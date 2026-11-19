/**
 * @file    dwin_uart.c
 * @brief   迪文屏 UART 通信实现 (DGUS II协议) - STM32F0 SPL
 */

#include "dwin_uart.h"
#include <stdio.h>
#include <string.h>

/* 全局变量 */
static DWIN_FrameState_t  g_frame_state = DWIN_FRAME_STATE_HEADER0;
/* 帧组装缓冲: ISR 按字节拼装一帧, 拼完拷进下面的帧队列 */
static DWIN_RecvFrame_t   g_recv_frame;
static DWIN_TouchCallback_t g_touch_callback = NULL;
static DWIN_VPReadCallback_t g_vpread_callback = NULL;

/* 已收完整帧队列: ISR 拼好一帧拷进队列, 主循环 DWIN_ProcessRecvFrame 一次取完分发.
 *
 * 以前只有一个帧缓冲 + 就绪标志: 主循环被阻塞时(每秒的屏显刷新+打印、切页后的
 * delay_ms(300)、设置落盘 ~65ms), 只要屏连发两帧, 前一帧就被后一帧覆盖 ——
 * 屏上按键的上报就这样丢了(改完设置 1~2 秒落盘时最容易碰上);
 * 主循环读帧时中断还可能正往同一缓冲写下一帧, 读到半新半旧的数据.
 *
 * 只存分发要用的前几个数据字节(VP 2 + 字数 1 + 值 2). 屏对 0x82 写指令的
 * "OK" 应答(5A A5 03 82 4F 4B)分发用不到, 不入队, 免得占满队列. */
#define DWIN_FQ_SIZE       16u     /* 队列深度, 必须是 2 的幂 */
#define DWIN_FQ_DATA_MAX   6u
typedef struct {
    uint8_t cmd;
    uint8_t n;                         /* 有效数据字节数 */
    uint8_t d[DWIN_FQ_DATA_MAX];
} DWIN_QFrame_t;
static volatile DWIN_QFrame_t g_fq[DWIN_FQ_SIZE];
static volatile uint8_t g_fq_head = 0;   /* ISR 写入位置 */
static volatile uint8_t g_fq_tail = 0;   /* 主循环读取位置 */

/* 串口回显测试开关 (默认由 DWIN_ECHO_TEST_ENABLE 决定) */
static volatile uint8_t g_echo_enable = DWIN_ECHO_TEST_ENABLE;

static uint8_t g_vp_to_icon_map[32];
static uint8_t g_vp_count = 0;
static uint16_t g_last_read_vp = 0xFFFF;
static uint8_t g_data_prev_5a = 0;   /* 数据区上一个字节是 0x5A (用于识别 5A A5 帧头) */

/**
 * @brief 初始化迪文屏 UART 通信
 * @note 引脚和USART由 sys.h 中 UART_SWAP 宏控制
 */
void DWIN_UART_Init(uint32_t baudrate)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    if (DWIN_HW_RCC == RCC_APB2Periph_USART1)
        RCC_APB2PeriphClockCmd(DWIN_HW_RCC, ENABLE);
    else
        RCC_APB1PeriphClockCmd(DWIN_HW_RCC, ENABLE);

    GPIO_PinAFConfig(GPIOA, DWIN_HW_TX_SRC, DWIN_HW_AF);
    GPIO_PinAFConfig(GPIOA, DWIN_HW_RX_SRC, DWIN_HW_AF);

    /* TX */
    GPIO_InitStructure.GPIO_Pin = DWIN_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DWIN_USART_GPIO_PORT, &GPIO_InitStructure);

    /* RX */
    GPIO_InitStructure.GPIO_Pin = DWIN_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(DWIN_USART_GPIO_PORT, &GPIO_InitStructure);

    /* USART配置 */
    USART_InitStructure.USART_BaudRate = baudrate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(DWIN_USART, &USART_InitStructure);

    /* 中断配置 */
    NVIC_InitStructure.NVIC_IRQChannel = DWIN_HW_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(DWIN_USART, USART_IT_RXNE, ENABLE);
    USART_Cmd(DWIN_USART, ENABLE);

    memset(&g_recv_frame, 0, sizeof(g_recv_frame));
    memset(g_vp_to_icon_map, 0xFF, sizeof(g_vp_to_icon_map));
}

void DWIN_UART_SendByte(uint8_t data)
{
    while (USART_GetFlagStatus(DWIN_USART, USART_FLAG_TXE) == RESET);
    USART_SendData(DWIN_USART, data);
}

void DWIN_UART_SendData(const uint8_t *data, uint16_t len)
{
    while (len--) {
        DWIN_UART_SendByte(*data++);
    }
}

void DWIN_SendFrame(uint8_t cmd, uint16_t vp_addr, const uint8_t *data, uint8_t data_len)
{
    uint8_t frame[256];
    uint16_t index = 0;

    frame[index++] = DWIN_FRAME_HEADER0;
    frame[index++] = DWIN_FRAME_HEADER1;
    frame[index++] = data_len + 3;
    frame[index++] = cmd;
    frame[index++] = (vp_addr >> 8) & 0xFF;
    frame[index++] = vp_addr & 0xFF;

    if (data && data_len > 0) {
        memcpy(&frame[index], data, data_len);
        index += data_len;
    }

    DWIN_UART_SendData(frame, index);
}

void DWIN_ReadVar(uint16_t vp_addr, uint8_t data_len)
{
    uint8_t frame[7];
    g_last_read_vp = vp_addr;
    frame[0] = DWIN_FRAME_HEADER0;
    frame[1] = DWIN_FRAME_HEADER1;
    frame[2] = 0x04;
    frame[3] = DWIN_CMD_READ_VAR;
    frame[4] = (vp_addr >> 8) & 0xFF;
    frame[5] = vp_addr & 0xFF;
    frame[6] = data_len;
    DWIN_UART_SendData(frame, 7);
}

void DWIN_SwitchPage(uint16_t page_id)
{
    uint8_t data[4];
    data[0] = 0x5A;
    data[1] = 0x01;
    data[2] = (page_id >> 8) & 0xFF;
    data[3] = page_id & 0xFF;
    DWIN_SendFrame(DWIN_CMD_WRITE_VAR, DWIN_SYS_VP_PAGE_SWITCH, data, 4);
}

void DWIN_SetBacklight(uint8_t brightness, uint8_t standby_brightness, uint16_t standby_time_ms)
{
    uint8_t data[4];
    data[0] = brightness;
    data[1] = standby_brightness;
    data[2] = (standby_time_ms >> 8) & 0xFF;
    data[3] = standby_time_ms & 0xFF;
    DWIN_SendFrame(DWIN_CMD_WRITE_VAR, DWIN_SYS_VP_BACKLIGHT, data, 4);
}

void DWIN_WriteVar(uint16_t vp_addr, uint16_t value)
{
    uint8_t data[2];
    data[0] = (value >> 8) & 0xFF;
    data[1] = value & 0xFF;
    DWIN_SendFrame(DWIN_CMD_WRITE_VAR, vp_addr, data, 2);
}

/* 写4字节单精度浮点变量 (IEEE754, 高字节在前) */
void DWIN_WriteVarFloat(uint16_t vp_addr, float value)
{
    uint8_t data[4];
    uint32_t u;
    memcpy(&u, &value, 4);
    data[0] = (u >> 24) & 0xFF;
    data[1] = (u >> 16) & 0xFF;
    data[2] = (u >> 8) & 0xFF;
    data[3] = u & 0xFF;
    DWIN_SendFrame(DWIN_CMD_WRITE_VAR, vp_addr, data, 4);
}

void DWIN_WritePic(uint16_t vp_addr, const uint8_t *data, uint16_t data_len)
{
    uint16_t offset = 0;
    uint16_t current_vp = vp_addr;

    while (offset < data_len) {
        uint16_t remaining = data_len - offset;
        uint8_t chunk_len = (remaining > 248) ? 248 : (uint8_t)remaining;

        DWIN_SendFrame(DWIN_CMD_WRITE_PIC, current_vp, data + offset, chunk_len);

        offset += chunk_len;
        current_vp += chunk_len / 2;

        uint32_t delay_cnt = 20000;
        while (delay_cnt--);
    }
}

void DWIN_WriteCurve(uint8_t ch_mode, const uint16_t *data, uint8_t word_count)
{
    uint8_t ch_count = 0;
    uint8_t i, ch;
    uint16_t index = 0;
    uint8_t frame[256];
    uint16_t total_words;
    uint16_t data_bytes;

    for (i = 0; i < 8; i++) {
        if (ch_mode & (1 << i)) ch_count++;
    }
    if (ch_count == 0 || word_count == 0) return;

    total_words = (uint16_t)word_count * ch_count;
    data_bytes = total_words * 2;

    if (data_bytes + 1 + 5 > 256) return;

    frame[0] = DWIN_FRAME_HEADER0;
    frame[1] = DWIN_FRAME_HEADER1;
    frame[2] = (uint8_t)(data_bytes + 2);
    frame[3] = DWIN_CMD_WRITE_CURVE;
    frame[4] = ch_mode;
    index = 5;

    /* 大端序(高字节在前), data按活跃通道顺序排列 */
    for (i = 0; i < word_count; i++) {
        uint8_t active_ch = 0;
        for (ch = 0; ch < 8; ch++) {
            if (ch_mode & (1 << ch)) {
                uint16_t val = data[active_ch * word_count + i];
                frame[index++] = (val >> 8) & 0xFF;
                frame[index++] = val & 0xFF;
                active_ch++;
            }
        }
    }

    DWIN_UART_SendData(frame, index);
}

void DWIN_SetTouchVP(uint8_t icon_id, uint16_t vp_addr)
{
    if (g_vp_count < 32) {
        g_vp_to_icon_map[vp_addr & 0x1F] = icon_id;
        g_vp_count++;
    }
}

void DWIN_RegisterTouchCallback(DWIN_TouchCallback_t callback)
{
    g_touch_callback = callback;
}

void DWIN_RegisterVPReadCallback(DWIN_VPReadCallback_t callback)
{
    g_vpread_callback = callback;
}

/* ISR 内调用: g_recv_frame 收完一帧, 拷进帧队列 (只收分发需要的帧) */
static void DWIN_RxFramePush(void)
{
    volatile DWIN_QFrame_t *q;
    uint8_t next;
    uint8_t n;
    uint8_t i;

    /* 分发只用 0x83 读应答/自动上传(≥5 字节数据) 和 0x82 主动上报(≥4 字节数据) */
    if (!((g_recv_frame.cmd == DWIN_CMD_READ_VAR  && g_recv_frame.data_index >= 5) ||
          (g_recv_frame.cmd == DWIN_CMD_WRITE_VAR && g_recv_frame.data_index >= 4))) {
        return;
    }
    next = (uint8_t)((g_fq_head + 1u) & (DWIN_FQ_SIZE - 1u));
    if (next == g_fq_tail) {
        return;   /* 队列满(主循环太久没取): 丢弃本帧 */
    }
    q = &g_fq[g_fq_head];
    n = (g_recv_frame.data_index < DWIN_FQ_DATA_MAX) ? g_recv_frame.data_index
                                                      : (uint8_t)DWIN_FQ_DATA_MAX;
    q->cmd = g_recv_frame.cmd;
    q->n   = n;
    for (i = 0; i < n; i++) {
        q->d[i] = g_recv_frame.data[i];
    }
    g_fq_head = next;   /* 数据写完再发布 */
}

void DWIN_USART_IRQHandler(void)
{
    uint8_t data;
    uint8_t overrun = 0;

    /* 接收溢出(ORE)必须在这里清, 否则整机卡死.
     *
     * 写 Flash 时关中断/取指停顿 20~40ms, 屏这期间发来的字节必然溢出
     * (USART 只有 1 字节接收缓冲). F0 的 ORE 只能写 ICR.ORECF 清, 读 RDR
     * 清不掉; RXNEIE=1 时 ORE 会一直申请中断 —— 不清的话本函数每次进来
     * RXNE=0 什么都不做就退出, 又立刻重入, 主循环被永久饿死.
     * 溢出说明当前帧已丢字节: RDR 里溢出前的那个字节照常解析(它可能正好
     * 补完一帧), 之后丢弃未完成的帧, 从下一个帧头重新同步. */
    if (USART_GetFlagStatus(DWIN_USART, USART_FLAG_ORE) != RESET) {
        USART_ClearFlag(DWIN_USART, USART_FLAG_ORE);
        overrun = 1;
    }

    if (USART_GetITStatus(DWIN_USART, USART_IT_RXNE) != RESET) {
        data = USART_ReceiveData(DWIN_USART);

        /* 回显测试模式: 每个字节原样发回, 不做DGUS帧解析 */
        if (g_echo_enable) {
            while (USART_GetFlagStatus(DWIN_USART, USART_FLAG_TXE) == RESET);
            USART_SendData(DWIN_USART, data);
            return;
        }

        switch (g_frame_state) {
            case DWIN_FRAME_STATE_HEADER0:
                if (data == DWIN_FRAME_HEADER0)
                    g_frame_state = DWIN_FRAME_STATE_HEADER1;
                break;

            case DWIN_FRAME_STATE_HEADER1:
                if (data == DWIN_FRAME_HEADER1)
                    g_frame_state = DWIN_FRAME_STATE_LENGTH;
                else
                    g_frame_state = DWIN_FRAME_STATE_HEADER0;
                break;

            case DWIN_FRAME_STATE_LENGTH:
                g_recv_frame.len = data;
                g_frame_state = DWIN_FRAME_STATE_CMD;
                break;

            case DWIN_FRAME_STATE_CMD:
                g_recv_frame.cmd = data;
                g_recv_frame.data_index = 0;
                g_data_prev_5a = 0;
                if (g_recv_frame.len <= 1) {
                    /* 无数据的命令帧: 分发用不到, 直接等下一帧 */
                    g_frame_state = DWIN_FRAME_STATE_HEADER0;
                } else {
                    g_frame_state = DWIN_FRAME_STATE_DATA;
                }
                break;

            case DWIN_FRAME_STATE_DATA:
                /* 帧截断重同步: 数据区里出现完整帧头 5A A5, 说明当前帧的 len
                 * 不可信(丢字节/干扰), 丢掉当前帧, 按新帧从 LENGTH 继续解析.
                 *
                 * 只认 5A A5 两个字节. 单个 0x5A 是正常数据(例如萃取温度 90 = 0x5A):
                 * 以前遇到单个 0x5A 就重同步, 会把这一帧丢掉, 还会因为卡在等 0xA5
                 * 把紧跟的下一帧(可能就是一次触摸)一起吃掉, 屏也不会重发.
                 * 写 Flash 期间溢出丢字节的半帧由 ORE 处理直接丢弃, 不靠这里. */
                if (g_data_prev_5a && data == DWIN_FRAME_HEADER1) {
                    g_data_prev_5a = 0;
                    g_frame_state = DWIN_FRAME_STATE_LENGTH;
                    break;
                }
                g_data_prev_5a = (data == DWIN_FRAME_HEADER0) ? 1u : 0u;
                if (g_recv_frame.data_index < sizeof(g_recv_frame.data)) {
                    g_recv_frame.data[g_recv_frame.data_index] = data;
                } else {
                    /* 写入超界 (异常长帧 / len 字段被破坏 / EMI 干扰):
                     *   主动丢弃整帧, 回到 HEADER0 重新等下一个 0x5A 0xA5,
                     *   防止越界读 + 同步错位污染后续整批帧. */
                    g_recv_frame.data_index = 0;
                    g_frame_state = DWIN_FRAME_STATE_HEADER0;
                    break;
                }
                g_recv_frame.data_index++;
                /* 帧完成条件: 已收到 len-1 个数据 (匹配 DGUS 帧长度)
                 *   或 data_index 触顶 (异常长帧 / EMI 干扰) 立即标完成, 防越界读. */
                if (g_recv_frame.data_index >= g_recv_frame.len - 1
                    || g_recv_frame.data_index >= sizeof(g_recv_frame.data)) {
                    /* 帧完成: 入队, 主循环取 */
                    DWIN_RxFramePush();
                    g_frame_state = DWIN_FRAME_STATE_HEADER0;
                }
                break;

            default:
                g_frame_state = DWIN_FRAME_STATE_HEADER0;
                break;
        }
    }

    if (overrun) {
        g_frame_state = DWIN_FRAME_STATE_HEADER0;   /* 丢弃已缺字节的半帧 */
    }
}

void DWIN_ProcessRecvFrame(void)
{
    /* 一次取完队列里所有已收完的帧, 按 cmd 分发 (见 g_fq 说明).
     * 先把需要的字段拷到局部变量, 回调里可以放心收发, 取完这一格才归还给 ISR. */
    while (g_fq_tail != g_fq_head) {
        const volatile DWIN_QFrame_t *f = &g_fq[g_fq_tail];
        uint8_t  cmd     = f->cmd;
        uint16_t vp_addr = (uint16_t)((f->d[0] << 8) | f->d[1]);
        uint16_t value;
        uint8_t  icon_id;

        if (cmd == DWIN_CMD_READ_VAR) {
            /* 0x83帧: 既包括主动读的应答, 也包括控件"数据自动上传"的主动帧.
             * 两者格式相同, 统一交给VP读回调(回调内按VP地址区分处理).
             * d[2] = word数(每word=2字节) */
            value = (f->d[2] >= 1) ? (uint16_t)((f->d[3] << 8) | f->d[4]) : 0u;
        } else {
            /* 迪文屏主动上报: 0x82 [VP_H] [VP_L] [data...] */
            value = (uint16_t)((f->d[2] << 8) | f->d[3]);
        }
        g_fq_tail = (uint8_t)((g_fq_tail + 1u) & (DWIN_FQ_SIZE - 1u));

        if (g_vpread_callback) {
            g_vpread_callback(vp_addr, value);
        }
        if (cmd == DWIN_CMD_READ_VAR && vp_addr == g_last_read_vp) {
            g_last_read_vp = 0xFFFF;
        }
        /* 触摸按键映射: 若该VP绑定了图标, 同时触发触摸回调(如翻页按钮) */
        icon_id = g_vp_to_icon_map[vp_addr & 0x1F];
        if (icon_id != 0xFF && g_touch_callback) {
            g_touch_callback(icon_id);
        }
    }
}

void (*DWIN_GetDebugSendFunc(void))(uint8_t)
{
    return NULL;
}

/* 串口回显测试开关: 切换时复位帧解析状态, 避免残留半解析帧 */
void DWIN_DisplayTemperature(float temp)
{
    int16_t temp_int = (int16_t)(temp * 10.0f);
    DWIN_WriteVar(DWIN_VP_ADDR_TEMP, (uint16_t)temp_int);
}

void DWIN_DisplayTargetTemp(float temp)
{
    int16_t temp_int = (int16_t)(temp * 10.0f);
    DWIN_WriteVar(DWIN_VP_ADDR_TARGET_TEMP, (uint16_t)temp_int);
}

void DWIN_DisplayPower(uint8_t power)
{
    DWIN_WriteVar(DWIN_VP_ADDR_POWER, power);
}

void DWIN_DisplayBrewState(uint8_t state)
{
    DWIN_WriteVar(DWIN_VP_ADDR_STATE, state);
}

void DWIN_DisplayBrewTimer(uint32_t time)
{
    uint16_t time_sec = (uint16_t)(time / 1000);
    DWIN_WriteVar(DWIN_VP_ADDR_TIMER, time_sec);
}

void DWIN_DisplayBrewCount(uint8_t count)
{
    DWIN_WriteVar(DWIN_VP_ADDR_COUNT, count);
}

void DWIN_DisplayPressure(float pressure)
{
    uint16_t pres_int = (uint16_t)(pressure * 10.0f);
    DWIN_WriteVar(DWIN_VP_ADDR_PRESSURE, pres_int);
}

/* 设置文本显示控件内容: 写入ASCII字符串到文本数据VP.
 * str: ASCII字符串, 最长受帧长限制(0x82单帧数据≤248字节). */
/* 汉字Unicode→GBK 码表 (按需添加) */
static const uint16_t utf8_to_gbk_table[][2] = {
    {0x52A0, 0xBCD3},  /* 加 */
    {0x70ED, 0xC8C8},  /* 热 */
    {0x8403, 0xDDCD},  /* 萃 */
    {0x53D6, 0xC8A1},  /* 取 */
    {0x6E29, 0xCEC2},  /* 温 */
    {0x5EA6, 0xB6C8},  /* 度 */
    {0x51B2, 0xB3E5},  /* 冲 */
    {0x6CE1, 0xC5DD},  /* 泡 */
    {0x5934, 0xCDB7},  /* 头 */
    {0x676F, 0xB1AD},  /* 杯 */
    {0x6570, 0xCAFD},  /* 数 */
    {0x7EDF, 0xCDB3},  /* 统 */
    {0x8BA1, 0xBCC6},  /* 计 */
    {0x7167, 0xD5D5},  /* 照 */
    {0x660E, 0xC3F7},  /* 明 */
    {0x9884, 0xD4A4},  /* 预 */
    {0x6D78, 0xBDFE},  /* 浸 */
    {0x65F6, 0xCAB1},  /* 时 */
    {0x95F4, 0xBCE4},  /* 间 */
    {0x624B, 0xCAD6},  /* 手 */
    {0x52A8, 0xB6AF},  /* 动 */
    {0x6A21, 0xC4A3},  /* 模 */
    {0x5F0F, 0xCABD},  /* 式 */
    {0x53D8, 0xB1E4},  /* 变 */
    {0x538B, 0xD1B9},  /* 压 */
    {0x529B, 0xC1A6},  /* 力 */
    {0x9891, 0xC6B5},  /* 频 */
    {0x7387, 0xC2CA},  /* 率 */
    {0x706F, 0xB5C6},  /* 灯 */
    {0x5149, 0xB9E2},  /* 光 */
    {0x8C03, 0xB5F7},  /* 调 */
    {0x8282, 0xBDDA},  /* 节 */
    {0x51FA, 0xB3F6},  /* 出 */
    {0x5382, 0xB3A7},  /* 厂 */
    {0x8BBE, 0xC9E8},  /* 设 */
    {0x7F6E, 0xD6C3},  /* 置 */
    {0x503C, 0xD6B5},  /* 值 */
    {0x66F2, 0xC7FA},  /* 曲 */
    {0x7EBF, 0xCFDF},  /* 线 */
    {0x91CD, 0xD6D8},  /* 重 */
    {0x786E, 0xC8B7},  /* 确 */
    {0x8BA4, 0xC8CF},  /* 认 */
    {0x6062, 0xBBD6},  /* 恢 */
    {0x590D, 0xB8B4},  /* 复 */
    {0x65B0, 0xD0C2},  /* 新 */
    {0x636E, 0xBEDD},  /* 据 */
    {0x662F, 0xCAC7},  /* 是 */
    {0x5426, 0xB7F1},  /* 否 */
    {0x66FF, 0xCCE6},  /* 替 */
    {0x6362, 0xBBBB},  /* 换 */
    {0x5E76, 0xB2A2},  /* 并 */
    {0x4FDD, 0xB1A3},  /* 保 */
    {0x5B58, 0xB4E6},  /* 存 */
};

static uint16_t utf16_to_gbk(uint16_t ucs2)
{
    uint16_t i;
    for (i = 0; i < sizeof(utf8_to_gbk_table)/sizeof(utf8_to_gbk_table[0]); i++) {
        if (utf8_to_gbk_table[i][0] == ucs2) {
            return utf8_to_gbk_table[i][1];
        }
    }
    return 0;  /* 未找到返回0 */
}

/* 设置文本显示控件内容: 传入文本数据VP,
 * 写入UTF-8字符串(内部编码为GBK/ASCII).
 * 汉字按utf8_to_gbk_table查表转换(GBK大端序: 高字节在前); 英文/数字ASCII直通. */
void DWIN_SetText(uint16_t vp_addr, const char *utf8)
{
    uint8_t gbk_buf[128];
    uint16_t gbk_len = 0;
    const uint8_t *p = (const uint8_t *)utf8;
    uint16_t i;
    if (utf8 == NULL) return;

    printf("[DWIN_SetText] VP=0x%04X in=\"%s\" ", vp_addr, utf8);

    while (*p && gbk_len < sizeof(gbk_buf) - 5) {
        uint8_t c = *p;
        if (c < 0x80) {
            /* ASCII 英文/数字 */
            gbk_buf[gbk_len++] = c;
            p++;
        } else if ((c & 0xE0) == 0xC0 && (p[1] & 0xC0) == 0x80) {
            /* 2字节UTF-8 → GBK (BMP内) */
            uint16_t ucs2 = ((c & 0x1F) << 6) | (p[1] & 0x3F);
            uint16_t gbk = utf16_to_gbk(ucs2);
            if (gbk) {
                gbk_buf[gbk_len++] = (uint8_t)(gbk >> 8);
                gbk_buf[gbk_len++] = (uint8_t)(gbk & 0xFF);
            }
            /* 查表失败: 跳过该字符(避免出现"?"等乱字符) */
            p += 2;
        } else if ((c & 0xF0) == 0xE0 && (p[1] & 0xC0) == 0x80 && (p[2] & 0xC0) == 0x80) {
            /* 3字节UTF-8 (含中文常见) */
            uint16_t ucs2 = ((c & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F);
            uint16_t gbk = utf16_to_gbk(ucs2);
            if (gbk) {
                gbk_buf[gbk_len++] = (uint8_t)(gbk >> 8);
                gbk_buf[gbk_len++] = (uint8_t)(gbk & 0xFF);
            }
            /* 查表失败: 跳过 */
            p += 3;
        } else {
            /* 不合法UTF-8, 跳过1字节 */
            p++;
        }
    }

    /* 末尾追加结束符 0xFFFF: 示例显示在最后一个字符后加FFFF可清除后面残留文本.
     * (不用00 00, 避免干扰DGUS解析) */
    gbk_buf[gbk_len++] = 0xFF;
    gbk_buf[gbk_len++] = 0xFF;

    DWIN_SendFrame(DWIN_CMD_WRITE_VAR, vp_addr, gbk_buf, (uint8_t)gbk_len);

    printf("frame=5A A5 %02X 82 %02X %02X", (uint8_t)(gbk_len + 3),
           (uint8_t)(vp_addr >> 8), (uint8_t)(vp_addr & 0xFF));
    for (i = 0; i < gbk_len; i++) printf(" %02X", gbk_buf[i]);
    printf("\r\n");
}

/* 修改文本显示控件颜色: 写控件描述符(SP)+3偏移的2字节颜色值.
 * 例: 5A A5 05 82 88 03 F8 00 → DWIN_SetTextColor(0x8803, 0xF800) */
void DWIN_SetTextColor(uint16_t sp, uint16_t color)
{
    printf("[DWIN_SetTextColor] SP=0x%04X color=0x%04X -> VP=0x%04X\r\n",
           sp, color, (uint16_t)(sp + 3));
    DWIN_WriteVar((uint16_t)(sp + 3), color);
}

void DWIN_DisplayPumpPower(uint8_t power)
{
    DWIN_WriteVar(DWIN_VP_ADDR_PUMP_POWER, power);
}

/*==============================================================
 * 蜂鸣器触发 (迪文屏扬声器, MCU 无硬件蜂鸣器)
 *
 * DGUS 蜂鸣器 VP=0x00A0, value 单位 = 8ms.
 * 例 value=0x0D = 13 → 响 13 * 8ms = 104ms.
 *
 * 本地事件 (如面板 KEY1/KEY2/KEY3 按下) 调 DWIN_BuzzerPlayOnce() 即可.
 *==============================================================*/
#define DWIN_BUZZER_VP   0x00A0u
#define DWIN_BUZZER_DUR  0x000Du   /* 13 * 8ms = 104ms */

void DWIN_BuzzerPlayOnce(void)
{
    DWIN_WriteVar(DWIN_BUZZER_VP, DWIN_BUZZER_DUR);
}
