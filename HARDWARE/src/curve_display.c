/**
 * @file    curve_display.c
 * @brief   迪文屏动态曲线显示模块实现 - STM32标准库(SPL)
 *
 * 绘制方式: 0x82写缓冲区数据 + 0x82更新0x300状态(指针+长度)
 */

#include "curve_display.h"
#include "dwin_uart.h"
#include "delay.h"

static CURVE_Display_t g_curve;

/* 通道0状态地址=0x300, 通道1=0x302, 通道2=0x304, 通道3=0x306 */
#define CH_STATUS_ADDR(ch)  (CURVE_STATUS_BASE + (ch) * 2)

static void CURVE_UpdateStatus(CURVE_Channel_t ch)
{
    uint8_t status[4];
    uint16_t ptr = g_curve.channels[ch].ptr;
    uint16_t len = ptr;   /* 每通道独立长度=本通道已写点数(不再用全局point_count) */
    if (len > CURVE_BUF_SIZE) len = CURVE_BUF_SIZE;

    status[0] = (ptr >> 8) & 0xFF;
    status[1] = ptr & 0xFF;
    status[2] = (len >> 8) & 0xFF;
    status[3] = len & 0xFF;
    DWIN_SendFrame(0x82, CH_STATUS_ADDR(ch), status, 4);
}

uint8_t CURVE_Init(void)
{
    g_curve.channels[CURVE_CH0].buf_base = CURVE_BUF_CH0;
    g_curve.channels[CURVE_CH0].ptr = 0;
    g_curve.channels[CURVE_CH0].value = 0;
    g_curve.channels[CURVE_CH0].enabled = 1;

    g_curve.channels[CURVE_CH1].buf_base = CURVE_BUF_CH1;
    g_curve.channels[CURVE_CH1].ptr = 0;
    g_curve.channels[CURVE_CH1].value = 0;
    g_curve.channels[CURVE_CH1].enabled = 0;   /* 只画压力, 停用温度/目标线 */

    g_curve.channels[CURVE_CH2].buf_base = 0x2000;
    g_curve.channels[CURVE_CH2].ptr = 0;
    g_curve.channels[CURVE_CH2].value = 0;
    g_curve.channels[CURVE_CH2].enabled = 0;

    g_curve.channels[CURVE_CH3].buf_base = 0x2800;
    g_curve.channels[CURVE_CH3].ptr = 0;
    g_curve.channels[CURVE_CH3].value = 0;
    g_curve.channels[CURVE_CH3].enabled = 0;

    g_curve.sample_interval = CURVE_SAMPLE_INTERVAL;
    g_curve.last_sample_time = 0;
    g_curve.paused = 1;
    g_curve.point_count = 0;

    return 0;
}

uint8_t CURVE_EnableChannel(CURVE_Channel_t ch)
{
    if (ch >= CURVE_CHANNEL_MAX) return 1;
    g_curve.channels[ch].enabled = 1;
    return 0;
}

uint8_t CURVE_DisableChannel(CURVE_Channel_t ch)
{
    if (ch >= CURVE_CHANNEL_MAX) return 1;
    g_curve.channels[ch].enabled = 0;
    return 0;
}

void CURVE_Update(float current_temp, float target_temp)
{
    uint32_t current_time;

    if (g_curve.paused) return;

    current_time = GetTick();
    if (current_time - g_curve.last_sample_time < g_curve.sample_interval) return;
    g_curve.last_sample_time = current_time;

    if (g_curve.channels[CURVE_CH0].enabled) {
        CURVE_WriteTemp(CURVE_CH0, current_temp);
    }
    if (g_curve.channels[CURVE_CH1].enabled) {
        CURVE_WriteTemp(CURVE_CH1, target_temp);
    }
}

/* 只绘制压力: CH0 写入 压力×5 (0-16Bar → 0-80, 适配Y轴0-100) */
void CURVE_UpdatePressure(float pressure_bar)
{
    uint32_t current_time;
    float scaled;

    if (g_curve.paused) return;

    current_time = GetTick();
    if (current_time - g_curve.last_sample_time < g_curve.sample_interval) return;
    g_curve.last_sample_time = current_time;

    if (pressure_bar < 0.0f) pressure_bar = 0.0f;
    scaled = pressure_bar * CURVE_PRES_SCALE;
    if (scaled > (float)CURVE_Y_MAX) scaled = (float)CURVE_Y_MAX;

    CURVE_WritePoint(CURVE_CH0, (uint16_t)scaled);
}

/* 双通道对比曲线: CH1 预铺完整参考曲线 (之后长度不再变), CH0 出水时用 0x84 实时追加.
 * 两条曲线的 X 步长由 main.c 的 Curve_ApplyXStep 同时设置, 保证始终一致. */
static uint16_t CURVE_ScalePres(float bar)
{
    float scaled;
    if (bar < 0.0f) bar = 0.0f;
    scaled = bar * CURVE_PRES_SCALE;
    if (scaled > (float)CURVE_Y_MAX) scaled = (float)CURVE_Y_MAX;
    return (uint16_t)scaled;
}

/* 追加一个实时采样点: CH0 写 real_bar (0x84, ch_mode=0x01).
 * CH1 保持预铺长度不动: 实际曲线录过预设结束点时, 预设曲线就停在它的最后一点, 不补 0. */
void CURVE_DualAppend(float real_bar, float ref_bar)
{
    uint16_t ch0_data[1];
    (void)ref_bar;   /* 参考曲线已预铺, 这里忽略 */

    ch0_data[0] = CURVE_ScalePres(real_bar);
    DWIN_WriteCurve(0x01, ch0_data, 1);
}

/* 配置曲线 X 轴间隔(像素步长) + 数据源通道: 写 SP+9 偏移的 u16 值.
 * u16 = (src_ch << 8) | step, 即高字节=数据源通道 (0-7), 低字节=像素步长. */
void CURVE_SetXStep(uint16_t sp_addr, uint8_t step, uint8_t src_ch)
{
    uint16_t vp   = (uint16_t)(sp_addr + 9u);
    uint16_t word = (uint16_t)(((uint16_t)src_ch << 8) | (uint16_t)step);
    DWIN_WriteVar(vp, word);
}

/* X 坐标标签控件地址表: 0xD254 起, 每 4 字节一变量, 共 10 个 */
static const uint16_t kCurveLabelVP[10] = {
    0xD254, 0xD258, 0xD25C, 0xD260, 0xD264,
    0xD268, 0xD26C, 0xD270, 0xD274, 0xD278,
};

/* 按当前步长重写 10 个 X 坐标标签控件:
 *   step=4 (量程 50s):  5, 10, 15, ..., 50  (步长大, 同屏时间短)
 *   step=2 (量程 100s): 10, 20, 30, ..., 100 (步长小, 同屏时间长)
 *   其它 step → 不动 (避免显示错位) */
void CURVE_SetXLabels(uint8_t step)
{
    uint16_t base_val = 0;
    uint16_t step_val = 0;
    uint8_t  i;

    if (step == 4) {
        base_val = 5;   step_val = 5;   /* 第 i 个标签 = base_val + i * step_val */
    } else if (step == 2) {
        base_val = 10;  step_val = 10;
    } else {
        return;
    }

    for (i = 0; i < 10; i++) {
        uint16_t v = (uint16_t)(base_val + (uint16_t)i * step_val);
        DWIN_WriteVar(kCurveLabelVP[i], v);
    }
}

/* 只绘制温度: CH0 写入 温度×2/3 (0-150℃ → 0-100, 适配Y轴0-100) */
void CURVE_UpdateTemp(float temp_c)
{
    uint32_t current_time;
    float scaled;

    if (g_curve.paused) return;

    current_time = GetTick();
    if (current_time - g_curve.last_sample_time < g_curve.sample_interval) return;
    g_curve.last_sample_time = current_time;

    if (temp_c < 0.0f) temp_c = 0.0f;
    scaled = temp_c * CURVE_TEMP_SCALE;
    if (scaled > (float)CURVE_Y_MAX) scaled = (float)CURVE_Y_MAX;

    CURVE_WritePoint(CURVE_CH0, (uint16_t)scaled);
}

void CURVE_WritePoint(CURVE_Channel_t ch, uint16_t value)
{
    if (ch >= CURVE_CHANNEL_MAX) return;
    if (!g_curve.channels[ch].enabled) return;

    if (value > CURVE_Y_MAX) value = CURVE_Y_MAX;

    g_curve.channels[ch].value = value;

    /* 写数据到缓冲区当前位置 */
    DWIN_WriteVar(g_curve.channels[ch].buf_base + g_curve.channels[ch].ptr, value);

    /* 更新指针, 循环写入 */
    g_curve.channels[ch].ptr++;
    if (g_curve.channels[ch].ptr >= CURVE_BUF_SIZE) {
        g_curve.channels[ch].ptr = 0;
    }

    /* 递增总数据点计数 */
    g_curve.point_count++;

    /* 更新状态: 指针位置 + 有效数据长度 */
    CURVE_UpdateStatus(ch);
}

void CURVE_WriteTemp(CURVE_Channel_t ch, float temp)
{
    uint16_t val;

    if (temp < (float)CURVE_Y_MIN) temp = (float)CURVE_Y_MIN;
    if (temp > (float)CURVE_Y_MAX) temp = (float)CURVE_Y_MAX;

    val = (uint16_t)temp;
    CURVE_WritePoint(ch, val);
}

void CURVE_Clear(void)
{
    uint8_t i;

    for (i = 0; i < CURVE_CHANNEL_MAX; i++) {
        g_curve.channels[i].ptr = 0;
        g_curve.channels[i].value = 0;
    }
    g_curve.point_count = 0;

    /* 重置所有通道状态 */
    for (i = 0; i < CURVE_CHANNEL_MAX; i++) {
        if (g_curve.channels[i].enabled) {
            CURVE_UpdateStatus((CURVE_Channel_t)i);
        }
    }
}

void CURVE_ClearChannel(CURVE_Channel_t ch)
{
    if (ch >= CURVE_CHANNEL_MAX) return;
    g_curve.channels[ch].ptr = 0;
    g_curve.channels[ch].value = 0;
    CURVE_UpdateStatus(ch);
}

/* 预铺参考曲线(定点档位)到 CH1:
 *   Y = raw[i] * mul_x100 / 100, 钳位到 [CURVE_Y_MIN, CURVE_Y_MAX]
 * 例: 档位(0~40) 1档=1.25Hz, Y=Hz x 10 -> mul_x100=1250 (档位40 -> Y=500)
 * 每点横向重复 repeat 次(拉宽), 分块批量快速写入CH1.
 * 每帧最多写CHUNK_WORDS个点, 避免单帧过长被屏截断; 全部写完再更新一次状态. */
void CURVE_DrawRefFixed(const uint8_t *raw, uint16_t count, uint16_t mul_x100, uint8_t repeat)
{
    #define CHUNK_WORDS    100    /* 每帧点数(每点2字节, 100点=200字节) */

    uint8_t buf[CHUNK_WORDS * 2];
    uint8_t status[4];
    uint16_t i, r;
    uint16_t ptr = 0;       /* CH1缓冲区总写入点数 */
    uint16_t chunk_n = 0;   /* 当前块已积累点数 */
    uint16_t chunk_base = 0;

    if (raw == NULL || count == 0 || repeat == 0) return;

    g_curve.channels[CURVE_CH1].enabled = 1;
    g_curve.channels[CURVE_CH1].ptr = 0;

    for (i = 0; i < count; i++) {
        uint32_t scaled = (uint32_t)raw[i] * (uint32_t)mul_x100 / 100u;
        uint16_t val;
        if (scaled > (uint32_t)CURVE_Y_MAX) scaled = (uint32_t)CURVE_Y_MAX;
        val = (uint16_t)scaled;

        for (r = 0; r < repeat; r++) {
            if (ptr >= CURVE_BUF_SIZE) break;
            buf[chunk_n * 2]     = (val >> 8) & 0xFF;
            buf[chunk_n * 2 + 1] = val & 0xFF;
            chunk_n++;
            ptr++;

            /* 块满 → 发一帧 */
            if (chunk_n >= CHUNK_WORDS) {
                DWIN_SendFrame(0x82, CURVE_BUF_CH1 + chunk_base,
                               buf, (uint8_t)(chunk_n * 2));
                delay_ms(5);
                chunk_base += chunk_n;
                chunk_n = 0;
            }
        }
    }

    /* 发送剩余不满一块的数据 */
    if (chunk_n > 0) {
        DWIN_SendFrame(0x82, CURVE_BUF_CH1 + chunk_base,
                       buf, (uint8_t)(chunk_n * 2));
        delay_ms(5);
    }

    /* 一次性更新状态: 指针=长度=总点数 */
    status[0] = (ptr >> 8) & 0xFF;
    status[1] = ptr & 0xFF;
    status[2] = (ptr >> 8) & 0xFF;
    status[3] = ptr & 0xFF;
    DWIN_SendFrame(0x82, CH_STATUS_ADDR(CURVE_CH1), status, 4);

    g_curve.channels[CURVE_CH1].ptr = ptr;
}

void CURVE_SetSampleInterval(uint32_t interval)
{
    if (interval < CURVE_SAMPLE_FAST) interval = CURVE_SAMPLE_FAST;
    if (interval > CURVE_SAMPLE_SLOW) interval = CURVE_SAMPLE_SLOW;
    g_curve.sample_interval = interval;
}

uint32_t CURVE_GetSampleInterval(void) { return g_curve.sample_interval; }

void CURVE_Pause(void) { g_curve.paused = 1; }

void CURVE_Resume(void)
{
    g_curve.paused = 0;
    g_curve.last_sample_time = GetTick();
}

uint8_t CURVE_IsPaused(void) { return g_curve.paused; }

uint16_t CURVE_GetPointCount(void) { return g_curve.point_count; }

void CURVE_ResetPointCount(void) { g_curve.point_count = 0; }

void CURVE_WriteHistory(CURVE_Channel_t ch, const uint16_t *data, uint16_t count)
{
    uint16_t i;

    if (ch >= CURVE_CHANNEL_MAX || data == NULL || count == 0) return;
    if (!g_curve.channels[ch].enabled) return;

    for (i = 0; i < count; i++) {
        uint16_t val = data[i];
        if (val > CURVE_Y_MAX) val = CURVE_Y_MAX;

        DWIN_WriteVar(g_curve.channels[ch].buf_base + g_curve.channels[ch].ptr, val);
        g_curve.channels[ch].ptr++;
        if (g_curve.channels[ch].ptr >= CURVE_BUF_SIZE) {
            g_curve.channels[ch].ptr = 0;
        }
        g_curve.point_count++;
        delay_ms(5);
    }

    CURVE_UpdateStatus(ch);
}
