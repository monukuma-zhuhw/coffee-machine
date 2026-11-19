/**
 * @file    dwin_uart.h
 * @brief   迪文屏 UART 通信接口 (DGUS II协议) - STM32F0 SPL
 */

#ifndef __DWIN_UART_H
#define __DWIN_UART_H

#include "sys.h"
#include <stdint.h>

/* UART 配置参数 - 由 sys.h 中的 UART_SWAP 宏控制 */
#define DWIN_USART                    DWIN_HW_USART
#define DWIN_USART_TX_PIN            DWIN_HW_TX_PIN
#define DWIN_USART_RX_PIN            DWIN_HW_RX_PIN
#define DWIN_USART_GPIO_PORT         GPIOA
#define DWIN_USART_AF                DWIN_HW_AF

/* 默认波特率 */
#define DWIN_DEFAULT_BAUDRATE         115200

/* ---------- 串口回显测试功能 ----------
 * 开启后 DWIN 串口收到的每个字节原样发回(中断内直接回发),
 * 且不再进入 DGUS 帧解析. 用于串口链路/波特率测试.
 * 上电默认状态由此宏决定 (编译期固定, 无运行时开关). */
#define DWIN_ECHO_TEST_ENABLE         0

/* DGUS II 协议命令字 */
#define DWIN_CMD_WRITE_REG            0x80
#define DWIN_CMD_READ_REG             0x81
#define DWIN_CMD_WRITE_VAR            0x82
#define DWIN_CMD_READ_VAR             0x83
#define DWIN_CMD_WRITE_CURVE          0x84
#define DWIN_CMD_WRITE_PIC            0xA5
#define DWIN_CMD_PLAY_MUSIC           0xA3

/* DGUS II 系统VP地址 (通过0x82写入) */
#define DWIN_SYS_VP_BACKLIGHT         0x0082
#define DWIN_SYS_VP_PAGE_SWITCH       0x0084

/* 旧寄存器地址 (0x80/0x81) */
#define DWIN_REG_BACKLIGHT            0x0001
#define DWIN_REG_PAGE_ID              0x0003
#define DWIN_REG_CUR_PAGE             0x0004

/* DGUS II 协议帧头 */
#define DWIN_FRAME_HEADER0            0x5A
#define DWIN_FRAME_HEADER1            0xA5

/* 帧状态 */
typedef enum {
    DWIN_FRAME_STATE_HEADER0 = 0,
    DWIN_FRAME_STATE_HEADER1,
    DWIN_FRAME_STATE_LENGTH,
    DWIN_FRAME_STATE_CMD,
    DWIN_FRAME_STATE_DATA
} DWIN_FrameState_t;

/* DGUS 接收帧结构 */
typedef struct {
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  data[256];
    uint8_t  data_index;
} DWIN_RecvFrame_t;

/* 触控事件回调函数类型 */
typedef void (*DWIN_TouchCallback_t)(uint8_t icon_id);

/* VP读取回调函数类型 */
typedef void (*DWIN_VPReadCallback_t)(uint16_t vp_addr, uint16_t value);

void DWIN_UART_Init(uint32_t baudrate);
void DWIN_UART_SendByte(uint8_t data);
void DWIN_UART_SendData(const uint8_t *data, uint16_t len);
void DWIN_SendFrame(uint8_t cmd, uint16_t vp_addr, const uint8_t *data, uint8_t data_len);
void DWIN_ReadVar(uint16_t vp_addr, uint8_t data_len);
void DWIN_SwitchPage(uint16_t page_id);
void DWIN_SetBacklight(uint8_t brightness, uint8_t standby_brightness, uint16_t standby_time_ms);
void DWIN_WriteVar(uint16_t vp_addr, uint16_t value);
void DWIN_WriteVarFloat(uint16_t vp_addr, float value);
void DWIN_WritePic(uint16_t vp_addr, const uint8_t *data, uint16_t data_len);
void DWIN_WriteCurve(uint8_t ch_mode, const uint16_t *data, uint8_t word_count);
void DWIN_SetTouchVP(uint8_t icon_id, uint16_t vp_addr);
void DWIN_RegisterTouchCallback(DWIN_TouchCallback_t callback);
void DWIN_RegisterVPReadCallback(DWIN_VPReadCallback_t callback);
void DWIN_USART_IRQHandler(void);
void DWIN_ProcessRecvFrame(void);
void (*DWIN_GetDebugSendFunc(void))(uint8_t);

/* 本地主动触发蜂鸣器: 写 VP=0x00A0=0x000D → 屏蜂鸣器响 104ms (13*8ms).
 * 用于面板按键 (KEY1/KEY2/KEY3) 按下等本地事件. MCU 无硬件蜂鸣器, 由迪文屏扬声器发声. */
void DWIN_BuzzerPlayOnce(void);

/* 屏幕参数 */
#define DWIN_SCREEN_WIDTH       400
#define DWIN_SCREEN_HEIGHT      960

/* 迪文屏VP地址 */
#define DWIN_VP_ADDR_TEMP        0x2000
#define DWIN_VP_ADDR_STATE       0x2001
#define DWIN_VP_ADDR_POWER       0x2002
#define DWIN_VP_ADDR_TARGET_TEMP 0x2003
#define DWIN_VP_ADDR_TIMER       0x2004
#define DWIN_VP_ADDR_COUNT       0x2005
#define DWIN_VP_ADDR_PRESSURE    0x2006
#define DWIN_VP_ADDR_PUMP_POWER  0x2007
#define DWIN_VP_ADDR_PREINFUSION 0xD000   /* 预浸泡值 */
#define DWIN_VP_ADDR_CUR_TEMP    0xD004   /* 温度值 */
#define DWIN_VP_ADDR_CUR_PRES    0xD008   /* 压力值 */
#define DWIN_VP_ADDR_BREW_ELAPSED 0xD00C  /* 萃取出水后经过时间(秒, BEV_BREW_RUN 开始后每秒刷新) */
#define DWIN_VP_ADDR_HEAD_TEMP   0xD010   /* 冲泡头(出水口)温度 */
#define DWIN_VP_ADDR_SEL_A       0xD100   /* 选择控件A */
#define DWIN_VP_ADDR_SEL_B       0xD104   /* 选择控件B */
#define DWIN_VP_ADDR_MODE_FLAG   0xD180   /* 模式标志: A选中=1, B选中=0 */

/* 曲线缓冲区系统接口 */
#define DWIN_CURVE_STATUS_BASE   0x0300
#define DWIN_CURVE_TRIGGER       0x0310
#define DWIN_CURVE_DATABLOCK     0x0312
#define DWIN_CURVE_CH0_BUF       0x1000
#define DWIN_CURVE_CH1_BUF       0x1800

void DWIN_DisplayTemperature(float temp);
void DWIN_DisplayTargetTemp(float temp);
void DWIN_DisplayPower(uint8_t power);
void DWIN_DisplayBrewState(uint8_t state);
void DWIN_DisplayBrewTimer(uint32_t time);
void DWIN_DisplayBrewCount(uint8_t count);
void DWIN_DisplayPressure(float pressure);
void DWIN_DisplayPumpPower(uint8_t power);
void DWIN_SetText(uint16_t vp_addr, const char *utf8);  /* 传UTF-8串, 内部转GBK/ASCII写入 */
void DWIN_SetTextColor(uint16_t sp, uint16_t color);     /* 写SP+3偏移处的颜色值 */

#endif
