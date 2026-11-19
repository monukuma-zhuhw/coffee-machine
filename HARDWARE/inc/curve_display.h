/**
 * @file    curve_display.h
 * @brief   迪文屏动态曲线显示模块接口 - STM32标准库(SPL)
 *
 * 曲线绘制原理:
 * - 每通道2Kwords缓冲区: CH0=0x1000, CH1=0x1800
 * - 通过0x82写缓冲区数据 + 更新0x300状态(指针+长度)
 * - DGUS曲线控件从缓冲区读取数据并显示
 * - Y轴取值范围0-100, 温度值直接写入(℃整数)
 */

#ifndef __CURVE_DISPLAY_H
#define __CURVE_DISPLAY_H

#include "sys.h"
#include <stdint.h>

/* 曲线通道数量 */
#define CURVE_CHANNEL_MAX       4

/* 曲线缓冲区基地址 (每通道2Kwords, 间隔0x800) */
#define CURVE_BUF_CH0           0x1000
#define CURVE_BUF_CH1           0x1800
#define CURVE_BUF_SIZE          0x0800  /* 每通道2Kwords */

/* 0x300-0x30F: 8通道状态, 每通道2字(指针+长度) */
#define CURVE_STATUS_BASE       0x0300

/* Y轴范围 (需与DGUS工程配置一致)
 * 当前变压模式 Y = 水泵频率(Hz) × 10 → 0~50Hz 对应 0~500 */
#define CURVE_Y_MIN             0
#define CURVE_Y_MAX             500

/* 曲线缩放: 输入值 ×10 再绘制
 *   变压模式: 频率(Hz) ×10 → 0~500 (频率 0~50Hz)
 *   压力模式: 压力(Bar) ×10 → 0~160 (压力 0~16Bar) */
#define CURVE_PRES_SCALE        10

/* 温度曲线缩放: 温度(℃)×0.6667 映射到Y轴 (0-150℃ → 0-100) */
#define CURVE_TEMP_SCALE        0.6667f

/* 采样配置 */
#define CURVE_SAMPLE_INTERVAL   100
#define CURVE_SAMPLE_FAST       100
#define CURVE_SAMPLE_SLOW       2000

typedef enum {
    CURVE_CH0 = 0,
    CURVE_CH1,
    CURVE_CH2,
    CURVE_CH3,
} CURVE_Channel_t;

typedef struct {
    uint16_t buf_base;       /* 缓冲区基地址 */
    uint16_t ptr;            /* 当前写入指针 */
    uint16_t value;          /* 当前值 */
    uint8_t  enabled;        /* 通道使能标志 */
} CURVE_ChannelConfig_t;

typedef struct {
    CURVE_ChannelConfig_t channels[CURVE_CHANNEL_MAX];
    uint32_t sample_interval;
    uint32_t last_sample_time;
    uint8_t  paused;
    uint16_t point_count;
} CURVE_Display_t;

uint8_t CURVE_Init(void);
uint8_t CURVE_EnableChannel(CURVE_Channel_t ch);
uint8_t CURVE_DisableChannel(CURVE_Channel_t ch);
void CURVE_Update(float current_temp, float target_temp);
void CURVE_UpdatePressure(float pressure_bar);
void CURVE_DualAppend(float real_bar, float ref_bar);   /* 0x84 给 CH0 追加一个实时点 (CH1 不动) */
/* 配置曲线 X 轴间隔(像素步长) + 数据源通道: 写 SP+9 偏移的 u16 值.
 * u16 高字节 = src_ch (0-7, 数据源通道 CH0-CH7), 低字节 = step (像素步长).
 * 例: SP=0xC000, step=2, src_ch=1 → 写 VP=0xC009=0x0102.
 * 屏命令: 5A A5 05 82 [SP+9] [byte_H byte_L]  其中 byte_H=src_ch, byte_L=step */
void CURVE_SetXStep(uint16_t sp_addr, uint8_t step, uint8_t src_ch);

/* 把 10 个 X 坐标标签控件 (VP 0xD254/0xD258/.../0xD278, 每4字节一变量) 同步到当前步长:
 *   step=4 (量程 50s):  5, 10, 15, ..., 50   (萃取开始后的秒数)
 *   step=2 (量程 100s): 10, 20, 30, ..., 100
 *   其它 step → 不动 (避免显示错位)
 * 屏曲线显示时, 这些控件显示对应 X 位置的时间刻度.
 * 注: 步长变化时必须同步重写, 否则标签还停留在旧步长的时间值, 与新 X 轴错位. */
void CURVE_SetXLabels(uint8_t step);
void CURVE_UpdateTemp(float temp_c);
/* 预铺参考曲线(定点档位)到 CH1:
 *   Y = raw[i] * mul_x100 / 100, 再钳位到 [CURVE_Y_MIN, CURVE_Y_MAX]
 *   raw     : 原始值数组 (uint8, 档位/百分比等小整数)
 *   mul_x100: 缩放系数x100. 例 档位->Y: 1档=1.25Hz, Y=Hz x 10 -> 1250
 *              (档位40 -> Y=500, 对应 Y 轴 0~50Hz)
 *   repeat  : 每个原始点横向重复次数(拉宽). 预设曲线与实时曲线 1:1 对齐时传 1 */
void CURVE_DrawRefFixed(const uint8_t *raw, uint16_t count, uint16_t mul_x100, uint8_t repeat);
void CURVE_WritePoint(CURVE_Channel_t ch, uint16_t value);
void CURVE_WriteTemp(CURVE_Channel_t ch, float temp);
void CURVE_Clear(void);
void CURVE_ClearChannel(CURVE_Channel_t ch);
void CURVE_SetSampleInterval(uint32_t interval);
uint32_t CURVE_GetSampleInterval(void);
void CURVE_Pause(void);
void CURVE_Resume(void);
uint8_t CURVE_IsPaused(void);
uint16_t CURVE_GetPointCount(void);
void CURVE_ResetPointCount(void);
void CURVE_WriteHistory(CURVE_Channel_t ch, const uint16_t *data, uint16_t count);

#endif
