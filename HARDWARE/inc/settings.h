/**
 * @file    settings.h
 * @brief   客户设定掉电保存模块 (主Flash末页模拟EEPROM) - STM32F0 SPL
 *
 * 存储介质: 主Flash最后一个1KB页(F030C8: 64KB, 第63页 0x0800FC00),
 *           选项字节在0x1FFFF800独立区域, 与本页无关.
 * 磨损策略: 内存镜像 + 脏标志 + 延迟合并落盘(Settings_Task, 修改停止5s后写一次),
 *           防连改(如旋钮连转)刷爆Flash(标称擦写~10K次).
 *
 * 用法: 开机 Settings_Init() 读Flash到内存镜像(无效则写默认值);
 *       运行中 Settings_Edit() 取指针修改字段, 改完调 Settings_MarkDirty();
 *       主循环慢速节拍调 Settings_Task(now) 自动延迟落盘;
 *       需要立即落盘时直接调 Settings_Save()(0=成功).
 */

#ifndef __SETTINGS_H
#define __SETTINGS_H

#include "sys.h"
#include <stdint.h>

/* 存储区: 主Flash末页(F030C8, 1KB/页, 第63页) */
#define SETTINGS_FLASH_PAGE       0x0800FC00u

/* 数据有效性标志 */
#define SETTINGS_MAGIC            0xC0FFu
#define SETTINGS_VERSION          0x0008u

/* 字段默认值(空片/数据损坏时使用) */
#define SETTINGS_DEF_BREW_TEMP    90.0f   /* 萃取目标温度(℃) */
#define SETTINGS_DEF_TARGET_PRES  9.0f    /* 萃取目标压力(Bar) */
#define SETTINGS_DEF_HEAD_TEMP    85.0f   /* 冲泡头目标温度(℃) */
#define SETTINGS_DEF_BREW_TIME_S  30u     /* 萃取出水时长(s) */
#define SETTINGS_DEF_HOT_TIME_S   30u     /* 热水出水时长(s) */
#define SETTINGS_DEF_LIGHT_LEVEL  10u     /* 灯板亮度档 0-10 (0=熄灭, 默认最亮) */
#define SETTINGS_DEF_BRIGHTNESS   10u     /* 屏幕背光 0-10 (默认最亮) */
#define SETTINGS_DEF_LANG         0u      /* 语言 0=中文 1=英文 */
#define SETTINGS_DEF_PREINFUSE_DURATION_S 5u   /* 预浸泡持续出水时间(秒, 整数 1-9), 屏端 0xD058 */
#define SETTINGS_DEF_PREINFUSE_STOP_S     3u   /* 预浸泡后停止出水时间(秒, 整数 1-9), 屏端 0xD05C */
#define SETTINGS_DEF_PREINFUSE_MASTER     1u   /* 预浸泡总开关 (0=跳过, 1=执行; 默认开), 屏端 0xD070 */
#define SETTINGS_DEF_CURVE_KNOB_VAL 0u    /* 曲线页旋钮值 0-8 (默认 0), 屏端 0xD250 */
#define SETTINGS_CURVE_KNOB_MIN   0u      /* 曲线页旋钮值最小值 */
#define SETTINGS_CURVE_KNOB_MAX   8u      /* 曲线页旋钮值最大值 (循环边界) */
#define SETTINGS_CUP_COUNT_MAX    9999u   /* 杯数统计上限 */

/* 延迟落盘: 最后一次修改后静置此时长才写Flash(ms).
 * 设定为低频编辑, 取短延时; 仍保留少量合并窗口防连点.
 * (慢速循环节拍1s, 实际提交在最后一次修改后1~2s) */
#define SETTINGS_COMMIT_DELAY_MS  1000u

/* 断电保存的设定集合.
 * 注意: 新增字段一律加在reserved之前, 并递增SETTINGS_VERSION
 * (版本不匹配按无效处理, 自动回默认值). */
typedef struct {
    uint16_t magic;          /* 有效标志 SETTINGS_MAGIC */
    uint16_t version;        /* 结构版本 */
    float    brew_temp;      /* 萃取目标温度(℃) */
    float    target_pres;    /* 萃取目标压力(Bar) */
    float    head_temp;      /* 冲泡头目标温度(℃) */
    uint16_t brew_time_s;    /* 萃取出水时长(s) */
    uint16_t hot_time_s;     /* 热水出水时长(s) */
    uint16_t cup_count;      /* 累计杯数统计 0-9999 */
    uint16_t light_level;    /* 灯板亮度档 0-10 (0=熄灭, 屏端 0xD050 上报时写入) */
    uint16_t brightness;     /* 屏幕背光 0-10 (独立于灯板) */
    uint16_t lang_mode;      /* 语言模式 0=中文 1=英文 */
    uint16_t preinfuse_duration_s;  /* 预浸泡持续出水时间(秒, 整数 1-9), 屏端 0xD058 */
    uint16_t preinfuse_stop_s;      /* 预浸泡后停止出水时间(秒, 整数 1-9), 屏端 0xD05C */
    uint16_t preinfuse_master;      /* 预浸泡总开关 (0=跳过, 1=执行; 默认 1), 屏端 0xD070.
                                     * OFF 时 0xD058/0xD05C 上报被 MCU 拒收, 不写 settings/flash */
    uint16_t curve_knob_val;        /* 曲线页旋钮值 0-8, 循环, 默认 0, 屏端 0xD250.
                                     * 旋钮左右旋转 ±1 循环, 改变即写入 flash. */
    uint16_t reserved;       /* 保留对齐/未来扩展 */
    uint16_t checksum;       /* 之前所有u16字累加和 */
} Settings_t;

void             Settings_Init(void);        /* 开机调用: Flash->内存镜像, 无效则装默认值并落盘 */
const Settings_t *Settings_Get(void);        /* 读当前设定(内存镜像, 只读) */
Settings_t      *Settings_Edit(void);        /* 改设定: 返回可写指针, 修改后必须调MarkDirty */
void             Settings_MarkDirty(void);   /* 置脏+记录修改时刻(触发延迟落盘) */
uint8_t          Settings_Save(void);        /* 立即写Flash, 返回0=成功 */
uint8_t          Settings_IsDirty(void);     /* 1=有未落盘修改 */
void             Settings_Task(u32 now);     /* 主循环慢速节拍调用: 延迟合并落盘 */
void             Settings_LoadDefaults(void);/* 恢复出厂默认: 重置g_settings为SETTINGS_DEF_*, 不写Flash(调用方决定) */

#endif
