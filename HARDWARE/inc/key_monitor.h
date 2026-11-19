/**
 * @file    key_monitor.h
 * @brief   按键状态监测模块 - STM32F0 SPL
 *
 * 功能:
 *   - 3路按键输入监测 (关=低电平, 开=高电平, 输入下拉)
 *   - 20ms软件消抖
 *   - 状态变化事件缓存, 供主循环轮询 (PopEvent)
 *
 * @note   按键GPIO: KEY1=PA5, KEY2=PA7, KEY3=PB7
 */

#ifndef __KEY_MONITOR_H
#define __KEY_MONITOR_H

#include "sys.h"
#include <stdint.h>

/* 按键引脚定义 (关=低, 开=高)
 * KEY1: PA5, KEY2: PA7, KEY3: PB7
 * 注意: PA5原为电源通道1; PA7原为AC泵PWM输出(泵已移至PC14) */
#define KEY1_PORT               GPIOA
#define KEY1_PIN                GPIO_Pin_5
#define KEY2_PORT               GPIOA
#define KEY2_PIN                GPIO_Pin_7
#define KEY3_PORT               GPIOB
#define KEY3_PIN                GPIO_Pin_7

#define KEY_COUNT               3
#define KEY_DEBOUNCE_MS         20      /* 消抖时间(ms) */

/* 按键状态 (电路: VCC→上拉→GPIO→按键→GND, 按下拉低) */
#define KEY_STATE_OFF           1       /* 高电平 (松开) */
#define KEY_STATE_ON            0       /* 低电平 (按下) */

void    KeyMonitor_Init(void);
void    KeyMonitor_Update(uint32_t now);                    /* 主循环周期调用(建议10ms) */
uint8_t KeyMonitor_GetState(uint8_t key_idx);               /* 1=开(高), 0=关(低) */
uint8_t KeyMonitor_PopEvent(uint8_t *key_idx, uint8_t *state); /* 1=取到事件, 0=无 */

#endif /* __KEY_MONITOR_H */
