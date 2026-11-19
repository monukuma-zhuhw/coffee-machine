/**
 * @file    brew_heater.h
 * @brief   冲泡头加热片模块 (PB10 / 继电器通断控温) - STM32F0 SPL
 *
 * 职责: 二次加热片贴在**冲泡头金属块**上, PA0 传感器测的就是这个金属块的温度.
 *       本模块负责把冲泡头温度稳定在设定温度附近 (滞回带内往复).
 *       设定温度 = 屏端「冲泡头温度」 (VP 0xD044 → settings.head_temp).
 *
 *       与锅炉温度**完全无关**: 锅炉 (boiler.c) 管的是锅炉水温, 各管各的.
 *
 * 控制策略 (无 PID, 继电器特性决定) — 滞回开关:
 *   控温目标: 冲泡头维持在 [设定−BREW_HYST_HIGH_C, 设定] 附近.
 *   1. 冲泡头 < 设定−BREW_HYST_LOW_C  → 满功率加热
 *   2. 冲泡头 >= 设定−BREW_HYST_HIGH_C → 停止加热
 *   两阈值构成滞回带, 防边界抖动. 例设定85℃, LOW=4, HIGH=0 → 81~85℃ 往复.
 *
 * 硬件约束: PB10 在 STM32F030C8 上没有任何定时器复用功能
 *   (datasheet Table 13: AF0 空, AF1=I2C1_SCL, AF4=USART3_TX,
 *    AF5=SPI2_SCK, AF6=I2C2_SCL), 且继电器本身也不适合高频.
 *
 * 极性: **高电平加热** (与锅炉 PA12 的低电平加热相反),
 *       GPIO 配下拉, 保证复位/上电瞬间为低=不加热.
 *
 * 用法:
 *   BrewHeater_Init();                              // 开机初始化 (PB10 默认不加热)
 *   BrewHeater_HeatStart(head_target);              // 启用控温, 传入冲泡头设定温度
 *   BrewHeater_Update(head_temp, now);              // 10ms 节拍调, 主循环里调
 *   BrewHeater_HeatStop();                          // 出饮结束, 强制断电
 */

#ifndef __BREW_HEATER_H
#define __BREW_HEATER_H

#include "sys.h"
#include <stdint.h>

/* 输出引脚 (PB10, 继电器驱动, 高电平吸合加热) */
#define BREW_HEATER_GPIO_PORT    GPIOB
#define BREW_HEATER_GPIO_PIN     GPIO_Pin_10

/*==============================================================
 * 控温参数 — 滞回开关 (两个阈值, 都可调)
 *
 *   冲泡头 <  (设定 − BREW_HYST_LOW_C)  → 加热片**满功率**工作
 *   冲泡头 >= (设定 − BREW_HYST_HIGH_C) → 加热片停止
 *
 * 两阈值构成滞回带, 避免在边界反复通断 (继电器寿命保护).
 * 例设定85℃, LOW=4, HIGH=0:
 *   冲泡头 < 81℃ → 满功率加热 → 升到 85℃ 停止 → 降到 <81℃ 再加热 …
 *
 * 调参指引:
 *   冲温严重 (明显超过设定温度) → 调小 BREW_HYST_HIGH_C (0 → 1 → 2),
 *                                  让加热片提前停止
 *   温度掉太低 (低于设定太多)   → 调大 BREW_HYST_LOW_C (4 → 6 → 8),
 *                                  让加热片更早开始加热
 *==============================================================*/

/* 滞回下沿 (℃): 冲泡头低于 设定−此值 → 满功率加热.
 * 调大 = 提前加热 (保温温度更高); 调小 = 让温度掉得更低才加热. */
#define BREW_HYST_LOW_C         3.0f

/* 滞回上沿 (℃): 冲泡头达到 设定−此值 → 停止加热.
 * 冲温严重时调大 (0 → 1 → 2), 让加热片提前停止. */
#define BREW_HYST_HIGH_C        0.0f

/* 决策粒度 (ms): 满功率时此值无意义 (立即通电), 仅停止判定用此节流,
 * 避免传感器抖动导致继电器高频动作. */
#define BREW_RELAY_TICK_MS      1000u

void    BrewHeater_Init(void);

/* 继电器通断: 高=吸合加热 */
void    BrewHeater_RelayOn(void);
void    BrewHeater_RelayOff(void);
uint8_t BrewHeater_RelayIsOn(void);

/* 启用/停用控温 */
void    BrewHeater_HeatStart(float head_target);   /* 启用 (萃取流程入口用) */
void    BrewHeater_HeatStop(void);                /* 停用 + 强制断电 (出饮结束用) */

/* 仅更新设定温度, 保持使能与滞回状态连续 (预浸泡→全压切换时用) */
void    BrewHeater_SetTarget(float head_target);

/* 10ms 节拍控制: head_temp=冲泡头实测温度(PA0), now=GetTick() */
void    BrewHeater_Update(float head_temp, uint32_t now);

/* 调试: 取当前状态 (duty 恒为 0 或 100) / 继电器状态 / 两个阈值 */
void    BrewHeater_GetTerms(float *duty_pct, uint8_t *relay_on,
                           float *low_thr, float *high_thr);

#endif /* __BREW_HEATER_H */