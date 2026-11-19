/**
 * @file    heater_control.h
 * @brief   锅炉加热器控制接口 (软件PWM控制PTC) - STM32F0 SPL
 *
 * @note PA12上没有任何定时器PWM复用功能(F030的TIM1_CH2在PA9, 与调试串口冲突),
 *       因此用软件PWM: PA12普通GPIO推挽输出, HEATER_PWM_Tick()由SysTick(1ms)
 *       驱动, 100ms周期/100级占空比(10Hz). PTC热惯性大(秒级), 10Hz足够平滑.
 *       低电平加热: 占空比内输出低=加热, 其余输出高=停止.
 */

#ifndef __HEATER_CONTROL_H
#define __HEATER_CONTROL_H

#include "sys.h"
#include <stdint.h>

/* 输出引脚 (PA12, 普通GPIO输出, 低=加热) */
#define HEATER_GPIO_PORT          GPIOA
#define HEATER_GPIO_PIN           GPIO_Pin_12

/* 软件PWM参数: 1ms节拍 x 100级 = 100ms周期(10Hz), 1%分辨率 */
#define HEATER_PWM_STEPS          100

/* 功率范围 */
#define HEATER_POWER_MIN          0
#define HEATER_POWER_MAX          100
#define HEATER_POWER_DEFAULT      0

/* 锅炉超温保护: 温度 >= HEATER_PROTECT_TEMP 时强制断开加热, 保护期间只能关不能开;
 * 降到 HEATER_PROTECT_RELEASE_TEMP 以下自动恢复 (回差防止在阈值附近反复通断) */
#define HEATER_PROTECT_TEMP           120.0f
#define HEATER_PROTECT_RELEASE_TEMP   115.0f

void HEATER_Init(void);
void HEATER_PWM_Tick(void);     /* SysTick(1ms)调用: 软件PWM翻转 */
uint8_t HEATER_SetPower(uint8_t percent);   /* 超温保护中只接受 0 */
uint8_t HEATER_GetPower(void);
void HEATER_Off(void);
uint8_t HEATER_CheckProtection(float current_temp);   /* 主循环10ms调用, 返回1=保护中 */
uint8_t HEATER_IsProtectionTriggered(void);

#endif
