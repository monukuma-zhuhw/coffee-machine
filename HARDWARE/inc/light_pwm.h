/**
 * @file    light_pwm.h
 * @brief   灯板硬件 PWM 调光模块接口 (PB1 / TIM3_CH4 / AF1) - STM32F0 SPL
 *
 * @note 设计: 硬件 PWM (TIM3_CH4 普通通道, 与现有定时器都不冲突)
 *   - PB1 配为复用推挽输出, AF1 = TIM3_CH4
 *   - TIM3 1kHz 周期, 10 级亮度档 (PSC=47, ARR=999)
 *   - TIM3 项目里完全未用 (只有 Library 库头文件提到), 不与 TIM1 (蒸汽加热器) 冲突
 *   - 与 TIM14 (AC 泵 update ISR) 在不同定时器, 也无冲突
 *   - 0% duty (level=0) → CCR=0 → PB1 一直低 (灯灭)
 *   - 100% duty (level=10) → CCR=1000 → PB1 一直高 (最亮)
 *   - 占空比 = level * 10%, 与原软件 PWM 档位一致
 *
 * 用法:
 *   LightPWM_Init();           // 初始化 TIM3_CH4 (PB1 推挽输出, 默认低)
 *   LightPWM_SetLevel(0~10);   // 设置亮度档位 (0=灯灭, 10=最亮)
 *   LightPWM_Off();            // 强制 duty=0 (灯灭)
 */

#ifndef __LIGHT_PWM_H
#define __LIGHT_PWM_H

#include "sys.h"
#include <stdint.h>

/* PWM 输出引脚 (PB1, TIM3_CH4 复用, AF1) */
#define LIGHT_PWM_GPIO_PORT      GPIOB
#define LIGHT_PWM_GPIO_PIN       GPIO_Pin_1
#define LIGHT_PWM_GPIO_PINSOURCE GPIO_PinSource1
#define LIGHT_PWM_GPIO_AF        GPIO_AF_1  /* AF1 = TIM3_CH4 */

/* 亮度档位 (0-10) */
#define LIGHT_LEVEL_MIN          0
#define LIGHT_LEVEL_MAX          10

/* TIM3 时序: 48MHz / (47+1) = 1MHz, 1MHz / (999+1) = 1kHz PWM */
#define LIGHT_PWM_PSC            47u
#define LIGHT_PWM_ARR            999u

void LightPWM_Init(void);
void LightPWM_SetLevel(uint8_t level);  /* 0-10, 限幅 */
void LightPWM_Off(void);                /* 强制 duty=0 (灯灭) */

#endif /* __LIGHT_PWM_H */