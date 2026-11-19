/**
 * @file    ac_pump.h
 * @brief   AC泵调速控制接口 (PWM频率调制) - STM32F0 SPL
 *
 * 原理: TIM14产生1ms周期中断, ISR中计数翻转PC14
 *       频率范围 0Hz(停) ~ 50Hz(全速), 占空比30%
 *       频率单位 0.25Hz (qhz), 支持旋钮 1.25Hz 细粒度调节
 *       功率0.1%级 (0~1000) 线性映射到 0~50Hz
 */

#ifndef __AC_PUMP_H
#define __AC_PUMP_H

#include "sys.h"
#include <stdint.h>

/* Gate输出引脚 (PC14 + PC15 镜像, BSRR/BRR 同时操作两引脚实现 atomic 同步) */
#define AC_PUMP_GATE_PIN            (GPIO_Pin_14 | GPIO_Pin_15)   /* PC14 + PC15 */
#define AC_PUMP_GATE_PORT           GPIOC

/* 延时定时器: TIM14 (16位, APB1) */
#define AC_PUMP_DELAY_TIMER         TIM14
#define AC_PUMP_DELAY_TIMER_RCC     RCC_APB1Periph_TIM14
#define AC_PUMP_DELAY_IRQN          TIM14_IRQn

/* 频率参数: 单位 = 0.25Hz (qhz), 支持旋钮 1.25Hz 步进 (5 qhz/格)
 * 范围 0~50Hz → 0~200 qhz. 0 = 停止 */
#define AC_FREQ_MIN_QHZ             0u              /* 0Hz (停) */
#define AC_FREQ_MAX_QHZ             200u            /* 50Hz (全速) */
#define AC_FREQ_STEP_QHZ            5u              /* 1.25Hz = 5 qhz (旋钮1格) */

/* 功率范围: 0.1% 级 (0~1000), 线性映射到 0~50Hz */
#define AC_PUMP_POWER_MIN           0u              /* 0.0%  (停) */
#define AC_PUMP_POWER_MAX           1000u           /* 100.0% */

void AC_Pump_Init(void);
void AC_Pump_SetPower(uint16_t deci_percent);   /* 0~1000 = 0.0%~100.0% */
void AC_Pump_SetFrequency(uint16_t qhz);         /* 0~200 = 0~50Hz (0.25Hz单位) */
uint16_t AC_Pump_GetPower(void);                /* 返回 0~1000 */
void AC_Pump_On(void);
void AC_Pump_Off(void);
uint8_t AC_Pump_IsOn(void);
uint16_t AC_Pump_GetFrequency(void);     /* 当前频率(0.25Hz单位 0~200), 0=停止 */

/* ISR入口, 在stm32f0xx_it.c中调用 */
void AC_Pump_ZCD_IRQHandler(void);       /* 空桩函数, 保留兼容 */
void AC_Pump_DelayTimer_IRQHandler(void);

#endif
