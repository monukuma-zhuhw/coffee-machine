/**
 * @file    encoder.h
 * @brief   增量式正交脉冲旋钮接口 (EXTI软件解码) - STM32F0 SPL
 *
 * 原理: A/B双相正交信号 → 双边沿EXTI中断 → 查表判方向
 *       PB9=A相(EXTI9), PB8=B相(EXTI8), 共用 EXTI4_15_IRQHandler
 *       一个机械档位 = 4个边沿(1个正交周期) = ±1档
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "sys.h"
#include <stdint.h>

/* 引脚配置 */
#define ENC_PORT            GPIOB
#define ENC_A_PIN           GPIO_Pin_9      /* A相 PB9 */
#define ENC_B_PIN           GPIO_Pin_8      /* B相 PB8 */
#define ENC_A_EXTI_LINE     EXTI_Line9
#define ENC_B_EXTI_LINE     EXTI_Line8
#define ENC_A_PIN_SOURCE    EXTI_PinSource9
#define ENC_B_PIN_SOURCE    EXTI_PinSource8
#define ENC_PORT_SOURCE     EXTI_PortSourceGPIOB
#define ENC_EXTI_IRQN       EXTI4_15_IRQn

/* 每档位对应的边沿数 (标准EC11: 4边沿/档) */
#define ENC_EDGES_PER_STEP  4

void     Encoder_Init(void);
int32_t  Encoder_PopDelta(void);        /* 取自上次以来的档位增量并清零 */
int32_t  Encoder_GetRawCount(void);     /* 原始边沿累计计数(调试用) */

/* ISR入口, 在stm32f0xx_it.c的EXTI4_15_IRQHandler中调用 */
void     Encoder_EXTI_IRQHandler(void);

#endif
