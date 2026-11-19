/**
 * @file    light_pwm.c
 * @brief   灯板硬件 PWM 调光模块实现 (PB1 / TIM3_CH4 / AF1) - STM32F0 SPL
 *
 * 设计: 硬件 PWM (TIM3_CH4 普通通道)
 *   - PB1 复用 AF1 → TIM3_CH4 输出 PWM
 *   - TIM3 时基: 48MHz / (47+1) = 1MHz, ARR=999 → 1kHz PWM
 *   - 占空比 CCR = level * 100 (0~1000) → 0%/10%/.../100%
 *   - 修改亮度档只需 TIM_SetCompare4() 一条寄存器写, 不进中断不消耗 CPU
 *
 * 为什么选 TIM3_CH4 on PB1:
 *   STM32F030 的 PB6 只能映射到 TIM16_CH1N (互补通道, 不适合调光),
 *   PB1 = TIM14_CH1 (AF0) 但 TIM14 已被 AC 泵 [ac_pump.c] 占用,
 *   PB1 = TIM3_CH4 (AF1) 普通通道, TIM3 项目里完全未用, 不冲突.
 *   PB1 当前未被任何模块使用 (原 POWER_CH2 是占位, 未调用).
 */

#include "light_pwm.h"

/**
 * @brief 初始化灯板硬件 PWM (PB1 -> TIM3_CH4)
 * @note  PB1 配为复用推挽输出, AF1=TIM3. TIM3 1kHz, 占空比初始 0.
 */
void LightPWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef tb;
    TIM_OCInitTypeDef oc;

    /* 使能 GPIOB 时钟 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* PB1 复用推挽输出 */
    GPIO_InitStructure.GPIO_Pin   = LIGHT_PWM_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(LIGHT_PWM_GPIO_PORT, &GPIO_InitStructure);

    /* PB1 -> AF1 = TIM3_CH4 */
    GPIO_PinAFConfig(LIGHT_PWM_GPIO_PORT, LIGHT_PWM_GPIO_PINSOURCE, LIGHT_PWM_GPIO_AF);

    /* TIM3 时钟 (APB1, 项目里目前完全未用) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 时基: 48MHz / (47+1) = 1MHz, 1MHz / (999+1) = 1kHz */
    tb.TIM_Prescaler     = LIGHT_PWM_PSC;
    tb.TIM_Period        = LIGHT_PWM_ARR;
    tb.TIM_CounterMode   = TIM_CounterMode_Up;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &tb);

    /* PWM 模式 1 输出到 CH4 (CCR < CNT 时输出高) */
    oc.TIM_OCMode      = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_Pulse       = 0;                  /* 初始 0% 占空比 (灯灭) */
    oc.TIM_OCPolarity  = TIM_OCPolarity_High;
    TIM_OC4Init(TIM3, &oc);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief 设置亮度档位 (0-10)
 * @note  level=0 → CCR=0 (灯灭)
 *        level=10 → CCR=1000 (最亮)
 *        level * 10% 占空比, 与原软件 PWM 一致
 *        仅一条寄存器写, 不进中断, 不消耗 CPU
 */
void LightPWM_SetLevel(uint8_t level)
{
    uint16_t ccr;

    if (level > LIGHT_LEVEL_MAX) level = LIGHT_LEVEL_MAX;

    /* level=0..10 → CCR = level * 100 = level * (ARR+1)/10 */
    ccr = (uint16_t)(level * 100u);
    TIM_SetCompare4(TIM3, ccr);
}

/**
 * @brief 关灯 (强制 duty=0)
 */
void LightPWM_Off(void)
{
    TIM_SetCompare4(TIM3, 0);
}
