/**
 * @file    steam_heater.c
 * @brief   蒸汽加热器PWM控制实现 (PA11 / TIM1_CH4) - STM32F0 SPL
 *
 * 复用 TIM1 (锅炉加热器用 CH2, 这里用 CH4), 共用时基与主输出配置.
 * 低电平加热: 极性Low, 空闲态高(功率0时不加热).
 */

#include "steam_heater.h"

/**
 * @brief 初始化蒸汽加热器PWM (PA11 -> TIM1_CH4)
 * @note  如果 TIM1 已由锅炉加热器初始化, 则只配置CH4和GPIO.
 */
void STEAM_HeaterInit(void)
{
    GPIO_InitTypeDef     GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef timebase;
    TIM_OCInitTypeDef      ocinit;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(STEAM_HEATER_TIMER_RCC, ENABLE);

    /* PA11 -> AF2 = TIM1_CH4 */
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_2);

    GPIO_InitStructure.GPIO_Pin = STEAM_HEATER_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(STEAM_HEATER_GPIO_PORT, &GPIO_InitStructure);

    /* 检查TIM1是否已经配置(锅炉加热器先 Init 的情况)
     * 若已配置, 仅初始化CH4; 否则完整配置时基. */
    if ((STEAM_HEATER_TIMER->CR1 & TIM_CR1_CEN) == 0) {
        /* TIM1 基础配置 (同锅炉加热器参数一致) */
        timebase.TIM_Period = STEAM_HEATER_PWM_RESOLUTION - 1;
        timebase.TIM_Prescaler = STEAM_HEATER_PRESCALER;
        timebase.TIM_ClockDivision = TIM_CKD_DIV1;
        timebase.TIM_CounterMode = TIM_CounterMode_Up;
        timebase.TIM_RepetitionCounter = 0;
        TIM_TimeBaseInit(STEAM_HEATER_TIMER, &timebase);
    }

    /* 通道4 PWM模式配置
     * 低电平加热: 极性Low, 空闲态Set(高) */
    ocinit.TIM_OCMode = TIM_OCMode_PWM1;
    ocinit.TIM_OutputState = TIM_OutputState_Enable;
    ocinit.TIM_Pulse = 0;
    ocinit.TIM_OCPolarity = TIM_OCPolarity_Low;
    ocinit.TIM_OCNPolarity = TIM_OCNPolarity_High;
    ocinit.TIM_OCIdleState = TIM_OCIdleState_Set;
    TIM_OC4Init(STEAM_HEATER_TIMER, &ocinit);
    TIM_OC4PreloadConfig(STEAM_HEATER_TIMER, TIM_OCPreload_Enable);

    /* TIM1主输出使能 (如果之前没开) */
    if ((STEAM_HEATER_TIMER->BDTR & TIM_BDTR_MOE) == 0) {
        TIM_CtrlPWMOutputs(STEAM_HEATER_TIMER, ENABLE);
        TIM_Cmd(STEAM_HEATER_TIMER, ENABLE);
    }
}

uint8_t STEAM_HeaterSetPower(uint8_t percent)
{
    uint16_t duty;

    if (percent < STEAM_HEATER_POWER_MIN || percent > STEAM_HEATER_POWER_MAX)
        return 1;

    duty = (uint16_t)((uint32_t)STEAM_HEATER_PWM_RESOLUTION * percent / 100);
    if (duty > STEAM_HEATER_PWM_RESOLUTION)
        duty = STEAM_HEATER_PWM_RESOLUTION;

    TIM_SetCompare4(STEAM_HEATER_TIMER, duty);
    return 0;
}

void    STEAM_HeaterOn(void)    { STEAM_HeaterSetPower(STEAM_HEATER_POWER_MAX); }
void    STEAM_HeaterOff(void)   { STEAM_HeaterSetPower(STEAM_HEATER_POWER_MIN); }
