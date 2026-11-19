/**
 * @file    gpio_ctrl.c
 * @brief   通用GPIO控制实现 (STM32F0 SPL)
 */
#include "gpio_ctrl.h"

/**
 * @brief 初始化GPIO为推挽输出
 * @param port GPIO端口 (GPIOA~GPIOF)
 * @param pin  引脚 (GPIO_Pin_0~GPIO_Pin_15)
 */
void GPIO_CtrlInit(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* 使能对应端口时钟 (F0: AHB总线) */
    if (port == GPIOA)      RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    else if (port == GPIOB) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    else if (port == GPIOC) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    else if (port == GPIOD) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);
    else if (port == GPIOF) RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);

    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(port, &GPIO_InitStructure);

    /* 默认拉低 */
    GPIO_ResetBits(port, pin);
}

/**
 * @brief 初始化所有已定义的GPIO组
 * @note  PB1 已改为 TIM3_CH4 灯板 PWM 输出, 不再作为 POWER_CH2 GPIO.
 *        PB1 已改为 TIM3_CH4 灯板 PWM 输出, 不再作为 POWER_CH2 GPIO.
 */
void GPIO_CtrlInitAll(void)
{
    GPIO_CtrlInit(POWER_CH1_PORT, POWER_CH1_PIN);
}
