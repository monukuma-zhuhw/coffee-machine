/**
 * @file    water_level.c
 * @brief   水箱水位传感器实现 - STM32F0 SPL
 *
 * PC13数字输入(上拉): 传感器检测到水箱有水时拉低, 缺水/断线时为高.
 * 抽水前必须查询WATER_IsPresent(), 为0时禁止泵运行(防空转).
 */

#include "water_level.h"

void WATER_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = WATER_SENSOR_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;   /* 上拉: 断线默认缺水(高) */
    GPIO_Init(WATER_SENSOR_PORT, &GPIO_InitStructure);
}

uint8_t WATER_IsPresent(void)
{
    /* 低电平=水箱有水 */
    return (GPIO_ReadInputDataBit(WATER_SENSOR_PORT, WATER_SENSOR_PIN) == Bit_RESET) ? 1 : 0;
}
