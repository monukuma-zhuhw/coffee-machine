/**
 * @file    water_level.h
 * @brief   水箱水位传感器接口 - STM32F0 SPL
 * @note    PC13输入: 低电平=水箱有水, 高电平=水箱缺水
 *          输入上拉: 传感器未接/断线时读高(缺水), 默认禁止抽水
 */

#ifndef __WATER_LEVEL_H
#define __WATER_LEVEL_H

#include "sys.h"
#include <stdint.h>

/* 水箱水位传感器引脚 (PC13, 低=有水, 高=缺水) */
#define WATER_SENSOR_PORT        GPIOC
#define WATER_SENSOR_PIN         GPIO_Pin_13

void    WATER_Sensor_Init(void);

/* 返回1=水箱有水(PC13低), 0=水箱缺水(PC13高) */
uint8_t WATER_IsPresent(void);

#endif /* __WATER_LEVEL_H */
