/**
 * @file    steam_heater.h
 * @brief   蒸汽加热器控制接口 (PWM控制PTC) - STM32F0 SPL
 * @note    PA11 / TIM1_CH4 (AF2), 同锅炉加热器(TIM1_CH2)共用TIM1
 */

#ifndef __STEAM_HEATER_H
#define __STEAM_HEATER_H

#include "sys.h"
#include <stdint.h>

/* PWM 配置 - 复用 TIM1, 通道4, PA11 AF2 */
#define STEAM_HEATER_TIMER        TIM1
#define STEAM_HEATER_TIMER_RCC    RCC_APB2Periph_TIM1
#define STEAM_HEATER_CHANNEL      TIM_Channel_4
#define STEAM_HEATER_GPIO_PORT    GPIOA
#define STEAM_HEATER_GPIO_PIN     GPIO_Pin_11

/* PWM 参数 (同锅炉加热器: 1kHz, 1000级分辨率) */
#define STEAM_HEATER_PWM_FREQ     1000
#define STEAM_HEATER_PWM_RESOLUTION 1000
#define STEAM_HEATER_PRESCALER    47    /* 48MHz/(47+1)=1MHz, 1MHz/1000=1kHz */

/* 功率范围 */
#define STEAM_HEATER_POWER_MIN    0
#define STEAM_HEATER_POWER_MAX    100
#define STEAM_HEATER_POWER_DEFAULT 0

void    STEAM_HeaterInit(void);
uint8_t STEAM_HeaterSetPower(uint8_t percent);
void    STEAM_HeaterOn(void);
void    STEAM_HeaterOff(void);

#endif /* __STEAM_HEATER_H */
