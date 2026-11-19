/**
 * @file    outlet_temp_sensor.h
 * @brief   出水口温度传感器接口 (NTC热敏电阻) - STM32F0 SPL
 * @note    PA0 / ADC_CH0, 与锅炉温度传感器(PA4/CH4)共用ADC1
 */

#ifndef __OUTLET_TEMP_SENSOR_H
#define __OUTLET_TEMP_SENSOR_H

#include "sys.h"
#include <stdint.h>

/* ADC GPIO - PA0 ADC_Channel_0 */
#define OTEMP_ADC_CHANNEL          ADC_Channel_0
#define OTEMP_ADC_GPIO_PORT        GPIOA
#define OTEMP_ADC_GPIO_PIN         GPIO_Pin_0

/* NTC 参数与锅炉温度传感器相同 (TPS210908) */
#define OTEMP_NTC_R100             6733
#define OTEMP_NTC_B25_100          4002
#define OTEMP_NTC_R_REF            5100        /* 实际板上分压电阻 Ω */
#define OTEMP_NTC_V_REF            3300

#define OTEMP_MIN                  0.0f
#define OTEMP_MAX                  150.0f
#define OTEMP_FILTER_ALPHA         0.15f

void   OTEMP_Sensor_Init(void);
float  OTEMP_ReadTemperature(void);
uint16_t OTEMP_ReadADC(void);
float  OTEMP_GetLastTemperature(void);

#endif /* __OUTLET_TEMP_SENSOR_H */
