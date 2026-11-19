/**
 * @file    temperature_sensor.h
 * @brief   温度传感器模块接口 (NTC热敏电阻 TPS210908) - STM32F0 SPL
 * @note    NTC: R100=6.733KΩ±1%, B25/100=4002K±1%
 *          电路: VCC -- R_REF(5.1KΩ, 实际板上分压电阻) --+-- NTC -- GND
 *                                      |
 *                                   ADC (PA0)
 */

#ifndef __TEMPERATURE_SENSOR_H
#define __TEMPERATURE_SENSOR_H

#include "sys.h"
#include <stdint.h>

/* ADC GPIO - PA4 ADC_Channel_4 */
#define TEMP_ADC_CHANNEL           ADC_Channel_4
#define TEMP_ADC_GPIO_PORT         GPIOA
#define TEMP_ADC_GPIO_PIN          GPIO_Pin_4

/* NTC 参数 (TPS210908 规格书) */
#define NTC_R100                   6733        /* 100℃电阻值 Ω */
#define NTC_B25_100                4002        /* B25/100 值 K */
#define NTC_R_REF                  5100        /* 实际板上分压电阻 Ω */
#define NTC_V_REF                  3300        /* 参考电压 mV */

/* LUT 参数 */
#define NTC_LUT_MIN_TEMP           0           /* LUT最低温度 ℃ */
#define NTC_LUT_MAX_TEMP           150         /* LUT最高温度 ℃ */
#define NTC_LUT_STEP               5           /* LUT步进 ℃ */
#define NTC_LUT_SIZE               ((NTC_LUT_MAX_TEMP - NTC_LUT_MIN_TEMP) / NTC_LUT_STEP + 1)

/* 温度范围 */
#define TEMP_MIN                   0.0f
#define TEMP_MAX                   150.0f
#define TEMP_DEFAULT_FILTER_ALPHA  0.15f

void TEMP_Sensor_Init(void);
float TEMP_ReadTemperature(void);
uint16_t TEMP_ReadADC(void);
float TEMP_ADCToTemperature(uint16_t adc_value);
float TEMP_GetLastTemperature(void);

#endif
