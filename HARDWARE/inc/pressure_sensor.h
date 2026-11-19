/**
 * @file    pressure_sensor.h
 * @brief   水压传感器模块接口 (MD-IS-TZ-) - STM32F0 SPL
 * @note    传感器: 线性输出, 0~16 Bar 对应 0.5~4.5V
 *          电路: VCC -- 传感器 -- ADC(PA7)
 *
 *          数据对应关系:
 *            压力(Bar) | 0   1    2  ...  16
 *            输出(V)   | 0.5 0.75 1.0 ... 4.5
 *          公式: V = 0.5 + 0.25 * P  →  P = (V - 0.5) * 4
 */

#ifndef __PRESSURE_SENSOR_H
#define __PRESSURE_SENSOR_H

#include "sys.h"
#include <stdint.h>

/* ADC GPIO */
#define PRES_ADC_CHANNEL           ADC_Channel_6
#define PRES_ADC_GPIO_PORT         GPIOA
#define PRES_ADC_GPIO_PIN          GPIO_Pin_6

/* 传感器参数 */
#define PRES_V_OFFSET              500       /* 0 Bar 输出电压(mV) */
#define PRES_V_PER_BAR             250       /* 每Bar电压增量(mV) */
#define PRES_V_REF                 3300      /* ADC参考电压(mV) */
#define PRES_ADC_MAX               4095      /* 12位ADC最大值 */

/* 压力范围 */
#define PRES_MIN                   0.0f      /* 最小压力(Bar) */
#define PRES_MAX                   16.0f     /* 最大压力(Bar) */
#define PRES_DEFAULT_FILTER_ALPHA  0.2f      /* 低通滤波系数 */

void PRES_Sensor_Init(void);
float PRES_ReadPressure(void);
uint16_t PRES_ReadADC(void);
float PRES_ADCToPressure(uint16_t adc_value);

#endif
