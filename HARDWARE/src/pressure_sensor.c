/**
 * @file    pressure_sensor.c
 * @brief   水压传感器模块实现 (MD-IS-TZ-, 0.5-4.5V线性输出) - STM32F0 SPL
 * @note    电路: VCC -- 传感器 -- ADC(PA7)
 *          线性关系: V = 0.5 + 0.25*P  →  P = (V - 0.5) * 4
 *
 *          与温度传感器共用ADC1, 每次读取前切换通道
 */

#include "pressure_sensor.h"

static float g_filter_alpha = PRES_DEFAULT_FILTER_ALPHA;
static float g_filtered_pres = 0.0f;
static uint8_t g_filter_initialized = 0;

void PRES_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = PRES_ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(PRES_ADC_GPIO_PORT, &GPIO_InitStructure);

    /* ADC1已由TEMP_Sensor_Init初始化, 此处不重复 */

    g_filter_initialized = 0;
}

uint16_t PRES_ReadADC(void)
{
    /* 清除通道选择, 只选压力通道 (与温度共用ADC1, CHSELR是累加寄存器需先清零) */
    ADC1->CHSELR = 0;
    ADC_ChannelConfig(ADC1, PRES_ADC_CHANNEL, ADC_SampleTime_239_5Cycles);

    ADC_StartOfConversion(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

/**
 * @brief  ADC值转压力 (线性公式)
 * @param  adc_value 12位ADC采样值 (0-4095)
 * @return 压力值(Bar), 超出范围返回-1.0f
 * @note   V_mV = adc * Vref / 4095
 *         P    = (V_mV - V_offset) / V_per_bar
 */
float PRES_ADCToPressure(uint16_t adc_value)
{
    int32_t v_mv;
    float pressure;

    /* 计算ADC对应电压(mV) */
    v_mv = (int32_t)adc_value * PRES_V_REF / PRES_ADC_MAX;

    /* 低于偏移电压(0Bar=0.5V), 视为无压力或断线 */
    if (v_mv < PRES_V_OFFSET) {
        return 0.0f;
    }

    /* 线性转换: P = (V - 500mV) / 250mV/Bar */
    pressure = (float)(v_mv - PRES_V_OFFSET) / (float)PRES_V_PER_BAR;

    /* 范围限制 */
    if (pressure < PRES_MIN) pressure = PRES_MIN;
    if (pressure > PRES_MAX) pressure = PRES_MAX;

    return pressure;
}

float PRES_ReadPressure(void)
{
    uint16_t adc_value;
    float pressure;

    adc_value = PRES_ReadADC();
    pressure = PRES_ADCToPressure(adc_value);

    if (g_filter_initialized == 0) {
        g_filtered_pres = pressure;
        g_filter_initialized = 1;
    } else {
        g_filtered_pres = g_filter_alpha * pressure + (1.0f - g_filter_alpha) * g_filtered_pres;
    }

    return g_filtered_pres;
}

