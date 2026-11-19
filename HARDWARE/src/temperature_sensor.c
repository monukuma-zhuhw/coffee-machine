/**
 * @file    temperature_sensor.c
 * @brief   温度传感器模块实现 (NTC TPS210908, LUT查表法) - STM32F0 SPL
 * @note    电路: VCC--R_REF(5.1K, 实际板上分压电阻)--+--NTC--GND
 *                                |
 *                             ADC(PA4)
 *          ADC值随温度升高而降低(NTC电阻减小)
 */

#include "temperature_sensor.h"

/* ADC→温度查找表 (5℃间隔, 0-150℃)
 * 计算公式: R = R100 * exp(B*(1/T - 1/T100))
 *           ADC = 4095 * R / (R + R_REF)
 * 参数: R100=6733Ω, B25/100=4002K, R_REF=5.1KΩ (实际板上分压电阻)
 */
static const uint16_t ntc_adc_lut[NTC_LUT_SIZE] = {
 4035, 4017, 3995, 3968, 3935, 3896,   /*  0~25℃  */
 3850, 3796, 3733, 3661, 3579, 3488,   /* 30~55℃  */
 3387, 3277, 3158, 3032, 2899, 2761,   /* 60~85℃  */
 2619, 2475, 2330, 2186, 2045, 1908,   /* 90~115℃ */
 1775, 1648, 1527, 1412, 1304, 1204,   /*120~145℃ */
 1110                                  /* 150℃    */
};

static float g_filter_alpha = TEMP_DEFAULT_FILTER_ALPHA;
static float g_filtered_temp = 0.0f;
static uint8_t g_filter_initialized = 0;

void TEMP_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = TEMP_ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(TEMP_ADC_GPIO_PORT, &GPIO_InitStructure);

    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_ChannelConfig(ADC1, TEMP_ADC_CHANNEL, ADC_SampleTime_239_5Cycles);

    ADC_Cmd(ADC1, ENABLE);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);

    g_filter_initialized = 0;
}

uint16_t TEMP_ReadADC(void)
{
    /* 清除通道选择, 只选温度通道 (与压力共用ADC1, CHSELR是累加寄存器需先清零) */
    ADC1->CHSELR = 0;
    ADC_ChannelConfig(ADC1, TEMP_ADC_CHANNEL, ADC_SampleTime_239_5Cycles);

    ADC_StartOfConversion(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

/**
 * @brief  ADC值转温度 (LUT查表+线性插值)
 * @param  adc_value 12位ADC采样值 (0-4095)
 * @return 温度值(℃), 超出范围返回-999.0f
 * @note   ADC值随温度升高单调递减, LUT从低温(高ADC)到高温(低ADC)排列
 */
float TEMP_ADCToTemperature(uint16_t adc_value)
{
    int16_t lo, hi, mid;

    if (adc_value >= ntc_adc_lut[0])
        return (float)NTC_LUT_MIN_TEMP;
    if (adc_value <= ntc_adc_lut[NTC_LUT_SIZE - 1])
        return (float)NTC_LUT_MAX_TEMP;

    /* 二分查找: ntc_adc_lut[lo] > adc_value >= ntc_adc_lut[hi] */
    lo = 0;
    hi = NTC_LUT_SIZE - 1;
    while (hi - lo > 1) {
        mid = (lo + hi) / 2;
        if (adc_value >= ntc_adc_lut[mid])
            hi = mid;
        else
            lo = mid;
    }

    /* 线性插值 */
    {
        int16_t temp_lo = NTC_LUT_MIN_TEMP + lo * NTC_LUT_STEP;
        int16_t temp_hi = NTC_LUT_MIN_TEMP + hi * NTC_LUT_STEP;
        uint16_t adc_lo = ntc_adc_lut[lo];
        uint16_t adc_hi = ntc_adc_lut[hi];
        int32_t delta_adc = (int32_t)adc_lo - (int32_t)adc_hi;

        if (delta_adc == 0)
            return (float)(temp_lo + temp_hi) / 2.0f;

        return (float)temp_lo + (float)(temp_hi - temp_lo) * (float)(adc_lo - adc_value) / (float)delta_adc;
    }
}

float TEMP_ReadTemperature(void)
{
    uint16_t adc_value;
    float temperature;

    adc_value = TEMP_ReadADC();
    temperature = TEMP_ADCToTemperature(adc_value);

    if (temperature < TEMP_MIN - 5.0f || temperature > TEMP_MAX + 5.0f)
        return -999.0f;

    if (g_filter_initialized == 0) {
        g_filtered_temp = temperature;
        g_filter_initialized = 1;
    } else {
        g_filtered_temp = g_filter_alpha * temperature + (1.0f - g_filter_alpha) * g_filtered_temp;
    }

    return g_filtered_temp;
}

float TEMP_GetLastTemperature(void) { return g_filter_initialized ? g_filtered_temp : 0.0f; }
