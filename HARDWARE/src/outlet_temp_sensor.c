/**
 * @file    outlet_temp_sensor.c
 * @brief   出水口温度传感器实现 (NTC, PA0/ADC_CH0) - STM32F0 SPL
 *
 * 硬件: NTC + 5.1K分压(实际板上分压电阻), PA0 -> ADC1_CH0.
 * 与锅炉温度传感器(PA4/CH4)共用ADC1, 读取前清零CHSELR再切换通道.
 */

#include "outlet_temp_sensor.h"

/* ADC->温度查找表 (5℃间隔, 0-150℃), 与锅炉温度传感器完全相同
 * R100=6733Ω, B25/100=4002K, R_REF=5.1KΩ (实际板上分压电阻) */
static const uint16_t otemp_adc_lut[] = {
 4035, 4017, 3995, 3968, 3935, 3896,   /*  0~25℃  */
 3850, 3796, 3733, 3661, 3579, 3488,   /* 30~55℃  */
 3387, 3277, 3158, 3032, 2899, 2761,   /* 60~85℃  */
 2619, 2475, 2330, 2186, 2045, 1908,   /* 90~115℃ */
 1775, 1648, 1527, 1412, 1304, 1204,   /*120~145℃ */
 1110                                  /* 150℃    */
};

#define OTEMP_LUT_SIZE   (sizeof(otemp_adc_lut) / sizeof(otemp_adc_lut[0]))

static float    g_filter_alpha = OTEMP_FILTER_ALPHA;
static float    g_filtered_temp = 0.0f;
static uint8_t  g_filter_initialized = 0;

void OTEMP_Sensor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = OTEMP_ADC_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(OTEMP_ADC_GPIO_PORT, &GPIO_InitStructure);

    /* ADC1 可能已由温度/压力传感器初始化, 这里仅配置通道采样时间 */
    ADC_ChannelConfig(ADC1, OTEMP_ADC_CHANNEL, ADC_SampleTime_239_5Cycles);

    /* 确保ADC已使能 */
    if ((ADC1->CR & ADC_CR_ADEN) == 0) {
        ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
        ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
        ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_TRGO;
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
        ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
        ADC_Init(ADC1, &ADC_InitStructure);
        ADC_Cmd(ADC1, ENABLE);
        while (ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);
    }

    g_filter_initialized = 0;
}

uint16_t OTEMP_ReadADC(void)
{
    /* 清除通道选择, 只选出水口温度通道 */
    ADC1->CHSELR = 0;
    ADC_ChannelConfig(ADC1, OTEMP_ADC_CHANNEL, ADC_SampleTime_239_5Cycles);

    ADC_StartOfConversion(ADC1);
    while (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
    return ADC_GetConversionValue(ADC1);
}

float OTEMP_ReadTemperature(void)
{
    uint16_t adc_val = OTEMP_ReadADC();
    float temp;

    /* 查表+线性插值, 同锅炉温度传感器算法 (二分查找) */
    if (adc_val >= otemp_adc_lut[0]) {
        temp = 0.0f;
    } else if (adc_val <= otemp_adc_lut[OTEMP_LUT_SIZE - 1]) {
        temp = 150.0f;
    } else {
        int16_t lo = 0, hi = OTEMP_LUT_SIZE - 1, mid;
        /* 二分查找: otemp_adc_lut[lo] > adc_val >= otemp_adc_lut[hi] */
        while (hi - lo > 1) {
            mid = (lo + hi) / 2;
            if (adc_val >= otemp_adc_lut[mid])
                hi = mid;
            else
                lo = mid;
        }
        {
            float t_low = (float)(lo * 5);
            float t_high = (float)(hi * 5);
            float ratio = (float)(otemp_adc_lut[lo] - adc_val) /
                          (float)(otemp_adc_lut[lo] - otemp_adc_lut[hi]);
            temp = t_low + ratio * (t_high - t_low);
        }
    }

    /* 一阶低通滤波 */
    if (!g_filter_initialized) {
        g_filtered_temp = temp;
        g_filter_initialized = 1;
    } else {
        g_filtered_temp = g_filter_alpha * temp +
                          (1.0f - g_filter_alpha) * g_filtered_temp;
    }
    return g_filtered_temp;
}

float OTEMP_GetLastTemperature(void)
{
    return g_filtered_temp;
}
