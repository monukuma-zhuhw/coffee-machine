/**
 * @file    gpio_ctrl.h
 * @brief   通用GPIO控制接口
 */
#ifndef __GPIO_CTRL_H
#define __GPIO_CTRL_H

#include "sys.h"

/* GPIO组定义 (自行修改端口和引脚)
 * 注意: PA0=出水口温度ADC(NTC,CH0), PA2/PA3=USART2(迪文屏),
 *       PA4=锅炉温度ADC(NTC,CH4), PA5=按键1, PA6=压力ADC(CH6),
 *       PA7=按键2(AC泵PWM已移至PC14), PA9/PA10=USART1(调试),
 *       PA11=锅炉加热器PWM(TIM1_CH4), PA12=蒸汽加热器PWM(TIM1_CH2),
 *       PB2=锅炉出水阀, PB7=按键3, PB8/PB9=旋钮编码器(EXTI),
 *       PB11=蒸汽出水阀, PB12=蒸汽进水泵, PB13=蒸汽输出 已占用
 */
#define POWER_CH1_PORT    GPIOA
#define POWER_CH1_PIN     GPIO_Pin_5

void GPIO_CtrlInit(GPIO_TypeDef *port, uint16_t pin);
void GPIO_CtrlInitAll(void);

#endif
