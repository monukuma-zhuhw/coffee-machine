/**
 * @file    steam.h
 * @brief   蒸汽控制 (进水泵 + 蒸汽输出) - STM32F0 SPL
 *
 * 硬件:
 *   PB12 → RLY1 → 蒸汽进水泵 (高=继电器导通)
 *   PB13 → RLY7 → 蒸汽输出   (高=继电器导通)
 * 说明: 蒸汽加热由 steam_heater.c (PA12/TIM1_CH2) 单独控制
 *       (PB11原为蒸汽出水阀, 已改为热水出水阀, 由boiler.c控制)
 */

#ifndef __STEAM_H
#define __STEAM_H

#include "sys.h"
#include <stdint.h>

/* 蒸汽输出继电器 PB13 (高=导通) */
#define STEAM_OUT_PORT          GPIOB
#define STEAM_OUT_PIN           GPIO_Pin_13

/* 蒸汽进水泵继电器 PB12 (高=导通) */
#define STEAM_PUMP_PORT         GPIOB
#define STEAM_PUMP_PIN          GPIO_Pin_12

void    Steam_Init(void);       /* PB12/PB13输出, 默认关(低) */

/* 独立控制 (蒸汽流程: 进水泵与蒸汽输出同时工作) */
void    Steam_PumpOn(void);     /* 进水泵开 (PB12高) */
void    Steam_PumpOff(void);    /* 进水泵关 */
void    Steam_ValveOpen(void);  /* 蒸汽阀开 (PB13高) */
void    Steam_ValveClose(void); /* 蒸汽阀关 */

#endif
