/**
 * @file    steam.c
 * @brief   蒸汽锅炉继电器控制实现 - STM32F0 SPL
 */

#include "steam.h"

/* 蒸汽进水泵独立控制 (PB12, 高=抽水) */
void Steam_PumpOn(void)
{
    GPIO_SetBits(STEAM_PUMP_PORT, STEAM_PUMP_PIN);
}

void Steam_PumpOff(void)
{
    GPIO_ResetBits(STEAM_PUMP_PORT, STEAM_PUMP_PIN);
}

/* 蒸汽阀门独立控制 (PB13, 高=开) */
void Steam_ValveOpen(void)
{
    GPIO_SetBits(STEAM_OUT_PORT, STEAM_OUT_PIN);
}

void Steam_ValveClose(void)
{
    GPIO_ResetBits(STEAM_OUT_PORT, STEAM_OUT_PIN);
}

void Steam_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* PB12进水泵 + PB13蒸汽输出, 推挽输出, 默认关(低)
     * (PB11已改为热水出水阀, 由boiler.c控制) */
    GPIO_InitStructure.GPIO_Pin = STEAM_PUMP_PIN | STEAM_OUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_ResetBits(STEAM_PUMP_PORT, STEAM_PUMP_PIN);
    GPIO_ResetBits(STEAM_OUT_PORT, STEAM_OUT_PIN);
}
