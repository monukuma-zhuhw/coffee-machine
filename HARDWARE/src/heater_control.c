/**
 * @file    heater_control.c
 * @brief   锅炉加热器控制模块实现 (软件PWM, PA12) - STM32F0 SPL
 *
 * PA12无定时器PWM复用, 用软件PWM:
 *   HEATER_PWM_Tick()由SysTick每1ms调用一次, 100ms为一个周期(100级),
 *   计数器 < 占空比 → 引脚低(加热), 否则引脚高(停止).
 *   低电平加热, 功率0/初始化/保护时引脚恒高(故障安全方向=断).
 */

#include "heater_control.h"
#include <stdio.h>

static volatile uint8_t  g_pwm_duty = 0;        /* 占空比0-100 (加热%) */
static volatile uint8_t  g_pwm_counter = 0;     /* 1ms节拍计数 0-99 */
static uint8_t g_power_percent = 0;
static uint8_t g_protection_triggered = 0;      /* 1=超温保护中 */

/* 引脚直接操作: BSRR置位=高(不加热), BRR复位=低(加热) */
#define HEATER_PIN_HIGH()   (HEATER_GPIO_PORT->BSRR = HEATER_GPIO_PIN)
#define HEATER_PIN_LOW()    (HEATER_GPIO_PORT->BRR  = HEATER_GPIO_PIN)

/**
 * @brief 初始化加热器控制模块 (PA12 GPIO输出, 默认高=不加热)
 */
void HEATER_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = HEATER_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(HEATER_GPIO_PORT, &GPIO_InitStructure);

    g_pwm_duty = 0;
    g_pwm_counter = 0;
    g_power_percent = 0;
    HEATER_PIN_HIGH();     /* 低电平加热: 默认高=停止 */
}

/**
 * @brief 软件PWM节拍, SysTick_Handler每1ms调用一次
 */
void HEATER_PWM_Tick(void)
{
    if (g_pwm_counter < g_pwm_duty) {
        HEATER_PIN_LOW();      /* 低=加热 */
    } else {
        HEATER_PIN_HIGH();     /* 高=停止 */
    }
    if (++g_pwm_counter >= HEATER_PWM_STEPS) {
        g_pwm_counter = 0;
    }
}

uint8_t HEATER_SetPower(uint8_t percent)
{
    if (percent < HEATER_POWER_MIN || percent > HEATER_POWER_MAX)
        return 1;

    /* 超温保护中: 只能关不能开 (锅炉PID每10ms写一次功率, 在这里统一挡住) */
    if (g_protection_triggered) {
        percent = 0;
    }

    g_pwm_duty = percent;
    g_power_percent = percent;

    if (percent == 0) {
        HEATER_PIN_HIGH();     /* 立即停止加热, 不等当前周期走完 */
        g_pwm_counter = 0;
    }
    return 0;
}

uint8_t HEATER_GetPower(void) { return g_power_percent; }
void HEATER_Off(void) { HEATER_SetPower(HEATER_POWER_MIN); }

/* 锅炉超温保护 (主循环10ms调用, 放在锅炉PID之前):
 * 温度 >= HEATER_PROTECT_TEMP 进入保护并立即断开加热, 之后 HEATER_SetPower 只接受 0;
 * 降到 HEATER_PROTECT_RELEASE_TEMP 以下自动解除, 加热交还锅炉PID. 返回 1=保护中 */
uint8_t HEATER_CheckProtection(float current_temp)
{
    if (!g_protection_triggered && current_temp >= HEATER_PROTECT_TEMP) {
        g_protection_triggered = 1;
        HEATER_Off();
        printf(">>> Boiler over-temp (%d C >= %d C): heater forced off\r\n",
               (int)current_temp, (int)HEATER_PROTECT_TEMP);
    } else if (g_protection_triggered && current_temp < HEATER_PROTECT_RELEASE_TEMP) {
        g_protection_triggered = 0;
        printf(">>> Boiler temp %d C < %d C: over-temp protection released\r\n",
               (int)current_temp, (int)HEATER_PROTECT_RELEASE_TEMP);
    }
    return g_protection_triggered;
}

uint8_t HEATER_IsProtectionTriggered(void) { return g_protection_triggered; }
