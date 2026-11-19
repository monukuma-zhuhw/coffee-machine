/**
 * @file    ac_pump.c
 * @brief   AC泵调速控制实现 (PWM频率调制) - STM32F0 SPL
 *
 * 工作原理:
 *   TIM14产生1ms周期中断, ISR中按周期/高电平时间驱动PC14
 *   PC15 镜像 PC14 电平 (AC_PUMP_GATE_PIN = PC14|PC15, 一次 BSRR/BRR 写操作两引脚)
 *   频率 0~50Hz (单位0.25Hz qhz, 0~200), 占空比30%方波
 *   功率 0.1%级 (0~1000) 线性映射到 0~50Hz:
 *     qhz = 200 * deci / 1000, period_ms = 4000 / qhz
 */

#include "ac_pump.h"

static volatile uint16_t g_power = 0;                   /* 目标功率(0.1%单位, 0~1000) */
static volatile uint8_t  g_enabled = 0;                  /* 使能标志 */
static volatile uint16_t g_frequency_qhz = 0;            /* 当前频率(0.25Hz单位 0~200), 0=停止 */
static volatile uint32_t g_period_ms = 0;                /* 周期(ms), 0=停止 */
static volatile uint32_t g_high_ms = 0;                  /* 高电平时间(ms), 占空比30% */
static volatile uint32_t g_toggle_counter = 0;           /* 1ms计数器 */
static volatile uint8_t  g_pin_level = 0;                /* PC14/PC15 当前电平 (镜像同步) */

/* TIM14时钟: 48MHz, PSC=47 → 1MHz, ARR=999 → 1ms周期 */
#define TIM14_PSC  47
#define TIM14_ARR  999

/* qhz (0.25Hz单位) → 周期(ms): period = 1000 / (qhz/4) = 4000 / qhz */
static void FrequencyToPeriod(uint16_t qhz)
{
    if (qhz == 0) {
        g_frequency_qhz = 0;
        g_period_ms = 0;
        g_high_ms = 0;
    } else {
        if (qhz > AC_FREQ_MAX_QHZ) qhz = AC_FREQ_MAX_QHZ;
        g_frequency_qhz = qhz;
        g_period_ms = 4000u / qhz;
        g_high_ms = (g_period_ms * 30u) / 100u;
        if (g_high_ms == 0) g_high_ms = 1;
    }
}

/* 功率 (0.1%单位 0~1000) → 频率 (qhz): 线性映射 0~1000 → 0~200 (0~50Hz) */
static void PowerToPeriod(uint16_t deci_percent)
{
    uint16_t qhz;

    if (deci_percent == 0) {
        FrequencyToPeriod(0);
    } else {
        qhz = (uint16_t)((uint32_t)AC_FREQ_MAX_QHZ * deci_percent / 1000u);
        FrequencyToPeriod(qhz);
    }
}

void AC_Pump_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_InitStructure;

    /* 使能时钟 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    RCC_APB1PeriphClockCmd(AC_PUMP_DELAY_TIMER_RCC, ENABLE);

    /* Gate输出引脚 (PC14, 默认低) */
    GPIO_InitStructure.GPIO_Pin = AC_PUMP_GATE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(AC_PUMP_GATE_PORT, &GPIO_InitStructure);
    AC_PUMP_GATE_PORT->BRR = AC_PUMP_GATE_PIN;

    /* TIM14配置: 1ms周期中断 */
    TIM_InitStructure.TIM_Prescaler = TIM14_PSC;
    TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_InitStructure.TIM_Period = TIM14_ARR;
    TIM_InitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(AC_PUMP_DELAY_TIMER, &TIM_InitStructure);

    /* 清除TIM_TimeBaseInit产生的UIF悬挂 */
    TIM_ClearFlag(AC_PUMP_DELAY_TIMER, TIM_FLAG_Update);

    TIM_ITConfig(AC_PUMP_DELAY_TIMER, TIM_IT_Update, ENABLE);

    /* NVIC配置 */
    NVIC_InitStructure.NVIC_IRQChannel = AC_PUMP_DELAY_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* 初始状态: 关闭 (TIM14不启动, 由AC_Pump_On启动) */
    g_enabled = 0;
    g_power = 0;
    g_frequency_qhz = 0;
    g_pin_level = 0;
    g_toggle_counter = 0;
    PowerToPeriod(0);
}

void AC_Pump_SetPower(uint16_t deci_percent)
{
    if (deci_percent > AC_PUMP_POWER_MAX) deci_percent = AC_PUMP_POWER_MAX;
    g_power = deci_percent;
    PowerToPeriod(deci_percent);

    /* 功率变为0时, 立即拉低PC14并重置计数器 */
    if (deci_percent == 0) {
        AC_PUMP_GATE_PORT->BRR = AC_PUMP_GATE_PIN;
        g_pin_level = 0;
        g_toggle_counter = 0;
    }
}

/* 直接设频率 (0.25Hz单位 0~200 = 0~50Hz). 同步更新 g_power 便于读回. */
void AC_Pump_SetFrequency(uint16_t qhz)
{
    if (qhz > AC_FREQ_MAX_QHZ) qhz = AC_FREQ_MAX_QHZ;

    if (qhz == 0) {
        g_power = 0;
    } else {
        /* 反算功率 (0.1%单位): qhz 0~200 → deci 0~1000 */
        g_power = (uint16_t)((uint32_t)qhz * 1000u / AC_FREQ_MAX_QHZ);
    }

    FrequencyToPeriod(qhz);

    if (qhz == 0) {
        AC_PUMP_GATE_PORT->BRR = AC_PUMP_GATE_PIN;
        g_pin_level = 0;
        g_toggle_counter = 0;
    }
}

uint16_t AC_Pump_GetPower(void)
{
    return g_power;
}

void AC_Pump_On(void)
{
    g_enabled = 1;
    g_toggle_counter = 0;
    if (g_period_ms > 0) {
        g_pin_level = 1;
        AC_PUMP_GATE_PORT->BSRR = AC_PUMP_GATE_PIN;
    }
    TIM_Cmd(AC_PUMP_DELAY_TIMER, ENABLE);
}

void AC_Pump_Off(void)
{
    g_enabled = 0;
    AC_PUMP_GATE_PORT->BRR = AC_PUMP_GATE_PIN;
    g_pin_level = 0;
    g_toggle_counter = 0;
    TIM_Cmd(AC_PUMP_DELAY_TIMER, DISABLE);
}

uint8_t AC_Pump_IsOn(void)
{
    return g_enabled;
}

uint16_t AC_Pump_GetFrequency(void)
{
    return g_frequency_qhz;
}

/* ZCD中断空桩函数, 保留stm32f0xx_it.c兼容 */
void AC_Pump_ZCD_IRQHandler(void)
{
}

/* TIM14中断: 每1ms触发, 输出30%占空比PWM */
void AC_Pump_DelayTimer_IRQHandler(void)
{
    if (TIM_GetITStatus(AC_PUMP_DELAY_TIMER, TIM_IT_Update) != RESET) {
        TIM_ClearITPendingBit(AC_PUMP_DELAY_TIMER, TIM_IT_Update);

        /* 未使能或功率为0时不输出 */
        if (!g_enabled || g_period_ms == 0) {
            return;
        }

        g_toggle_counter++;
        if (g_toggle_counter >= g_period_ms) {
            g_toggle_counter = 0;
            g_pin_level = 1;
            AC_PUMP_GATE_PORT->BSRR = AC_PUMP_GATE_PIN;
        } else if (g_toggle_counter >= g_high_ms && g_pin_level) {
            g_pin_level = 0;
            AC_PUMP_GATE_PORT->BRR = AC_PUMP_GATE_PIN;
        }
    }
}
