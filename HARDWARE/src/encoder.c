/**
 * @file    encoder.c
 * @brief   增量式正交脉冲旋钮实现 (EXTI软件解码) - STM32F0 SPL
 *
 * 工作原理:
 *   PB9(A) / PB8(B) 双相正交信号, 上拉输入, 双边沿EXTI中断
 *   每次中断读当前(A,B)两位, 与上次状态组成4bit索引查正交转移表
 *   转移表输出 +1/-1/0 累加到 g_raw_count
 *   一个机械档位 = 4个边沿, 对外档位增量 = raw变化 / 4
 */

#include "encoder.h"

static volatile int32_t g_raw_count = 0;    /* 边沿累计计数 */
static volatile uint8_t g_last_state = 0;   /* 上次(A<<1|B)状态 */
static int32_t g_last_step_raw = 0;         /* 上次PopDelta时的raw基准 */

/* 正交转移表: index = (prev<<2)|curr, 值为方向增量
 * 无效跳变(同时变两位)输出0
 */
static const int8_t s_qtable[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};

static uint8_t Encoder_ReadState(void)
{
    uint8_t a = (ENC_PORT->IDR & ENC_A_PIN) ? 1 : 0;
    uint8_t b = (ENC_PORT->IDR & ENC_B_PIN) ? 1 : 0;
    return (uint8_t)((a << 1) | b);
}

void Encoder_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* A/B相输入, 下拉 (旋钮高电平有效: 平时低, 转动输出高) */
    GPIO_InitStructure.GPIO_Pin = ENC_A_PIN | ENC_B_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(ENC_PORT, &GPIO_InitStructure);

    /* EXTI映射到PB8/PB9 */
    SYSCFG_EXTILineConfig(ENC_PORT_SOURCE, ENC_A_PIN_SOURCE);
    SYSCFG_EXTILineConfig(ENC_PORT_SOURCE, ENC_B_PIN_SOURCE);

    /* A/B双边沿触发 */
    EXTI_InitStructure.EXTI_Line = ENC_A_EXTI_LINE | ENC_B_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = ENC_EXTI_IRQN;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    g_raw_count = 0;
    g_last_step_raw = 0;
    g_last_state = Encoder_ReadState();
}

int32_t Encoder_GetRawCount(void)
{
    return g_raw_count;
}

/* 返回自上次调用以来的整档位增量, 不足一档的余量保留 */
int32_t Encoder_PopDelta(void)
{
    int32_t raw;
    int32_t diff;
    int32_t steps;

    raw = g_raw_count;
    diff = raw - g_last_step_raw;
    steps = diff / ENC_EDGES_PER_STEP;

    if (steps != 0) {
        /* 消费掉整档部分, 余数保留到下次 */
        g_last_step_raw += steps * ENC_EDGES_PER_STEP;
    }
    return steps;
}

void Encoder_EXTI_IRQHandler(void)
{
    uint8_t handled = 0;

    if (EXTI_GetITStatus(ENC_A_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(ENC_A_EXTI_LINE);
        handled = 1;
    }
    if (EXTI_GetITStatus(ENC_B_EXTI_LINE) != RESET) {
        EXTI_ClearITPendingBit(ENC_B_EXTI_LINE);
        handled = 1;
    }

    if (handled) {
        uint8_t curr = Encoder_ReadState();
        uint8_t idx = (uint8_t)((g_last_state << 2) | curr);
        g_raw_count += s_qtable[idx & 0x0F];
        g_last_state = curr;
    }
}
