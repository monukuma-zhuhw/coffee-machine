/**
 * @file    brew_heater.c
 * @brief   冲泡头加热片模块实现 (PB10 / 继电器通断控温) - STM32F0 SPL
 *
 * 控制对象: **冲泡头金属块温度** (PA0 传感器贴在加热片/冲泡头上),
 *          与锅炉水温完全无关, 两条独立控制回路.
 *
 * 控制用"滞回开关"而非 PID:
 *   继电器只有通/断两态, 精度要求低 (冲泡头在设定−4 ~ 设定 之间即可),
 *   故用最简单的两态开关, 靠两个阈值构成滞回带防抖动.
 *
 * 控温目标: 让冲泡头在 [设定−BREW_HYST_HIGH_C, 设定] 附近往复.
 *   例设定85℃, LOW=4, HIGH=0 → 低于81℃满功率加热, 到85℃停止.
 */

#include "brew_heater.h"
#include "delay.h"     /* GetTick() */

/* 引脚直接操作: BSRR置位=高(继电器吸合=加热), BRR复位=低(停止) */
#define BREWHEATER_PIN_ON()     (BREW_HEATER_GPIO_PORT->BSRR = BREW_HEATER_GPIO_PIN)
#define BREWHEATER_PIN_OFF()    (BREW_HEATER_GPIO_PORT->BRR  = BREW_HEATER_GPIO_PIN)

static uint8_t g_enabled = 0;          /* 控温使能标志 */
static uint8_t g_relay_state = 0;      /* 继电器当前是否吸合 */
static float   g_head_target = 0.0f;    /* 冲泡头设定温度 (℃), 来自 settings.head_temp */

/* 决策节流 (ms): 防止传感器抖动导致继电器高频动作 */
static uint32_t g_relay_last_tick = 0;

/* 调试用 */
static float    g_dbg_duty = 0.0f;      /* 0 或 100 (满功率/停止) */

/**
 * @brief 初始化 PB10 为推挽输出, 默认继电器断开 (不加热)
 */
void BrewHeater_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    /* 下拉: 复位态为低 = 继电器断开 = 不加热 (故障安全方向) */
    GPIO_InitStructure.GPIO_Pin   = BREW_HEATER_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
    GPIO_Init(BREW_HEATER_GPIO_PORT, &GPIO_InitStructure);
    BREWHEATER_PIN_OFF();

    g_enabled        = 0;
    g_relay_state    = 0;
    g_head_target    = 0.0f;
    g_relay_last_tick = 0;
    g_dbg_duty       = 0.0f;
}

/**
 * @brief 继电器吸合 (加热)
 */
void BrewHeater_RelayOn(void)
{
    BREWHEATER_PIN_ON();
    g_relay_state = 1;
}

/**
 * @brief 继电器断开 (停止加热)
 */
void BrewHeater_RelayOff(void)
{
    BREWHEATER_PIN_OFF();
    g_relay_state = 0;
}

uint8_t BrewHeater_RelayIsOn(void) { return g_relay_state; }

/**
 * @brief 启用控温 (萃取流程入口)
 * @param  head_target 冲泡头设定温度 (℃), 取自 settings.head_temp
 */
void BrewHeater_HeatStart(float head_target)
{
    g_head_target     = head_target;
    g_enabled         = 1;
    g_relay_last_tick = GetTick();
    g_dbg_duty        = 0.0f;
    BrewHeater_RelayOff();
}

/**
 * @brief 仅更新设定温度, 保持使能与滞回状态连续
 * @note  预浸泡→全压切换时调用; 滞回状态由继电器实际状态决定, 无需额外处理
 */
void BrewHeater_SetTarget(float head_target)
{
    g_head_target = head_target;
}

/**
 * @brief 停用控温 (出饮结束), 强制断开继电器
 */
void BrewHeater_HeatStop(void)
{
    g_enabled     = 0;
    g_head_target = 0.0f;
    g_dbg_duty    = 0.0f;
    BrewHeater_RelayOff();
}

/**
 * @brief 滞回开关控温 (主循环 10ms 节拍调用, 内部 1 秒节流)
 * @param  head_temp 冲泡头实测温度 (℃), PA0 传感器贴在冲泡头金属块上
 * @param  now       GetTick() 毫秒时戳
 *
 * 规则 (满功率 / 停止 两态, 无中间档):
 *   冲泡头 <  (设定 − BREW_HYST_LOW_C)  → 满功率加热
 *   冲泡头 >= (设定 − BREW_HYST_HIGH_C) → 停止加热
 *   两阈值之间维持当前状态 (滞回带, 防抖动)
 *
 * 加热启动 (进入低阈值) 立即生效, 不等节流窗口 —— 保证响应速度.
 * 停止判定走 1 秒节流, 防传感器噪声导致继电器频繁断开重吸.
 *
 * 与锅炉温度无关: 锅炉 (boiler.c) 管锅炉水温, 本模块管冲泡头温度, 两条独立回路.
 */
void BrewHeater_Update(float head_temp, uint32_t now)
{
    float low_thr, high_thr;
    uint8_t timed_out;

    /* 未启用: 保持断开 */
    if (!g_enabled) {
        return;
    }

    /* 传感器无效: 安全断开 */
    if (head_temp < -100.0f) {
        BrewHeater_RelayOff();
        g_dbg_duty = 0.0f;
        return;
    }

    low_thr  = g_head_target - BREW_HYST_LOW_C;    /* 例 85−4 = 81℃ */
    high_thr = g_head_target - BREW_HYST_HIGH_C;   /* 例 85−0 = 85℃ */

    timed_out = (now - g_relay_last_tick >= BREW_RELAY_TICK_MS) ? 1u : 0u;
    if (timed_out) {
        g_relay_last_tick = now;
    }

    /* 加热启动: 低于下沿立即满功率, 不等节流 (响应优先) */
    if (head_temp < low_thr) {
        BrewHeater_RelayOn();
        g_dbg_duty = 100.0f;
        return;
    }

    /* 停止判定: 达到上沿即停, 走节流防抖动 */
    if (head_temp >= high_thr && timed_out) {
        BrewHeater_RelayOff();
        g_dbg_duty = 0.0f;
        return;
    }

    /* 滞回带内: 维持当前状态, g_dbg_duty 保持上次值 */
}

/**
 * @brief 取当前状态与两个阈值 (调试用)
 */
void BrewHeater_GetTerms(float *duty_pct, uint8_t *relay_on,
                         float *low_thr, float *high_thr)
{
    if (duty_pct) *duty_pct = g_dbg_duty;
    if (relay_on) *relay_on = g_relay_state;
    if (low_thr)  *low_thr  = g_head_target - BREW_HYST_LOW_C;
    if (high_thr) *high_thr = g_head_target - BREW_HYST_HIGH_C;
}