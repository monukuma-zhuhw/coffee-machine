/**
 * @file    key_monitor.c
 * @brief   按键状态监测模块实现 (输入下拉, 软件消抖) - STM32F0 SPL
 *
 * 开机防误触发:
 *   1. KeyMonitor_Init() 只配 GPIO 不读电平 (避免读切换瞬态)
 *   2. 开机 KEY_BOOT_QUIET_MS 内 KeyMonitor_Update() 只同步电平,
 *      不生成任何事件 (覆盖 GPIO 切换瞬态 + 外部干扰窗口)
 */

#include "key_monitor.h"

/* 开机稳定期: 此期间 Update 只同步电平, 不生成任何按键事件.
 * 防 PA5 从 POWER_CH1 输出切换为 KEY1 输入时的瞬态毛刺或
 * 外部干扰/按键抖动导致的误触发. */
#define KEY_BOOT_QUIET_MS     100u

static GPIO_TypeDef *const s_key_port[KEY_COUNT] = {
    KEY1_PORT, KEY2_PORT, KEY3_PORT
};
static const uint16_t s_key_pin[KEY_COUNT] = {
    KEY1_PIN, KEY2_PIN, KEY3_PIN
};

static uint8_t  g_key_state[KEY_COUNT];       /* 已确认状态(消抖后) */
static uint8_t  g_key_raw_last[KEY_COUNT];    /* 上次原始采样 */
static uint32_t g_key_change_tick[KEY_COUNT]; /* 原始电平变化时刻 */
static uint8_t  g_key_event[KEY_COUNT];       /* 待处理事件标志 */
static uint8_t  g_key_event_state[KEY_COUNT]; /* 事件对应的新状态 */
static uint32_t g_key_init_tick;              /* KeyMonitor_Init 调用时刻 */

void KeyMonitor_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    uint8_t i;

    /* 按端口使能时钟 (按键引脚换端口后无需改这里) */
    for (i = 0; i < KEY_COUNT; i++) {
        if (s_key_port[i] == GPIOA)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
        else if (s_key_port[i] == GPIOB)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
        else if (s_key_port[i] == GPIOC)
            RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    }

    /* 按键输入: 上拉 (电路: VCC→上拉→GPIO→按键→GND)
     * 松开=被外部上拉拉到高电平, 按下=按键闭合拉到低电平 */
    GPIO_InitStructure.GPIO_Pin = KEY1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_2;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    for (i = 0; i < KEY_COUNT; i++) {
        GPIO_InitStructure.GPIO_Pin = s_key_pin[i];
        GPIO_Init(s_key_port[i], &GPIO_InitStructure);

        /* 状态清零: 真实电平由首次 Update 在开机稳定期内同步,
         * 避免立即读切换瞬态电平 (PA5 之前是 POWER_CH1 输出态) */
        g_key_state[i] = 0;
        g_key_raw_last[i] = 0;
        g_key_change_tick[i] = 0;
        g_key_event[i] = 0;
        g_key_event_state[i] = 0;
    }
    g_key_init_tick = GetTick();
}

/**
 * @brief  按键扫描 (主循环周期调用, 建议10ms一次)
 * @param  now 当前系统tick (GetTick())
 * @note   原始电平变化刷新计时起点; 新电平稳定保持超过消抖时间
 *         才确认为状态变化并缓存事件.
 *         开机 KEY_BOOT_QUIET_MS 内只同步电平不出事件.
 */
void KeyMonitor_Update(uint32_t now)
{
    uint8_t i;

    /* 开机稳定期: 仅同步电平, 不生成任何事件
     * 防止 Init 阶段 GPIO 模式切换瞬态或外部干扰误触发按键 */
    if ((now - g_key_init_tick) < KEY_BOOT_QUIET_MS) {
        for (i = 0; i < KEY_COUNT; i++) {
            uint8_t raw = GPIO_ReadInputDataBit(s_key_port[i], s_key_pin[i]);
            g_key_raw_last[i] = raw;
            g_key_state[i] = raw;
            g_key_change_tick[i] = now;
        }
        return;
    }

    for (i = 0; i < KEY_COUNT; i++) {
        uint8_t raw = GPIO_ReadInputDataBit(s_key_port[i], s_key_pin[i]);

        if (raw != g_key_raw_last[i]) {
            /* 原始电平变化: 记录时刻, 开始消抖计时 */
            g_key_raw_last[i] = raw;
            g_key_change_tick[i] = now;
        } else if (raw != g_key_state[i] &&
                   (now - g_key_change_tick[i]) >= KEY_DEBOUNCE_MS) {
            /* 新电平稳定超过消抖时间: 确认状态变化 */
            g_key_state[i] = raw;
            g_key_event[i] = 1;
            g_key_event_state[i] = raw;
        }
    }
}

uint8_t KeyMonitor_GetState(uint8_t key_idx)
{
    if (key_idx >= KEY_COUNT) return 0;
    return g_key_state[key_idx];
}

/**
 * @brief  取出一个按键状态变化事件 (FIFO按按键序号)
 * @param  key_idx 输出: 按键序号 (0~KEY_COUNT-1)
 * @param  state   输出: 新状态 (KEY_STATE_ON/OFF)
 * @return 1=取到事件, 0=当前无事件
 */
uint8_t KeyMonitor_PopEvent(uint8_t *key_idx, uint8_t *state)
{
    uint8_t i;

    for (i = 0; i < KEY_COUNT; i++) {
        if (g_key_event[i]) {
            g_key_event[i] = 0;
            if (key_idx) *key_idx = i;
            if (state)   *state = g_key_event_state[i];
            return 1;
        }
    }
    return 0;
}
