/**
 * @file    settings.c
 * @brief   客户设定掉电保存实现 - STM32F0 SPL
 *
 * 写Flash流程: 擦末页 -> 按半字编程 -> Lock -> 回读校验.
 * 擦写期间CPU取指会停等(单Bank Flash特性), 中断响应延迟~20-40ms,
 * 但不会造成数据损坏, SysTick/定时器计数不受影响.
 */

#include "settings.h"
#include "flash_op.h"
#include "delay.h"
#include "stm32f0xx_flash.h"
#include <string.h>
#include <stdio.h>

/* 擦写 flash 的关中断临界区宏见 flash_op.h
 * (F0 单 Bank: erase/program 期间 CPU 不能从 flash 取指,
 *  此时任何中断触发都会因 flash 总线忙触发总线错误 -> HardFault -> Lockup) */

/* 擦页之后的"落定"延时(ms).
 *
 * F0 擦完一页 Flash 内部需要一段恢复时间才能接着编程, 连着写最容易出事.
 * 这里把擦除和编程拆成两段临界区, 中间 delay_ms 让 SysTick 跑起来:
 *   1) 给 Flash 控制器真实的恢复时间(墙钟时间, 不是空转轮数)
 *   2) 擦除那 ~30ms 里中断是关的, 拆开后泵/加热的时序不至于冻结太久
 */
#define SETTINGS_ERASE_SETTLE_MS   30u

/* Settings_t 落盘用的对齐缓冲(半字写入, 长度取偶) */
#define SETTINGS_FLASH_BUF_MAX   64u

static Settings_t g_settings;          /* 内存镜像, 一切读写都走这里 */
static uint8_t    g_dirty = 0;         /* 有未落盘修改 */
static u32        g_last_change = 0;   /* 最后一次置脏时刻(ms) */

/* 计算checksum字段之前所有u16字的累加和 */
static uint16_t Settings_CalcChecksum(const Settings_t *p)
{
    const uint16_t *w = (const uint16_t *)p;
    uint16_t count = (uint16_t)(((const uint8_t *)&p->checksum
                                 - (const uint8_t *)p) / 2);
    uint16_t i;
    uint16_t sum = 0;

    for (i = 0; i < count; i++) {
        sum += w[i];
    }
    return sum;
}

void Settings_LoadDefaults(void)
{
    memset(&g_settings, 0, sizeof(g_settings));
    g_settings.magic        = SETTINGS_MAGIC;
    g_settings.version      = SETTINGS_VERSION;
    g_settings.brew_temp    = SETTINGS_DEF_BREW_TEMP;
    g_settings.target_pres  = SETTINGS_DEF_TARGET_PRES;
    g_settings.head_temp    = SETTINGS_DEF_HEAD_TEMP;
    g_settings.brew_time_s  = SETTINGS_DEF_BREW_TIME_S;
    g_settings.hot_time_s   = SETTINGS_DEF_HOT_TIME_S;
    g_settings.cup_count    = 0;                          /* 杯数从0开始 */
    g_settings.light_level  = SETTINGS_DEF_LIGHT_LEVEL;   /* 灯板默认最亮档 */
    g_settings.brightness   = SETTINGS_DEF_BRIGHTNESS;    /* 屏幕背光默认最亮 */
    g_settings.lang_mode    = SETTINGS_DEF_LANG;          /* 默认中文 */
    g_settings.preinfuse_duration_s = SETTINGS_DEF_PREINFUSE_DURATION_S;
    g_settings.preinfuse_stop_s     = SETTINGS_DEF_PREINFUSE_STOP_S;
    g_settings.preinfuse_master     = SETTINGS_DEF_PREINFUSE_MASTER;
    g_settings.curve_knob_val       = SETTINGS_DEF_CURVE_KNOB_VAL;
    g_settings.reserved     = 0xFFFFu;
    g_settings.checksum     = Settings_CalcChecksum(&g_settings);
}

/* Flash中的数据是否有效: 标志/版本/校验和/数值范围全过才认 */
static uint8_t Settings_FlashValid(void)
{
    const Settings_t *p = (const Settings_t *)SETTINGS_FLASH_PAGE;

    if (p->magic != SETTINGS_MAGIC)         return 0;  /* 空片(0xFFFF)或无关数据 */
    if (p->version != SETTINGS_VERSION)     return 0;  /* 结构已改版, 回默认 */
    if (p->checksum != Settings_CalcChecksum(p)) return 0;

    /* 数值合理性: 防校验和碰巧通过的坏数据 */
    if (p->brew_temp < 20.0f || p->brew_temp > 110.0f)  return 0;
    if (p->target_pres < 1.0f || p->target_pres > 16.0f) return 0;
    if (p->head_temp < 20.0f || p->head_temp > 110.0f)  return 0;
    if (p->brew_time_s < 15 || p->brew_time_s > 99)    return 0;
    if (p->hot_time_s == 0 || p->hot_time_s > 300)      return 0;
    if (p->cup_count > SETTINGS_CUP_COUNT_MAX)          return 0;
    if (p->light_level > 10)                            return 0;
    if (p->brightness > 10)                             return 0;
    if (p->lang_mode > 1)                               return 0;
    if (p->preinfuse_duration_s < 1 || p->preinfuse_duration_s > 9) return 0;
    if (p->preinfuse_stop_s     < 1 || p->preinfuse_stop_s     > 9) return 0;
    if (p->preinfuse_master > 1)                              return 0;
    if (p->curve_knob_val > SETTINGS_CURVE_KNOB_MAX)         return 0;

    return 1;
}

void Settings_Init(void)
{
    if (Settings_FlashValid()) {
        memcpy(&g_settings, (const void *)SETTINGS_FLASH_PAGE, sizeof(g_settings));
        printf("[Settings] loaded: temp=%.0fC head=%.0fC pres=%.0fBar time=%us/%us "
               "cups=%u backlight=%u lang=%u light_level=%u\r\n",
               g_settings.brew_temp, g_settings.head_temp, g_settings.target_pres,
               g_settings.brew_time_s, g_settings.hot_time_s, g_settings.cup_count,
               g_settings.brightness, g_settings.lang_mode, g_settings.light_level);
    } else {
        /* 首次上电/数据无效: 装默认值并落盘 */
        printf("[Settings] flash invalid, using defaults\r\n");
        Settings_LoadDefaults();
        (void)Settings_Save();
    }
    g_dirty = 0;
    g_last_change = GetTick();
}

const Settings_t *Settings_Get(void)
{
    return &g_settings;
}

Settings_t *Settings_Edit(void)
{
    return &g_settings;
}

void Settings_MarkDirty(void)
{
    g_dirty = 1;
    g_last_change = GetTick();
}

uint8_t Settings_IsDirty(void)
{
    return g_dirty;
}

uint8_t Settings_Save(void)
{
    uint8_t st_erase;
    uint8_t st_prog;
    uint8_t buf[SETTINGS_FLASH_BUF_MAX];
    u32 len;

    /* 刷新标志/版本/校验和后再落盘 */
    g_settings.magic   = SETTINGS_MAGIC;
    g_settings.version = SETTINGS_VERSION;
    g_settings.checksum = Settings_CalcChecksum(&g_settings);

    /* 半字写入, 长度取偶 */
    len = (u32)sizeof(g_settings);
    len = (len + 1u) & ~1u;
    if (len > SETTINGS_FLASH_BUF_MAX) return 1;
    memset(buf, 0xFF, len);
    memcpy(buf, &g_settings, sizeof(g_settings));

    /* ===== 第 1 段: 擦页, 关中断 ~20~40ms (物理下限) =====
     * 关中断: 写 flash 期间 CPU 不能从 flash 取指, 中断触发会导致总线错误
     * -> Lockup. 注意: 临界区内不调 printf (printf 占用大栈, 会触发栈溢出). */
    FLASH_OP_CRITICAL_BEGIN();
    FLASH_Unlock();
    st_erase = Flash_ErasePage(SETTINGS_FLASH_PAGE);
    FLASH_Lock();
    FLASH_OP_CRITICAL_END();

    /* 擦除后落定: 让 SysTick 跑起来, 同时给 Flash 控制器恢复时间 */
    delay_ms(SETTINGS_ERASE_SETTLE_MS);

    /* ===== 第 2 段: 编程, 关中断 ~1ms ===== */
    FLASH_OP_CRITICAL_BEGIN();
    FLASH_Unlock();
    st_prog = st_erase;
    if (st_prog == 0u) {
        st_prog = Flash_ProgramBlock(SETTINGS_FLASH_PAGE, buf, (uint16_t)len);
    }
    FLASH_Lock();
    FLASH_OP_CRITICAL_END();

    if (st_prog != 0u) {
        printf("[Settings] save FAILED: erase=%u prog=%u\r\n",
               (unsigned)st_erase, (unsigned)st_prog);
        return 1;
    }

    /* 回读校验: 不一致视为失败 */
    if (memcmp((const void *)SETTINGS_FLASH_PAGE,
               (const void *)&g_settings, sizeof(g_settings)) != 0) {
        printf("[Settings] save FAILED: verify mismatch\r\n");
        return 1;
    }

    g_dirty = 0;
    return 0;
}

void Settings_Task(u32 now)
{
    if (!g_dirty) {
        return;
    }
    /* 修改停止静置SETTINGS_COMMIT_DELAY_MS后才落盘(合并连改) */
    if (now - g_last_change < SETTINGS_COMMIT_DELAY_MS) {
        return;
    }
    if (Settings_Save() != 0) {
        /* 写失败: 保持脏标志, 下一节拍重试 */
        return;
    }
}
