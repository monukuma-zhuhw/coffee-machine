/**
 * @file    language.c
 * @brief   中英文语言切换实现 - STM32F0 SPL
 *
 * 0xD060 = 语言标志 (0=中文, 1=英文), 屏端触控改变.
 * 映射表第一列(0x8000-0x8880)是SP描述指针, 需读SP本身2字节得到真正的文本数据VP,
 * 再往该VP写文本.
 *
 * 本文件须以UTF-8 BOM保存, 确保ARMCC把中文字符串字面量识别为UTF-8.
 */

#include "language.h"
#include "dwin_uart.h"
#include <string.h>
#include <stdio.h>

typedef struct {
    uint16_t    sp;       /* SP描述指针 */
    const char *cn;       /* UTF-8中文 */
    const char *en;       /* ASCII英文 */
} LangText_t;

/* SP描述指针 -> 中/英文. 来源: 文本映射表.txt (0x8000-0x8880).
 * 0x8780/0x87C0 原表英文与中文错位, 已按中文语义修正为 BREW TEMP / HEAD TEMP. */
static const LangText_t g_text[] = {
    {0x8000, "萃取温度", "BREW TEMP"},
    {0x8040, "冲泡头温度", "HEAD TEMP"},
    {0x8080, "杯数统计", "CUP COUNT"},
    {0x80C0, "照明", "LIGHT"},
    {0x8100, "预浸泡", "PRE SOAK"},
    {0x8140, "萃取时间", "BREW TIME"},
    {0x8180, "手动模式", "MANUAL"},
    {0x81C0, "变压模式", "VAR PRESS"},
    {0x8200, "杯数", "CUP NUM"},
    {0x8240, "预浸泡", "PRE SOAK"},
    {0x8280, "萃取温度", "BREW TEMP"},
    {0x82C0, "冲泡头温度", "HEAD TEMP"},
    {0x8300, "计时", "TIMING"},
    {0x8340, "灯光调节", "LIGHT ADJ"},
    {0x8380, "出厂设置", "FACT SET"},
    {0x83C0, "预浸泡", "PRE SOAK"},
    {0x8400, "压力值", "PRESS VAL"},
    {0x8440, "温度", "TEMP"},
    {0x8480, "时间", "TIME"},
    {0x84C0, "预浸泡", "PRE SOAK"},
    {0x8500, "温度", "TEMP"},
    {0x8540, "压力值", "PRESS VAL"},
    {0x8580, "时间", "TIME"},
    {0x85C0, "曲线模式", "CURVE MODE"},
    {0x8640, "频率", "FRQ"},
    {0x8680, "时间(S)", "TIME(S)"},
    {0x86C0, "杯数", "CUP NUM"},
    {0x8700, "重置", "RESET"},
    {0x8740, "预浸泡", "PRE SOAK"},
    {0x8780, "萃取温度", "BREW TEMP"},            /* 原表英文错位, 已修正 */
    {0x87C0, "冲泡头温度", "HEAD TEMP"},          /* 原表英文错位, 已修正 */
    {0x8800, "计时", "TIMING"},
    {0x8840, "灯光调节", "LIGHT ADJ"},
    {0x8880, "确认恢复出厂设置?", "Confirm Restore Factory Settings?"},
    {0x8940, "新数据是否替换并保存?", "Replace And Save New Data ?"},   /* page12 变压模式保存确认框 */
    {0x8980, "是", "Yes"},
    {0x89C0, "否", "No"},
};

#define TEXT_COUNT  (sizeof(g_text) / sizeof(g_text[0]))

static uint16_t g_d060 = 0;
static uint8_t  g_lang_applied = 0xFF;   /* 初值非法, 缓存好后首次必刷 */

/* 从SP读出的文本数据VP缓存 */
static uint16_t g_text_data_vp[TEXT_COUNT];
static uint8_t  g_vp_cached = 0;

/* 读取状态机 */
static uint8_t  g_read_idx = 0;
static uint16_t g_pending_read_vp = 0xFFFF;

typedef enum {
    LANG_STATE_INIT = 0,    /* 开机: 异步读取所有SP缓存数据VP */
    LANG_STATE_READY,       /* 缓存完成, 等待语言切换 */
    LANG_STATE_APPLYING     /* 正在写文本 */
} LangState_t;

static LangState_t g_lang_state = LANG_STATE_INIT;
static uint8_t g_apply_idx = 0;

/* SP读请求超时重发: 应答仍可能丢失 (例如写 Flash 期间串口溢出丢字节,
 * 接收帧队列满). 丢失后原逻辑没有重试, g_lang_state 会永久卡在
 * LANG_STATE_INIT. 这里记录发出时刻, 超时未收到对应应答则重发同一个
 * SP 读, 而不是依赖"永远不丢帧"这个不成立的假设. */
#define LANG_READ_TIMEOUT_MS  300u
static uint32_t g_now_cache = 0;      /* Language_Poll每次传入的now, 供OnVPReadResp内部取用 */
static uint32_t g_read_send_tick = 0; /* 当前挂起SP读请求的发送时刻 */

void Language_OnD060(uint16_t value) { g_d060 = value; }

uint16_t Language_GetPendingReadVP(void) { return g_pending_read_vp; }

void Language_OnVPReadResp(uint16_t value)
{
    printf("[Lang] on_resp val=0x%04X idx=%u state=%u\r\n", value, (uint16_t)g_read_idx, (uint8_t)g_lang_state);  /* [DEBUG] */
    if (g_lang_state != LANG_STATE_INIT) return;
    if (g_read_idx >= TEXT_COUNT) return;

    g_text_data_vp[g_read_idx] = value;
    g_read_idx++;

    if (g_read_idx >= TEXT_COUNT) {
        g_vp_cached = 1;
        g_lang_state = LANG_STATE_READY;
        g_pending_read_vp = 0xFFFF;
        printf("[Lang] all %u text VPs cached\r\n", (uint16_t)TEXT_COUNT);
    } else {
        g_pending_read_vp = g_text[g_read_idx].sp;   /* 读下一个SP本身得数据VP */
        g_read_send_tick = g_now_cache;
        DWIN_ReadVar(g_pending_read_vp, 1);
    }
}

void Language_Apply(uint8_t lang)
{
    uint16_t i;
    for (i = 0; i < TEXT_COUNT; i++) {
        const char *s = (lang == LANG_EN) ? g_text[i].en : g_text[i].cn;
        DWIN_SetText(g_text_data_vp[i], s);
    }
    printf("[Lang] apply %s (%u texts)\r\n", lang ? "EN" : "CN", (uint16_t)TEXT_COUNT);
}

void Language_Poll(uint32_t now)
{
    g_now_cache = now;
    switch (g_lang_state) {
    case LANG_STATE_INIT:
        if (g_pending_read_vp == 0xFFFF) {
            /* 启动读取第一个SP本身 */
            g_read_idx = 0;
            g_pending_read_vp = g_text[0].sp;
            g_read_send_tick = now;
            printf("[Lang] INIT: send read SP=0x%04X\r\n", g_pending_read_vp);  /* [DEBUG] */
            DWIN_ReadVar(g_pending_read_vp, 1);
        } else if (now - g_read_send_tick >= LANG_READ_TIMEOUT_MS) {
            /* 超时未收到应答(大概率被其它VP流量抢占单帧接收缓冲覆盖丢失): 重发 */
            g_read_send_tick = now;
            printf("[Lang] SP=0x%04X read timeout, resend (idx=%u)\r\n",
                   g_pending_read_vp, (uint16_t)g_read_idx);  /* [DEBUG] */
            DWIN_ReadVar(g_pending_read_vp, 1);
        }
        break;

    case LANG_STATE_READY: {
        uint8_t desired = g_d060 ? LANG_EN : LANG_CN;
        if (desired != g_lang_applied) {
            g_lang_applied = desired;
            g_lang_state = LANG_STATE_APPLYING;
            g_apply_idx = 0;
        }
        break;
    }

    case LANG_STATE_APPLYING:
        if (g_apply_idx < TEXT_COUNT) {
            const char *s = (g_lang_applied == LANG_EN)
                            ? g_text[g_apply_idx].en
                            : g_text[g_apply_idx].cn;
            DWIN_SetText(g_text_data_vp[g_apply_idx], s);
            g_apply_idx++;
        } else {
            g_lang_state = LANG_STATE_READY;
        }
        break;

    default:
        g_lang_state = LANG_STATE_INIT;
        break;
    }
}

/* ============================================================
 * 文字颜色随变量值变化规则表
 * ============================================================ */

typedef struct {
    uint16_t var_vp;     /* 变量VP地址(如0xD218) */
    uint16_t text_sp;    /* 文字控件SP描述指针(如0x81C0) */
    uint32_t color0;     /* value==0 时的颜色(RGB888) */
    uint32_t color1;     /* value!=0 时的颜色(RGB888) */
} TextColorRule_t;

/* RGB888 -> RGB565 (DWIN_SetTextColor写2字节颜色) */
#define RGB888_TO_RGB565(c)  (uint16_t)((((c) >> 19) << 11) | (((((c) >> 16) & 0xFF) >> 2) << 5) | (((c) >> 3) & 0x1F))

static const TextColorRule_t g_color_rules[] = {
    {0xD22C, 0x8700, 0x0000FF, 0xFFFFFF},   /* 0=纯蓝, 1=白 */
};

#define COLOR_RULE_COUNT  (sizeof(g_color_rules) / sizeof(g_color_rules[0]))

/* 每个规则上次的值, 用于变化检测(只有值变了才写颜色, 避免重复写).
 * 初值0xFFFF表示"未收到过上报", 首次上报只记录不写色, 从第二次开始比较变化. */
#define COLOR_VAL_UNINIT  0xFFFF
static uint16_t g_color_last_val[COLOR_RULE_COUNT];
static uint8_t  g_color_last_inited = 0;

void Language_OnTextColorVar(uint16_t vp, uint16_t value)
{
    uint16_t i;
    if (!g_color_last_inited) {
        for (i = 0; i < COLOR_RULE_COUNT; i++) {
            g_color_last_val[i] = COLOR_VAL_UNINIT;
        }
        g_color_last_inited = 1;
    }
    for (i = 0; i < COLOR_RULE_COUNT; i++) {
        if (g_color_rules[i].var_vp == vp) {
            if (g_color_last_val[i] == COLOR_VAL_UNINIT) {
                /* 首次收到该变量上报, 只记录初值, 不写色(避免开机误触发) */
                g_color_last_val[i] = value;
                return;
            }
            if (value == g_color_last_val[i]) return;  /* 值未变, 跳过 */
            g_color_last_val[i] = value;

            uint32_t rgb888 = (value == 0) ? g_color_rules[i].color0 : g_color_rules[i].color1;
            uint16_t rgb565 = RGB888_TO_RGB565(rgb888);
            DWIN_SetTextColor(g_color_rules[i].text_sp, rgb565);
            printf("[Lang] color var=0x%04X val=%u SP=0x%04X color888=0x%06X color565=0x%04X\r\n",
                   vp, value, g_color_rules[i].text_sp, rgb888, rgb565);
            return;
        }
    }
}
