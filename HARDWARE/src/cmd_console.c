/**
 * @file    cmd_console.c
 * @brief   串口调试命令控制台实现 - STM32F0 SPL
 *
 * 数据流: 调试串口(DEBUG_USART) RXNE中断 → CmdConsole_IsrByte (逐字节入行缓冲)
 *         → CmdConsole_Poll (主循环解析执行)
 *
 * 命令表: 加一行 {名字, 处理函数, 说明} 即可扩展新命令.
 */

#include "cmd_console.h"

#if SYS_UART_CMD_ENABLE

#include <stdio.h>
#include <string.h>

/* ---- 行缓冲 (ISR 写, 主循环读, 用 volatile + 双缓冲避免竞态) ---- */
static volatile char     s_line[CMD_CONSOLE_LINE_MAX];
static volatile uint8_t  s_len  = 0;            /* 当前行已写字节数 */
static volatile uint8_t  s_done = 0;            /* 行完成标志 */
static char              s_exec[CMD_CONSOLE_LINE_MAX];  /* 主循环侧解析副本 */

/*---------------------------------------------------------------
 * 命令处理函数表
 *-----------------------------------------------------------*/
typedef void (*CmdHandler_t)(void);

/* 前置声明: 实际实现在 main.c (需要调用出饮流程) */
extern void Cmd_Trigger_Extract(void);        /* /extract → 萃取 */
extern void Cmd_Trigger_HotWater(void);       /* /hot     → 热水 */
extern void Cmd_Trigger_Steam(void);          /* /steam   → 蒸汽 */
extern void Cmd_Trigger_Cold(void);           /* /cold    → 降温冲洗(泵满功率30s, 加热关) */
extern void Cmd_Trigger_Curve(void);          /* /curve   → 变压模式(跳page4, 泵跟预设曲线) */

typedef struct {
    const char   *name;       /* 命令名 (不含前导 '/') */
    CmdHandler_t  handler;
    const char   *help;       /* 帮助说明 */
} CmdEntry_t;

static const CmdEntry_t s_cmd_table[] = {
    { "extract", Cmd_Trigger_Extract, "Start brew process (same as KEY1)" },
    { "hot",     Cmd_Trigger_HotWater, "Start hot water (same as KEY2)" },
    { "steam",   Cmd_Trigger_Steam,   "Start steam (same as KEY3)" },
    { "cold",    Cmd_Trigger_Cold,    "Cool flush: pump 100% 30s, heater off" },
    { "curve",   Cmd_Trigger_Curve,   "Var-pressure: page4, pump follows preset curve" },
};

static void Cmd_Help(void)
{
    uint8_t i;
    printf("\r\n=== Commands ===\r\n");
    for (i = 0; i < sizeof(s_cmd_table)/sizeof(s_cmd_table[0]); i++) {
        printf("  /%-8s : %s\r\n", s_cmd_table[i].name, s_cmd_table[i].help);
    }
    printf("  /%-8s : %s\r\n", "help", "Show this list");
    printf("==================\r\n");
}

/*---------------------------------------------------------------
 * ISR 侧: 收字节入行缓冲
 *-----------------------------------------------------------*/
void CmdConsole_IsrByte(uint8_t byte)
{
    /* 上一行还没被主循环消费 → 丢弃新字节, 防止缓冲错乱 */
    if (s_done) {
        return;
    }

    if (byte == '\r' || byte == '\n') {
        /* 换行: 结束本行 */
        if (s_len > 0) {
            s_line[s_len] = '\0';
            s_done = 1;
        }
        s_len = 0;
        return;
    }

    /* 可打印字符才收, 忽略回车以外的控制字符 */
    if (byte < 0x20 || byte > 0x7E) {
        return;
    }

    /* 溢出保护: 丢弃多余字符直到换行 */
    if (s_len >= (CMD_CONSOLE_LINE_MAX - 1)) {
        s_len = 0;
        return;
    }

    s_line[s_len++] = (char)byte;
}

/*---------------------------------------------------------------
 * 主循环侧: 取完整行 → 解析 → 执行
 *-----------------------------------------------------------*/
void CmdConsole_Poll(void)
{
    char  *p;
    char  *arg;
    uint8_t i;

    if (!s_done) {
        return;
    }

    /* 原子取走: 关中断拷贝到 s_exec, 清标志, 再开中断 */
    __disable_irq();
    memcpy(s_exec, (const char *)s_line, CMD_CONSOLE_LINE_MAX);
    s_done = 0;
    __enable_irq();

    /* 跳过大端/小端的字符串开头空白 */
    p = s_exec;
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') {
        return;   /* 空行 */
    }

    /* 可选前导 '/' */
    if (*p == '/') {
        p++;
    }

    /* 命令名与参数用空格分隔: 命令名到第一个空格为止 */
    arg = p;
    while (*arg != '\0' && *arg != ' ' && *arg != '\t') arg++;
    if (*arg != '\0') {
        *arg++ = '\0';      /* 命令名结束 */
        while (*arg == ' ' || *arg == '\t') arg++;   /* 跳过参数前空白 */
    }

    /* 在命令表里查找 */
    for (i = 0; i < sizeof(s_cmd_table)/sizeof(s_cmd_table[0]); i++) {
        if (strcmp(p, s_cmd_table[i].name) == 0) {
            printf(">>> CMD: %s\r\n", s_cmd_table[i].name);
            s_cmd_table[i].handler();
            return;
        }
    }

    if (strcmp(p, "help") == 0) {
        Cmd_Help();
        return;
    }

    printf(">>> CMD unknown: %s (try /help)\r\n", p);
}

#endif /* SYS_UART_CMD_ENABLE */