/**
 * @file    cmd_console.h
 * @brief   串口调试命令控制台 - STM32F0 SPL
 *
 * 用途: 通过调试串口 (USART1, PA9/PA10, 115200) 发送文本命令控制机器,
 *       便于调试时无需按面板按键.
 *
 * 协议: 行结束符 (\r 或 \n) 触发解析. 命令前导 '/' 可选.
 *   例: /extract<CR>   → 触发萃取流程 (等同面板 KEY1)
 *       /help<CR>      → 列出所有命令
 *
 * 接收路径: USART1_IRQHandler (逐字节) → 行缓冲 → CmdConsole_Poll() (主循环)
 *           解析不放在 ISR 里, 避免长字符串比较占用中断时间.
 *
 * 扩展: 在 cmd_table[] 里加一行 {命令名, 处理函数, 说明} 即可.
 */

#ifndef __CMD_CONSOLE_H
#define __CMD_CONSOLE_H

#include "sys.h"
#include <stdint.h>

#if SYS_UART_CMD_ENABLE

/* 行缓冲最大长度 (含结尾 \0) */
#define CMD_CONSOLE_LINE_MAX      32

/* 调试串口 IRQ 每收到1字节调用: 存入行缓冲, 遇换行置完成标志 */
void CmdConsole_IsrByte(uint8_t byte);

/* 主循环调用: 有完整行则解析执行 */
void CmdConsole_Poll(void);

#endif /* SYS_UART_CMD_ENABLE */

#endif /* __CMD_CONSOLE_H */