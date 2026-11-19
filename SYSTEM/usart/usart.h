#ifndef __USART_H
#define __USART_H
#include "stdio.h"
#include "sys.h"

/* 调试串口初始化 (Rx 中断使能).
 * 引脚/外设由 sys.h 的 UART_SWAP 宏决定:
 *   未定义 → USART1 (PA9/PA10)
 *   定义   → USART2 (PA2/PA3) */
void uart_init(u32 bound);

#endif