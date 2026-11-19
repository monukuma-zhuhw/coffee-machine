#ifndef __SYS_H
#define __SYS_H
#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "stdlib.h"
#include <string.h>

//0,不支持ucos
//1,支持ucos
//#define SYSTEM_SUPPORT_UCOS		0
#define UART_SWAP

/* 串口命令控制台开关 (调试用)
 * 0/注释掉: 禁用, 调试串口纯输出(printf)
 * 1: 启用, 调试串口可接收文本命令 (/extract /hot /steam /help)
 * 注意: 命令接收独占"调试串口"(DEBUG_USART), 与 DWIN 屏通信口互不影响,
 *       两口由 UART_SWAP 宏区分, 切换 UART_SWAP 时命令口自动跟随. */
#define SYS_UART_CMD_ENABLE     1

/* 屏端 VP 上报调试打印开关
 * 注释掉(当前): 屏蔽 "[vp] read vp=..." 逐条打印 (抓控温曲线时噪音太大)
 * 取消注释:     打开, 每条 0x83 上报都打印 (排查屏端交互时打开) */
//#define DEBUG_PRINT_VP_READ

/* 串口引脚交换控制
 * 默认: 调试串口=USART1(PA9/PA10), 迪文屏=USART2(PA2/PA3)
 * 定义 UART_SWAP 后: 调试串口=USART2(PA2/PA3), 迪文屏=USART1(PA9/PA10)
 */
#ifdef UART_SWAP
  /* 调试串口: USART2 PA2/PA3 */
  #define DEBUG_USART              USART2
  #define DEBUG_USART_IRQn         USART2_IRQn
  #define DEBUG_USART_RCC          RCC_APB1Periph_USART2
  #define DEBUG_USART_TX_PIN       GPIO_Pin_2
  #define DEBUG_USART_RX_PIN       GPIO_Pin_3
  #define DEBUG_USART_TX_SRC       GPIO_PinSource2
  #define DEBUG_USART_RX_SRC       GPIO_PinSource3
  #define DEBUG_USART_AF           GPIO_AF_1

  /* 迪文屏串口: USART1 PA9/PA10 */
  #define DWIN_HW_USART            USART1
  #define DWIN_HW_IRQn             USART1_IRQn
  #define DWIN_HW_RCC              RCC_APB2Periph_USART1
  #define DWIN_HW_TX_PIN           GPIO_Pin_9
  #define DWIN_HW_RX_PIN           GPIO_Pin_10
  #define DWIN_HW_TX_SRC           GPIO_PinSource9
  #define DWIN_HW_RX_SRC           GPIO_PinSource10
  #define DWIN_HW_AF               GPIO_AF_1
#else
  /* 调试串口: USART1 PA9/PA10 */
  #define DEBUG_USART              USART1
  #define DEBUG_USART_IRQn         USART1_IRQn
  #define DEBUG_USART_RCC          RCC_APB2Periph_USART1
  #define DEBUG_USART_TX_PIN       GPIO_Pin_9
  #define DEBUG_USART_RX_PIN       GPIO_Pin_10
  #define DEBUG_USART_TX_SRC       GPIO_PinSource9
  #define DEBUG_USART_RX_SRC       GPIO_PinSource10
  #define DEBUG_USART_AF           GPIO_AF_1

  /* 迪文屏串口: USART2 PA2/PA3 */
  #define DWIN_HW_USART            USART2
  #define DWIN_HW_IRQn             USART2_IRQn
  #define DWIN_HW_RCC              RCC_APB1Periph_USART2
  #define DWIN_HW_TX_PIN           GPIO_Pin_2
  #define DWIN_HW_RX_PIN           GPIO_Pin_3
  #define DWIN_HW_TX_SRC           GPIO_PinSource2
  #define DWIN_HW_RX_SRC           GPIO_PinSource3
  #define DWIN_HW_AF               GPIO_AF_1
#endif

/* F0 (Cortex-M0) 不支持位带操作, 改为直接寄存器访问 */
#define PAout(n)   ((GPIOA->ODR & (1<<(n))) ? 1 : 0)
#define PAin(n)    ((GPIOA->IDR & (1<<(n))) ? 1 : 0)

#define PBout(n)   ((GPIOB->ODR & (1<<(n))) ? 1 : 0)
#define PBin(n)    ((GPIOB->IDR & (1<<(n))) ? 1 : 0)

#define PCout(n)   ((GPIOC->ODR & (1<<(n))) ? 1 : 0)
#define PCin(n)    ((GPIOC->IDR & (1<<(n))) ? 1 : 0)

#define PDout(n)   ((GPIOD->ODR & (1<<(n))) ? 1 : 0)
#define PDin(n)    ((GPIOD->IDR & (1<<(n))) ? 1 : 0)

#define PFout(n)   ((GPIOF->ODR & (1<<(n))) ? 1 : 0)
#define PFin(n)    ((GPIOF->IDR & (1<<(n))) ? 1 : 0)

typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;
typedef const int16_t sc16;
typedef const int8_t sc8;

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;
typedef __I int16_t vsc16;
typedef __I int8_t vsc8;

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;
typedef const uint16_t uc16;
typedef const uint8_t uc8;

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;
typedef __I uint16_t vuc16;
typedef __I uint8_t vuc8;

#endif
