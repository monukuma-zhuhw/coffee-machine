/**
  * @file    stm32f0xx_it.c
  * @brief   中断服务函数
  */

#include "stm32f0xx_it.h"
#include "dwin_uart.h"
#include "ac_pump.h"
#include "encoder.h"
#include "heater_control.h"
#include "light_pwm.h"
#include "brew_heater.h"
#include "usart.h"
#if SYS_UART_CMD_ENABLE
#include "cmd_console.h"
#endif

/* delay.c 中的tick变量 */
extern volatile u32 g_tick;

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
    while (1) {}
}

void SVC_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    g_tick++;
    HEATER_PWM_Tick();   /* 锅炉加热器软件PWM节拍 (PA12, 无定时器PWM复用) */
    /* 萃取加热片 PB10 是继电器通断 (秒级), 萃取加热片控温在 main 循环里做 */
    /* 灯板已改用 TIM3_CH4 硬件 PWM (PB1), 无需 SysTick ISR 调用 */
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/******************************************************************************/

/*==============================================================
 * 串口中断分工 (严格隔离, 互不影响)
 *   DWIN 屏口 (DWIN_HW_USART): 只跑 DGUS 帧解析, 与命令系统无关
 *   调试串口 (DEBUG_USART):   输出 printf + (可选)命令接收
 * 两口由 UART_SWAP 决定谁是谁, 命令接收用 DEBUG_USART 宏自动跟随.
 *==============================================================*/
#if SYS_UART_CMD_ENABLE
static void DebugUsart_RxByte(void)
{
    if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET) {
        CmdConsole_IsrByte((uint8_t)(DEBUG_USART->RDR & 0xFF));
    }
    /* 溢出标志: F0 的 ORE 只能写 ICR.ORECF 清 (读 RDR 清不掉).
     * 不清的话 RXNEIE=1 使 ORE 持续申请中断, 主循环被饿死 (写 Flash 期间
     * 串口工具/键盘发来字节就会溢出). */
    if (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_ORE) != RESET) {
        USART_ClearFlag(DEBUG_USART, USART_FLAG_ORE);
    }
}
#endif

void USART1_IRQHandler(void)
{
#ifdef UART_SWAP
    DWIN_USART_IRQHandler();   /* UART_SWAP: USART1 = DWIN 屏 */
#elif SYS_UART_CMD_ENABLE
    /* 默认: USART1 = 调试串口 → 命令接收.
     * 必须在这里读一次 RDD 清掉 RXNE 标志, 否则 RX 中断会反复重入把 CPU 占满
     * (表现: 主循环跑不动, 发给屏的指令全部失效). */
    DebugUsart_RxByte();
#endif
}

void USART2_IRQHandler(void)
{
#ifndef UART_SWAP
    DWIN_USART_IRQHandler();   /* 默认: USART2 = DWIN 屏 */
#elif SYS_UART_CMD_ENABLE
    DebugUsart_RxByte();       /* UART_SWAP: USART2 = 调试串口 → 命令接收 */
#endif
}

void EXTI0_1_IRQHandler(void)
{
    AC_Pump_ZCD_IRQHandler();
}

void EXTI4_15_IRQHandler(void)
{
    Encoder_EXTI_IRQHandler();
}

void TIM14_IRQHandler(void)
{
    AC_Pump_DelayTimer_IRQHandler();
}
