#include "sys.h"
#include "usart.h"

#if SYSTEM_SUPPORT_UCOS
#include "includes.h"
#endif

int fputc(int ch, FILE *f)
{
    USART_SendData(DEBUG_USART, (uint8_t) ch);
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_TXE) == RESET) {}
    return ch;
}

int fgetc(FILE *f)
{
    while (USART_GetFlagStatus(DEBUG_USART, USART_FLAG_RXNE) == RESET);
    return (int)USART_ReceiveData(DEBUG_USART);
}

static void debug_usart_init(u32 bound, FunctionalState rx_it_enable)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    if (DEBUG_USART_RCC == RCC_APB2Periph_USART1)
        RCC_APB2PeriphClockCmd(DEBUG_USART_RCC, ENABLE);
    else
        RCC_APB1PeriphClockCmd(DEBUG_USART_RCC, ENABLE);

    GPIO_PinAFConfig(GPIOA, DEBUG_USART_TX_SRC, DEBUG_USART_AF);
    GPIO_PinAFConfig(GPIOA, DEBUG_USART_RX_SRC, DEBUG_USART_AF);

    /* TX */
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* RX */
    GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* NVIC */
    NVIC_InitStructure.NVIC_IRQChannel = DEBUG_USART_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = rx_it_enable;
    NVIC_Init(&NVIC_InitStructure);

    /* USART */
    USART_InitStructure.USART_BaudRate = bound;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(DEBUG_USART, &USART_InitStructure);

    USART_ITConfig(DEBUG_USART, USART_IT_RXNE, rx_it_enable);
    USART_Cmd(DEBUG_USART, ENABLE);
}

void uart_init(u32 bound)
{
#if SYS_UART_CMD_ENABLE
    /* 命令控制台启用: 开 RX 中断接收命令 */
    debug_usart_init(bound, ENABLE);
#else
    /* 只用 printf 输出: 不开 RX 中断.
     * 否则 RXNE 标志没人清(无 ISR 处理), 中断会反复重入把 CPU 占满,
     * 表现为主循环跑不动、发给屏的指令全部失效. */
    debug_usart_init(bound, DISABLE);
#endif
}
