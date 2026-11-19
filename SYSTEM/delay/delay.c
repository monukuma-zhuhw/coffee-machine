#include "delay.h"

volatile u32 g_tick = 0;
static u32 fac_us_loop = 1;

void delay_init(u8 SYSCLK)
{
    (void)SYSCLK;   /* 不再依赖入参, 用真实主频SystemCoreClock */

    /* 确保SystemCoreClock为当前真实HCLK */
    SystemCoreClockUpdate();

    /* 粗略us忙等系数(按主频, 循环约4周期/次) */
    fac_us_loop = SystemCoreClock / 4000000;
    if (fac_us_loop == 0) fac_us_loop = 1;

    /* SysTick用HCLK直接作时钟源, 1ms周期中断 */
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL  = 0;
    /* ENABLE=1, TICKINT=1, CLKSOURCE=1(HCLK直接) */
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_CLKSOURCE_Msk;
}

void delay_ms(u32 nms)
{
    u32 start = g_tick;
    while (g_tick - start < nms);
}

void delay_us(u32 nus)
{
    u32 count = nus * fac_us_loop;
    while (count--) __NOP();
}

u32 GetTick(void)
{
    return g_tick;
}
