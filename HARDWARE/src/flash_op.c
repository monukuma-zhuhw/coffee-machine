/**
 * @file    flash_op.c
 * @brief   主Flash擦写底层封装实现 - STM32F0 SPL
 *
 * 见 flash_op.h 的说明: 这里不用 SPL 的 FLASH_ErasePage/FLASH_ProgramHalfWord,
 * 改为带"等空闲/状态校验/写后回读"的自实现, 规避 F0 上 BSY 时序竞态导致的
 * "控制器状态错乱 -> 之后所有擦写超时 -> 机器卡死".
 *
 * 最重要的一条: **关中断窗口要短**.
 *   F0 单Bank, 擦写期间 CPU 不能从 Flash 取指, 所以擦写全程必须关中断.
 *   擦一页 2KB 本身就要 20~40ms(物理下限, 省不掉), 但编程部分完全可以做到
 *   ~10ms. 之前逐半字调 Flash_ProgramHalfWord, 每半字 5 次寄存器往返,
 *   204 个半字把窗口撑到 ~200ms —— 期间屏的串口字节全部溢出丢失,
 *   泵/加热时序冻结, 表现为"死机".
 *
 * 注意: 所有函数都必须在 FLASH_OP_CRITICAL_BEGIN/END 之间调用
 *       (擦写期间不能有中断).
 */

#include "flash_op.h"
#include "stm32f0xx_flash.h"

/* 等 Flash 空闲(BSY清零)的有界超时. SPL 的 FLASH_ER_PRG_TIMEOUT=0x000B0000
 * 约等于 0.7 秒, 这里用更小的值, 避免"残留BSY"把机器卡那么久. */
#define FLASH_BUSY_SPIN_MAX     2000000u

/* 等 BSY 清零. 返回1=已空闲, 0=超时(控制器卡住) */
static uint8_t Flash_WaitIdle(void)
{
    uint32_t t = FLASH_BUSY_SPIN_MAX;

    while ((FLASH->SR & FLASH_SR_BSY) != 0u) {
        if (t == 0u) {
            return 0u;
        }
        t--;
    }
    return 1u;
}

/* 清错误标志(EOP/PGERR/WRPERR 写1清零).
 * 用 SPL 的宏而不是写死数字, 免得和 F0 的 SR 位定义对不上. */
static void Flash_ClearErrFlags(void)
{
    FLASH->SR = (uint32_t)(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
}

/* 擦一个 2KB 页. page_addr 必须页对齐(低11位为0) */
uint8_t Flash_ErasePage(uint32_t page_addr)
{
    /* 1) 先确保 Flash 空闲: 上一轮若卡在 BSY, 这里最多等一小会儿 */
    if (!Flash_WaitIdle()) {
        return 1u;   /* 控制器还卡着, 放弃本次操作(不擦, 免得状态更乱) */
    }
    Flash_ClearErrFlags();

    /* 2) 发起擦除 */
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = page_addr;
    FLASH->CR |= FLASH_CR_STRT;

    /* 3) 等完成 */
    if (!Flash_WaitIdle()) {
        FLASH->CR &= ~FLASH_CR_PER;
        return 2u;
    }
    FLASH->CR &= ~FLASH_CR_PER;

    /* 4) 状态校验: PGERR/WRPERR 都是失败 */
    if ((FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) != 0u) {
        Flash_ClearErrFlags();
        return 3u;
    }
    /* 5) 回读确认: 擦除后首字节必须是 0xFF */
    if (*(volatile uint16_t *)page_addr != 0xFFFFu) {
        return 4u;
    }
    return 0u;
}

/* 写一个半字 */
uint8_t Flash_ProgramHalfWord(uint32_t addr, uint16_t data)
{
    if (!Flash_WaitIdle()) {
        return 1u;
    }
    Flash_ClearErrFlags();

    FLASH->CR |= FLASH_CR_PG;
    *(volatile uint16_t *)addr = data;

    if (!Flash_WaitIdle()) {
        FLASH->CR &= ~FLASH_CR_PG;
        return 2u;
    }
    FLASH->CR &= ~FLASH_CR_PG;

    if ((FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) != 0u) {
        Flash_ClearErrFlags();
        return 3u;
    }
    /* 回读确认 */
    if (*(volatile uint16_t *)addr != data) {
        return 4u;
    }
    return 0u;
}

/* 写一个字(两个半字) */
uint8_t Flash_ProgramWord(uint32_t addr, uint32_t data)
{
    uint8_t rc;

    rc = Flash_ProgramHalfWord(addr, (uint16_t)(data & 0xFFFFu));
    if (rc != 0u) return rc;

    rc = Flash_ProgramHalfWord(addr + 2u, (uint16_t)((data >> 16) & 0xFFFFu));
    return rc;
}

/* 连续写一段数据(偶数长度), 小端半字写入.
 * PG 位全程保持置位, 每写完一个半字只等 BSY 清 + 清一次 EOP,
 * 整段写完再统一查 PGERR/WRPERR 并回读校验. */
uint8_t Flash_ProgramBlock(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint16_t i;
    uint8_t rc = 0u;

    if (data == NULL) return 1u;
    if ((len & 1u) != 0u) return 1u;   /* 半字写入, 长度必须偶数 */
    if (len == 0u) return 0u;

    if (!Flash_WaitIdle()) {
        return 1u;
    }
    Flash_ClearErrFlags();

    /* PG 置位后连续写: 每个半字只需等 BSY 清 + 清 EOP */
    FLASH->CR |= FLASH_CR_PG;

    for (i = 0; i < len; i += 2u) {
        uint16_t hw = (uint16_t)((uint16_t)data[i]
                                | ((uint16_t)data[i + 1u] << 8));
        *(volatile uint16_t *)(addr + i) = hw;

        if (!Flash_WaitIdle()) {
            rc = 2u;
            break;
        }
        /* 清 EOP, 为下一个半字做准备 (不清 PGERR/WRPERR, 留到段末统一判) */
        FLASH->SR = (uint32_t)FLASH_FLAG_EOP;

        if ((FLASH->SR & (FLASH_SR_PGERR | FLASH_SR_WRPERR)) != 0u) {
            rc = 3u;
            break;
        }
    }

    FLASH->CR &= ~FLASH_CR_PG;

    if (rc != 0u) {
        Flash_ClearErrFlags();
        return rc;
    }

    /* 回读校验: 整段一次比完 */
    for (i = 0; i < len; i += 2u) {
        uint16_t hw = (uint16_t)((uint16_t)data[i]
                                | ((uint16_t)data[i + 1u] << 8));
        if (*(volatile uint16_t *)(addr + i) != hw) {
            return 4u;
        }
    }
    return 0u;
}
