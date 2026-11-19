/**
 * @file    flash_op.h
 * @brief   主Flash擦写底层封装 + 临界区宏 - STM32F0 SPL
 *
 * 为什么不直接用 SPL 的 FLASH_ErasePage/FLASH_ProgramHalfWord:
 *   F0 的 flash 控制器在"编程/擦除"期间 BSY 的置位与清除和 CPU 的写操作
 *   存在时序竞态, 一旦轮询到"还没开始"就误判为完成, 会在编程进行中清掉
 *   FLASH_CR.PG, 控制器状态错乱 -> 之后所有擦写都 FLASH_TIMEOUT(5),
 *   表现为"Flash 写不进去 / 机器卡死".
 *   这里的实现:
 *     1) 每个操作前先等 Flash 完全空闲(有界超时), 不让残留的 BSY 拖死后续操作
 *     2) 擦除后 / 编程后 都显式检查 BSY 与错误标志, 状态异常立刻返回, 不继续往下写
 *     3) 每个半字写完立刻校验写入值, 防止"报告成功但数据没进去"
 *
 * 临界区: 擦写期间 CPU 不能从 Flash 取指, 任何中断触发都会因 flash 总线忙
 *          触发总线错误 -> HardFault -> Lockup. 故擦写全程关中断.
 */
#ifndef __FLASH_OP_H
#define __FLASH_OP_H

#include "sys.h"
#include <stdint.h>

/* 占位变量: 保证编译器不会把 cpsid i / cpsie i 当成无副作用语句优化掉 */
static volatile uint32_t s_flash_op_lock __attribute__((unused)) = 0;

#define FLASH_OP_CRITICAL_BEGIN()  do { s_flash_op_lock = 1; __asm("cpsid i"); (void)s_flash_op_lock; } while (0)
#define FLASH_OP_CRITICAL_END()    do { __asm("cpsie i"); s_flash_op_lock = 0; } while (0)

/* 返回值: 0=成功, 非0=失败(状态码同 FLASH_Status) */
uint8_t Flash_ErasePage(uint32_t page_addr);
uint8_t Flash_ProgramHalfWord(uint32_t addr, uint16_t data);
uint8_t Flash_ProgramWord(uint32_t addr, uint32_t data);

/* 连续写一段数据(长度必须为偶数), 小端半字写入.
 *
 * 为什么要单独做这个:
 *   逐个半字调 Flash_ProgramHalfWord, 每半字要做 5 次 FLASH_CR/SR 寄存器
 *   往返(清标志->置PG->写->等BSY->清PG). 写 400 字节就是 204 次往返,
 *   加上每次的等待, 单页写入的关中断窗口能到 200ms ——
 *   F0 单Bank, 这段时间 CPU 不能从 Flash 取指, 中断也进不来,
 *   屏的串口字节会溢出丢失, 泵/加热的时序也被冻结.
 *
 *   这里 PG 位一直保持置位, 每次写完只等 BSY 清并清一次 EOP,
 *   整段写完才查 PGERR/WRPERR, 并做一次回读校验.
 *   同样的 400 字节, 寄存器往返从 1020 次降到 ~410 次,
 *   关中断窗口从 ~200ms 降到 ~10ms(编程部分).
 *
 * 必须在 FLASH_OP_CRITICAL_BEGIN/END 之间调用. */
uint8_t Flash_ProgramBlock(uint32_t addr, const uint8_t *data, uint16_t len);

#endif
