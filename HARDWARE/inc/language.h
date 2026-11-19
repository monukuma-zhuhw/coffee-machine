/**
 * @file    language.h
 * @brief   中英文语言切换模块 - STM32F0 SPL
 *
 * 0xD060: 语言标志 (0=中文, 1=英文), 屏端触控改变.
 * 切换时遍历文本控件表(0x8000-0x8880):
 *   1. 开机异步读取每个描述指针(SP)+6 处存储的文本数据VP并缓存
 *   2. 语言变化时, 用DWIN_SetText往缓存的数据VP写对应语言文本
 */

#ifndef __LANGUAGE_H
#define __LANGUAGE_H

#include "sys.h"
#include <stdint.h>

#define LANG_CN   0u
#define LANG_EN   1u

/* 屏上报 0xD060 时由 on_vpread 转发调用 */
void Language_OnD060(uint16_t value);

/* 返回当前正在等待 0x83 读响应的 VP 地址(无则 0xFFFF). on_vpread 用它判断是否转发. */
uint16_t Language_GetPendingReadVP(void);

/* 把 0x83 读响应(数据VP)转发给语言模块 */
void Language_OnVPReadResp(uint16_t value);

/* 立即应用指定语言(0中1英)到所有文本控件; 前提:数据VP已缓存好 */
void Language_Apply(uint8_t lang);

/* 主循环调用: 初始化读数据VP + 检测语言变化刷新文本
 * now: GetTick(), 用于SP读请求超时重发 (见.c文件说明) */
void Language_Poll(uint32_t now);

/* 根据变量VP的值刷新对应文字控件颜色; on_vpread 在收到任意VP上报时调用 */
void Language_OnTextColorVar(uint16_t vp, uint16_t value);

#endif /* __LANGUAGE_H */
