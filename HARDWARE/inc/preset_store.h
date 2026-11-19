/**
 * @file    preset_store.h
 * @brief   预设水泵曲线掉电存储 (9组 x 400点, 档位制)
 *
 * 档位定义: 0 = 停泵, 1~40, 每档 1.25Hz  (40档 = 50Hz)
 *   - 与 AC 泵原生分辨率完全对齐: qhz(0.25Hz单位) = 档位 * 5, 无损
 *   - 曲线 Y 轴: Y = Hz * 10 = 档位 * 12.5  (0~500)
 *
 * 存储介质: 主Flash 0x0800B000 起连续 9 个 2KB 页, 一组独占一页
 * 存储格式 (页内):
 *   +0x000  uint16 magic    'PR' (0x5052)
 *   +0x002  uint16 version  1
 *   +0x004  uint16 crc16    (400 个档位字节的累加和)
 *   +0x006  uint16 num      点数 = 400
 *   +0x008  uint8  data[400] 档位 (0~40)
 *   页内其余 1640 字节保持擦除态 0xFF (不编程, 省时间)
 *
 * 为什么一组一页: STM32F030 Flash 擦除粒度是整页 2KB,
 * Flash 只能把位 1->0, 改一个字节必须先擦整页再重写整页数据,
 * 所以每组必须独占一页, 擦一组才不会毁掉其它组.
 *
 * 落盘代价: 擦页 ~20-40ms + 编程 204 个半字 ~8ms, 全程关中断, 约 30-50ms.
 * 掉电保护: 擦页与编程之间掉电 -> 该槽变空白, crc 校验不通过 -> 判定无效,
 *            自动回退到出厂曲线, 不会把坏数据当成有效数据用.
 */
#ifndef __PRESET_STORE_H
#define __PRESET_STORE_H

#include "sys.h"
#include <stdint.h>

#define PRESET_SLOT_NUM       9u       /* 9 组预设 */
#define PRESET_POINT_NUM      400u     /* 每组点数 (250ms/点 -> 100秒), 含结束标志 */
#define PRESET_END_MARK       200u     /* 曲线结束标志: 档位值200, 正常档位只有0~40, 不可能出现.
                                         * 数组里第一个遇到它的位置就是本条曲线的有效点数. */
#define PRESET_GEAR_MAX       40u      /* 档位上限 (40档 x 1.25Hz = 50Hz) */
#define PRESET_GEAR_HZ_X100   125u     /* 1 档 = 1.25Hz = 125/100 Hz */
#define PRESET_GEAR_QHZ       5u       /* 1 档 = 5 qhz (0.25Hz单位) */
#define PRESET_CURVE_MUL_X100 1250u    /* 档位->曲线Y: Y = 档位 x 12.5 (Hz x 10) */

#define PRESET_FLASH_BASE     0x0800B000u  /* 预设区起始 (2KB 页对齐) */
#define PRESET_SLOT_SIZE      0x800u      /* 每组 1 个 Flash 页 = 2KB */

/* 异步"恢复出厂"的写页间隔(ms).
 * 每写一页要关中断 ~40ms (擦页20-40 + 编程8), 连续写9页会卡 360ms,
 * 期间 SysTick 停走 -> 泵/加热时序冻结. 故拆成每 PRESET_RESTORE_GAP_MS 写一页,
 * 页与页之间主循环正常运行, 最长单次阻塞只有 ~40ms.
 *
 * 间隔不能太短: F030 擦一页 Flash 是 20~40ms 的高电流脉冲, 连续 9 次擦除
 * (尤其锅炉PTC加热片正在加热时) 可能把 3.3V 轨拉塌导致 MCU 掉电复位.
 * 300ms 给电源足够的恢复时间 (9页共约3秒, 用户无感). */
#define PRESET_RESTORE_GAP_MS 300u

/* 出厂预设曲线表 (固化在固件里, 由 tools/gen_preset_defaults.py 生成) */
extern const uint8_t g_preset_defaults[PRESET_SLOT_NUM][PRESET_POINT_NUM];

uint8_t        Preset_Init(void);              /* 开机自检: 校验9槽, 缺失的写入固化出厂曲线 */
uint8_t        Preset_Load(uint8_t slot);      /* 读槽到 RAM 工作区 (无效则生成出厂曲线) */
uint8_t        Preset_Save(uint8_t slot);      /* RAM 工作区写回槽 (擦页+编程+回读校验) */
void           Preset_Factory(uint8_t slot);   /* 生成出厂曲线到 RAM 工作区 (不写Flash) */
uint8_t        Preset_RestoreFactory(uint8_t slot); /* 该槽恢复出厂曲线并立即落盘 */
uint8_t        Preset_RestoreAllStart(void);   /* 启动"9组全部恢复出厂"(异步, 每次写一页) */
uint8_t        Preset_RestoreAllTask(uint32_t now); /* 主循环调用: 每次写一页, 返回1=全部完成 */
uint8_t        Preset_RestoreAllBusy(void);     /* 1=正在恢复出厂中 */
uint8_t        Preset_SlotValid(uint8_t slot); /* 该槽 Flash 数据是否有效 (实时读Flash校验) */
uint8_t        Preset_SlotOk(uint8_t slot);    /* 该槽有效性的缓存值 (不读Flash) */
uint8_t        *Preset_Data(void);              /* RAM 工作区指针 (当前组 400 点, 可写: 录制时写实际值) */
uint8_t        Preset_CurrentSlot(void);       /* 当前槽号 */
void           Preset_SetCurrentSlot(uint8_t slot);
uint8_t        Preset_LoadedFromFlash(void);   /* 1=工作区来自Flash, 0=出厂曲线(尚未落盘) */
uint8_t        Preset_PointGear(uint16_t idx); /* 第 idx 点档位 */
float          Preset_PointHz(uint16_t idx);   /* 第 idx 点频率 (Hz) */
uint16_t       Preset_Length(void);            /* 当前工作区有效点数 (到结束标志为止, 无标志则=400) */
void           Preset_SetLength(uint16_t len); /* 结束标志设到第 len 点, 有效点数随之更新 (不写Flash) */

#endif
