/**
 * @file    preset_store.c
 * @brief   预设水泵曲线掉电存储实现 (9组 x 400点, 档位制)
 *
 * Flash 布局 (每组独占一个 2KB 页, 页内只编程 408 字节):
 *   +0x000 magic 'PR' | +0x002 version | +0x004 crc16 | +0x006 点数
 *   +0x008 data[400] 档位字节, 其余保持擦除态 0xFF
 *
 * 擦除粒度 2KB 是硬约束: 改一个字节必须先擦整页再重写,
 * 所以一组一页, 擦一组不影响其它组.
 */

#include "preset_store.h"
#include "flash_op.h"
#include "delay.h"
#include "stm32f0xx_flash.h"
#include <string.h>
#include <stdio.h>

#define PRESET_MAGIC       0x5052u   /* 'PR' */
#define PRESET_VERSION     0x0001u
#define PRESET_HDR_SIZE    8u        /* magic(2)+version(2)+crc(2)+num(2) */

/* 擦页之后的"落定"延时(ms).
 *
 * F0 擦完一页 2KB Flash, 内部需要一段恢复时间才能接着编程.
 * 之前把"擦除"和"编程"连着做(中间只靠 BSY 标志), 表现就是:
 *   写一次成功 -> 过一会儿紧接着再写一次 -> 死机.
 *
 * 这里把擦除和编程拆成两段临界区, 中间 delay_ms 让 SysTick 跑起来:
 *   1) 给 Flash 控制器真实的恢复时间(墙钟时间, 不是空转轮数)
 *   2) 擦除那 ~30ms 里中断是关的, 拆开后泵/加热的时序不至于冻结太久
 */
#define PRESET_ERASE_SETTLE_MS   30u

typedef struct {
    uint16_t magic;
    uint16_t version;
    uint16_t crc;
    uint16_t num;
} Preset_Hdr_t;

/* RAM 工作区: 当前正在查看/编辑的那一组 (400 字节) */
static uint8_t g_work[PRESET_POINT_NUM];
static uint8_t g_slot_ok[PRESET_SLOT_NUM];   /* 各槽 Flash 数据有效性 */
static uint8_t g_cur_slot   = 0u;            /* 当前槽号 */
static uint8_t g_from_flash = 0u;            /* 1=工作区来自Flash, 0=出厂曲线(尚未落盘) */
static uint8_t g_area_ok    = 0u;            /* 预设区与代码区不重叠 */
static uint16_t g_work_len   = PRESET_POINT_NUM;  /* 有效点数(到结束标志为止) */

/* 找曲线结束标志: 返回第一个 PRESET_END_MARK 的下标, 没找到则返回 PRESET_POINT_NUM.
 * 标志本身占一个位置, 所以有效点数最多 PRESET_POINT_NUM-1. */
static uint16_t Preset_FindEnd(const uint8_t *p)
{
    uint16_t i;
    for (i = 0; i < PRESET_POINT_NUM; i++) {
        if (p[i] == (uint8_t)PRESET_END_MARK) {
            break;
        }
    }
    return i;
}

/* 异步"全部恢复出厂"状态 */
static uint8_t  g_restore_busy = 0u;         /* 1=正在恢复 */
static uint8_t  g_restore_slot = 0u;         /* 下一个要写的槽 */
static uint8_t  g_restore_keep = 0u;         /* 恢复前当前选中的槽(结束时重新载入) */
static uint32_t g_restore_tick = 0u;         /* 上次写页时刻 */

/* 链接器 scatter-loading 符号: 执行区 ER_IROM1 的末尾 (代码+RO 的下一个可用地址).
 * RW 初始化镜像紧随其后 (当前 ~350B), 故检查时留 1KB 余量.
 * 预设区必须完全在它之上, 否则擦预设页会把代码擦掉. */
extern uint32_t Image$$ER_IROM1$$Limit;

#define PRESET_AREA_MARGIN   1024u   /* ER_IROM1 末尾到预设区起始的最小余量 */

/* 档位字节累加和 (与 settings.c 的 u16 字累加和同一思路) */
static uint16_t Preset_CalcCrc(const uint8_t *p, uint16_t n)
{
    uint16_t i;
    uint16_t s = 0;

    for (i = 0; i < n; i++) {
        s = (uint16_t)(s + p[i]);
    }
    return s;
}

static uint32_t Preset_SlotAddr(uint8_t slot)
{
    return PRESET_FLASH_BASE + (uint32_t)slot * PRESET_SLOT_SIZE;
}

uint8_t Preset_SlotValid(uint8_t slot)
{
    const Preset_Hdr_t *h;

    if (slot >= PRESET_SLOT_NUM) return 0u;

    h = (const Preset_Hdr_t *)Preset_SlotAddr(slot);
    if (h->magic   != PRESET_MAGIC)    return 0u;   /* 空片(0xFF)或无关数据 */
    if (h->version != PRESET_VERSION)  return 0u;
    if (h->num     != PRESET_POINT_NUM) return 0u;
    if (h->crc     != Preset_CalcCrc((const uint8_t *)(h + 1), PRESET_POINT_NUM)) return 0u;

    return 1u;
}

/* 缓存的有效性 (开机自检/写盘时更新), 不再读Flash. */
uint8_t Preset_SlotOk(uint8_t slot)
{
    if (slot >= PRESET_SLOT_NUM) return 0u;
    return g_slot_ok[slot];
}

uint8_t Preset_Init(void)
{
    uint8_t i;
    uint8_t valid = 0;
    uint8_t restored = 0;
    uint32_t img_end = (uint32_t)&Image$$ER_IROM1$$Limit;

    /* 安全检查: 代码镜像必须整个在预设区之下 (含 1KB 余量给 RW 初始化镜像).
     * 工程 IROM1 必须从 0xFC00 缩到 0xB000, 否则链接器可能把代码放到
     * 0x0800B000 之后, 一旦保存预设就会擦掉代码 -> 整机变砖. */
    if ((img_end + PRESET_AREA_MARGIN) > PRESET_FLASH_BASE) {
        g_area_ok = 0u;
        printf("[Preset] FATAL: image ends 0x%08X, overlaps preset area 0x%08X!\r\n",
               (unsigned)img_end, (unsigned)PRESET_FLASH_BASE);
        printf("[Preset] IROM1 MUST be shrunk to 0xB000, saving is disabled!\r\n");
    } else {
        g_area_ok = 1u;
        printf("[Preset] image ends 0x%08X < preset base 0x%08X, area OK\r\n",
               (unsigned)img_end, (unsigned)PRESET_FLASH_BASE);
    }

    for (i = 0; i < PRESET_SLOT_NUM; i++) {
        if (Preset_SlotValid(i)) {
            g_slot_ok[i] = 1u;
            valid++;
        } else {
            /* 该槽缺失(首次上电 / 掉电写坏 / 换过固件): 把固化在固件里的
             * 出厂曲线写进去. 开机时机(屏还没起来, 泵/加热未启动)最安全. */
            g_slot_ok[i] = 0u;
            if (Preset_RestoreFactory(i) == 0u) {
                restored++;
            }
        }
    }
    printf("[Preset] init: %u/%u slots valid, %u restored from factory\r\n",
           (unsigned)valid, (unsigned)PRESET_SLOT_NUM, (unsigned)restored);
    return 0u;
}

void Preset_Factory(uint8_t slot)
{
    uint16_t i;

    if (slot >= PRESET_SLOT_NUM) return;

    for (i = 0; i < PRESET_POINT_NUM; i++) {
        g_work[i] = g_preset_defaults[slot][i];
    }
    g_work_len   = Preset_FindEnd(g_work);
    g_cur_slot   = slot;
    g_from_flash = 0u;
}

uint8_t Preset_Load(uint8_t slot)
{
    if (slot >= PRESET_SLOT_NUM) return 1u;

    if (Preset_SlotValid(slot)) {
        memcpy(g_work,
               (const void *)(Preset_SlotAddr(slot) + PRESET_HDR_SIZE),
               PRESET_POINT_NUM);
        g_work_len       = Preset_FindEnd(g_work);
        g_slot_ok[slot] = 1u;
        g_from_flash    = 1u;
    } else {
        /* 该槽无有效数据 (首次上电 / 掉电写坏) -> 用出厂曲线兜底, 不写Flash */
        Preset_Factory(slot);
        g_slot_ok[slot] = 0u;
    }
    g_cur_slot = slot;
    return 0u;
}

uint8_t Preset_Save(uint8_t slot)
{
    Preset_Hdr_t hdr;
    const uint16_t *hp;
    uint8_t rc_erase;
    uint8_t rc_prog;
    uint32_t addr;
    uint16_t i;

    if (slot >= PRESET_SLOT_NUM) return 1u;

    /* 预设区与代码区重叠时禁止写入 (擦页会毁掉代码) */
    if (!g_area_ok) {
        printf("[Preset] save slot%u REFUSED: preset area overlaps code\r\n",
               (unsigned)slot);
        return 1u;
    }

    hdr.magic   = PRESET_MAGIC;
    hdr.version = PRESET_VERSION;
    hdr.num     = PRESET_POINT_NUM;
    hdr.crc     = Preset_CalcCrc(g_work, PRESET_POINT_NUM);
    hp          = (const uint16_t *)&hdr;
    addr        = Preset_SlotAddr(slot);

    /* ===== 第 1 段: 擦整页 (2KB), 一组一页, 不影响其它组 =====
     * 关中断 ~20~40ms (物理下限, 省不掉). */
    FLASH_OP_CRITICAL_BEGIN();
    FLASH_Unlock();
    rc_erase = Flash_ErasePage(addr);
    FLASH_Lock();
    FLASH_OP_CRITICAL_END();

    /* 擦除后落定: 让 SysTick 跑起来(泵/加热时序不冻结), 同时给 F0 Flash
     * 控制器真正的恢复时间. 擦除和编程之间隔开, 是"连着写两次就死"的关键. */
    delay_ms(PRESET_ERASE_SETTLE_MS);

    /* ===== 第 2 段: 编程, 关中断 ~10ms =====
     * 头部 4 个半字 + 400 个档位字节 (一次连续写, PG 位全程置位). */
    FLASH_OP_CRITICAL_BEGIN();
    FLASH_Unlock();
    rc_prog = rc_erase;
    for (i = 0; i < (PRESET_HDR_SIZE / 2u) && rc_prog == 0u; i++) {
        rc_prog = Flash_ProgramHalfWord(addr + i * 2u, hp[i]);
    }
    if (rc_prog == 0u) {
        rc_prog = Flash_ProgramBlock(addr + PRESET_HDR_SIZE,
                                     g_work, PRESET_POINT_NUM);
    }
    FLASH_Lock();
    FLASH_OP_CRITICAL_END();

    /* ===== 以下全部在临界区外 =====
     * printf 会把一大段库代码拉进临界区 (settings.c 里明确写了
     * "临界区内不调 printf"), 一律挪到外面. */
    if (rc_prog != 0u) {
        printf("[Preset] save slot%u FAILED: rc=%u\r\n",
               (unsigned)slot, (unsigned)rc_prog);
        return 1u;
    }

    /* 回读校验: 不一致视为失败 (不更新有效标志) */
    if (memcmp((const void *)(addr + PRESET_HDR_SIZE),
               g_work, PRESET_POINT_NUM) != 0) {
        printf("[Preset] save slot%u FAILED: verify mismatch\r\n",
               (unsigned)slot);
        return 1u;
    }

    g_slot_ok[slot] = 1u;
    g_from_flash    = 1u;
    g_cur_slot      = slot;
    printf("[Preset] save slot%u ok\r\n", (unsigned)slot);
    return 0u;
}

/* 某一组恢复出厂曲线并立即落盘.
 * 不改变"当前选中槽"(由调用方负责重新载入显示). */
uint8_t Preset_RestoreFactory(uint8_t slot)
{
    uint8_t keep = g_cur_slot;
    uint8_t rc;

    if (slot >= PRESET_SLOT_NUM) return 1u;

    Preset_Factory(slot);        /* 生成出厂曲线到 RAM 工作区 */
    rc = Preset_Save(slot);      /* 落盘 (含 g_area_ok 保护) */
    g_cur_slot = keep;           /* 恢复当前选中槽 */
    return rc;
}

/* 启动"9 组全部恢复出厂"(异步).
 * 拆成每 PRESET_RESTORE_GAP_MS 写一页, 避免连续 9 页把主循环卡住 360ms. */
uint8_t Preset_RestoreAllStart(void)
{
    if (!g_area_ok) {
        printf("[Preset] restore-all REFUSED: preset area overlaps code\r\n");
        return 1u;
    }
    g_restore_slot = 0u;
    g_restore_keep = g_cur_slot;
    g_restore_busy = 1u;
    g_restore_tick = 0u;         /* 令首次 task 立即执行 */
    printf("[Preset] restore-all started\r\n");
    return 0u;
}

/* 主循环调用: 每次最多写一页, 返回 1 = 9 组全部恢复完毕 */
uint8_t Preset_RestoreAllTask(uint32_t now)
{
    if (!g_restore_busy) return 1u;

    if ((uint32_t)(now - g_restore_tick) < PRESET_RESTORE_GAP_MS) return 0u;
    g_restore_tick = now;

    (void)Preset_RestoreFactory(g_restore_slot);
    printf("[Preset] restore slot %u done\r\n", (unsigned)g_restore_slot);
    g_restore_slot++;

    if (g_restore_slot >= PRESET_SLOT_NUM) {
        g_restore_busy = 0u;
        /* 恢复过程中工作区被最后一槽的出厂曲线占着, 重新载入当前选中槽 */
        (void)Preset_Load(g_restore_keep);
        printf("[Preset] restore-all done (9 slots)\r\n");
        return 1u;
    }
    return 0u;
}

uint8_t Preset_RestoreAllBusy(void)
{
    return g_restore_busy;
}

uint8_t *Preset_Data(void)
{
    return g_work;
}

uint8_t Preset_CurrentSlot(void)
{
    return g_cur_slot;
}

void Preset_SetCurrentSlot(uint8_t slot)
{
    if (slot < PRESET_SLOT_NUM) {
        g_cur_slot = slot;
    }
}

uint8_t Preset_LoadedFromFlash(void)
{
    return g_from_flash;
}

uint8_t Preset_PointGear(uint16_t idx)
{
    if (idx >= PRESET_POINT_NUM) return 0u;
    return g_work[idx];
}

/* 当前工作区的有效点数 (结束标志的下标; 无标志则=PRESET_POINT_NUM) */
uint16_t Preset_Length(void)
{
    return g_work_len;
}

/* 把工作区曲线的结束标志设到第 len 点 (不写Flash), 有效点数按实际结束标志重算.
 * 变压萃取录到的实际曲线和预设长度不同时, 保存前用它把结束标志移到实际曲线末尾.
 * 加长时调用方须已写好 [原有效点数, len) 的数据(覆盖掉原结束标志), 否则长度不变. */
void Preset_SetLength(uint16_t len)
{
    if (len >= PRESET_POINT_NUM) {
        return;   /* 放不下结束标志 */
    }
    g_work[len] = (uint8_t)PRESET_END_MARK;
    g_work_len  = Preset_FindEnd(g_work);
}

float Preset_PointHz(uint16_t idx)
{
    if (idx >= PRESET_POINT_NUM) return 0.0f;
    return (float)g_work[idx] * (float)PRESET_GEAR_HZ_X100 / 100.0f;
}
