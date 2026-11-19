#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
生成 9 组出厂预设水泵曲线 (档位制) -> HARDWARE/src/preset_defaults.c

档位定义: 0=停泵, 1~40, 每档 1.25Hz  (40档 = 50Hz)
曲线点数: 396 个有效点 (每点 250ms -> 99 秒, 与 MCU 实际曲线最长 396 点一致),
          下标 396 (第 397 个位置) 放结束标志 200, 其余填 0xFF

结束标志: 档位值 200 (正常档位只有 0~40, 不可能出现), 表示曲线数据到此结束.
        MCU 扫描到第一个 200 的位置就是本条曲线的有效点数.
        数组容量 PRESET_POINT_NUM=400, 所以有效点最多 399 个.

曲线形状: 预浸泡段(定值) -> 线性爬升段 -> 满频保持段
9 组按"预浸泡档位"和"爬升快慢"区分, 供现场在屏端微调后另存.

改曲线只需改本文件下面的 PROFILES 表, 然后 python tools/gen_preset_defaults.py
"""
import os

POINT_NUM = 400
DATA_NUM  = 396     # 有效数据点数 (99 秒, 对应 main.c 的 CURVE_REC_MAX_POINTS)
END_MARK  = 200     # 曲线结束标志 (档位值, 正常 0~40 不会出现)
SLOT_NUM  = 9
GEAR_MAX  = 40

# (预浸泡档位, 爬升段点数)  -- 峰值统一 40 档 (50Hz)
PROFILES = [
    ( 8, 360),
    (10, 320),
    (12, 280),
    (14, 240),
    (16, 200),
    (18, 160),
    (20, 120),
    (22,  80),
    (24,  40),
]

PRE_POINTS = 40          # 预浸泡段点数 (40 点 = 10 秒)


def gen_curve(pre_gear, ramp_points):
    """生成一条曲线: 预浸泡定值 -> 线性爬升 -> 峰值保持 (DATA_NUM 个有效点)"""
    c = [0] * DATA_NUM
    peak = GEAR_MAX
    for i in range(DATA_NUM):
        if i < PRE_POINTS:
            c[i] = pre_gear
        elif i < PRE_POINTS + ramp_points:
            # 线性插值 pre -> peak
            span = ramp_points
            idx = i - PRE_POINTS
            c[i] = pre_gear + (peak - pre_gear) * idx // span
        else:
            c[i] = peak
    return c


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, '..', 'HARDWARE', 'src', 'preset_defaults.c')
    out = os.path.normpath(out)

    lines = []
    lines.append('/**')
    lines.append(' * @file    preset_defaults.c')
    lines.append(' * @brief   9 组出厂预设水泵曲线 (档位制, 固化在固件里)')
    lines.append(' *')
    lines.append(' * 档位定义: 0=停泵, 1~40, 每档 1.25Hz (40档 = 50Hz)')
    lines.append(' * 有效点数: %d 点 x 250ms = %g 秒, 之后是结束标志 %d, 其余填 0xFF'
                 % (DATA_NUM, DATA_NUM * 0.25, END_MARK))
    lines.append(' *')
    lines.append(' * 本表由 tools/gen_preset_defaults.py 生成, 直接改脚本里的 PROFILES 再重跑即可.')
    lines.append(' * 也可直接在本文件里手改数值 (uint8 数组, 每行 20 个点).')
    lines.append(' */')
    lines.append('')
    lines.append('#include "preset_store.h"')
    lines.append('')
    lines.append('/* 出厂预设曲线: [槽号][点号] = 档位 (0~40) */')
    lines.append('const uint8_t g_preset_defaults[PRESET_SLOT_NUM][PRESET_POINT_NUM] = {')

    for slot, (pre, ramp) in enumerate(PROFILES):
        curve = gen_curve(pre, ramp)
        lines.append('    { /* 槽 %d: 预浸泡 %d 档, 爬升 %d 点, 峰值 %d 档 */'
                     % (slot, pre, ramp, GEAR_MAX))
        for row in range(0, DATA_NUM, 20):
            chunk = curve[row:row + 20]
            lines.append('        ' + ' '.join('%3d,' % v for v in chunk))
        # 补齐到数组末尾: 结束标志 + 0xFF 填充
        tail = [END_MARK] + [0xFF] * (POINT_NUM - DATA_NUM - 1)
        lines.append('        ' + ' '.join('%3d,' % v for v in tail))
        lines.append('    },')
    lines.append('};')
    lines.append('')

    with open(out, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines))

    print('written: %s (%d bytes)' % (out, os.path.getsize(out)))


if __name__ == '__main__':
    main()
