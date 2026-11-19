/**
 * @file    boiler.h
 * @brief   锅炉加热+出水阀门控制模块 - STM32F0 SPL
 *
 * 功能:
 *   - 上电待机(不加热), 触发后PID连续调节加热功率(PA11 PWM), 目标水温88℃
 *   - 萃取出水阀门(PB2, 高=开): 水温进入[目标±1℃]开阀出水, 超出关阀
 *   - 热水出水阀门(PB11, 高=开): 与萃取阀互锁, 同一时刻仅一个为高
 *   - 开阀出水/水泵加水时, PID持续补偿冷水涌入, 保持水温±1℃
 *   - 水位保护(PC13, 低=水箱有水): 缺水时禁止泵运行(防空转), 不影响加热
 */

#ifndef __BOILER_H
#define __BOILER_H

#include "sys.h"
#include <stdint.h>

/* 出水阀门引脚 (PB2, 高=开) */
#define VALVE_PORT              GPIOB
#define VALVE_PIN               GPIO_Pin_2

/* 热水出水阀门引脚 (PB11, 高=开), 与萃取阀互锁: 同一时刻仅一个为高 */
#define HOT_VALVE_PORT          GPIOB
#define HOT_VALVE_PIN           GPIO_Pin_11

/* 目标水温与控制容差 */
#define BOILER_TARGET_TEMP      88.0f    /* 目标水温(℃) */
#define BOILER_TEMP_TOL         1.0f     /* 温度容差 ±℃ (进入此区间开阀) */
#define BOILER_PREHEAT_TEMP     60.0f    /* 预热阈值: 低于此全功率, 达到后转PID限功率段.
                                               * 目标90℃时对应误差33%, 属于 cap=100% 区间, 行为一致 */

/*==============================================================
 * 加热 PID 参数 — 双 Profile (待机控温 / 出饮控温)
 *
 * 职责划分: 锅炉只管锅炉水温, 全程单套参数 (待机/萃取/热水一致).
 *           萃取出水口温度由 brew_heater.c (PB10 加热片) 独立闭环控温.
 *           实测 (目标90℃) 功率 100%→10% 升温速率几乎不变 (0.46~0.52℃/s),
 *           说明热量主要来自加热棒蓄热, 必须早期大幅限功率.
 *
 * 原双 Profile 说明 (保留参考):
 *   待机: 无冷水进入, 加热器只需维持温度 → 关键是**防冲温**
 *         (实测 100%→10% 功率时升温速率几乎不变, 热量来自加热棒蓄热)
 *         → 增益保守 + cap 严格递减 + 冲温预测前馈开启
 *   出饮: 冷水持续冲入锅炉, 温度被持续拉低 → 关键是**快速补偿**
 *         → 增益高 (尤其 Ki) + cap 宽松 + 冲温前馈关闭 (冷水会带走热量)
 *
 * 所有分段边界用**温差百分比** (相对 g_target_temp), 不用绝对温度,
 * 因为目标温度可调 (85~99℃). 例: 目标90℃时 25% = 22.5℃; 目标85℃时 25% = 21.3℃.
 *==============================================================*/

/* PID 工作模式已移除: 锅炉全程单套参数 */

/* 增益调度分界 (温差占目标温度的百分比) */
#define BOILER_PID_BAND_FAR_PCT     16.7f   /* 远/中段: 目标90℃时≈15℃ */
#define BOILER_PID_BAND_NEAR_PCT    3.3f    /* 中/近段: 目标90℃时≈3℃ */

/* 积分死区 (温差占目标温度的百分比): 目标90℃时≈1.5℃ */
#define BOILER_PID_DEADBAND_PCT     1.7f

/* 最低加热功率 (%): 未到目标温度时保底, 防止 PID 输出 0% 导致温度卡在目标下方 */
#define BOILER_MIN_POWER          1.0f

/* 分段输出上限: 误差越大上限越低 (提前减速, 防加热棒蓄热过多) */
#define BOILER_CAP_T1_PCT           25.0f   /* > 25% → 100% */
#define BOILER_CAP_T2_PCT           15.0f   /* > 15% →  50%  */
#define BOILER_CAP_T3_PCT            8.0f   /* >  8% →  28%  */
#define BOILER_CAP_T4_PCT            5.0f   /* >  5% →  18%  */
#define BOILER_CAP_1                 100.0f
#define BOILER_CAP_2                 50.0f
#define BOILER_CAP_3                 28.0f
#define BOILER_CAP_4                 18.0f
                                                 /* <=  5% → PID 自由精调 */

/* 段1: 远温差, 大误差, 快速升温 */
#define BOILER_PID_KP_FAR       12.0f
#define BOILER_PID_KI_FAR       0.60f
#define BOILER_PID_KD_FAR       2.0f

/* 段2: 中温差, 匀速接近 */
#define BOILER_PID_KP_MID       8.0f
#define BOILER_PID_KI_MID       0.35f
#define BOILER_PID_KD_MID       3.0f

/* 段3: 近温差, 抑制过冲精调 */
#define BOILER_PID_KP_NEAR      3.0f
#define BOILER_PID_KI_NEAR      0.12f
#define BOILER_PID_KD_NEAR      8.0f

/* 超温硬保护带 (温差占目标温度的百分比): 目标90℃时≈1.5℃.
 * 两套 Profile 共用 — 出饮时若意外超温也必须硬关断. */
#define BOILER_OVERHEAT_BAND_PCT    1.7f

/* 冲温预测前馈参数 (全程启用) */
#define BOILER_FF_LOOKAHEAD_S     6.0f    /* 前瞻时间(秒) */
#define BOILER_FF_MARGIN_PCT      1.0f    /* 安全边界(占目标%) */
#define BOILER_FF_CAP             8.0f    /* 预测会冲时的输出上限(%) */
#define BOILER_FF_MIN_RATE        0.15f   /* 最小有效升温速率(℃/s), 低于此不触发 */

/* 降温冲洗参数 */
#define BOILER_COOL_TEMP        70.0f    /* 降温目标(℃), 低于此结束冲洗转加热 */
#define BOILER_COOL_PUMP_FREQ   200      /* 降温时水泵抽冷水频率 (0.25Hz单位, 200=50Hz) */
#define BOILER_HOLD_PUMP_FREQ   200      /* 保温出水时水泵补水频率 (0.25Hz单位, 200=50Hz) */
#define BOILER_HOLD_DURATION_MS 120000u  /* 保温出水持续时间(ms), 到时停机 */

/* 出水阶段水泵频率曲线(相对进入HOLDING的时间):
 *   0-6s:   10Hz→30Hz 线性上升
 *   6-12s:  30Hz 保持
 *   12-18s: 30Hz→50Hz 线性上升
 *   18-33s: 50Hz 保持
 *   33s后:  停泵 */
#define PUMP_CURVE_T1_MS        6000u    /* 第一段上升结束 */
#define PUMP_CURVE_T2_MS        12000u   /* 30Hz保持结束 */
#define PUMP_CURVE_T3_MS        18000u   /* 第二段上升结束 */
#define PUMP_CURVE_T4_MS        33000u   /* 50Hz保持结束, 之后停泵 */
#define PUMP_CURVE_F_LOW        40       /* 起始频率 (0.25Hz单位, 40=10Hz) */
#define PUMP_CURVE_F_MID        120      /* 中间频率 (0.25Hz单位, 120=30Hz) */
#define PUMP_CURVE_F_HIGH       200      /* 最高频率 (0.25Hz单位, 200=50Hz) */

/* 锅炉工作状态 */
typedef enum {
    BOILER_IDLE = 0,    /* 待机: 加热关/泵关/阀关 */
    BOILER_COOLING,     /* 降温: 加热关/泵抽冷水/阀开, 到<40℃转加热 */
    BOILER_HEATING,     /* 加热: PID加热/泵关/阀关, 到目标温转保温 */
    BOILER_HOLDING,     /* 保温出水: PID加热/阀开/泵补水, 维持±1℃ */
    BOILER_HEAT_HOLD    /* 纯加热保持: PID加热到目标并维持, 不开阀不出水(出饮任务控制出水) */
} Boiler_State_t;

void    Boiler_Init(void);
void    Boiler_Update(float cur_temp, uint32_t now);   /* 主循环调用 */
void    Boiler_SetTarget(float temp);
float   Boiler_GetTarget(void);
uint8_t Boiler_IsValveOpen(void);
uint8_t Boiler_GetHeaterPower(void);
void    Boiler_ValveOpen(void);       /* 开萃取阀 (互锁: 自动关热水阀) */
void    Boiler_ValveClose(void);
void    Boiler_HotValveOpen(void);    /* 开热水阀 (互锁: 自动关萃取阀) */
void    Boiler_HotValveClose(void);
uint8_t Boiler_HotValveIsOpen(void);  /* 1=热水阀打开 */
void    Boiler_AllValvesClose(void);  /* 同时关闭两个阀门 */

/* 启动完整流程: 温度>40先降温→加热到目标→开阀出水+补水保温.
 * 由增量调节控件点击触发. 温度判断在Update内根据实时温度进行. */
void    Boiler_StartCycle(void);
Boiler_State_t Boiler_GetState(void);

/* 纯加热: 进入HEAT_HOLD态, PID加热到目标温度并维持, 不开阀不出水.
 * 出水由外部(出饮任务)控制. 配合Boiler_SetExternalControl使用. */
void    Boiler_HeatStart(void);
void    Boiler_HeatStop(void);       /* 取消纯加热, 回IDLE */

/* 外部接管: 置1后Boiler_Update不干预任何输出(泵/阀/加热),
 * 由外部任务直接控制. 置0恢复正常状态机. */
void    Boiler_SetExternalControl(uint8_t en);

#endif
