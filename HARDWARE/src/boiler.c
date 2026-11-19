/**
 * @file    boiler.c
 * @brief   锅炉加热+出水阀门控制实现 - STM32F0 SPL
 *
 * 工作原理:
 *   1. 上电Boiler_Init后进入IDLE待机(不加热), 由Boiler_StartCycle/
 *      Boiler_HeatStart触发才开始加热, 目标88℃
 *   2. 主循环每次调用Boiler_Update(温度): 按状态机PID计算 → HEATER_SetPower(0-100%)
 *   3. HEATING/HOLDING: |温度-目标|<=1℃ 开阀(PB2高)出水
 *   4. 开阀出水时冷水涌入使温度下降 → PID误差变正 → 提高加热功率补偿
 */

#include "boiler.h"
#include "pid_controller.h"
#include "heater_control.h"
#include "ac_pump.h"
#include "water_level.h"

static PID_Controller_t g_boiler_pid;
static float   g_target_temp = BOILER_TARGET_TEMP;
static uint8_t g_valve_open = 0;
static uint8_t g_hot_valve_open = 0;
static uint8_t g_heater_power = 0;
static Boiler_State_t g_boiler_state = BOILER_IDLE;
static uint32_t g_hold_start = 0;     /* 保温开始时刻(ms) */
static uint8_t g_external_ctrl = 0;   /* 外部接管标志: Update直接返回 */

void Boiler_ValveOpen(void)
{
    /* 互锁: 开萃取阀前强制关热水阀 */
    GPIO_ResetBits(HOT_VALVE_PORT, HOT_VALVE_PIN);
    g_hot_valve_open = 0;
    GPIO_SetBits(VALVE_PORT, VALVE_PIN);   /* 高=开 */
    g_valve_open = 1;
}

void Boiler_ValveClose(void)
{
    GPIO_ResetBits(VALVE_PORT, VALVE_PIN); /* 低=关 */
    g_valve_open = 0;
}

void Boiler_HotValveOpen(void)
{
    /* 互锁: 开热水阀前强制关萃取阀 */
    GPIO_ResetBits(VALVE_PORT, VALVE_PIN);
    g_valve_open = 0;
    GPIO_SetBits(HOT_VALVE_PORT, HOT_VALVE_PIN);   /* 高=开 */
    g_hot_valve_open = 1;
}

void Boiler_HotValveClose(void)
{
    GPIO_ResetBits(HOT_VALVE_PORT, HOT_VALVE_PIN); /* 低=关 */
    g_hot_valve_open = 0;
}

uint8_t Boiler_HotValveIsOpen(void) { return g_hot_valve_open; }

void Boiler_AllValvesClose(void)
{
    GPIO_ResetBits(VALVE_PORT, VALVE_PIN);          /* 萃取阀关 */
    GPIO_ResetBits(HOT_VALVE_PORT, HOT_VALVE_PIN);  /* 热水阀关 */
    g_valve_open = 0;
    g_hot_valve_open = 0;
}

void Boiler_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* PB2萃取阀 + PB11热水阀输出, 默认关(低), 两阀互锁 */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin = VALVE_PIN | HOT_VALVE_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(VALVE_PORT, &GPIO_InitStructure);
    Boiler_ValveClose();
    Boiler_HotValveClose();

    /* 加热PID 初始用中段增益 (每次 Boiler_PIDStep 会按误差重新选段) */
    PID_Init(&g_boiler_pid, BOILER_PID_KP_MID, BOILER_PID_KI_MID, BOILER_PID_KD_MID);
    PID_SetOutputLimits(&g_boiler_pid, 0.0f, 100.0f);
    /* 积分限幅必须 ≥ (目标维持功率 / Ki).
     * 88℃稳态需 ~21% 功率: I ≥ 21/0.2 = 105. 设 150 留余量. */
    PID_SetIntegralLimit(&g_boiler_pid, 150.0f);
    PID_SetSetpoint(&g_boiler_pid, g_target_temp);
    PID_Reset(&g_boiler_pid);

    /* 待机: 加热关/泵关/阀关, 等待触发 */
    HEATER_Off();
    g_heater_power = 0;
    g_boiler_state = BOILER_IDLE;
    g_external_ctrl = 0;
}

/* 触发完整流程: 统一先进降温态, 若温度已<40℃, Update会立即转加热 */
void Boiler_StartCycle(void)
{
    PID_Reset(&g_boiler_pid);
    g_boiler_state = BOILER_COOLING;
}

/* 纯加热: 进入HEAT_HOLD, PID加热到目标并维持, 阀/泵交给外部控制 */
void Boiler_HeatStart(void)
{
    Boiler_AllValvesClose();
    AC_Pump_Off();
    PID_Reset(&g_boiler_pid);
    PID_SetAuto(&g_boiler_pid);   /* BugFix: 必须Enable PID, 否则HEAT_HOLD≥70°C时PID_Calculate返回manual_output=0 */
    g_boiler_state = BOILER_HEAT_HOLD;
}

/* 取消纯加热, 回IDLE */
void Boiler_HeatStop(void)
{
    if (g_boiler_state == BOILER_HEAT_HOLD) {
        g_boiler_state = BOILER_IDLE;
    }
}

void Boiler_SetExternalControl(uint8_t en)
{
    g_external_ctrl = en ? 1 : 0;
}

Boiler_State_t Boiler_GetState(void) { return g_boiler_state; }

/*==============================================================
 * PID 增益调度 + 分段输出上限 (按当前 Profile)
 *
 * 所有分段边界用**温差百分比** (相对 g_target_temp), 不用绝对温度,
 * 因为目标温度可调 (85~99℃). 例: 目标90℃时 25% = 22.5℃; 目标85℃时 25% = 21.3℃.
 *
 * 超温保护: 超过目标 + OVERHEAT_BAND_PCT 强制输出 0 并清积分 (两套 Profile 共用).
 *==============================================================*/
static float Boiler_PIDStep(float cur_temp, uint32_t now)
{
    float err = g_target_temp - cur_temp;
    float aerr = (err < 0.0f) ? -err : err;
    float kp, ki, kd, cap, out;
    /* 目标温度的 1% 对应的温差 (℃), 目标 0 时兜底为 1 防止除零 */
    float pct = (g_target_temp > 1.0f) ? (g_target_temp * 0.01f) : 1.0f;

    /* 超温硬保护: 超过目标 1.7% (目标90℃时≈1.5℃) 强制关断 */
    if (err < -BOILER_OVERHEAT_BAND_PCT * pct) {
        PID_Reset(&g_boiler_pid);
        return 0.0f;
    }

    /* 增益调度: 按温差百分比分段 */
    if (aerr > BOILER_PID_BAND_FAR_PCT * pct) {
        kp = BOILER_PID_KP_FAR;  ki = BOILER_PID_KI_FAR;  kd = BOILER_PID_KD_FAR;
    } else if (aerr > BOILER_PID_BAND_NEAR_PCT * pct) {
        kp = BOILER_PID_KP_MID;  ki = BOILER_PID_KI_MID;  kd = BOILER_PID_KD_MID;
    } else {
        kp = BOILER_PID_KP_NEAR; ki = BOILER_PID_KI_NEAR; kd = BOILER_PID_KD_NEAR;
    }

    /* 分段输出上限: 误差越小 → 上限越低 (提前减速, 防加热棒蓄热过多) */
    if      (aerr > BOILER_CAP_T1_PCT * pct) cap = BOILER_CAP_1;
    else if (aerr > BOILER_CAP_T2_PCT * pct) cap = BOILER_CAP_2;
    else if (aerr > BOILER_CAP_T3_PCT * pct) cap = BOILER_CAP_3;
    else if (aerr > BOILER_CAP_T4_PCT * pct) cap = BOILER_CAP_4;
    else                                      cap = 100.0f;   /* 误差 <=5%: PID 自由精调 */

    /* 冲温预测前馈: 升温速率线性外推, 预测会冲就提前收功率 */
    {
        float rate = g_boiler_pid.deriv_filter;           /* ℃/s, 上升为正 */
        float predicted = cur_temp + rate * BOILER_FF_LOOKAHEAD_S;
        float safe_limit = g_target_temp * (1.0f - BOILER_FF_MARGIN_PCT * 0.01f);
        if (rate > BOILER_FF_MIN_RATE && predicted > safe_limit && cap > BOILER_FF_CAP) {
            cap = BOILER_FF_CAP;
        }
    }

    /* 积分死区: |误差| < DEADBAND 时冻结积分, 防止过冲后积分滞后.
     * 已越过目标(err<0)时允许积分, 让积分快速回退清零. */
    if (aerr > BOILER_PID_DEADBAND_PCT * pct || err < 0.0f) {
        PID_SetIntegralLimit(&g_boiler_pid, PID_INTEGRAL_LIMIT);
    } else {
        PID_SetIntegralLimit(&g_boiler_pid, 0.0f);
    }

    PID_SetTuning(&g_boiler_pid, kp, ki, kd);

    /* 用输出上限替代默认 0~100 限幅, 让 PID 内部抗饱和逻辑感知到 cap */
    PID_SetOutputLimits(&g_boiler_pid, 0.0f, cap);

    out = PID_Calculate(&g_boiler_pid, g_target_temp, cur_temp, now);

    /* 最低功率保护: 还没到目标温度时至少给 BOILER_MIN_POWER 功率.
     * 否则 PID 可能接近目标时输出 0%, 锅炉靠余热冲过目标后
     * 散热降温, 又因为输出 0 无法回升 → 温度在目标下方卡死/振荡. */
    if (err > 0.0f && out < BOILER_MIN_POWER) {
        out = BOILER_MIN_POWER;
    }

    return out;
}

void Boiler_Update(float cur_temp, uint32_t now)
{
    float out;
    float diff;

    /* 外部接管期间不干预任何输出 */
    if (g_external_ctrl) {
        return;
    }

    /* 温度无效: 安全停机 */
    if (cur_temp < -100.0f) {
        HEATER_Off();
        g_heater_power = 0;
        AC_Pump_Off();
        Boiler_ValveClose();
        return;
    }
    switch (g_boiler_state) {
    case BOILER_IDLE:
        /* 待机: 全部关闭 */
        HEATER_Off();
        g_heater_power = 0;
        AC_Pump_Off();
        Boiler_ValveClose();
        break;

    case BOILER_COOLING:
        /* 降温: 加热关, 泵抽冷水, 阀开出水, 直到<40℃ */
        if (cur_temp < BOILER_COOL_TEMP) {
            /* 降温完成 → 转加热 */
            AC_Pump_Off();
            Boiler_ValveClose();
            PID_Reset(&g_boiler_pid);
            PID_SetAuto(&g_boiler_pid);
            g_boiler_state = BOILER_HEATING;
        } else {
            HEATER_Off();
            g_heater_power = 0;
            if (WATER_IsPresent()) {
                /* 水箱有水才允许抽水 */
                if (!AC_Pump_IsOn()) {
                    AC_Pump_SetFrequency(BOILER_COOL_PUMP_FREQ);
                    AC_Pump_On();
                }
            } else {
                /* 水箱缺水: 停泵防空转, 保持等待 */
                if (AC_Pump_IsOn()) AC_Pump_Off();
            }
            if (!g_valve_open) Boiler_ValveOpen();
        }
        break;

    case BOILER_HEATING:
        /* 加热: 泵不抽水, 阀关. 到达目标温转保温 */
        AC_Pump_Off();
        if (g_valve_open) Boiler_ValveClose();

        if (cur_temp < BOILER_PREHEAT_TEMP) {
            /* 预热段: 低于70℃全功率快速升温.
             * 注: 此处不再每周期 PID_Reset, 因为 Reset 会清掉 last_time,
             *      导致下一帧跨入 PID 段时 dt 爆炸, 触发 derivative 尖峰.
             *      PID_Reset 仅在状态进入 BOILER_HEATING 时执行一次
             *      (见 BOILER_COOLING → BOILER_HEATING 转换处). */
            g_heater_power = 100;
            HEATER_SetPower(100);
        } else {
            /* 70℃后转PID: 增益调度, 随接近目标误差减小, 功率递减, 平稳到位不超调 */
            out = Boiler_PIDStep(cur_temp, now);
            g_heater_power = (uint8_t)out;
            HEATER_SetPower(g_heater_power);
        }

        /* 进入目标±容差 → 转保温出水 */
        diff = cur_temp - g_target_temp;
        if (diff < 0.0f) diff = -diff;
        if (diff <= BOILER_TEMP_TOL) {
            Boiler_ValveOpen();
            g_hold_start = now;
            AC_Pump_SetFrequency(PUMP_CURVE_F_LOW);   /* 出水曲线起点10Hz */
            AC_Pump_On();
            g_boiler_state = BOILER_HOLDING;
        }
        break;

    case BOILER_HOLDING:
        /* 保温出水: PID持续加热补偿, 阀开出水, 泵补水. 维持±1℃ */
        /* 保温满2分钟 → 停机(关加热/泵/阀), 回IDLE等待下次触发 */
        if (now - g_hold_start >= BOILER_HOLD_DURATION_MS) {
            HEATER_Off();
            g_heater_power = 0;
            AC_Pump_Off();
            Boiler_ValveClose();
            g_boiler_state = BOILER_IDLE;
            break;
        }

        out = PID_Calculate(&g_boiler_pid, g_target_temp, cur_temp, now);
        g_heater_power = (uint8_t)out;
        HEATER_SetPower(g_heater_power);

        if (!g_valve_open) Boiler_ValveOpen();

        /* 出水阶段水泵频率曲线 (相对进入HOLDING的时间elapsed) */
        {
            uint32_t elapsed = now - g_hold_start;
            uint16_t freq;   /* 0.25Hz单位 (qhz), 0~200 = 0~50Hz */

            if (elapsed < PUMP_CURVE_T1_MS) {
                /* 0-4s: 10→30Hz 线性 */
                freq = PUMP_CURVE_F_LOW +
                    (uint32_t)(PUMP_CURVE_F_MID - PUMP_CURVE_F_LOW) * elapsed / PUMP_CURVE_T1_MS;
            } else if (elapsed < PUMP_CURVE_T2_MS) {
                /* 4-8s: 30Hz 保持 */
                freq = PUMP_CURVE_F_MID;
            } else if (elapsed < PUMP_CURVE_T3_MS) {
                /* 8-12s: 30→50Hz 线性 */
                freq = PUMP_CURVE_F_MID +
                    (uint32_t)(PUMP_CURVE_F_HIGH - PUMP_CURVE_F_MID)
                    * (elapsed - PUMP_CURVE_T2_MS) / (PUMP_CURVE_T3_MS - PUMP_CURVE_T2_MS);
            } else if (elapsed < PUMP_CURVE_T4_MS) {
                /* 12-22s: 50Hz 保持 */
                freq = PUMP_CURVE_F_HIGH;
            } else {
                /* 22s后: 停泵 */
                freq = 0;
            }

            if (!WATER_IsPresent()) {
                /* 水箱缺水: 停泵防空转 (加热和阀门不受影响) */
                if (AC_Pump_IsOn()) AC_Pump_Off();
            } else if (freq == 0) {
                if (AC_Pump_IsOn()) AC_Pump_Off();
            } else {
                AC_Pump_SetFrequency(freq);
                if (!AC_Pump_IsOn()) AC_Pump_On();
            }
        }
        break;

    case BOILER_HEAT_HOLD:
        /* 纯加热保持: 预热段全功率, 70℃后PID到目标并维持.
         * 到温后不开阀不出水, 泵/阀由出饮任务控制, 此处不干预 */
        if (cur_temp < BOILER_PREHEAT_TEMP) {
            /* 预热段全功率. 不在此处 PID_Reset: 见 BOILER_HEATING 注释. */
            g_heater_power = 100;
            HEATER_SetPower(100);
        } else {
            out = Boiler_PIDStep(cur_temp, now);
            g_heater_power = (uint8_t)out;
            HEATER_SetPower(g_heater_power);
        }
        break;

    default:
        g_boiler_state = BOILER_IDLE;
        break;
    }
}

void Boiler_SetTarget(float temp)
{
    g_target_temp = temp;
    PID_SetSetpoint(&g_boiler_pid, temp);
}

float   Boiler_GetTarget(void)      { return g_target_temp; }
uint8_t Boiler_IsValveOpen(void)    { return g_valve_open; }
/* 返回加热器实际功率 (读 heater_control, 而非本地 g_heater_power 影子变量).
 * 外部接管(external_ctrl)或外部直接 HEATER_Off() 时, 本地影子不会更新,
 * 直接返回影子会导致调试打印/面板显示的功率与实际输出不一致. */
uint8_t Boiler_GetHeaterPower(void) { return HEATER_GetPower(); }
