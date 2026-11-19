/**
 * @file    pid_controller.c
 * @brief   PID控制器实现 - STM32标准库(SPL)
 * @author  Claude Code
 * @date    2026-05-11
 */

#include "pid_controller.h"
#include <math.h>

/**
 * @brief 初始化PID控制器
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    // 设置PID参数
    PID_SetTuning(pid, Kp, Ki, Kd);

    // 设置默认限幅
    PID_SetOutputLimits(pid, PID_OUTPUT_MIN, PID_OUTPUT_MAX);
    PID_SetIntegralLimit(pid, PID_INTEGRAL_LIMIT);

    // 初始化为手动模式，默认禁用
    PID_SetManual(pid, 0.0f);
    PID_Disable(pid);

    // 重置状态
    PID_Reset(pid);
}

/**
 * @brief PID计算
 * @note 使用"对测量值求导"的工业PID:
 *       output = Kp*e + Ki*∫e - Kd*(dInput/dt)
 *       上升时 D 为负抑制加热, 下降时 D 为正加大加热.
 *       相比 D-on-error 消除了目标值跳变时的 derivative kick. */
float PID_Calculate(PID_Controller_t *pid, float setpoint, float input, uint32_t current_time)
{
    float output;
    float P, I, D;
    float error;
    float dt;
    float input_rate;       /* dInput/dt (上升为正) */
    float inter_time;
    uint8_t saturated;      /* 输出饱和标志, 用于抗饱和 */

    // 检查是否使能
    if (!pid->enabled) {
        pid->last_output = pid->manual_output;
        return pid->manual_output;
    }

    // 计算误差
    error = setpoint - input;

    /* 计算时间步长(ms -> s).
     * 关键: 如果 last_time==0 (刚 Reset), 钳到 PID_SAMPLE_TIME,
     *       避免 dt 爆炸导致 derivative 尖峰. */
    inter_time = (float)(current_time - pid->last_time);
    if (pid->last_time == 0 || inter_time < 1.0f) {
        inter_time = (float)PID_SAMPLE_TIME;  /* 默认用采样周期 */
    }
    if (inter_time > (float)PID_DT_MAX_MS) {
        inter_time = (float)PID_DT_MAX_MS;    /* 防止长时间未调用 */
    }
    dt = inter_time / 1000.0f;

    // 比例项
    P = pid->Kp * error;

    // 积分项(带限幅, 抗饱和逻辑见输出钳位后)
    if (fabsf(pid->Ki) > 0.01f) {
        pid->integral += error * dt;

        // 积分限幅(对称: 上下都限, 避免单边饱和)
        if (pid->integral > pid->integral_limit) {
            pid->integral = pid->integral_limit;
        } else if (pid->integral < -pid->integral_limit) {
            pid->integral = -pid->integral_limit;
        }
    } else {
        pid->integral = 0.0f;
    }
    I = pid->Ki * pid->integral;

    // 微分项(对测量值求导 + 一阶低通滤波)
    if (fabsf(pid->Kd) > 0.01f && inter_time > 0.01f) {
        /* 工业惯例: D on input, 上升为正.
         * 首帧(last_time==0)跳过微分, 避免 prev_input=0 导致 input_rate 爆炸. */
        if (pid->last_time == 0) {
            input_rate = 0.0f;
        } else {
            input_rate = (input - pid->prev_input) / inter_time;
        }
        /* 一阶低通: derivative_filtered = α·new + (1-α)·old */
        pid->deriv_filter = PID_DERIV_FILTER_ALPHA * input_rate +
                            (1.0f - PID_DERIV_FILTER_ALPHA) * pid->deriv_filter;
        /* D = -Kd * rate: 上升→抑制, 下降→加大 */
        D = -pid->Kd * pid->deriv_filter;
    } else {
        D = 0.0f;
    }

    // 计算原始输出
    output = P + I + D;

    // 输出限幅
    saturated = 0;
    if (output > pid->output_max) {
        output = pid->output_max;
        saturated = 1;
    } else if (output < pid->output_min) {
        output = pid->output_min;
        saturated = 1;
    }

    /* 抗积分饱和 (anti-windup):
     * 当输出被钳到极限, 且误差方向与饱和方向一致 (会继续推积分),
     * 停止本帧的积分累积, 避免积分器持续单向累积导致后续恢复时大超调.
     * 仅在 Ki>0 时生效. */
    if (saturated && fabsf(pid->Ki) > 0.01f) {
        if ((output >= pid->output_max && error > 0.0f) ||
            (output <= pid->output_min && error < 0.0f)) {
            /* 回滚本帧累积的积分量 */
            pid->integral -= error * dt;
            I = pid->Ki * pid->integral;
            output = P + I + D;
            /* 重新钳位 (回滚后可能仍在极限内或外) */
            if (output > pid->output_max) output = pid->output_max;
            else if (output < pid->output_min) output = pid->output_min;
        }
    }

    // 保存状态
    pid->prev_error = error;
    pid->prev_input = input;
    pid->last_time = current_time;
    pid->setpoint = setpoint;
    pid->last_output = output;

    return output;
}

/**
 * @brief 重置PID状态
 */
void PID_Reset(PID_Controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_input = 0.0f;
    pid->deriv_filter = 0.0f;
    pid->last_time = 0;   /* PID_Calculate 检测到 0 会用 PID_SAMPLE_TIME 作默认 dt */
}

/**
 * @brief 设置PID参数
 */
void PID_SetTuning(PID_Controller_t *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

/**
 * @brief 设置输出限幅
 */
void PID_SetOutputLimits(PID_Controller_t *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

/**
 * @brief 设置积分限幅
 */
void PID_SetIntegralLimit(PID_Controller_t *pid, float limit)
{
    pid->integral_limit = limit;
}

/**
 * @brief 设置目标值
 */
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

/**
 * @brief 使能PID控制器
 */
void PID_Enable(PID_Controller_t *pid)
{
    pid->enabled = 1;
}

/**
 * @brief 禁用PID控制器
 */
void PID_Disable(PID_Controller_t *pid)
{
    pid->enabled = 0;
}

/**
 * @brief 设置手动模式
 */
void PID_SetManual(PID_Controller_t *pid, float manual_output)
{
    pid->manual_output = manual_output;
    PID_Disable(pid);
}

/**
 * @brief 设置自动模式
 */
void PID_SetAuto(PID_Controller_t *pid)
{
    PID_Enable(pid);
}