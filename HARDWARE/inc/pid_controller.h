/**
 * @file    pid_controller.h
 * @brief   PID控制器接口 - STM32标准库(SPL)
 * @author  Claude Code
 * @date    2026-05-11
 *
 * 功能说明：
 * - 标准PID算法实现
 * - 支持参数在线调整
 * - 输出限幅和积分抗饱和
 * - 用于咖啡机温度精确控制
 */

#ifndef __PID_CONTROLLER_H
#define __PID_CONTROLLER_H

#include "sys.h"
#include <stdint.h>

/* ============================================================ */
/* 默认参数配置                                                 */
/* ============================================================ */

/* PID参数（需根据实际系统调优）
 * 注意: 此实现微分项对"测量值"求导(D on input), 与传统教材"D on error"符号相反.
 *       当输入上升(温度升), D项为负→抑制加热; 输入下降, D项为正→加大加热.
 *       优点: 目标值跳变时无 derivative kick, 适合过程控制. */
#define PID_DEFAULT_KP           10.0f              // 默认比例系数
#define PID_DEFAULT_KI           0.5f               // 默认积分系数
#define PID_DEFAULT_KD           2.0f               // 默认微分系数(对测量值求导, 符号已反转)

/* 输出限幅 */
#define PID_OUTPUT_MIN           0.0f               // 最小输出(%)
#define PID_OUTPUT_MAX           100.0f             // 最大输出(%)

/* 积分限幅 */
#define PID_INTEGRAL_LIMIT       50.0f              // 积分项限幅

/* 采样时间 */
#define PID_SAMPLE_TIME          10                 // 采样时间(ms)

/* 微分项一阶低通滤波系数: derivative_filtered = α·new + (1-α)·old
 * 0=最强滤波(无D响应), 1=无滤波(原值). 推荐 0.2~0.4 平滑尖峰. */
#define PID_DERIV_FILTER_ALPHA   0.3f

/* dt 上限: 防止 last_time=0(刚Reset)导致 dt 爆炸 */
#define PID_DT_MAX_MS            1000u              // dt 钳到 1s 以内

/* ============================================================ */
/* 数据结构                                                     */
/* ============================================================ */

/**
 * @brief PID配置结构
 */
typedef struct {
    /* PID参数 */
    float Kp;                   // 比例系数
    float Ki;                   // 积分系数
    float Kd;                   // 微分系数

    /* 限幅参数 */
    float output_min;          // 输出最小值
    float output_max;          // 输出最大值
    float integral_limit;      // 积分项限幅

    /* 内部变量 */
    float setpoint;             // 目标值
    float integral;            // 积分项
    float prev_error;           // 上次误差
    float prev_input;           // 上次输入(用于微分项计算)
    float deriv_filter;         // 微分项一阶低通滤波状态(对输入求导的平滑值)
    uint32_t last_time;         // 上次计算时间(ms)

    /* 控制标志 */
    uint8_t auto_mode;          // 自动模式标志(1=自动, 0=手动)
    uint8_t enabled;            // PID使能标志
    float   manual_output;      // 手动模式输出值
    float   last_output;        // 上次计算输出值
} PID_Controller_t;

/* ============================================================ */
/* 接口函数                                                     */
/* ============================================================ */

/**
 * @brief 初始化PID控制器
 * @param pid PID控制器指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief PID计算
 * @param pid PID控制器指针
 * @param setpoint 目标值
 * @param input 当前输入值
 * @param current_time 当前时间(ms)
 * @return PID输出值(0-100)
 * @note 在主循环中定期调用
 */
float PID_Calculate(PID_Controller_t *pid, float setpoint, float input, uint32_t current_time);

/**
 * @brief 重置PID状态
 * @param pid PID控制器指针
 * @note 清除积分项和历史误差
 */
void PID_Reset(PID_Controller_t *pid);

/**
 * @brief 设置PID参数
 * @param pid PID控制器指针
 * @param Kp 比例系数
 * @param Ki 积分系数
 * @param Kd 微分系数
 */
void PID_SetTuning(PID_Controller_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief 设置输出限幅
 * @param pid PID控制器指针
 * @param min 最小值
 * @param max 最大值
 */
void PID_SetOutputLimits(PID_Controller_t *pid, float min, float max);

/**
 * @brief 设置积分限幅
 * @param pid PID控制器指针
 * @param limit 积分项限幅
 */
void PID_SetIntegralLimit(PID_Controller_t *pid, float limit);

/**
 * @brief 设置目标值
 * @param pid PID控制器指针
 * @param setpoint 目标值
 */
void PID_SetSetpoint(PID_Controller_t *pid, float setpoint);

/**
 * @brief 使能PID控制器
 * @param pid PID控制器指针
 */
void PID_Enable(PID_Controller_t *pid);

/**
 * @brief 禁用PID控制器
 * @param pid PID控制器指针
 */
void PID_Disable(PID_Controller_t *pid);

/**
 * @brief 设置手动模式
 * @param pid PID控制器指针
 * @param manual_output 手动输出值
 */
void PID_SetManual(PID_Controller_t *pid, float manual_output);

/**
 * @brief 设置自动模式
 * @param pid PID控制器指针
 */
void PID_SetAuto(PID_Controller_t *pid);

#endif /* __PID_CONTROLLER_H */