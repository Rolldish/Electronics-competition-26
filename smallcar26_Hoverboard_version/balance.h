#ifndef __BALANCE_H
#define __BALANCE_H

#include "pid.h"
#include "imu.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== 平衡控制参数 (需根据实际硬件整定) ========== */

/* ===================================================================
 * 直立环 (角度 PD) — 内环, 最高优先级
 *
 * 公式: output = Kp * angle_error(°) + Kd * gyro_rate(°/s)
 *
 * 两种调参风格 (均可工作, 取决于机械系统):
 *   [A] P 主导型 (WHEELTEC 风格): Kp≈280, Kd≈1.5
 *       响应快, 刚性强, 依赖大 P 维持直立, Kd 仅轻度阻尼
 *       → 适用于: 电机力矩大、齿轮间隙小、重心低的系统
 *   [B] D 主导型 (旧 smallcar26 风格): Kp≈200, Kd≈300
 *       对角度偏差响应温和, 依赖大 Kd 快速抑制角速度防止倾倒
 *       → 适用于: 电机响应慢、需要强阻尼、机械间隙较大的系统
 *
 * 推荐从 P 主导型开始调参:
 *   1. Kd=0, 逐步增大 Kp 直到小车能短暂站立 (以 ~5Hz 震荡)
 *   2. 逐步增大 Kd 直到震荡消失
 *   3. 如果 Kd 需调至 >100 才能稳定, 说明机械系统适合 D 主导型
 * =================================================================== */
#define BALANCE_ANGLE_KP        800.0f  /* 角度比例系数 (大步提高, 穿透死区+补偿IMU延迟) */
#define BALANCE_ANGLE_KD        400.0f  /* 角速度阻尼 (同步提高, 维持阻尼比) */
#define GYRO_D_FILTER_ALPHA      0.1f  /* D 项滤波: 1.0=不过滤 */
#define BALANCE_ANGLE_OUT_MAX  8000.0f  /* 角度环输出限幅 */

/* ===================================================================
 * 速度环 (速度 PI) — 外环
 *
 * 公式 (WHEELTEC): velocity = filtered_enc_pulses * Kp/100 + integral * Ki/100
 * 公式 (smallcar26): speed_output = Kp * speed_error(rps) + Ki * integral(rps*s)
 *
 * 速度环输出是角度偏移量 (°):
 *   speed_output > 0 → 目标前倾 → 加速前进 → 形成负反馈速度闭环
 *
 * 参考 WHEELTEC (编码器脉冲输入): Kp≈252, Ki≈1.25
 * smallcar26 使用 rps 作输入, 参数需单独整定 (量纲不同!)
 * =================================================================== */
#define BALANCE_SPEED_KP         0.0f   /* 速度比例 (提高, 推车刹车靠P, 不靠I) */
#define BALANCE_SPEED_KI         0.0f   /* 速度积分 (降低, 只消除残余偏差, 不主导) */
#define BALANCE_SPEED_OUT_MAX    6.0f   /* 速度环输出限幅 (°) */

/* 速度环低通滤波 */
#define SPEED_FILTER_ALPHA     0.14f  /* 新值权重 (0~1), WHEELTEC 默认 */

/* 转向环 (偏航 PD) */
#define BALANCE_TURN_KP         0.0f  /* 转向比例 (关掉) */
#define BALANCE_TURN_KD         0.0f  /* 偏航阻尼 (先关掉, 站住再说) */
#define BALANCE_TURN_OUT_MAX  2000.0f /* 转向输出限幅 */

/* 控制参数 */
#define BALANCE_DT            0.005f   /* 控制周期 5ms (200Hz) */
#define BALANCE_TARGET_ANGLE  0.0f     /* 目标直立角度 (°) */
#define BALANCE_ANGLE_TRIM    1.8f     /* 机械重心偏移: 前倾→增大, 后倾→减小 */
#define BALANCE_TARGET_SPEED  0.0f     /* 目标速度 (rps, 0 = 静止) */
#define BALANCE_TARGET_TURN   0.0f     /* 目标转向角速度 (°/s, 0 = 不转) */

/* 安全保护 */
#define BALANCE_MAX_ANGLE     50.0f    /* 最大允许倾角 (°, 参考 WHEELTEC 50°) */
#define BALANCE_LOW_VOLTAGE    9.5f    /* 电池低压保护阈值 (V, 3S 锂电 3.2*3=9.6V) */

/* 编码器死区 (rps) — 滤除量化噪声, 防止积分漂移 */
#define SPEED_DEAD_ZONE        0.1f     /* 降低死区, 更早检测到漂移 */

/* 抗积分饱和: 角度误差超过此值则冻结速度环积分 */
#define ANGLE_ERROR_IWINDUP    8.0f

/* ========== 平衡状态 ========== */
typedef enum {
    BALANCE_STOP    = 0,   /* 停止 (电机锁定) */
    BALANCE_RUNNING = 1    /* 平衡运行 */
} BalanceState;

/* ========== 停止原因 (调试用) ========== */
typedef enum {
    STOP_REASON_NONE    = 0,   /* 未停止 (RUNNING) */
    STOP_REASON_INIT    = 1,   /* 初始化默认停止 */
    STOP_REASON_IMU     = 2,   /* IMU 读取失败 */
    STOP_REASON_ANGLE   = 3,   /* 倾角过大 (>BALANCE_MAX_ANGLE) */
    STOP_REASON_VOLTAGE = 4,   /* 电池电压过低 */
    STOP_REASON_USER    = 5    /* 用户主动调用 */
} BalanceStopReason;

/* ========== 调试数据 (主循环中可用于 OLED 显示) ========== */
typedef struct {
    float pitch;            /* 当前俯仰角 (°) */
    float pitch_rate;       /* 当前角速度 (°/s) */
    float yaw_rate;         /* 当前偏航角速度 (°/s, gyro.z) */
    float speed_left;       /* 左轮速度 (rps) */
    float speed_right;      /* 右轮速度 (rps) */
    float speed_filtered;   /* 低通滤波后的平均速度 */
    float angle_output;     /* 角度环输出 */
    float speed_output;     /* 速度环输出 (角度偏移 °) */
    float turn_output;      /* 转向环输出 */
    int   pwm_left;         /* 左轮 PWM */
    int   pwm_right;        /* 右轮 PWM */
    float battery_voltage;  /* 电池电压 (V) */
    BalanceState state;     /* 当前状态 */
    BalanceStopReason stop_reason;  /* 最近一次停止原因 */
} BalanceDebug;

/* ========== API ========== */
void Balance_Init(void);
void Balance_Update(void);    /* 在 5ms 定时器中断/主循环中调用 */
void Balance_Start(void);
void Balance_Stop(BalanceStopReason reason);
void Balance_SetTargetSpeed(float speed_rps);   /* 设置目标速度 (前进为正) */
void Balance_SetTargetTurn(float turn_dps);     /* 设置目标转向角速度 (°/s, 左转为正) */

/* 外部访问: 调试数据 */
extern BalanceDebug balance_debug;

/* 电池电压测量 (需 ADC 实现, 返回电压值 V) */
extern float Balance_ReadBatteryVoltage(void);

#endif /* __BALANCE_H */
