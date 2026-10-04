#ifndef __IMU_H
#define __IMU_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/* ========== 姿态数据结构 ========== */
typedef struct {
    float pitch;        /* 俯仰角 (°) — 前后倾斜 (正值 = 前倾, AXIS6 Pitch) */
    float pitch_rate;   /* 俯仰角速度 (°/s) — 陀螺仪 X 轴 (gyro.x) */
    float roll;         /* 横滚角 (°) — 左右倾斜 (AXIS6 Roll) */
    float yaw;          /* 偏航角 (°) — 陀螺仪 Z 轴积分 */
    float yaw_rate;     /* 偏航角速度 (°/s) — 陀螺仪 Z 轴 (gyro.z, 用于转向 PD) */
} IMU_Attitude;

/* ========== IMU 状态 ========== */
typedef struct {
    float pitch;
    float roll;
    float yaw;

    float gyro_offset_x;        /* 陀螺仪 X 轴零偏 (°/s) */
    float gyro_offset_y;
    float gyro_offset_z;

    float comp_filter_alpha;    /* 互补滤波系数 (可选, 典型值 0.95~0.98) */

    uint32_t last_update_tick;  /* 上次更新时间 (Tick) */
    bool     initialized;
} IMU_State;

/* ========== API ========== */
bool IMU_Init(float comp_alpha);
bool IMU_CalibrateGyro(uint16_t samples);
bool IMU_Update(IMU_Attitude *att, float dt_s);
void IMU_Reset(void);

/* 外部访问 (用于调试) */
extern IMU_State imu_state;

#endif /* __IMU_H */
