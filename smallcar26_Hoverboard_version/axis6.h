#ifndef __AXIS6_H
#define __AXIS6_H

#include "i2c_soft.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== AXIS6 设备地址 & 寄存器 ========== */
#define AXIS6_ADDR                  0x48
#define AXIS6_I2C_TIMEOUT_MS        10

#define AXIS6_REG_GYRO              0xAA   /* 陀螺仪原始数据 (6 bytes: wx, wy, wz) */
#define AXIS6_REG_ANGLE             0xBB   /* 姿态角 (6 bytes: Roll, Pitch, Yaw) */

/* ========== 解锁密钥 ========== */
#define AXIS6_KEY_BYTE0             0x13
#define AXIS6_KEY_BYTE1             0x8E
#define AXIS6_KEY_BYTE2             0x5F

/* ========== 比例因子 ========== */
#define AXIS6_GYRO_SCALE            (2000.0f / 32768.0f)   /* °/s per LSB */
#define AXIS6_ANGLE_SCALE           (180.0f / 32768.0f)    /* ° per LSB */

/* ========== 数据结构 ========== */
typedef struct {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t pitch_raw;
    int16_t roll_raw;
    int16_t yaw_raw;
} AXIS6_RawData;

/* ========== 全局变量 ========== */
extern float axis6_gyro_scale;
extern float axis6_angle_scale;
extern int   axis6_error;   /* 初始化失败步骤码 (0=OK) */

/* ========== API ========== */
bool AXIS6_Init(void);
bool AXIS6_ReadRawData(AXIS6_RawData *raw);
bool AXIS6_Unlock(void);
bool AXIS6_SendCommand(uint8_t b0, uint8_t b1, uint8_t b2);

#endif /* __AXIS6_H */
