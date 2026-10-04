/******************************************************************
 * @file    axis6.c
 * @brief   AXIS6 6轴姿态传感器驱动 (I2C 软件模拟)
 * @note    使用 GPIO 模拟 I2C (PA0=SDA, PA1=SCL)，无需硬件 I2C
 *          传感器地址: 0x48
 *          通信协议: 解锁 → 命令/读取 → (可选)保存
 * @version 2.0 — 软件 I2C 版本, 移除 MPU6050 遗留代码
 ******************************************************************/

#include "axis6.h"
#include "delay.h"

/* ========== 全局变量 ========== */
float axis6_gyro_scale  = AXIS6_GYRO_SCALE;
float axis6_angle_scale = AXIS6_ANGLE_SCALE;
int   axis6_error       = 0;   /* 0=OK, 1=Unlock, 2=Cmd, 3=ReadData */

/* ========== 底层 I2C 写/读 (封装 i2c_soft) ========== */

static bool AXIS6_WriteData(uint8_t *data, uint8_t len)
{
    if (len == 0 || len > 8) return false;

    IIC_Start();
    IIC_Send_Byte(AXIS6_ADDR << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return false; }

    for (uint8_t i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (IIC_Wait_Ack()) { IIC_Stop(); return false; }
    }
    IIC_Stop();
    return true;
}

static bool AXIS6_ReadRegister(uint8_t reg, uint8_t *data, uint8_t len)
{
    if (len > 14) return false;

    /* 写寄存器地址 */
    IIC_Start();
    IIC_Send_Byte(AXIS6_ADDR << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return false; }
    IIC_Send_Byte(reg);
    if (IIC_Wait_Ack()) { IIC_Stop(); return false; }
    IIC_Stop();

    /* Repeated START → 读数据 */
    IIC_Start();
    IIC_Send_Byte((AXIS6_ADDR << 1) | 0x01);
    if (IIC_Wait_Ack()) { IIC_Stop(); return false; }

    for (uint8_t i = 0; i < len; i++) {
        data[i] = IIC_Read_Byte();
        if (i < len - 1) {
            IIC_Send_Ack(0);    /* ACK: 继续读取 */
        } else {
            IIC_Send_Ack(1);    /* NACK: 最后一个字节 */
        }
    }
    IIC_Stop();
    return true;
}

/* ========== 解锁传感器 (写保护密钥) ========== */
bool AXIS6_Unlock(void)
{
    uint8_t key[3] = { AXIS6_KEY_BYTE0, AXIS6_KEY_BYTE1, AXIS6_KEY_BYTE2 };
    return AXIS6_WriteData(key, 3);
}

/* ========== 发送3字节命令 (自动解锁) ========== */
bool AXIS6_SendCommand(uint8_t b0, uint8_t b1, uint8_t b2)
{
    if (!AXIS6_Unlock()) return false;
    delay_us(10000);  /* 10ms 等待传感器处理解锁 */
    uint8_t cmd[3] = { b0, b1, b2 };
    return AXIS6_WriteData(cmd, 3);
}

/* ========== 读取原始数据 ========== */
bool AXIS6_ReadRawData(AXIS6_RawData *raw)
{
    uint8_t buf[6];

    /* 读取陀螺仪数据 (0xAA) */
    if (!AXIS6_ReadRegister(AXIS6_REG_GYRO, buf, 6))
        return false;
    raw->gyro_x = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    raw->gyro_y = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    raw->gyro_z = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

    /* 读取姿态角 (0xBB) */
    if (!AXIS6_ReadRegister(AXIS6_REG_ANGLE, buf, 6))
        return false;
    raw->roll_raw  = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
    raw->pitch_raw = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);
    raw->yaw_raw   = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);

    return true;
}

/* ========== 初始化 AXIS6 传感器 ========== */
bool AXIS6_Init(void)
{
    axis6_error = 0;

    /* 1. 初始化软件 I2C 引脚 (PA0=SDA, PA1=SCL) */
    I2C_Soft_Init();
    delay_us(100000);  /* 100ms 等待传感器上电稳定 */

    /* 2. 解锁传感器 */
    axis6_error = 1;
    if (!AXIS6_Unlock()) return false;
    delay_us(10000);

    /* 3. 发送 Yaw 归零校准命令 */
    axis6_error = 2;
    if (!AXIS6_SendCommand(0x0A, 0x04, 0x00)) return false;
    delay_us(10000);

    /* 4. 发送空数据帧 (保持通信) */
    {
        uint8_t s[3] = {0};
        AXIS6_WriteData(s, 3);
    }
    delay_us(10000);

    /* 5. 验证通信: 尝试读取数据 (最多重试3次) */
    axis6_error = 3;
    AXIS6_RawData raw;
    for (uint8_t r = 0; r < 3; r++) {
        if (AXIS6_ReadRawData(&raw)) {
            axis6_error = 0;
            return true;
        }
        delay_us(5000);
    }
    return false;
}
