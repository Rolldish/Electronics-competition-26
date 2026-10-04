/******************************************************************
 * @file    i2c_soft.c
 * @brief   I2C 软件模拟 (GPIO bit-bang)，标准模式 (~100kHz)
 * @note    使用 GPIO 模拟 I2C 时序，PA0=SDA, PA1=SCL
 *          适配 MSPM0G3507，无需硬件 I2C 外设
 * @version 1.0
 * @date    2026
 ******************************************************************/

#include "i2c_soft.h"
#include "delay.h"

/* ========== 引脚初始化 ========== */
void I2C_Soft_Init(void)
{
    /*
     * SDA (PA0) 和 SCL (PA1) 初始化为输出高电平。
     *
     * 软件 I2C 通过 SDA_IN()/SDA_OUT() 切换方向来模拟开漏:
     *   - 输出 LOW  = 推挽输出低电平 (驱动总线 LOW)
     *   - 输出 HIGH = 推挽输出高电平 (驱动总线 HIGH)
     *   - 读取时     = 切换到输入模式 + 内部上拉 (从机拉 LOW 表示 ACK)
     *
     * 注意: 可靠的 I2C 通信需要在 SDA/SCL 上外接 4.7kΩ 上拉电阻到 VCC(3.3V)。
     * 内部上拉 (~40kΩ) 仅作为调试用的弱上拉, 长线或高速时不可靠。
     */

    /* SDA: 输出高电平 */
    DL_GPIO_initDigitalOutput(I2C_SOFT_SDA_IOMUX);
    DL_GPIO_setPins(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN);
    DL_GPIO_enableOutput(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN);

    /* SCL: 输出高电平 */
    DL_GPIO_initDigitalOutput(I2C_SOFT_SCL_IOMUX);
    DL_GPIO_setPins(I2C_SOFT_PORT, I2C_SOFT_SCL_PIN);
    DL_GPIO_enableOutput(I2C_SOFT_PORT, I2C_SOFT_SCL_PIN);

    /* 确保总线起始状态: SDA=HIGH, SCL=HIGH (空闲状态) */
    DL_GPIO_setPins(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN | I2C_SOFT_SCL_PIN);
    delay_us(10);
}

/* ========== 起始信号 ========== */
void IIC_Start(void)
{
    SDA_OUT();
    SDA(1);
    SCL(1);
    delay_us(5);
    SDA(0);
    delay_us(5);
    SCL(0);
    delay_us(5);
}

/* ========== 停止信号 ========== */
void IIC_Stop(void)
{
    SDA_OUT();
    SCL(0);
    delay_us(5);
    SDA(0);
    delay_us(5);
    SCL(1);
    delay_us(5);
    SDA(1);
    delay_us(5);
}

/* ========== 发送应答 ========== */
void IIC_Send_Ack(uint8_t ack)
{
    SDA_OUT();
    SCL(0);
    delay_us(5);
    if (ack) {
        SDA(1);    /* NACK */
    } else {
        SDA(0);    /* ACK */
    }
    delay_us(5);
    SCL(1);
    delay_us(5);
    delay_us(5);
    SCL(0);
    delay_us(5);
    SDA(1);
}

/* ========== 等待应答 ========== */
uint8_t IIC_Wait_Ack(void)
{
    uint8_t timeout = 0;

    SDA_IN();
    SDA(1);
    delay_us(5);

    SCL(1);
    delay_us(5);

    while (SDA_GET() && (timeout < 255)) {
        timeout++;
        delay_us(1);
    }

    SCL(0);
    delay_us(5);
    SDA_OUT();

    if (timeout >= 255) {
        IIC_Stop();
        return 1;   /* 无应答 */
    }
    return 0;       /* 收到应答 */
}

/* ========== 发送一个字节 ========== */
void IIC_Send_Byte(uint8_t dat)
{
    uint8_t i;
    SDA_OUT();
    SCL(0);
    delay_us(5);

    for (i = 0; i < 8; i++) {
        if (dat & 0x80) {
            SDA(1);
        } else {
            SDA(0);
        }
        delay_us(5);
        SCL(1);
        delay_us(5);
        SCL(0);
        delay_us(5);
        dat <<= 1;
    }
}

/* ========== 读取一个字节 ========== */
uint8_t IIC_Read_Byte(void)
{
    uint8_t i;
    uint8_t receive = 0;

    SDA_IN();
    SDA(1);
    delay_us(5);

    for (i = 0; i < 8; i++) {
        SCL(0);
        delay_us(5);
        SCL(1);
        delay_us(5);
        receive <<= 1;
        if (SDA_GET()) {
            receive |= 1;
        }
        delay_us(5);
    }

    SCL(0);
    delay_us(5);
    SDA_OUT();

    return receive;
}

/* ========== 写寄存器 (单字节) ========== */
uint8_t I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
    return I2C_WriteReg(dev_addr, reg_addr, &data, 1);
}

/* ========== 读寄存器 (单字节) ========== */
uint8_t I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data)
{
    return I2C_ReadReg(dev_addr, reg_addr, data, 1);
}

/* ========== 发送命令 (无寄存器地址) ========== */
uint8_t I2C_SendCmd(uint8_t dev_addr, uint8_t cmd)
{
    IIC_Start();
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    IIC_Send_Byte(cmd);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    IIC_Stop();
    return 1;
}

/* ========== 写原始数据 (无寄存器地址) ========== */
void I2C_WriteData(uint8_t dev_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return; }
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (IIC_Wait_Ack()) { IIC_Stop(); return; }
    }
    IIC_Stop();
}

/* ========== 读原始数据 (无寄存器地址) ========== */
void I2C_ReadData(uint8_t dev_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte((dev_addr << 1) | 0x01);
    if (IIC_Wait_Ack()) { IIC_Stop(); return; }
    for (i = 0; i < len; i++) {
        data[i] = IIC_Read_Byte();
        if (i < len - 1) {
            IIC_Send_Ack(0);
        } else {
            IIC_Send_Ack(1);
        }
    }
    IIC_Stop();
}

/* ========== 写寄存器 (多字节) ========== */
uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    for (i = 0; i < len; i++) {
        IIC_Send_Byte(data[i]);
        if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    }
    IIC_Stop();
    return 1;
}

/* ========== 读寄存器 (多字节, 带 Repeated START) ========== */
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    uint8_t i;
    IIC_Start();
    IIC_Send_Byte(dev_addr << 1);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    IIC_Send_Byte(reg_addr);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte((dev_addr << 1) | 0x01);
    if (IIC_Wait_Ack()) { IIC_Stop(); return 0; }
    for (i = 0; i < len; i++) {
        data[i] = IIC_Read_Byte();
        if (i < len - 1) {
            IIC_Send_Ack(0);
        } else {
            IIC_Send_Ack(1);
        }
    }
    IIC_Stop();
    return 1;
}
