#ifndef __I2C_SOFT_H
#define __I2C_SOFT_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== I2C 软件模拟引脚 (PA0=SDA, PA1=SCL) ========== */
#define I2C_SOFT_PORT       GPIOA
#define I2C_SOFT_SDA_PIN    DL_GPIO_PIN_0
#define I2C_SOFT_SCL_PIN    DL_GPIO_PIN_1
#define I2C_SOFT_SDA_IOMUX  (IOMUX_PINCM1)   /* PA0 */
#define I2C_SOFT_SCL_IOMUX  (IOMUX_PINCM2)   /* PA1 */

/* ========== SDA 方向切换宏 (模拟开漏输出) ========== */
#define SDA_OUT()   {                                                \
                        DL_GPIO_initDigitalOutput(I2C_SOFT_SDA_IOMUX); \
                        DL_GPIO_setPins(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN); \
                        DL_GPIO_enableOutput(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN); \
                    }

#define SDA_IN()    { DL_GPIO_initDigitalInput(I2C_SOFT_SDA_IOMUX); }

#define SDA_GET()   ( ((DL_GPIO_readPins(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN) & I2C_SOFT_SDA_PIN) > 0) ? 1 : 0 )

#define SDA(x)      ( (x) ? (DL_GPIO_setPins(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN)) : (DL_GPIO_clearPins(I2C_SOFT_PORT, I2C_SOFT_SDA_PIN)) )
#define SCL(x)      ( (x) ? (DL_GPIO_setPins(I2C_SOFT_PORT, I2C_SOFT_SCL_PIN)) : (DL_GPIO_clearPins(I2C_SOFT_PORT, I2C_SOFT_SCL_PIN)) )

/* ========== API ========== */
void I2C_Soft_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Ack(uint8_t ack);
uint8_t IIC_Wait_Ack(void);
void IIC_Send_Byte(uint8_t dat);
uint8_t IIC_Read_Byte(void);

uint8_t I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
uint8_t I2C_ReadReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint8_t len);
uint8_t I2C_WriteByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);
uint8_t I2C_ReadByte(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data);
uint8_t I2C_SendCmd(uint8_t dev_addr, uint8_t cmd);
void I2C_WriteData(uint8_t dev_addr, uint8_t *data, uint8_t len);
void I2C_ReadData(uint8_t dev_addr, uint8_t *data, uint8_t len);

#endif /* __I2C_SOFT_H */
