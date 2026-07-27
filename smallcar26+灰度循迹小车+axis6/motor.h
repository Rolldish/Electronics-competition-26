#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>

/* ========== 电机驱动引脚 (SysConfig 自动生成, 见 ti_msp_dl_config.h) ========== */
/* 左轮 A: AIN1=PA14, AIN2=PA13  →  AIN_PORT, AIN_AIN1_PIN, AIN_AIN2_PIN */
/* 右轮 B: BIN1=PA16, BIN2=PA17  →  BIN_PORT, BIN_BIN1_PIN, BIN_BIN2_PIN */

/* PWM 通道 (TIMA1) */
#define GPIO_PWM_0_C0_IDX   DL_TIMER_CC_0_INDEX   /* 左轮 */
#define GPIO_PWM_0_C1_IDX   DL_TIMER_CC_1_INDEX   /* 右轮 */

/* ========== PWM 限幅 ========== */
#define PWM_MAX_LIMIT   7000
#define PWM_MIN_LIMIT  -7000

/* ========== API ========== */
void Motor_Init(void);
void Set_PWM(int pwmA, int pwmB);
int  limit_PWM(int value, int low, int high);

#endif /* __MOTOR_H */
