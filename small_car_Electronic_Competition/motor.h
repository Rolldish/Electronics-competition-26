#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdlib.h>
#include "main.h"


// // 电机A（左轮）方向引脚 → AIN1=PA14，AIN2=PA13
// #define AIN_PORT        GPIOA            // 端口固定为GPIOA
// #define AIN_AIN1_PIN    DL_GPIO_PIN_14   // PA14
// #define AIN_AIN2_PIN    DL_GPIO_PIN_13   // PA13

// // 电机B（右轮）方向引脚 → BIN1=PA16，BIN2=PA17
// #define BIN_PORT        GPIOA            // 端口固定为GPIOA
// #define BIN_BIN1_PIN    DL_GPIO_PIN_16   // PA16
// #define BIN_BIN2_PIN    DL_GPIO_PIN_17   // PA17

// // PWM定时器（MSPM0G3507 定时器PWM通道，无需修改）
// #define PWM_0_INST      TIM0_INST
// #define GPIO_PWM_0_C0_IDX   0  // 左轮PWM通道
// #define GPIO_PWM_0_C1_IDX   1  // 右轮PWM通道

// ===================== 控制参数（无需修改） =====================
#define PWM_MAX_LIMIT     7000   // 轮趣代码默认PWM限幅
#define PWM_MIN_LIMIT     -7000
//#define TARGET_SPEED      150    // 目标速度（编码器脉冲值）
#define TRACKING_K        5.0f   // 循迹转向系数

// // ===================== 速度环PI参数（无需修改） =====================
// extern float Velocity_Kp;
// extern float Velocity_Ki;

// ===================== 外部全局变量（循迹偏差） =====================
extern int16_t track_error;

void Set_PWM(int pwmA,int pwmB);
int Velocity_A(int TargetVelocity, int CurrentVelocity);
int Velocity_B(int TargetVelocity, int CurrentVelocity);

void Motor_Init(void);
int limit_PWM(int value, int low, int high);




















#endif