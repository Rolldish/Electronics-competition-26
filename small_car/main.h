#ifndef __MAIN_H
#define __MAIN_H
#include "ti_msp_dl_config.h"
#include "delay.h"
#include "stdio.h"
#include "Uart.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include <string.h>


//===================== 传感器配置 =====================
extern unsigned short Anolog[8];
extern unsigned short white[8];
extern unsigned short black[8];
extern unsigned short Normal[8];
extern unsigned char rx_buff[256];
extern No_MCU_Sensor sensor;
extern unsigned char Digtal;  // 8路数字量核心变量
extern volatile uint32_t control_update_cnt;
//===================== 电机PID速度控制参数 =====================
#define TARGET_SPEED     10  // 目标速度（编码器脉冲/10ms）
#define Velocity_Kp        0.2f   // 比例系数
#define Velocity_Ki        0.8f    // 积分系数
#define PID_KD        0.2f    // 微分系数
#define PWM_MAX       95      // PWM最大占空比
#define PWM_MIN       0       // PWM最小占空比

#endif