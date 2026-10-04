#ifndef __ENCODER_H
#define __ENCODER_H
#include "ti_msp_dl_config.h"
#include <stdint.h> 
// 声明你的全局计数变量
volatile int32_t Get_Encoder_countA;
volatile int32_t Get_Encoder_countB;
// 方向取反宏（调试用）
#define ENCODER_LEFT_REVERSE  0   // 若正转计数减少，改为1
#define ENCODER_RIGHT_REVERSE 0
void Encoder_Init();
float Get_Speed_Left();   // 返回 rps
float Get_Speed_Right();
#endif


    







