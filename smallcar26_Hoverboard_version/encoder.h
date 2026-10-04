#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

/* ========== 全局编码器计数值 ========== */
extern volatile int32_t Get_Encoder_countA;   /* 左轮 */
extern volatile int32_t Get_Encoder_countB;   /* 右轮 */

/* ========== 方向取反宏 (调试用) ========== */
#define ENCODER_LEFT_REVERSE   0
#define ENCODER_RIGHT_REVERSE  0

/* ========== 编码器参数: 每转脉冲数 × 4 倍频 ========== */
#define ENCODER_TICKS_PER_REV_LEFT   400.0f   /* 左轮每圈脉冲数 */
#define ENCODER_TICKS_PER_REV_RIGHT  400.0f   /* 右轮每圈脉冲数 */
#define ENCODER_DT                   0.005f   /* 速度计算周期 5ms */

/* ========== API ========== */
void  Encoder_Init(void);
float Get_Speed_Left(void);     /* 返回 rps (圈/秒) */
float Get_Speed_Right(void);

#endif /* __ENCODER_H */
