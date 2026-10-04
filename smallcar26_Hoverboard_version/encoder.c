#include "encoder.h"

/* ========== 全局变量 ========== */
volatile int32_t Get_Encoder_countA = 0;
volatile int32_t Get_Encoder_countB = 0;

/* 速度计算 */
static int32_t last_left_cnt  = 0;
static int32_t last_right_cnt = 0;

/* ========== 初始化 ========== */
void Encoder_Init(void)
{
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    last_left_cnt      = 0;
    last_right_cnt     = 0;
}

/* ========== 读取左轮速度 (rps) ========== */
float Get_Speed_Left(void)
{
    int32_t current = Get_Encoder_countA;
#if ENCODER_LEFT_REVERSE
    current = -current;
#endif
    int32_t delta = current - last_left_cnt;
    last_left_cnt = current;
    return (float)delta / (ENCODER_TICKS_PER_REV_LEFT * ENCODER_DT);
}

/* ========== 读取右轮速度 (rps) ========== */
float Get_Speed_Right(void)
{
    int32_t current = Get_Encoder_countB;
#if ENCODER_RIGHT_REVERSE
    current = -current;
#endif
    int32_t delta = current - last_right_cnt;
    last_right_cnt = current;
    return (float)delta / (ENCODER_TICKS_PER_REV_RIGHT * ENCODER_DT);
}

/* ========== 编码器中断处理 (GPIOA + GPIOB → GROUP1) ========== */
void GROUP1_IRQHandler(void)
{
    uint32_t statusA, statusB;

    /* 读取 GPIOA 和 GPIOB 的中断状态 */
    statusA = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    statusB = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    /* ---- 左编码器 (ENCODERA: PA25=E1A, PA26=E1B) ---- */
    if (statusA & ENCODERA_E1A_PIN) {
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN)) {
            Get_Encoder_countA++;
        } else {
            Get_Encoder_countA--;
        }
    }
    if (statusA & ENCODERA_E1B_PIN) {   /* 独立 if, 不是 else if — 两路可能同时触发 */
        if (!DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN)) {
            Get_Encoder_countA--;
        } else {
            Get_Encoder_countA++;
        }
    }

    /* ---- 右编码器 (ENCODERB: PB20=E2A, PB24=E2B) ---- */
    if (statusB & ENCODERB_E2A_PIN) {
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN)) {
            Get_Encoder_countB++;
        } else {
            Get_Encoder_countB--;
        }
    }
    if (statusB & ENCODERB_E2B_PIN) {   /* 独立 if, 不是 else if — 两路可能同时触发 */
        if (!DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN)) {
            Get_Encoder_countB--;
        } else {
            Get_Encoder_countB++;
        }
    }

    /* 清除中断标志 (编码器) */
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT,
        ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}
