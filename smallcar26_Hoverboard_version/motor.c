#include "motor.h"

/* ========== PWM 限幅 ========== */
int limit_PWM(int value, int low, int high)
{
    if (value > high) return high;
    else if (value < low) return low;
    else return value;
}

/* ========== 设置左右电机 PWM ========== */
void Set_PWM(int pwmA, int pwmB)
{
    /* --- 左轮 (A) 方向 + 转速 --- */
    if (pwmA > 0) {
        /* 正转: AIN1=H, AIN2=L */
        DL_GPIO_setPins(AIN_PORT, AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,
            (uint32_t)abs(pwmA), GPIO_PWM_0_C0_IDX);
    } else {
        /* 反转: AIN2=H, AIN1=L */
        DL_GPIO_setPins(AIN_PORT, AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,
            (uint32_t)abs(pwmA), GPIO_PWM_0_C0_IDX);
    }

    /* --- 右轮 (B) 方向 + 转速 --- */
    if (pwmB > 0) {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN2_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,
            (uint32_t)abs(pwmB), GPIO_PWM_0_C1_IDX);
    } else {
        DL_GPIO_setPins(BIN_PORT, BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,
            (uint32_t)abs(pwmB), GPIO_PWM_0_C1_IDX);
    }
}

/* ========== 电机初始化 ========== */
void Motor_Init(void)
{
    /* 确保方向引脚初始为低电平 */
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);

    /* 停止 PWM */
    Set_PWM(0, 0);
}
