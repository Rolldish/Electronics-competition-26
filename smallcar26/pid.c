#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->Kp         = kp;
    pid->Ki         = ki;
    pid->Kd         = kd;
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
    pid->out_min    = out_min;
    pid->out_max    = out_max;
    /* 积分限幅独立设置: 默认与输出限幅一致, 可通过外部修改 */
    pid->integral_max = out_max;
    pid->integral_min = out_min;
    pid->integral_decay = 0.98f;  /* 默认纯积分, 不衰减 */
}

float PID_Update(PID_t *pid, float target, float actual, float dt)
{
    float error = target - actual;

    /* ---- P 项 ---- */
    float p_term = pid->Kp * error;

    /* ---- I 项 (漏积分 + 独立限幅 + clamping 抗饱和) ----
     * 漏积分 (integral_decay < 1.0):
     *   每次先衰减再累加: integral = integral*decay + error*dt
     *   防止编码器偏置等 DC 误差导致积分无限增长
     *   参考 WHEELTEC: Encoder_bias *= 0.86 (等效衰减)
     */
    pid->integral = pid->integral * pid->integral_decay + error * dt;

    /* 积分独立限幅 (可大于输出限幅, 用于速度环位移累积) */
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    if (pid->integral < pid->integral_min)
        pid->integral = pid->integral_min;

    float i_term = pid->Ki * pid->integral;

    /* ---- D 项 (微分先行/误差微分) ----
     * 注意: 角度环不使用此 D 项 (直接用 gyro_rate 替代)
     * 速度环 Kd=0, 转向环 Kd=0 (gyro.z 直接反馈)
     * 此 D 项保留给需要经典 PID 的场景
     */
    float derivative = (error - pid->last_error) / dt;
    float d_term = pid->Kd * derivative;

    /* 保存误差 (必须在输出限幅之前, 否则破坏 D 项) */
    pid->last_error = error;

    /* ---- 输出 = P + I + D ---- */
    float output = p_term + i_term + d_term;

    /* ---- 输出限幅 + clamping 抗饱和 ----
     * 如果输出超出限幅, 回退本次积分累积 (防止继续向同方向饱和)
     * 这比简单清零更精细: 只撤销导致饱和的那部分积分
     */
    if (output > pid->out_max) {
        /* 如果积分项在助推饱和方向, 回退多余部分 */
        if (i_term > 0.0f) {
            pid->integral -= (output - pid->out_max) / pid->Ki;
            /* 确保不回退到反向 */
            if (pid->integral < pid->integral_min)
                pid->integral = pid->integral_min;
        }
        output = pid->out_max;
    }
    if (output < pid->out_min) {
        if (i_term < 0.0f) {
            pid->integral -= (output - pid->out_min) / pid->Ki;
            if (pid->integral > pid->integral_max)
                pid->integral = pid->integral_max;
        }
        output = pid->out_min;
    }

    return output;
}

void PID_Clear_Integral(PID_t *pid)
{
    if (pid == 0) return;
    pid->integral   = 0.0f;
    pid->last_error = 0.0f;
}
