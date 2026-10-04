#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Update(PID_t *pid, float target, float actual, float dt)
{
    float error = target - actual;
    
    // 积分累积并限幅
    pid->integral += error * dt;
    if(pid->integral > pid->out_max) pid->integral = pid->out_max;
    if(pid->integral < pid->out_min) pid->integral = pid->out_min;
    
    // 微分
    float derivative = (error - pid->last_error) / dt;
    
    // 输出
    float output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;
    
    // 保存误差
    pid->last_error = error;
    
    // 输出限幅
    if(output > pid->out_max) output = pid->out_max;
    if(output < pid->out_min) output = pid->out_min;
    
    return output;
}

void PID_Clear_Integral(PID_t *pid)
{
    // 安全检查，防止传入空指针
    if (pid == 0) {
        return;
    }
    
    // 清空积分累计项
    pid->integral = 0.0f;
    
    // 同时清空上一次的误差，防止在状态切换瞬间计算微分 (D项) 时产生巨大的输出尖峰
    pid->last_error = 0.0f;
}