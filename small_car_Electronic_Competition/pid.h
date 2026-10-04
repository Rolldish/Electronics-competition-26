#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float out_min, out_max;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
float PID_Update(PID_t *pid, float target, float actual, float dt);
// 在 pid.h 的函数声明区域添加
void PID_Clear_Integral(PID_t *pid);
#endif