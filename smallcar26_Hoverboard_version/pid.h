#ifndef PID_H
#define PID_H

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float last_error;
    float out_min, out_max;           /* 输出限幅 */
    float integral_max, integral_min; /* 积分独立限幅 */
    float integral_decay;             /* 积分衰减: 1.0=纯积分, 0.99=漏积分(防偏置累积) */
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd,
               float out_min, float out_max);
float PID_Update(PID_t *pid, float target, float actual, float dt);
void  PID_Clear_Integral(PID_t *pid);

#endif /* PID_H */
