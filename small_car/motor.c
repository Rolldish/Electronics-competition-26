#include "main.h"
#include "motor.h"

// //速度环pi参数
// float Velocity_Kp = 0.1f;
// float Velocity_Ki = 0.4f;

int limit_PWM(int value, int low, int high)
{
    if(value>high) return high;
    else if(value<low) return low;
    else return value;
}




void  Set_PWM(int pwmA, int pwmB)
{   //左轮方向+转速控制
    if(pwmA>0)
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN2_PIN);
        DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST, abs(pwmA),GPIO_PWM_0_C0_IDX);
    }
     else
    {
        DL_GPIO_setPins(AIN_PORT,AIN_AIN1_PIN);
        DL_GPIO_clearPins(AIN_PORT,AIN_AIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,abs(pwmA),GPIO_PWM_0_C0_IDX);
    }

    // 右轮方向+转速控制
    if(pwmB>0)
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN2_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN1_PIN);
        DL_Timer_setCaptureCompareValue(PWM_0_INST,abs(pwmB),GPIO_PWM_0_C1_IDX);
    }
    else
    {
		DL_GPIO_setPins(BIN_PORT,BIN_BIN1_PIN);
        DL_GPIO_clearPins(BIN_PORT,BIN_BIN2_PIN);
		DL_Timer_setCaptureCompareValue(PWM_0_INST,abs(pwmB),GPIO_PWM_0_C1_IDX);
    }
}



int Velocity_A(int TargetVelocity, int CurrentVelocity)
{
    int Bias;
    static int ControlVelocityA,Last_biasA;

    Bias=TargetVelocity-CurrentVelocity;
    ControlVelocityA+=Velocity_Ki*(Bias-Last_biasA)+Velocity_Kp*Bias;
    Last_biasA=Bias;

    ControlVelocityA = limit_PWM(CurrentVelocity, PWM_MIN_LIMIT,PWM_MIN_LIMIT);
    return ControlVelocityA;

}

int Velocity_B(int TargetVelocity, int CurrentVelocity)
{
    int Bias;
    static int ControlVelocityB, Last_biasB;
    Bias = TargetVelocity - CurrentVelocity;
    ControlVelocityB += Velocity_Kp * Bias + Velocity_Ki * (Bias - Last_biasB);
    Last_biasB = Bias;
    return limit_PWM(ControlVelocityB, PWM_MIN_LIMIT, PWM_MAX_LIMIT);
}
void Motor_Init(void)
{
    DL_GPIO_clearPins(AIN_PORT, AIN_AIN1_PIN | AIN_AIN2_PIN);
    DL_GPIO_clearPins(BIN_PORT, BIN_BIN1_PIN | BIN_BIN2_PIN);
    Set_PWM(0,0);
}