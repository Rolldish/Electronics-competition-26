#include "encoder.h"
#include "main.h"

// 外部按钮标志（在 main.c 中定义）
extern volatile uint8_t button_press_flag;
extern volatile uint8_t Mode_press_flag;
// 静态变量保存上次计数值
static int32_t last_left_cnt = 0;
static int32_t last_right_cnt = 0;
// 转换系数：delta_cnt → rps
// 每转脉冲数 = 260 * 4 = 1040，周期 0.01s
#define TICKS_TO_RPS (1.0f / (740.0f * 0.01f))  // ≈ 0.0961538
#define LEFT_TICKS_TO_RPS   (1.0f / (740.0f * 0.01f))
#define RIGHT_TICKS_TO_RPS  (1.0f / (370.0f* 0.01f))
void Encoder_Init(void)
{
    // 清空计数（可选）
    Get_Encoder_countA = 0;
    Get_Encoder_countB = 0;
    last_left_cnt = 0;
    last_right_cnt = 0;
}
float Get_Speed_Left(void)
{
    int32_t current = Get_Encoder_countA;
#if ENCODER_LEFT_REVERSE
    current = -current;
#endif
    int32_t delta = current - last_left_cnt;
    last_left_cnt = current;
    return (float)delta *LEFT_TICKS_TO_RPS ;
}
float Get_Speed_Right(void)
{
    int32_t current = Get_Encoder_countB;
#if ENCODER_RIGHT_REVERSE
    current = -current;
#endif
    int32_t delta = current - last_right_cnt;
    last_right_cnt = current;
    return (float)delta * RIGHT_TICKS_TO_RPS;
}
void GROUP1_IRQHandler(void)
{    uint32_t statusA, statusB;
    //  ȡ ж  ź 
    statusA = DL_GPIO_getEnabledInterruptStatus(ENCODERA_PORT,ENCODERA_Mode_PIN|ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
    statusB = DL_GPIO_getEnabledInterruptStatus(ENCODERB_PORT,
                ENCODERB_E2A_PIN | ENCODERB_E2B_PIN | ENCODERB_BUTTON_PIN);
    
    
    //encoderA
    if(( statusA& ENCODERA_E1A_PIN)==ENCODERA_E1A_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1B_PIN))
        {
            Get_Encoder_countA++;
        }
        else
        {
            Get_Encoder_countA--;
        }
    }
    else if((statusA & ENCODERA_E1B_PIN)==ENCODERA_E1B_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERA_PORT,ENCODERA_E1A_PIN))
        {
            Get_Encoder_countA--;
        }
        else
        {
            Get_Encoder_countA++;
        }
    }
    
    //encoderB
    if((statusB & ENCODERB_E2A_PIN)==ENCODERB_E2A_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2B_PIN))
        {
            Get_Encoder_countB--;
        }
        else
        {
            Get_Encoder_countB++;
        }
    }
    else if((statusB & ENCODERB_E2B_PIN)==ENCODERB_E2B_PIN)
    {
        if(!DL_GPIO_readPins(ENCODERB_PORT,ENCODERB_E2A_PIN))
        {
            Get_Encoder_countB++;
        }                 
        else              
        {                 
            Get_Encoder_countB--;
        }
    
    
    }
     // ----- 处理按钮（与编码器B共用中断）-----
    if (statusB & ENCODERB_BUTTON_PIN) {
        button_press_flag = 1;   // 仅设置标志，具体处理在主循环中
    }
     // ----- 处理按钮（与编码器B共用中断）-----
    if (statusA & ENCODERA_Mode_PIN) {
        Mode_press_flag = 1;  
         // 仅设置标志，具体处理在主循环中
    }
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT,ENCODERA_E1A_PIN|ENCODERA_E1B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT,ENCODERB_E2A_PIN|ENCODERB_E2B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_BUTTON_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_Mode_PIN);
}
