#include "gray_sensor.h"

volatile uint8_t gray[8] = {0};
static float last_deviation = 0.0f;
static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

// 声明你的 Digtal 变量（来自 No_Mcu_Ganv_* 库）
extern uint16_t Digtal;

void Gray_Init(void)
{
    for(int i=0; i<8; i++) gray[i] = 0;
    last_deviation = 0.0f;
}

void Update_Gray_Sensors(void)
{
    // 将 Digtal 的每一位存入 gray 数组 (假设 bit0 为传感器0)
    for(int i = 0; i < 8; i++) {
        gray[i] = ((Digtal >> i) & 0x01)^1;
    }
}

float Gray_GetDeviation(void)
{
    int sum_weights = 0;
    int sum_values = 0;
    
    for(int i = 0; i < 8; i++) {
        if(gray[i]) {
            sum_weights += weights[i];
            sum_values++;
        }
    }
    
    if(sum_values == 0) {
        // 全白：保持上一次偏差
        return 0.0f;
    } else if(sum_values == 8) {
        // 全黑：返回0
        return 0.0f;
    } else {
        last_deviation = (float)sum_weights / sum_values;
        return last_deviation;
    }
}