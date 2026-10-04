#ifndef GRAY_SENSOR_H
#define GRAY_SENSOR_H

#include <stdint.h>

// 声明数组，由你在 Update_Gray_Sensors 中填充
extern volatile uint8_t gray[8];

void Gray_Init(void);
void Update_Gray_Sensors(void);  
float Gray_GetDeviation(void);   // 返回中心对称加权偏差 (-7 ~ +7)

#endif