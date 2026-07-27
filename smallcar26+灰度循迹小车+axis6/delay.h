#ifndef delay_h
#define delay_h
#include "ti_msp_dl_config.h"
extern volatile uint32_t Tick;
void delay_us(uint32_t us);
void Tick_delay(uint32_t t);
void SysTick_Handler(void);
#endif
