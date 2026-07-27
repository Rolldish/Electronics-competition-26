#include "delay.h"
#include "ti_msp_dl_config.h"   /* CPUCLK_FREQ */

volatile uint32_t Tick = 0;

void delay_us(uint32_t us) {
    /* 使用硬件精确延时 (80MHz → 80 cycles/μs) */
    delay_cycles((CPUCLK_FREQ / 1000000UL) * us);
}

void Tick_delay(uint32_t t) {
    uint32_t tEnd = Tick + t;
    while (Tick < tEnd);
}
