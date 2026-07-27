#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "axis6.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "balance.h"
#include <stdio.h>

/* ========== 常量 ========== */
#define OLED_REFRESH_MS    200    /* OLED 刷新周期 (ms) */
#define BALANCE_PERIOD_MS    5    /* 平衡控制周期 (ms) */

/* ========== 全局变量 ========== */
volatile uint32_t g_tick_ms    = 0;   /* 毫秒计数器 (SysTick 驱动) */
volatile bool     g_balance_trig = false;  /* 平衡控制触发标志 */

/* ========== 主程序 ========== */
int main(void)
{
    /* ---- 1. 硬件初始化 ---- */
    SYSCFG_DL_init();
    __enable_irq();

    /* ---- 2. I2C 使用 GPIO 模拟 (PA0=SDA, PA1=SCL), 无需硬件 I2C 中断 ---- */

    /* ---- 3. OLED 初始化 ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Bal Car Init...");
    OLED_Refresh_Gram();

    /* ---- 4. AXIS6 初始化 ---- */
    if (!AXIS6_Init()) {
        OLED_Clear();
        OLED_ShowString(0, 0,  (uint8_t *)"AXIS6 FAIL!");
        /* 显示错误步骤码: 1=Unlock, 2=Cmd, 3=Read */
        {
            char ebuf[12];
            ebuf[0] = 'E'; ebuf[1] = 'R'; ebuf[2] = 'R'; ebuf[3] = ':';
            ebuf[4] = '0' + axis6_error;
            ebuf[5] = '\0';
            OLED_ShowString(0, 16, (uint8_t *)ebuf);
        }
        OLED_ShowString(0, 32, (uint8_t *)"Check HW:");
        OLED_ShowString(0, 40, (uint8_t *)"PA0/PA1 pullup");
        OLED_Refresh_Gram();
        while (1) { __WFE(); }
    }

    /* ---- 5. IMU 初始化 + 校准 ---- */
    IMU_Init(0.00f);   /* alpha=0: 直接用 AXIS6 角度, 最小延迟 */

    OLED_Clear();
    OLED_ShowString(0, 0,  (uint8_t *)"Calibrating...");
    OLED_ShowString(0, 24, (uint8_t *)"Keep STILL!");
    OLED_Refresh_Gram();

    if (!IMU_CalibrateGyro(100)) {
        OLED_Clear();
        OLED_ShowString(0, 0,  (uint8_t *)"Gyro Cal FAIL!");
        OLED_Refresh_Gram();
        while (1) { __WFE(); }
    }

    /* ---- 6. 平衡控制初始化 ---- */
    Balance_Init();

    /* ---- 7. 使能外设中断 ---- */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);

    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);

    /* ---- 8. 启动 PWM ---- */
    DL_Timer_startCounter(PWM_0_INST);
    Balance_Start();

    OLED_Clear();
    OLED_ShowString(0, 0,  (uint8_t *)"BALANCE CAR");
    OLED_ShowString(0, 24, (uint8_t *)"RUNNING...");
    OLED_Refresh_Gram();

    /* ---- 9. 主循环: 平衡控制 + OLED 刷新 ---- */
    uint32_t last_balance = g_tick_ms;
    uint32_t last_oled    = g_tick_ms;

    while (1) {
        uint32_t now = g_tick_ms;

        /* ----- 平衡控制 (5ms) — 在主循环中执行, 不在 ISR 中 ----- */
        if (now - last_balance >= BALANCE_PERIOD_MS) {
            last_balance = now;
            Balance_Update();
        }

        /* ----- OLED 刷新 (200ms) ----- */
        if (now - last_oled >= OLED_REFRESH_MS) {
            last_oled = now;

            char buf[21];  /* OLED 一行最多 16 字符, 21 足够安全 */
            OLED_Clear();

            if (balance_debug.state == BALANCE_RUNNING) {
                OLED_ShowString(0, 0, (uint8_t *)"ST: RUN");
            } else {
                switch (balance_debug.stop_reason) {
                    case STOP_REASON_IMU:
                        OLED_ShowString(0, 0, (uint8_t *)"ST: IMU FAIL");
                        break;
                    case STOP_REASON_ANGLE:
                        OLED_ShowString(0, 0, (uint8_t *)"ST: ANGLE >60");
                        break;
                    case STOP_REASON_USER:
                        OLED_ShowString(0, 0, (uint8_t *)"ST: USER");
                        break;
                    default:  /* STOP_REASON_INIT / NONE */
                        OLED_ShowString(0, 0, (uint8_t *)"ST: INIT");
                        break;
                }
            }

            /* 俯仰角 (显示整数 + 1 位小数, 避免 sprintf %f 占用过多栈) */
            {
                int   p_int = (int)balance_debug.pitch;
                int   p_dec = (int)((balance_debug.pitch - (float)p_int) * 10.0f);
                if (p_dec < 0) p_dec = -p_dec;
                if (p_int == 0 && balance_debug.pitch < 0) {
                    /* 处理 -0.x 情况 */
                }
                /* 手动格式化避免 sprintf 浮点开销 */
                uint8_t idx = 0;
                buf[idx++] = 'P';
                buf[idx++] = ':';
                if (balance_debug.pitch < 0 && p_int == 0) buf[idx++] = '-';
                if (p_int < 0) { buf[idx++] = '-'; p_int = -p_int; }
                if (p_int >= 100) buf[idx++] = '0' + (p_int / 100) % 10;
                if (p_int >= 10)  buf[idx++] = '0' + (p_int / 10) % 10;
                buf[idx++] = '0' + (p_int % 10);
                buf[idx++] = '.';
                buf[idx++] = '0' + p_dec;
                buf[idx] = '\0';
            }
            OLED_ShowString(0, 12, (uint8_t *)buf);

            /* 角速度 */
            {
                int   r_int = (int)balance_debug.pitch_rate;
                uint8_t idx = 0;
                buf[idx++] = 'R';
                buf[idx++] = ':';
                if (r_int < 0) { buf[idx++] = '-'; r_int = -r_int; }
                if (r_int >= 1000) buf[idx++] = '0' + (r_int / 1000) % 10;
                if (r_int >= 100)  buf[idx++] = '0' + (r_int / 100) % 10;
                if (r_int >= 10)   buf[idx++] = '0' + (r_int / 10) % 10;
                buf[idx++] = '0' + (r_int % 10);
                buf[idx] = '\0';
            }
            OLED_ShowString(0, 22, (uint8_t *)buf);

            /* PWM */
            {
                uint8_t idx = 0;
                buf[idx++] = 'P';
                buf[idx++] = 'W';
                buf[idx++] = 'M';
                buf[idx++] = ':';
                int pwm = balance_debug.pwm_left;
                if (pwm < 0) { buf[idx++] = '-'; pwm = -pwm; }
                if (pwm >= 10000) buf[idx++] = '0' + (pwm / 10000) % 10;
                if (pwm >= 1000)  buf[idx++] = '0' + (pwm / 1000) % 10;
                if (pwm >= 100)   buf[idx++] = '0' + (pwm / 100) % 10;
                if (pwm >= 10)    buf[idx++] = '0' + (pwm / 10) % 10;
                buf[idx++] = '0' + (pwm % 10);
                buf[idx] = '\0';
            }
            OLED_ShowString(0, 32, (uint8_t *)buf);

            /* IMU 状态 */
            OLED_ShowString(0, 48,
                (balance_debug.pitch > -45.0f && balance_debug.pitch < 45.0f)
                    ? (uint8_t *)"IMU: OK"
                    : (uint8_t *)"IMU: OUT");

            OLED_Refresh_Gram();
        }

        __WFE();  /* 等待中断唤醒 */
    }
}

/* ========== SysTick 中断 (1ms) ========== */
void SysTick_Handler(void)
{
    g_tick_ms++;
    Tick = g_tick_ms;  /* 兼容旧代码中的 Tick 引用 */
}

/* ========== 旧定时器中断已废弃, 主循环替代 ========== */
void TIMER_0_INST_IRQHandler(void)
{
    /* 不再使用 — 平衡控制已移至主循环 */
    DL_Timer_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
}
