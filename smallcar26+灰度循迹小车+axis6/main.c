#include "ti_msp_dl_config.h"
#include "delay.h"
#include "oled.h"
#include "axis6.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "control.h"
#include "gray_sensor.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include <stdio.h>

/* ========== 常量 ========== */
#define OLED_REFRESH_MS    200    /* OLED 刷新周期 (ms) */
#define IMU_UPDATE_MS        5    /* IMU 更新周期 (ms) */

/* ========== 全局变量 ========== */
volatile uint32_t g_tick_ms       = 0;   /* 毫秒计数器 (SysTick 驱动) */
volatile uint32_t timer_interrupt_cnt = 0; /* 10ms 定时器计数 */

/* --- 灰度传感器 --- */
unsigned short white[8] = {1800,1800,1800,1800,1800,1800,1800,1800};
unsigned short black[8] = { 300, 300, 300, 300, 300, 300, 300, 300};
No_MCU_Sensor sensor;
unsigned char Digtal;

/* --- control.c 需要的全局变量 --- */
float    yawAngle             = 0.0f;
float    target_angle_0       = 0.0f;
float    target_angle_1       = -178.0f;
volatile uint8_t motor_started        = 0;
volatile uint8_t mode_stautes_flag    = 0;
// white_stop_threshold 在 control.c 中定义, 此处用 extern
extern volatile uint8_t white_stop_threshold;

/* --- IMU 姿态数据 --- */
static IMU_Attitude att;

/* --- OLED 字符串缓存 --- */
static char oled_buf[32];

/* ========== 主程序 ========== */
int main(void)
{
    /* ---- 1. 硬件初始化 ---- */
    SYSCFG_DL_init();
    __enable_irq();

    /* ---- 2. OLED 初始化 ---- */
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, (uint8_t *)"Track Car Init");
    OLED_Refresh_Gram();

    /* ---- 3. AXIS6 IMU 初始化 ---- */
    if (!AXIS6_Init()) {
        OLED_Clear();
        OLED_ShowString(0, 0,  (uint8_t *)"AXIS6 FAIL!");
        OLED_ShowString(0, 24, (uint8_t *)"Check PA0/PA1");
        OLED_Refresh_Gram();
        while (1) { __WFE(); }
    }
    IMU_Init(0.00f);   /* alpha=0: 直接用 AXIS6 角度 */

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

    /* ---- 4. 控制初始化 (PID, 编码器, 电机, 灰度) ---- */
    Control_Init();

    /* ---- 5. 使能外设中断 ---- */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);

    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* ---- 6. 启动 PWM 和 10ms 定时器 ---- */
    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(TIMER_0_INST);

    /* ---- 7. 灰度传感器初始化 ---- */
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    /* ---- 8. 上电自动启动 ---- */
    Tick_delay(100);
    motor_started = 1;

    OLED_Clear();
    OLED_ShowString(0, 0,  (uint8_t *)"TRACK CAR");
    OLED_ShowString(0, 24, (uint8_t *)"RUNNING...");
    OLED_Refresh_Gram();

    /* ---- 9. 主循环 ---- */
    uint32_t last_imu  = g_tick_ms;
    uint32_t last_oled = g_tick_ms;

    while (1) {
        uint32_t now = g_tick_ms;

        /* --- IMU 更新 (5ms) --- */
        if (now - last_imu >= IMU_UPDATE_MS) {
            last_imu = now;
            IMU_Update(&att, 0.005f);   /* 5ms 更新周期 */
            yawAngle = att.yaw;         /* 喂给 control.c */
        }

        /* --- 灰度传感器采样任务 --- */
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        Digtal = Get_Digtal_For_User(&sensor);

        /* --- OLED 刷新 (200ms) --- */
        if (now - last_oled >= OLED_REFRESH_MS) {
            last_oled = now;

            OLED_Clear();

            /* 显示 Yaw 角度 */
            sprintf(oled_buf, "YAW: %6.1f", (double)yawAngle);
            OLED_ShowString(0, 0, (uint8_t *)oled_buf);

            /* 显示模式 */
            sprintf(oled_buf, "Mode: %d", (mode_stautes_flag % 4) + 1);
            OLED_ShowString(0, 12, (uint8_t *)oled_buf);

            /* 显示目标速度 */
            extern volatile float debug_target_left;
            extern volatile float debug_target_right;
            sprintf(oled_buf, "TL:%.1f TR:%.1f",
                (double)debug_target_left, (double)debug_target_right);
            OLED_ShowString(0, 24, (uint8_t *)oled_buf);

            /* 显示编码器速度 */
            extern volatile float debug_actual_left;
            extern volatile float debug_actual_right;
            sprintf(oled_buf, "AL:%.1f AR:%.1f",
                (double)debug_actual_left, (double)debug_actual_right);
            OLED_ShowString(0, 36, (uint8_t *)oled_buf);

            /* 显示状态 */
            extern volatile int32_t debug_run_state;
            sprintf(oled_buf, "ST:%ld", (long)debug_run_state);
            OLED_ShowString(0, 48, (uint8_t *)oled_buf);

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

/* ========== 10ms 定时器中断 ========== */
void TIMER_0_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    timer_interrupt_cnt++;
    Control_Update();   /* 电机 PID 状态机更新 */
}
