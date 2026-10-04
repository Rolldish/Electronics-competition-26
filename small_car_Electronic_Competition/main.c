#include "ti_msp_dl_config.h"
#include "delay.h"
#include <stdio.h>   // 用于 sprintf                                                                                                                                                                             
#include <stdbool.h> // 用于 bool
#include "Uart.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"
#include "main.h"
#include "motor.h"
#include "encoder.h"
#include "gray_sensor.h"
#include "control.h"
#include "oled.h"
// 注意：这里我们不再需要 #include "jy61p.h" 了，全靠 DMA 解析
/* ================= 1. 全局变量定义 ================= */
unsigned short Anolog[8]={0};
unsigned short white[8]={1800,1800,1800,1800,1800,1800,1800,1800};
unsigned short black[8]={300,300,300,300,300,300,300,300};
unsigned short Normal[8];
unsigned char rx_buff[256]={0};
// 初始化传感器
No_MCU_Sensor sensor;
unsigned char Digtal;
// 全局变量
int32_t encoderA_cnt = 0, encoderB_cnt = 0;
int32_t PWMA = 0, PWMB = 0;
uint8_t Flag_Stop = 0; 
volatile uint32_t timer_interrupt_cnt = 0;
static uint32_t last_oled_time = 0;
extern volatile float debug_target_left;
extern volatile float debug_target_right;
extern volatile float debug_actual_left;
extern volatile float debug_actual_right;
extern volatile float debug_pwm_right;
extern volatile float debug_pwm_left;
extern float turn_p;
volatile uint8_t button_press_flag = 0;
volatile uint8_t Mode_press_flag = 0;
volatile uint8_t motor_started = 0;
static volatile uint8_t start_window_active = 0;
static volatile uint32_t window_start_time = 0;
extern volatile uint8_t white_stop_threshold;
extern volatile uint8_t white_event_count;
extern volatile uint8_t stop_flag;
extern volatile uint8_t first_white_handled;
extern volatile RunState run_state;
extern volatile int32_t debug_run_state;
extern void Control_ResetAfterStop(void);

volatile uint8_t mode_stautes_flag = 0;
extern volatile float BASE_RPS;
extern float STRAIGHT_SPEED_RPS;
extern volatile float MODE1_SPEED_RPS;  // 模式1 循迹速度
extern volatile float MODE2_SPEED_RPS;  // 模式2 循迹速度
extern volatile float MODE3_SPEED_RPS;  // 模式3 定时循迹速度
extern volatile uint32_t mode1_elapsed_ms; // 模式1 运行时间
extern volatile uint32_t mode2_elapsed_ms; // 模式2 运行时间
extern volatile uint32_t mode3_elapsed_ms; // 模式3 运行时间

#define DEBOUNCE_MS     20
#define START_WINDOW_MS 2000   // 上电后 2s 自动启动电机
// 在 control.c 中定义（替代宏）

float target_angle_0 = 0.0f;
float target_angle_1 = -178.0f;
/* ================= 2. JY61P DMA 接收区 (已注释) ================= */
// #define RX_LEN_MAX  (33 * 2)
// volatile uint8_t rxData[RX_LEN_MAX]; // DMA自动装载的数组
// float yawAngle = 0.0f;               // 存放解析出的偏航角
char oled_buf[32];                   // OLED 字符串缓存
volatile uint8_t button_cnt = 0;      // PB8 按键计数器
/* ================= 3. 辅助函数 ================= */
// 读取按钮电平
static uint8_t read_button(void) {
    return (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_BUTTON_PIN) == 0);
}
static uint8_t read_Mode(void) {
    return (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_Mode_PIN) == 0);
}
static uint8_t read_pb8(void) {
    return (DL_GPIO_readPins(COUNT_BTN_PORT, COUNT_BTN_BUTTON_CNT_PIN) == 0);
}
// 处理按钮按下：清除停止标志，恢复当前模式的循迹
void handle_button_press(void) {
    if (stop_flag) {
        Control_ResetAfterStop();   // 清除 stop_flag，PID 积分归零，模式状态重置
    }
}
// 处理 MODE 按键：循环切换 4 档参数预设（速度 + 转角灵敏度）
void handle_Mode_press(void) {
    mode_stautes_flag +=1;
    if(mode_stautes_flag%4+1 ==3){
        target_angle_0  =  -35.0f;
        target_angle_1  = -143.0f;
        turn_p = 0.15f;
        MODE1_SPEED_RPS = 1.5f;
        MODE2_SPEED_RPS = 1.5f;
        MODE3_SPEED_RPS = 2.0f;
        BASE_RPS = 1.5f;
        STRAIGHT_SPEED_RPS = 2.0f;

    }
    else if(mode_stautes_flag%4+1 == 1){
        target_angle_0  =  0.0f;
        target_angle_1  = -178.0f;
        turn_p = 0.15f;
        MODE1_SPEED_RPS = 1.5f;
        MODE2_SPEED_RPS = 1.5f;
        MODE3_SPEED_RPS = 2.0f;
        BASE_RPS = 1.5f;
        STRAIGHT_SPEED_RPS = 2.0f;
    }
    else if(mode_stautes_flag%4+1 == 2){
        target_angle_0  =  0.0f;
        target_angle_1  = -178.0f;
        turn_p = 0.1;   //转角速度
        MODE1_SPEED_RPS = 1.2f;
        MODE2_SPEED_RPS = 1.2f;
        MODE3_SPEED_RPS = 1.8f;
        BASE_RPS = 1.2f; //循迹速度
        STRAIGHT_SPEED_RPS = 1.0f;

    }
    else if(mode_stautes_flag%4+1 ==4){
        target_angle_0  =  -35.0f;
        target_angle_1  = -145.0f;
        turn_p = 0.1;   //转角速度
        MODE1_SPEED_RPS = 1.4f;
        MODE2_SPEED_RPS = 1.4f;
        MODE3_SPEED_RPS = 2.0f;
        BASE_RPS = 1.4f; //循迹速度
        STRAIGHT_SPEED_RPS = 1.0f;//全白直行速度

    }

}
/* ================= 4. 主程序 ================= */
int main(void)
{   
    // 1. 初始化底层
    SYSCFG_DL_init();
    
    // 2. 优先开启全局中断
    __enable_irq(); 
    // 3. 初始化 OLED 与 电机控制
    OLED_Init();
    OLED_Clear();
    Control_Init();
    
    // 使能中断（编码器、定时器）
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    /* --- 【已注释】JY61P DMA 自动搬运 --- */
    // DL_DMA_setSrcAddr(DMA, DMA_UART_JY61P_CHAN_ID, (uint32_t)(&UART_JY61P_INST->RXDATA));
    // DL_DMA_setDestAddr(DMA, DMA_UART_JY61P_CHAN_ID, (uint32_t)&rxData[0]);
    // DL_DMA_enableChannel(DMA, DMA_UART_JY61P_CHAN_ID);
    /* -------------------------------------- */
    
    // 启动定时器与传感器
    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(TIMER_0_INST);
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

    // 模式控制：由 PB8 按键 (button_cnt) 决定运动状态，上电默认 mode=0 静止
    // start_window_active = 0;  // 不再自动启动

    Tick_delay(100);
    /* === 进入主循环 === */
    while (1) {
        // ----------------- [二] OLED 显示刷新 (每100ms) -----------------
        if (timer_interrupt_cnt - last_oled_time >= 10) {
            last_oled_time = timer_interrupt_cnt;
            OLED_Clear();

            // 第一行：左轮编码器
            sprintf(oled_buf, "L_Enc:%ld", (long)Get_Encoder_countA);
            OLED_ShowString(0, 0, (uint8_t*)oled_buf);

            // 第二行：右轮编码器
            sprintf(oled_buf, "R_Enc:%ld", (long)Get_Encoder_countB);
            OLED_ShowString(0, 16, (uint8_t*)oled_buf);

            // 第三行：灰度传感器 (8路, 0=黑 1=白)
            sprintf(oled_buf, "GS:%u%u%u%u%u%u%u%u",
                    (unsigned int)((Digtal >> 7) & 1),
                    (unsigned int)((Digtal >> 6) & 1),
                    (unsigned int)((Digtal >> 5) & 1),
                    (unsigned int)((Digtal >> 4) & 1),
                    (unsigned int)((Digtal >> 3) & 1),
                    (unsigned int)((Digtal >> 2) & 1),
                    (unsigned int)((Digtal >> 1) & 1),
                    (unsigned int)((Digtal >> 0) & 1));
            OLED_ShowString(0, 32, (uint8_t*)oled_buf);

            // 第四行：当前模式 + 状态 + 运行时间
            {
                uint8_t mode = button_cnt % 4;
                char *mode_name;
                uint32_t elapsed;
                if (mode == 0) {
                    mode_name = "STOP";
                    elapsed = 0;
                } else if (mode == 1) {
                    mode_name = "TRK1";
                    elapsed = mode1_elapsed_ms;
                } else if (mode == 2) {
                    mode_name = "TRK2";
                    elapsed = mode2_elapsed_ms;
                } else {
                    mode_name = "TIME";
                    elapsed = mode3_elapsed_ms;
                }
                // 格式: "M1:TRACK  12.3s" 或 "M1 STOP!  12.3s"
                if (stop_flag) {
                    sprintf(oled_buf, "M%u STOP! %lu.%lus",
                            (unsigned int)mode,
                            (unsigned long)(elapsed / 1000),
                            (unsigned long)((elapsed / 100) % 10));
                } else {
                    sprintf(oled_buf, "M%u:%-5s %lu.%lus",
                            (unsigned int)mode, mode_name,
                            (unsigned long)(elapsed / 1000),
                            (unsigned long)((elapsed / 100) % 10));
                }
            }
            OLED_ShowString(0, 48, (uint8_t*)oled_buf);

            OLED_Refresh_Gram();
        }
        // ----------------- [三] 按键与启停控制逻辑 -----------------
        if (button_press_flag) {
            button_press_flag = 0;
            Tick_delay(DEBOUNCE_MS);
            if (read_button()) {
                handle_button_press();
            }
        }
        // （自动启动已禁用，由 PB8 按键切换模式控制）
        if(Mode_press_flag){
            Mode_press_flag=0;
            Tick_delay(DEBOUNCE_MS);
            if(read_Mode()){
                handle_Mode_press();
            }

        }
        // ----------------- PB8 按键计数（下降沿触发 + 消抖）-----------------
        {
            static uint8_t last_pb8 = 1;  // 上拉，默认高电平
            uint8_t pb8_now = read_pb8() ? 0 : 1;  // 按下为低电平 → 0
            if (last_pb8 == 1 && pb8_now == 0) {
                Tick_delay(DEBOUNCE_MS);
                if (read_pb8()) {
                    /* ===== 修改部分：PB8切换为0→1→2→3→0四模式循环 ===== */
                    button_cnt = (button_cnt + 1) % 4;
                    /* ===== 修改部分结束：四模式循环 ===== */
                }
            }
            last_pb8 = pb8_now;
        }

        // ----------------- [四] 灰度传感器任务 -----------------
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
        Digtal = Get_Digtal_For_User(&sensor);
        
    }
}
/* ================= 5. 定时器中断 (你的电机大脑) ================= */
void TIMER_0_INST_IRQHandler(void)
{
    DL_Timer_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    timer_interrupt_cnt++;
    Control_Update(); // 电机 PID 更新
}
