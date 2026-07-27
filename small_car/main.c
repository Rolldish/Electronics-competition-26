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
static uint32_t current_time = 0;
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

#define DEBOUNCE_MS     20
#define START_WINDOW_MS 1500
// 在 control.c 中定义（替代宏）

float target_angle_0 = 0.0f;
float target_angle_1 = -178.0f;
/* ================= 2. DMA 专属接收区 ================= */
#define RX_LEN_MAX  (33 * 2) 
volatile uint8_t rxData[RX_LEN_MAX]; // DMA自动装载的数组
float yawAngle = 0.0f;               // 存放解析出的偏航角
char oled_buf[32];                   // OLED 字符串缓存
/* ================= 3. 辅助函数 ================= */
// 读取按钮电平
static uint8_t read_button(void) {
    return (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_BUTTON_PIN) == 0);
}
static uint8_t read_Mode(void) {
    return (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_Mode_PIN) == 0);
}
// 处理按钮按下
void handle_button_press(void) {
    white_stop_threshold += 2;
    if (white_stop_threshold > 255) white_stop_threshold = 255;
    if (stop_flag) {
        Control_ResetAfterStop();
        motor_started = 0;
        start_window_active = 1;
        window_start_time = timer_interrupt_cnt;
        run_state = TRACKING;
        debug_run_state = 0;
    } else if (!motor_started) {
        if (!start_window_active) {
            start_window_active = 1;
            window_start_time = timer_interrupt_cnt;
        } else {
            window_start_time = timer_interrupt_cnt;
        }
    }
}
// 处理按钮按下
void handle_Mode_press(void) {
    mode_stautes_flag +=1;
    if(mode_stautes_flag%4+1 ==3){
        target_angle_0  =  -35.0f;
        target_angle_1  = -143.0f;
        turn_p = 0.15f;
        BASE_RPS = 1.5f; 
        STRAIGHT_SPEED_RPS    =  2.0f;
        
    }
    else if(mode_stautes_flag%4+1 == 1){
        target_angle_0  =  0.0f;
        target_angle_1  = -178.0f;
        turn_p = 0.15f;
        BASE_RPS = 1.5f; 
        STRAIGHT_SPEED_RPS    =  2.0f;
    }
    else if(mode_stautes_flag%4+1 == 2){
        target_angle_0  =  0.0f;
        target_angle_1  = -178.0f;
        turn_p = 0.1;   //转角速度
        BASE_RPS = 1.2f; //循迹速度
        STRAIGHT_SPEED_RPS = 1.0f;

    }
    else if(mode_stautes_flag%4+1 ==4){
        target_angle_0  =  -35.0f;
        target_angle_1  = -145.0f;
        turn_p = 0.1;   //转角速度
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
    /* --- 【核心！】启动 DMA 自动搬运工 --- */
    // 注意检查 DMA_UART_JY61P_CHAN_ID 和 UART_JY61P_INST 的宏名字是否和你 SysConfig 里匹配
    DL_DMA_setSrcAddr(DMA, DMA_UART_JY61P_CHAN_ID, (uint32_t)(&UART_JY61P_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_UART_JY61P_CHAN_ID, (uint32_t)&rxData[0]); 
    DL_DMA_enableChannel(DMA, DMA_UART_JY61P_CHAN_ID); 
    /* -------------------------------------- */
    
    // 启动定时器与传感器
    DL_Timer_startCounter(PWM_0_INST);
    DL_Timer_startCounter(TIMER_0_INST);
    No_MCU_Ganv_Sensor_Init(&sensor, white, black);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    
    Tick_delay(100);
    // 解析帧头的标志位
    bool hasFoundFrameHead = false; 
    uint8_t FrameHeadIndex[3] = {0}; 
    /* === 进入超级顺滑的主循环 === */
    while (1) {
        current_time = timer_interrupt_cnt;
        // ----------------- [一] DMA 数据暴力解析 -----------------
        bool found_angle = false; // 每次循环先假设没找到
        
        // 遍历整个 rxData 数组（留出 11 个字节的余量，防止越界）
        for (int i = 0; i < RX_LEN_MAX - 11; i++) { 
            // 目标明确：只找 0x55 0x53 (角度包)
            if (rxData[i] == 0x55 && rxData[i+1] == 0x53) { 
                
                // 找到了！角度包的第 6 字节是 Yaw 的低 8 位，第 7 字节是 Yaw 的高 8 位
                int16_t angleZ_int16 = rxData[i + 6] | (rxData[i + 7] << 8);
                
                // 计算真实角度
                yawAngle = (float)angleZ_int16 * 180.0f / 32768.0f;
                
                found_angle = true; // 打上标记：找到了！
                break; // 找一个就够了，直接跳出 for 循环，节省 CPU 算力
            }
        }
        // ----------------- [二] OLED 智能诊断与刷新 (每100ms) -----------------
        if (timer_interrupt_cnt - last_oled_time >= 10) { 
            last_oled_time = timer_interrupt_cnt;
            OLED_Clear(); 
            
            // --- 第一行显示 (y = 0) ---
            // 显示 "Circle:" 及其数值
            OLED_ShowString(0, 0, (uint8_t*)"Circle:");
            // 假设 "Circle:" 占用约 48 到 50 个像素宽度，数值从 x=64 开始显示是合理的
            OLED_ShowNumber(64, 0, ((white_stop_threshold - 1) / 2), 1, 12);
            // --- 第二行显示 (y = 16) ---
            if (found_angle) {
                // 如果找到了 55 53，就显示角度（保留两位小数）
                sprintf(oled_buf, "YAW: %6.2f", yawAngle);
            } else {
                // 如果屏幕显示这个，说明传感器吐出的数据里根本没有 55 53！
                sprintf(oled_buf, "NO 55 53!!"); 
            }
            // 在 x=0, y=16 的位置显示格式化好的 YAW 字符串
            OLED_ShowString(0, 16, (uint8_t*)oled_buf);
            OLED_ShowString(0, 30, (uint8_t*)"Mode:");
            OLED_ShowNumber(40, 30,mode_stautes_flag%4+1 ,1,12);
            // 如果你需要调试原始数据，可以放在第三行 (y = 32)
            // sprintf(oled_buf, "R:%02X %02X %02X %02X", rxData[0], rxData[1], rxData[2], rxData[3]);
            // OLED_ShowString(0, 32, (uint8_t*)oled_buf);
            
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
        if (start_window_active && !motor_started) {
            uint32_t elapsed_ms = (timer_interrupt_cnt - window_start_time) * 10;
            if (elapsed_ms >= START_WINDOW_MS) {
                start_window_active = 0;
                motor_started = 1;
                run_state = TRACKING;
                debug_run_state = 0;
            }
        }
        if(Mode_press_flag){
            Mode_press_flag=0;
            Tick_delay(DEBOUNCE_MS);
            if(read_Mode()){
                handle_Mode_press();
            }
            
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