#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "gray_sensor.h"
#include "motor.h"
#include "ti_msp_dl_config.h"
#include <math.h> 
// 定义全局变量（非 static）
RunState run_state = TRACKING;
extern volatile uint8_t motor_started;
static PID_t pid_left, pid_right;
static PID_t pid_angle;        // 角度 PID 控制器
volatile float debug_actual_left = 0.0f;
volatile float debug_actual_right = 0.0f;
volatile float debug_pwm_left = 0.0f;
volatile float debug_pwm_right = 0.0f;
volatile float debug_target_left = 0.0f;
volatile float debug_target_right = 0.0f;
volatile int32_t debug_run_state = 0;
extern float target_angle_0;
extern float target_angle_1 ;
extern volatile uint32_t timer_interrupt_cnt;
static uint8_t first_white_handled = 0;
static uint8_t white_detect_counter = 0;
static uint8_t white_event_count = 0;     // 全白事件次数
volatile uint8_t stop_flag = 0;             // 停止标志
volatile uint8_t white_stop_threshold = 1;
extern volatile uint8_t mode_stautes_flag;
float turn_p = 0.15f;

float BASE_RPS = 1.5f;      // 基本直道目标速度（圈/秒）
float STRAIGHT_SPEED_RPS    =  2.0f;  // 角度保持速度

// Straight 状态子状态
typedef enum {
    STRAIGHT_TURNING,   // 转向阶段（原地差速）
    STRAIGHT_HOLDING    // 保持阶段（角度 PID）
} StraightSubState;
static StraightSubState straight_substate = STRAIGHT_TURNING;
static uint8_t angle_settle_counter = 0;    // 角度稳定计数
static float target_angle = 0.0f;           // 当前目标角度
static uint8_t straight_entry_count = 0;    // 进入 STRAIGHT 的次数（用于交替角度）
// ========== 新增：DELAY_HOLD 状态相关变量 ==========
static uint32_t delay_hold_start_ticks = 0;     // 进入 DELAY_HOLD 时的 timer_interrupt_cnt
static float hold_target_left = 0.0f;           // 延时期间保持的左轮目标速度
static float hold_target_right = 0.0f;          // 延时期间保持的右轮目标速度
static uint8_t black_detect_counter = 0;        // 延时期间黑线检测防抖计数器
// 延时时间（单位：timer_interrupt_cnt 周期数，每个周期 10ms）
#define DELAY_HOLD_TICKS   (40U)   // 200ms 延时 (20 * 10ms)
// 延时期间黑线防抖阈值（连续多少次检测到黑线才认为真正出白区）
#define BLACK_DETECT_THRESHOLD   (2U)
// =================================================
extern float yawAngle;
void Control_Init(void)
{
    PID_Init(&pid_left, PID_LEFT_KP, PID_LEFT_KI, PID_LEFT_KD, PID_OUT_MIN, PID_OUT_MAX);
    PID_Init(&pid_right, PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD, PID_OUT_MIN, PID_OUT_MAX);
    PID_Init(&pid_angle, ANGLE_KP, ANGLE_KI, ANGLE_KD, -ANGLE_OUT_MAX, ANGLE_OUT_MAX);
    Gray_Init();
    Encoder_Init();
    Motor_Init();
    first_white_handled = 0;
    // 新增变量初始化
    delay_hold_start_ticks = 0;
    hold_target_left = hold_target_right = 0.0f;
    black_detect_counter = 0;
}
void Control_ResetAfterStop(void)
{
    white_event_count = 0;
    stop_flag = 0;
    first_white_handled = 0;
    white_detect_counter = 0;
    straight_entry_count = 0;
    // 新增变量复位
    delay_hold_start_ticks = 0;
    black_detect_counter = 0;
}
static uint8_t get_black_count(void)
{
    uint8_t cnt = 0;
    for (int i = 0; i < 8; i++) {
        if (gray[i]) cnt++;
    }
    return cnt;
}
static void no_gpio_output(void) { /* GPIO removed - no buzzer/LED */ }
static void start_gpio_output(void) { /* GPIO removed - no buzzer/LED */ }
// 计算角度误差（最短路径，输入角度范围 -180~180）
static float angle_error(float target, float current)
{
    float err = target - current;
    if (err > 180.0f) err -= 360.0f;
    else if (err < -180.0f) err += 360.0f;
    return err;
}
void Control_Update(void)
{   
    Update_Gray_Sensors();
    
    float actual_left = Get_Speed_Left();
    float actual_right = Get_Speed_Right();
    debug_actual_left = actual_left;
    debug_actual_right = actual_right;
    
    uint8_t black_cnt = get_black_count();
    float target_left, target_right;
    
    // handle_gpio_output removed - no buzzer/LED

    if (!motor_started) {
        Set_PWM(0, 0);
        return;
    }
    
    if (stop_flag) {
        Set_PWM(0, 0);
        debug_run_state = 99;
        return;
    }
    
    switch (run_state) {
        case TRACKING:
        {
            if (black_cnt == 0) {
                white_detect_counter++;
                if (white_detect_counter >= WHITE_DETECT_THRESHOLD) {
                    white_event_count++;
                    if (white_event_count >= white_stop_threshold) {
                        start_gpio_output();
                        stop_flag = 1;
                        Set_PWM(0, 0);
                        debug_run_state = 99;
                        return;
                    }
                    
                    // ========== 新增：判断是否启用延时 ==========
                    if (first_white_handled == 0) {
                        // 第一次全白（发车）：直接进入 STRAIGHT，不延时，保持原逻辑
                        run_state = STRAIGHT;
                        first_white_handled = 1;
                        yawAngle = 0.0f;
                        target_angle = target_angle_0;
                        straight_entry_count = 1;
                        straight_substate = STRAIGHT_TURNING;
                        angle_settle_counter = 0;
                        debug_run_state = 2;
                        PID_Clear_Integral(&pid_left);
                        PID_Clear_Integral(&pid_right);
                        PID_Clear_Integral(&pid_angle);
                    } else {
                        // 后续全白：进入 DELAY_HOLD 延时状态
                        run_state = DELAY_HOLD;
                        delay_hold_start_ticks = timer_interrupt_cnt;
                        // 保存进入全白前最后一刻的目标速度（可以使用当前速度环的目标值）
                        // 注意：此时 target_left/right 还未计算，需使用之前循迹周期保留的值
                        // 我们在 case TRACKING 末尾会计算 target，但这里是在检测到全白时立即进入，
                        // 因此需要提前保存上一次循迹计算出的目标速度。
                        // 简便方法：在这里重新基于当前偏差计算一次目标速度并保存
                        float deviation = Gray_GetDeviation();
                        float delta_rps = TURN_KP * deviation;
                        hold_target_left = BASE_RPS + delta_rps;
                        hold_target_right = BASE_RPS - delta_rps;
                        // // 限制速度范围（可选）
                        // if (hold_target_left < 0) hold_target_left = 0;
                        // if (hold_target_right < 0) hold_target_right = 0;
                        // if (hold_target_left > MAX_RPS) hold_target_left = MAX_RPS;
                        // if (hold_target_right > MAX_RPS) hold_target_right = MAX_RPS;
                        
                        black_detect_counter = 0;
                        debug_run_state = 3;  // 调试用，表示处于延时态
                    }
                    white_detect_counter = 0;
                    break;
                } else {
                    // 防抖等待期
                    if (first_white_handled == 0) {
                        target_left = 0.0f;
                        target_right = 0.0f;
                    } else {
                        target_left = STRAIGHT_SPEED_RPS;
                        target_right = STRAIGHT_SPEED_RPS;
                    }
                    break;
                }
            } else {
                white_detect_counter = 0;
            }
            
            // 正常循迹（黑线有效）
            float deviation = Gray_GetDeviation();
            float delta_rps = TURN_KP * deviation;
            target_left = BASE_RPS + delta_rps;
            target_right = BASE_RPS - delta_rps;
            break;
        } // ========== 新增：DELAY_HOLD 状态处理 ==========
        case DELAY_HOLD:
        {
            // 1. 检查是否重新检测到黑线（说明已经冲出白区或误触发）
            if (black_cnt > 0) {
                black_detect_counter++;
                if (black_detect_counter >= BLACK_DETECT_THRESHOLD) {
                    // 确实进入黑线区域，取消本次转向，回到循迹
                    run_state = TRACKING;
                    debug_run_state = 0;
                    start_gpio_output();
                    PID_Clear_Integral(&pid_left);
                    PID_Clear_Integral(&pid_right);
                    PID_Clear_Integral(&pid_angle);
                    white_detect_counter = 0;
                    black_detect_counter = 0;
                    // 重新计算循迹目标速度（使用当前偏差）
                    float deviation = Gray_GetDeviation();
                    float delta_rps = TURN_KP * deviation;
                    target_left = BASE_RPS + delta_rps;
                    target_right = BASE_RPS - delta_rps;
                    break;
                }
            } else {
                black_detect_counter = 0;
            }
            
            // 2. 检查延时是否结束
            uint32_t elapsed_ticks = timer_interrupt_cnt - delay_hold_start_ticks;
            if (elapsed_ticks >= DELAY_HOLD_TICKS) {
                // 延时结束，进入 STRAIGHT 转向阶段
                start_gpio_output();
                run_state = STRAIGHT;
                // 交替目标角度逻辑（与原来一致）
                if (straight_entry_count % 2 == 0)
                    target_angle = target_angle_0;
                else
                    target_angle = target_angle_1;
                straight_entry_count++;
                straight_substate = STRAIGHT_TURNING;
                angle_settle_counter = 0;
                debug_run_state = 2;
                PID_Clear_Integral(&pid_left);
                PID_Clear_Integral(&pid_right);
                PID_Clear_Integral(&pid_angle);
                // 注意：这里不需要立即设定 target_left/right，因为下面会根据 straight 子状态重新计算
                // 但我们仍需执行一次 straight 分支的速度计算，所以直接跳转到 straight 的代码？
                // 为了清晰，我们直接在此处接管控制：先不 break，而是让程序继续进入下面的 straight 处理？
                // 由于 switch 已匹配 case DELAY_HOLD，若不 break 会顺序执行到下一个 case？
                // 不安全，改用 goto 或重新组织逻辑。简单做法：直接给 target 赋值并 break，下一周期自然进入 straight。
                // 但为了行为一致，我们在这里调用 straight 的初始化后，立即设置目标速度为转向速度（0 或差速），
                // 然后 break，下一周期就会进入 STRAIGHT 状态。
                // 更简单：直接重新执行一遍 straight 子状态的处理？但这会导致代码重复。
                // 最干净的方法：将 straight 内部处理提取成函数，但为了代码独立，我们在延时结束时，
                // 手动设置转向期的目标速度（仿照原 straight_turning 起始逻辑）
                float current_angle = yawAngle;
                float angle_err = angle_error(target_angle, current_angle);
                if (fabsf(angle_err) <= 3.0f) {
                    target_left = 0.0f;
                    target_right = 0.0f;
                } else {
                    float turn_speed = fabsf(angle_err) * 0.2f;
                    if (turn_speed > OPENLOOP_TURN_SPEED) turn_speed = OPENLOOP_TURN_SPEED;
                    if (turn_speed < 0.15f) turn_speed = 0.15f;
                    if (angle_err > 0) {
                        target_left = -turn_speed;
                        target_right = turn_speed;
                    } else {
                        target_left = turn_speed;
                        target_right = -turn_speed;
                    }
                }
                break;  // 本周期按转向期速度运行，下周期进入 STRAIGHT 会正常处理
            }
            
            // 3. 延时尚未结束：保持最后时刻的目标速度，速度环闭环运行
            target_left = hold_target_left;
            target_right = hold_target_right;
            break;
        }
        // =================================================
        
        case STRAIGHT:
        {
            float current_angle = yawAngle;
            float angle_err = angle_error(target_angle, current_angle);
            
            // 白进黑最高优先级打断
            if (black_cnt > 0) {
                run_state = TRACKING;
                debug_run_state = 0;
                start_gpio_output();
                PID_Clear_Integral(&pid_left);
                PID_Clear_Integral(&pid_right);
                PID_Clear_Integral(&pid_angle);
                white_detect_counter = 0;
                float deviation = Gray_GetDeviation();
                float delta_rps = TURN_KP * deviation;
                target_left = BASE_RPS + delta_rps;
                target_right = BASE_RPS - delta_rps;
                break;
            }
            
            if (straight_substate == STRAIGHT_TURNING) {
                if (fabsf(angle_err) <= 3.0f) {
                    angle_settle_counter++;
                    if (angle_settle_counter >= ANGLE_SETTLE_COUNT) {
                        straight_substate = STRAIGHT_HOLDING;
                        angle_settle_counter = 0;
                        PID_Clear_Integral(&pid_angle);
                    }
                    target_left = 0.0f;
                    target_right = 0.0f;
                } else {
                    angle_settle_counter = 0;
                    
                    float turn_speed = fabsf(angle_err) * turn_p;
                    if (turn_speed > OPENLOOP_TURN_SPEED) turn_speed = OPENLOOP_TURN_SPEED;
                    if (turn_speed < 0.15f) turn_speed = 0.15f;
                    if (angle_err > 0) {
                        target_left = -turn_speed;
                        target_right = turn_speed;
                    } else {
                        target_left = +turn_speed;
                        target_right = -turn_speed;
                    }
                }
            } else { // STRAIGHT_HOLDING
                float correction = PID_Update(&pid_angle, target_angle, current_angle, DT);
                if (correction > ANGLE_OUT_MAX) correction = ANGLE_OUT_MAX;
                if (correction < -ANGLE_OUT_MAX) correction = -ANGLE_OUT_MAX;
                target_left = STRAIGHT_SPEED_RPS - correction;
                target_right = STRAIGHT_SPEED_RPS + correction;
            }
            break;
        }
        
        default:
            target_left = BASE_RPS;
            target_right = BASE_RPS;
            break;
    }
    
    debug_target_left = target_left;
    debug_target_right = target_right;
    
    float pwm_left = PID_Update(&pid_left, target_left, actual_left, DT);
    float pwm_right = PID_Update(&pid_right, target_right, actual_right, DT);
    
    debug_pwm_left = pwm_left;
    debug_pwm_right = pwm_right;
    
    Set_PWM((int)pwm_left, (int)pwm_right);
}


















































// void Control_Update(void)
// {
//     Update_Gray_Sensors();
    
//     float actual_left = Get_Speed_Left();
//     float actual_right = Get_Speed_Right();
//     debug_actual_left = actual_left;
//     debug_actual_right = actual_right;
    
//     float target_left, target_right;
    
//     switch (turn_state) {
//         case NORMAL_TRACKING:
//         {
//             // 检测右转直角（带连续计数滤波）
//             if (is_right_turn_detected()) {
//                 right_turn_detect_counter++;
//                 if (right_turn_detect_counter >= INTERSECTION_DETECT_COUNT) {
//                     // 检测到直角，进入延时直行状态
//                     turn_state = DELAY_LINEAR;
//                     state_start_time = timer_interrupt_cnt;
//                     right_turn_detect_counter = 0;
//                     debug_turn_state = 2;  // 调试：2表示延时状态
//                 }
//             } else {
//                 right_turn_detect_counter = 0;
//             }
            
//             // 正常循迹计算
//             float deviation = Gray_GetDeviation();
//             float delta_rps = TURN_KP * deviation;
//             target_left = BASE_RPS + delta_rps;
//             target_right = BASE_RPS - delta_rps;
//             break;
//         }
        
//         case DELAY_LINEAR:
//         {
//             // 延时1秒（100 * 10ms = 1000ms）
//             uint32_t elapsed_ms = (timer_interrupt_cnt - state_start_time) * 10;
//             if (elapsed_ms < 700) {
//                 // 延时期间继续正常循迹（保持直行）
//                 float deviation = Gray_GetDeviation();
//                 float delta_rps = TURN_KP * deviation;
//                 target_left = BASE_RPS + delta_rps;
//                 target_right = BASE_RPS - delta_rps;
//             } else {
//                 // 延时结束，进入原地右转状态
//                 turn_state = TURNING_RIGHT;
//                 state_start_time = timer_interrupt_cnt;
//                 debug_turn_state = 1;  // 1表示转弯状态
//                 // 第一次进入转弯状态，需要计算目标速度
//                 target_left = RIGHT_TURN_SPEED_RPS;
//                 target_right = -RIGHT_TURN_SPEED_RPS;
//             }
//             break;
//         }
        
//         case TURNING_RIGHT:
//         {
//             uint32_t elapsed_ms = (timer_interrupt_cnt - state_start_time) * 10;
//             if (elapsed_ms < RIGHT_TURN_DURATION_MS) {
//                 // 原地右转
//                 target_left = RIGHT_TURN_SPEED_RPS;
//                 target_right = -RIGHT_TURN_SPEED_RPS;
//             } else {
//                 // 转弯完成，恢复正常循迹
//                 turn_state = NORMAL_TRACKING;
//                 debug_turn_state = 0;
//                 // 重新计算偏差，避免突变
//                 float deviation = Gray_GetDeviation();
//                 float delta_rps = TURN_KP * deviation;
//                 target_left = BASE_RPS + delta_rps;
//                 target_right = BASE_RPS - delta_rps;
//             }
//             break;
//         }
        
//         default:
//             target_left = BASE_RPS;
//             target_right = BASE_RPS;
//             break;
//     }
    
//     debug_target_left = target_left;
//     debug_target_right = target_right;
    
//     float pwm_left = PID_Update(&pid_left, target_left, actual_left, DT);
//     float pwm_right = PID_Update(&pid_right, target_right, actual_right, DT);
    
//     debug_pwm_left = pwm_left;
//     debug_pwm_right = pwm_right;
    
//     Set_PWM((int)pwm_left, (int)pwm_right);
// }





































// void Control_Update(void)
// {   
//     float actual_left = Get_Speed_Left();
//     float actual_right = Get_Speed_Right();
    
  
//     // 1. 更新灰度数组（你需实现此函数）
//     Update_Gray_Sensors();
    
//     // 2. 计算偏差
//     float deviation = Gray_GetDeviation();
    
//     // 3. 计算左右目标速度（rps）
//     float delta_rps = TURN_KP * deviation;
//     float target_left = BASE_RPS + delta_rps;
//     float target_right = BASE_RPS - delta_rps;
    
   
    
//     // 5. PID 计算 PWM 输出
//     float pwm_left = PID_Update(&pid_left, target_left, actual_left, DT);
//     float pwm_right = PID_Update(&pid_right, target_right, actual_right, DT);
    
//     debug_target_left = target_left;
//     debug_target_right = target_right;
//     debug_actual_left = actual_left;
//     debug_actual_right = actual_right;
//     debug_pwm_left = pwm_left;
//     debug_pwm_right = pwm_right;
    
//     // 6. 设置电机 PWM
//     Set_PWM((int)pwm_left, (int)pwm_right);


