#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "gray_sensor.h"
#include "ti_msp_dl_config.h"

// ===== 外部引用 =====
extern unsigned char Digtal;                    // 灰度传感器数字量 (0=黑, 1=白)
extern volatile uint8_t button_cnt;             // PB8 按键计数 → 模式控制
extern volatile uint32_t timer_interrupt_cnt;   // 10ms 时基计数器

// ===== PID 控制器实例 =====
static PID_t pid_left, pid_right;

// ===== 调试变量 =====
volatile float debug_actual_left = 0.0f;
volatile float debug_actual_right = 0.0f;
volatile float debug_pwm_left = 0.0f;
volatile float debug_pwm_right = 0.0f;
volatile float debug_target_left = 0.0f;
volatile float debug_target_right = 0.0f;
volatile int32_t debug_run_state = 0;

// ===== 控制状态变量（main.c 引用）=====
RunState run_state = TRACKING;
volatile uint8_t stop_flag = 0;
volatile uint8_t white_stop_threshold = 1;
volatile uint8_t white_event_count = 0;
volatile uint8_t first_white_handled = 0;
float turn_p = 0.15f;
float BASE_RPS = 1.5f;
float STRAIGHT_SPEED_RPS = 2.0f;

// ===== 模式专属速度（可通过 MODE 按键或直接修改默认值调整）=====
volatile float MODE1_SPEED_RPS = MODE1_DEFAULT_SPEED_RPS;
volatile float MODE2_SPEED_RPS = MODE2_DEFAULT_SPEED_RPS; // 模式2分段线性速度规划
volatile float MODE3_SPEED_RPS = MODE3_DEFAULT_SPEED_RPS; // 模式3定时循迹

// ===== 模式运行计时器 =====
static uint32_t mode1_start_time = 0;           // 模式1 启动时刻
static uint8_t  mode1_active = 0;               // 模式1 已激活标志
static uint32_t mode2_start_time = 0;           // 模式2 启动时刻
static uint8_t  mode2_active = 0;               // 模式2 已激活标志
static uint32_t mode3_start_time = 0;           // 模式3 启动时刻
static uint8_t  mode3_active = 0;               // 模式3 已激活标志
static uint8_t  last_mode = 0;                  // 上一轮的模式（检测模式切换）
static uint8_t  mode0_initialized = 0;          // 模式0 初始化标志

/* ===== 修改部分：模式1黑色区域延时停车状态 ===== */
static volatile uint32_t mode1_black_start_time = 0; // 首次检测到黑色区域的时刻
static volatile uint8_t  mode1_black_active = 0;     // 1=已进入继续循迹计时阶段
/* ===== 修改部分结束：模式1黑色区域延时停车状态 ===== */

/* ===== 修改部分：按键切换后延时1秒再启动 ===== */
static volatile uint32_t mode_start_delay_time = 0; // 本次按键切换的时刻
static volatile uint8_t  mode_start_delay_active = 0; // 1=正在等待启动
/* ===== 修改部分结束：按键切换启动延时状态 ===== */

// ===== 模式运行时间（OLED 显示用）=====
volatile uint32_t mode1_elapsed_ms = 0;          // 模式1 已运行时间 (ms)
volatile uint32_t mode2_elapsed_ms = 0;          // 模式2 已运行时间 (ms)
volatile uint32_t mode3_elapsed_ms = 0;          // 模式3 已运行时间 (ms)

// 循迹偏差权重表：左负右正，中间对称
static const int8_t weights[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

/* ===== 修改部分：模式2/3共用的分段线性速度规划 =====
 * 加速段和减速段都只做线性比例计算。
 */
static float Control_GetLinearSpeed(float final_speed,
                                    uint32_t elapsed_ms,
                                    uint32_t accel_end_ms,
                                    uint32_t decel_start_ms,
                                    uint32_t stop_time_ms)
{
    // 起步阶段：目标速度随时间线性增加，因此目标加速度恒定。
    if (elapsed_ms < accel_end_ms) {
        return final_speed * ((float)elapsed_ms / (float)accel_end_ms);
    }

    // 匀速阶段：保持当前模式的预设速度。
    if (elapsed_ms < decel_start_ms) {
        return final_speed;
    }

    // 减速阶段：继续循迹，目标速度随时间线性减小到0。
    if (elapsed_ms < stop_time_ms) {
        uint32_t decel_duration_ms = stop_time_ms - decel_start_ms;
        uint32_t remaining_ms = stop_time_ms - elapsed_ms;
        return final_speed * ((float)remaining_ms / (float)decel_duration_ms);
    }

    return 0.0f;
}
/* ===== 修改部分结束：模式2/3分段线性速度规划 ===== */

/* ===== 修改部分：等待结束后统一启动所选模式 ===== */
static void Control_BeginModeExecution(uint8_t mode)
{
    PID_Clear_Integral(&pid_left);
    PID_Clear_Integral(&pid_right);

    mode1_active = 0U;
    mode2_active = 0U;
    mode3_active = 0U;

    if (mode == 1U) {
        mode1_start_time = timer_interrupt_cnt;
        mode1_elapsed_ms = 0U;
        mode1_active = 1U;
    } else if (mode == 2U) {
        mode2_start_time = timer_interrupt_cnt;
        mode2_elapsed_ms = 0U;
        mode2_active = 1U;
    } else if (mode == 3U) {
        mode3_start_time = timer_interrupt_cnt;
        mode3_elapsed_ms = 0U;
        mode3_active = 1U;
    }
}
/* ===== 修改部分结束：统一启动所选模式 ===== */

void Control_ResetAfterStop(void)
{
    white_event_count = 0;
    first_white_handled = 0;

    /* ===== 修改部分：停车恢复时清除模式1黑区延时状态 ===== */
    mode1_black_active = 0U;
    mode1_black_start_time = 0U;
    mode_start_delay_active = 0U;
    /* ===== 修改部分结束：清除模式1黑区延时状态 ===== */

    /* ===== 修改部分：停车后重新启动时清空PID历史 =====
     * stop_flag 最后清零，确保定时中断不会在复位尚未完成时提前驱动电机。
     */
    Control_BeginModeExecution(button_cnt % 4U);
    stop_flag = 0;
    /* ===== 修改部分结束：停车后重新启动 ===== */
}

void Control_Init(void)
{
    PID_Init(&pid_left,  PID_LEFT_KP,  PID_LEFT_KI,  PID_LEFT_KD,  PID_OUT_MIN, PID_OUT_MAX);
    PID_Init(&pid_right, PID_RIGHT_KP, PID_RIGHT_KI, PID_RIGHT_KD, PID_OUT_MIN, PID_OUT_MAX);
    Encoder_Init();
    Motor_Init();
    Gray_Init();
}

void Control_Update(void)
{
    float actual_left  = Get_Speed_Left();
    float actual_right = Get_Speed_Right();
    debug_actual_left  = actual_left;
    debug_actual_right = actual_right;

    uint8_t mode = button_cnt % 4;  // 0=停止, 1=黑区停车, 2=29s速度规划, 3=定时循迹

    // ---------- 模式切换检测：清零 PID 积分，清除停止标志 ----------
    if (mode != last_mode) {
        PID_Clear_Integral(&pid_left);
        PID_Clear_Integral(&pid_right);
        mode1_active = 0U;
        mode2_active = 0U;
        mode3_active = 0U;
        mode0_initialized = 0U;

        /* ===== 修改部分：切换模式时清除旧状态，并启动1秒等待 ===== */
        mode1_black_active = 0U;
        mode1_black_start_time = 0U;
        mode1_elapsed_ms = 0U;
        mode2_elapsed_ms = 0U;
        mode3_elapsed_ms = 0U;
        mode_start_delay_time = timer_interrupt_cnt;
        mode_start_delay_active = (mode == 0U) ? 0U : 1U;
        /* ===== 修改部分结束：按键切换后1秒启动等待 ===== */

        stop_flag = 0;          // 切换模式时清除停止标志
        last_mode = mode;
    }

    // ==================== 模式 0：静止 ====================
    if (mode == 0) {
        if (!mode0_initialized) {
            mode0_initialized = 1;
            stop_flag = 0;
            Control_ResetAfterStop();
        }
        Set_PWM(0, 0);
        debug_target_left  = 0.0f;
        debug_target_right = 0.0f;
        debug_pwm_left  = 0.0f;
        debug_pwm_right = 0.0f;
        return;
    }

    /* ===== 修改部分：模式1/2/3按键切换后等待1秒再执行 ===== */
    if (mode_start_delay_active) {
        uint32_t start_delay_ms =
            (timer_interrupt_cnt - mode_start_delay_time) * 10U;

        Set_PWM(0, 0);
        debug_target_left  = 0.0f;
        debug_target_right = 0.0f;
        debug_pwm_left  = 0.0f;
        debug_pwm_right = 0.0f;

        if (start_delay_ms < MODE_SWITCH_START_DELAY_MS) {
            return;
        }

        mode_start_delay_active = 0U;
        Control_BeginModeExecution(mode);
    }
    /* ===== 修改部分结束：按键切换启动等待 ===== */

    // ==================== 停止状态门控 ====================
    // stop_flag 一旦置位，持续保持 PWM=0，防止下一轮中断"穿过"检查重启电机
    if (stop_flag) {
        Set_PWM(0, 0);
        debug_target_left  = 0.0f;
        debug_target_right = 0.0f;
        debug_pwm_left  = 0.0f;
        debug_pwm_right = 0.0f;
        return;
    }

    // ==================== 更新运行时间（停车后冻结） ====================
    if (mode == 1 && mode1_active) {
        mode1_elapsed_ms = (timer_interrupt_cnt - mode1_start_time) * 10U;
    } else if (mode == 2 && mode2_active) {
        mode2_elapsed_ms = (timer_interrupt_cnt - mode2_start_time) * 10U;
    } else if (mode == 3 && mode3_active) {
        mode3_elapsed_ms = (timer_interrupt_cnt - mode3_start_time) * 10U;
    }

    /* ===== 修改部分：模式2/3在线性减速结束时速度归零并停车 =====
     * 运行计时从1秒按键等待结束后开始，因此等待时间不计入速度规划。
     */
    if (mode == 2U && mode2_elapsed_ms >= MODE2_STOP_TIME_MS) {
        mode2_elapsed_ms = MODE2_STOP_TIME_MS; // OLED固定显示设定的停车时刻
        stop_flag = 1U;
        Set_PWM(0, 0);
        PID_Clear_Integral(&pid_left);
        PID_Clear_Integral(&pid_right);
        debug_target_left  = 0.0f;
        debug_target_right = 0.0f;
        debug_pwm_left  = 0.0f;
        debug_pwm_right = 0.0f;
        return;
    }
    if (mode == 3U && mode3_elapsed_ms >= MODE3_STOP_TIME_MS) {
        mode3_elapsed_ms = MODE3_STOP_TIME_MS; // OLED固定显示设定的停车时刻
        stop_flag = 1U;
        Set_PWM(0, 0);
        PID_Clear_Integral(&pid_left);
        PID_Clear_Integral(&pid_right);
        debug_target_left  = 0.0f;
        debug_target_right = 0.0f;
        debug_pwm_left  = 0.0f;
        debug_pwm_right = 0.0f;
        return;
    }
    /* ===== 修改部分结束：模式2/3线性减速结束停车 ===== */

    // ==================== 模式 1 / 2 / 3：循迹 ====================
    // 更新灰度传感器数据 → gray[8]  (gray[i]=1 表示检测到黑线)
    Update_Gray_Sensors();

    // ---- 计算循迹偏差 (加权平均) ----
    float deviation = 0.0f;
    int   sum_values = 0;
    for (int i = 0; i < 8; i++) {
        if (gray[i]) {
            deviation  += weights[i];
            sum_values += 1;
        }
    }

    /* ===== 修改部分：模式1检测到黑色区域后继续循迹再停车 =====
     * 保留当前黑区判定条件 sum_values >= 3。
     * 首次检测到黑区时只启动计时，不立即停车；计时期间继续执行下方循迹PID。
     * 模式2改为完整执行设定的速度规划，因此不再使用黑区停车条件。
     */
    if (mode == 1U) {
        if (!mode1_black_active && sum_values >= 3) {
            mode1_black_active = 1U;
            mode1_black_start_time = timer_interrupt_cnt;
        }

        if (mode1_black_active) {
            uint32_t black_continue_ms =
                (timer_interrupt_cnt - mode1_black_start_time) * 10U;

            if (black_continue_ms >= MODE1_BLACK_CONTINUE_MS) {
                stop_flag = 1;
                Set_PWM(0, 0);
                PID_Clear_Integral(&pid_left);
                PID_Clear_Integral(&pid_right);
                debug_target_left  = 0.0f;
                debug_target_right = 0.0f;
                debug_pwm_left  = 0.0f;
                debug_pwm_right = 0.0f;
                return;
            }
        }
    }
    /* ===== 修改部分结束：模式1黑区继续循迹后停车 ===== */

    // ---- 归一化偏差 (-7 ~ +7) ----
    if (sum_values > 0 && sum_values < 8) {
        deviation = deviation / sum_values;
    } else {
        // 全白 (sum_values==0) 或不应到达的全黑分支：直行
        deviation = 0.0f;
    }

    /* ===== 修改部分：三种模式采用各自速度规划 ===== */
    // configured_speed 是最终设定速度，base_speed 是当前速度规划输出。
    float configured_speed;
    if (mode == 3U) {
        configured_speed = MODE3_SPEED_RPS;
    } else if (mode == 2U) {
        configured_speed = MODE2_SPEED_RPS;
    } else {
        configured_speed = MODE1_SPEED_RPS;
    }
    float base_speed;
    if (mode == 2U) {
        // 模式2：匀加速、匀速、匀减速。
        base_speed = Control_GetLinearSpeed(configured_speed,
                                            mode2_elapsed_ms,
                                            MODE2_ACCEL_END_MS,
                                            MODE2_DECEL_START_MS,
                                            MODE2_STOP_TIME_MS);
    } else if (mode == 3U) {
        // 模式3：与模式2共用线性算法，但采用模式3自己的时间参数。
        base_speed = Control_GetLinearSpeed(configured_speed,
                                            mode3_elapsed_ms,
                                            MODE3_ACCEL_END_MS,
                                            MODE3_DECEL_START_MS,
                                            MODE3_STOP_TIME_MS);
    } else {
        // 模式1：1秒按键等待结束后直接使用预设速度。
        base_speed = configured_speed;
    }

    /*
     * 起步及减速阶段同步缩放转向量：模式2和模式3减速期间
     * 都保持灰度循迹，同时转向修正随基础速度线性减小到0。
     */
    float speed_ratio = 0.0f;
    if (configured_speed > 0.0f) {
        speed_ratio = base_speed / configured_speed;
    }
    if (speed_ratio > 1.0f) {
        speed_ratio = 1.0f;
    }

    // deviation<0 → 线偏左 → 左轮减速，右轮加速
    // deviation>0 → 线偏右 → 右轮减速，左轮加速
    float turn_adjust = deviation * turn_p * speed_ratio;
    /* ===== 修改部分结束：三种模式速度规划及循迹修正 ===== */

    float target_left  = base_speed + turn_adjust;
    float target_right = base_speed - turn_adjust;

    /* ===== 修改部分：允许速度规划从真正的0速度起步 ===== */
    // 原来的最低0.1rps限幅会破坏零速起点，因此改为最低0rps。
    if (target_left  < 0.0f) target_left  = 0.0f;
    if (target_right < 0.0f) target_right = 0.0f;
    if (target_left  > 3.0f) target_left  = 3.0f;
    if (target_right > 3.0f) target_right = 3.0f;
    /* ===== 修改部分结束：目标速度限幅 ===== */

    // ---- PID 速度控制 ----
    float pwm_left  = PID_Update(&pid_left,  target_left,  actual_left,  DT);
    float pwm_right = PID_Update(&pid_right, target_right, actual_right, DT);

    debug_target_left  = target_left;
    debug_target_right = target_right;
    debug_pwm_left  = pwm_left;
    debug_pwm_right = pwm_right;

    Set_PWM((int)pwm_left, (int)pwm_right);
}
