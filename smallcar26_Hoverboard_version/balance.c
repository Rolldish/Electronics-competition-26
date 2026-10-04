#include "balance.h"
#include "axis6.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>

/* ========== PID 控制器实例 ========== */
static PID_t pid_angle;     /* 直立环 (角度 PD) — 内环 */
static PID_t pid_speed;     /* 速度环 (速度 PI) — 外环 */
static PID_t pid_turn;      /* 转向环 (偏航 PD) — 独立环 */

/* ========== 全局状态 ========== */
BalanceDebug balance_debug;

/* ========== 低通滤波状态 (参考 WHEELTEC: 一阶 IIR) ========== */
static float speed_filtered      = 0.0f;  /* 滤波后的平均速度 */
static float gyro_d_filtered     = 0.0f;  /* 滤波后的 gyro_rate (D 项降噪) */
static float turn_gyro_filtered  = 0.0f;  /* 滤波后的偏航角速度 */

/* ========== 用户设定的目标值 ========== */
static float user_target_speed = 0.0f;   /* 目标速度 (rps) */
static float user_target_turn  = 0.0f;   /* 目标偏航角速度 (°/s) */

/* ========== 初始化 ========== */
void Balance_Init(void)
{
    /* 直立环: PD 控制 (Ki=0)
     * 注意: 角度环的 D 项通过直接乘 gyro_rate 实现, 不走 PID_Update */
    PID_Init(&pid_angle,
        BALANCE_ANGLE_KP,
        0.0f,                          /* Ki = 0 (纯 PD) */
        BALANCE_ANGLE_KD,
        -BALANCE_ANGLE_OUT_MAX,
         BALANCE_ANGLE_OUT_MAX);

    /* 速度环: PI 控制 (Kd=0) */
    PID_Init(&pid_speed,
        BALANCE_SPEED_KP,
        BALANCE_SPEED_KI,
        0.0f,                          /* Kd = 0 (纯 PI) */
        -BALANCE_SPEED_OUT_MAX,
         BALANCE_SPEED_OUT_MAX);
    pid_speed.integral_decay = 0.98f;  /* 漏积分: 编码器偏置 ~250ms 后衰减, 防"突然前倾" */

    /* 转向环: PD 控制 (Ki=0)
     * 输入: 偏航角速度误差 (°/s), 输出: 差速 PWM */
    PID_Init(&pid_turn,
        BALANCE_TURN_KP,
        0.0f,                          /* Ki = 0 (纯 PD) */
        BALANCE_TURN_KD,
        -BALANCE_TURN_OUT_MAX,
         BALANCE_TURN_OUT_MAX);

    Motor_Init();
    Encoder_Init();

    /* 重置滤波状态 */
    speed_filtered      = 0.0f;
    gyro_d_filtered     = 0.0f;
    turn_gyro_filtered  = 0.0f;
    user_target_speed   = 0.0f;
    user_target_turn    = 0.0f;

    balance_debug.state = BALANCE_STOP;
    balance_debug.stop_reason = STOP_REASON_INIT;
    balance_debug.pitch        = 0.0f;
    balance_debug.pitch_rate   = 0.0f;
    balance_debug.yaw_rate     = 0.0f;
    balance_debug.speed_left   = 0.0f;
    balance_debug.speed_right  = 0.0f;
    balance_debug.speed_filtered = 0.0f;
    balance_debug.angle_output = 0.0f;
    balance_debug.speed_output = 0.0f;
    balance_debug.turn_output  = 0.0f;
    balance_debug.pwm_left     = 0;
    balance_debug.pwm_right    = 0;
    balance_debug.battery_voltage = 0.0f;
}

/* ========== 启动平衡 ========== */
void Balance_Start(void)
{
    if (balance_debug.state == BALANCE_STOP) {
        PID_Clear_Integral(&pid_angle);
        PID_Clear_Integral(&pid_speed);
        PID_Clear_Integral(&pid_turn);
        speed_filtered      = 0.0f;
        gyro_d_filtered     = 0.0f;
        turn_gyro_filtered  = 0.0f;
        balance_debug.state = BALANCE_RUNNING;
        balance_debug.stop_reason = STOP_REASON_NONE;
    }
}

/* ========== 停止平衡 ========== */
void Balance_Stop(BalanceStopReason reason)
{
    balance_debug.state = BALANCE_STOP;
    balance_debug.stop_reason = reason;
    Set_PWM(0, 0);
}

/* ========== 设置目标速度 ========== */
void Balance_SetTargetSpeed(float speed_rps)
{
    user_target_speed = speed_rps;
}

/* ========== 设置目标转向角速度 ========== */
void Balance_SetTargetTurn(float turn_dps)
{
    user_target_turn = turn_dps;
}

/* ========== 电池电压读取 (弱实现, 需用户根据硬件 ADC 实现) ========== */
__attribute__((weak)) float Balance_ReadBatteryVoltage(void)
{
    /* 默认返回 12.0V (模拟满电), 用户需根据实际 ADC 通道实现
     * 示例: return (float)DL_ADC12_getMemResult(...) * 3.3f * 11.0f / 4096.0f;
     */
    return 12.0f;
}

/* ========== 平衡控制更新 (每 5ms 调用一次, 200Hz) ========== */
void Balance_Update(void)
{
    /* ---- 1. 读取姿态 ---- */
    IMU_Attitude att;
    if (!IMU_Update(&att, BALANCE_DT)) {
        /* IMU 读取失败 → 紧急停止 */
        if (balance_debug.state == BALANCE_RUNNING) {
            Balance_Stop(STOP_REASON_IMU);
        }
        return;
    }
    balance_debug.pitch      = att.pitch;
    balance_debug.pitch_rate = att.pitch_rate;
    balance_debug.yaw_rate   = att.yaw_rate;   /* gyro.z 角速度 (°/s), 用于转向 PD */

    /* ---- 2. 读取轮速 ---- */
    float speed_left  = Get_Speed_Left();
    float speed_right = Get_Speed_Right();
    balance_debug.speed_left  = speed_left;
    balance_debug.speed_right = speed_right;

    /* ---- 3. 电池电压检测 ---- */
    balance_debug.battery_voltage = Balance_ReadBatteryVoltage();
    if (balance_debug.battery_voltage < BALANCE_LOW_VOLTAGE
        && balance_debug.battery_voltage > 0.1f) {  /* >0.1 防止未实现时误触发 */
        if (balance_debug.state == BALANCE_RUNNING) {
            Balance_Stop(STOP_REASON_VOLTAGE);
        }
        return;
    }

    /* ---- 4. 停止状态 → 电机断电, 清积分 ---- */
    if (balance_debug.state == BALANCE_STOP) {
        Set_PWM(0, 0);
        PID_Clear_Integral(&pid_angle);
        PID_Clear_Integral(&pid_speed);
        PID_Clear_Integral(&pid_turn);
        speed_filtered     = 0.0f;
        gyro_d_filtered    = 0.0f;
        turn_gyro_filtered = 0.0f;
        balance_debug.angle_output = 0.0f;
        balance_debug.speed_output = 0.0f;
        balance_debug.turn_output  = 0.0f;
        balance_debug.pwm_left     = 0;
        balance_debug.pwm_right    = 0;
        return;
    }

    /* ---- 5. 安全保护: 倾角过大 → 停机 ---- */
    if (fabsf(att.pitch) > BALANCE_MAX_ANGLE) {
        Balance_Stop(STOP_REASON_ANGLE);
        return;
    }

    /* ============================================================
     * 6. 三环串级 PID 控制
     *
     *   [速度 PI]           [角度 PD]          [电机]
     *   目标速度 ──→ 角度偏移 ──→ PWM ──→ 左轮+右轮
     *                ↑                  ↓
     *             编码器              [转向 PD]
     *                                  ↓
     *                               差速 ± ──→ 左轮-右轮
     *
     *   参考 WHEELTEC 架构:
     *     Motor_Left  = Angle_PD + Speed_PI + Turn_PD
     *     Motor_Right = Angle_PD + Speed_PI - Turn_PD
     *
     *   直立环采用直接 PD 计算 (gyro_rate 直接作为 D 项,
     *   比角度微分更平滑、延迟更低)
     * ============================================================ */

    /* ---- 6a. 速度环 (外环 PI) ----
     * 输入: 编码器平均速度 (rps), 经低通滤波
     * 输出: 目标角度偏移量 (°)
     * 正输出 = 前倾 = 加速前进
     *
     * 参考 WHEELTEC: 一阶低通滤波
     *   filtered = old * (1-alpha) + new * alpha
     *   alpha=0.14 为 WHEELTEC 默认
     */
    float avg_speed = (speed_left + speed_right) * 0.5f;

    /* 死区: 滤除编码器量化噪声, 防止积分随机游走 */
    if (fabsf(avg_speed) < SPEED_DEAD_ZONE) {
        avg_speed = 0.0f;
    }

    /* 一阶低通滤波 */
    speed_filtered = speed_filtered * (1.0f - SPEED_FILTER_ALPHA)
                   + avg_speed * SPEED_FILTER_ALPHA;
    balance_debug.speed_filtered = speed_filtered;

    /* 速度 PI 计算 (通过 PID_Update) */
    float speed_output = PID_Update(&pid_speed,
        user_target_speed, speed_filtered, BALANCE_DT);
    balance_debug.speed_output = speed_output;

    /* ---- 6b. 直立环 (内环 PD) ----
     * target_angle = TRIM + speed_offset
     * (小车有前进速度 → speed_offset 为正 → 前倾更多 → 加速追赶)
     * angle_error = pitch - target_angle
     * output = Kp * angle_error + Kd * gyro_rate
     */
    float target_angle = BALANCE_TARGET_ANGLE + BALANCE_ANGLE_TRIM + speed_output;
    float angle_error  = att.pitch - target_angle;

    /* 抗积分饱和: 角度误差过大时冻结速度环积分 */
    if (fabsf(angle_error) > ANGLE_ERROR_IWINDUP) {
        pid_speed.integral = pid_speed.integral;  /* 保持不变 */
    }

    /* ---- D 项 gyro_rate 低通滤波 (降噪) ----
     * Kd 直接乘 gyro_rate 会放大陀螺仪高频噪声 (±2°/s → ±400 PWM),
     * 导致电机在目标角度附近高频震颤, 表现为"抖动严重".
     * 用一阶低通滤波平滑 gyro_rate, 消除 D 项噪声放大
     */
    gyro_d_filtered = gyro_d_filtered * (1.0f - GYRO_D_FILTER_ALPHA)
                    + att.pitch_rate * GYRO_D_FILTER_ALPHA;

    /* 直立 PD 直接计算 (D 项用滤波后的 gyro_rate) */
    float angle_output = BALANCE_ANGLE_KP  * angle_error
                       + BALANCE_ANGLE_KD  * gyro_d_filtered;

    /* 输出限幅 */
    if (angle_output >  BALANCE_ANGLE_OUT_MAX)
        angle_output =  BALANCE_ANGLE_OUT_MAX;
    if (angle_output < -BALANCE_ANGLE_OUT_MAX)
        angle_output = -BALANCE_ANGLE_OUT_MAX;

    balance_debug.angle_output = angle_output;

    /* ---- 6c. 转向环 (偏航 PD) ----
     * 输入: gyro.z (偏航角速度 °/s) + 目标转向角速度
     * 输出: 差速 PWM (加到左轮、减自右轮, 或反之)
     *
     * 参考 WHEELTEC:
     *   Turn = Kp * Turn_Target + Kd * gyro_z
     *
     * Kd 作用: 抑制非期望的偏航旋转 (类似角速度阻尼)
     *         前进/后退时启用, 原地转向时可关闭
     */
    float turn_error = user_target_turn - att.yaw_rate;

    /* 转向 PD: 目标跟踪 + 扰动抑制 */
    float turn_output = BALANCE_TURN_KP * turn_error
                      + BALANCE_TURN_KD * (-att.yaw_rate);  /* Kd 阻尼 gyro.z 本身 */

    /* 转向输出限幅 */
    if (turn_output >  BALANCE_TURN_OUT_MAX)
        turn_output =  BALANCE_TURN_OUT_MAX;
    if (turn_output < -BALANCE_TURN_OUT_MAX)
        turn_output = -BALANCE_TURN_OUT_MAX;

    balance_debug.turn_output = turn_output;

    /* ---- 7. 电机混控与输出 ----
     * 参考 WHEELTEC:
     *   Motor_Left  = Angle + Speed + Turn
     *   Motor_Right = Angle + Speed - Turn
     *
     * Turn 为正 (左转) → 左轮减速, 右轮加速 → 差速左转
     */
    int pwm_left  = (int)(angle_output + turn_output);
    int pwm_right = (int)(angle_output - turn_output);

    /* 分别限幅 */
    pwm_left  = limit_PWM(pwm_left,  PWM_MIN_LIMIT, PWM_MAX_LIMIT);
    pwm_right = limit_PWM(pwm_right, PWM_MIN_LIMIT, PWM_MAX_LIMIT);

    balance_debug.pwm_left  = pwm_left;
    balance_debug.pwm_right = pwm_right;

    Set_PWM(pwm_left, pwm_right);
}
