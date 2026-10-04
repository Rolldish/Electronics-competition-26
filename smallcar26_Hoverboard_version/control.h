#ifndef CONTROL_H
#define CONTROL_H
// PID 参数（固定宏）

#define TURN_KP          0.1f       // 转向系数
#define PID_LEFT_KP      2000.0f
#define PID_LEFT_KI      200.0f
#define PID_LEFT_KD      5.0f
#define PID_RIGHT_KP    2000.0f
#define PID_RIGHT_KI     200.0f
#define PID_RIGHT_KD     5.0f
#define PID_OUT_MIN     -8000.0f
#define PID_OUT_MAX     8000.0f
#define DT               0.01f

// #define RIGHT_TURN_DURATION_MS  1500   // 右转持续时间（毫秒）
// #define RIGHT_TURN_SPEED_RPS    0.5f  // 转弯时的目标速度（圈/秒）
// #define INTERSECTION_DETECT_COUNT 2   // 连续检测到直角特征的次数（滤波
// 全白处理参数
#define WHITE_DETECT_THRESHOLD  10   // 连续全白次数（30ms）触发
#define PAUSE_DURATION_MS       500  // 暂停持续时间（毫秒）

void Control_Init(void);
void Control_Update(void);   // 在10ms中断中调用
// 你需要实现的函数：更新灰度数组 gray[8]
extern void Update_Gray_Sensors(void);
// 在 control.c 的全局区域添加

// 运行状态机
typedef enum {
    TRACKING = 0,
    PAUSE,
    STRAIGHT,
    STOP,
    DELAY_HOLD 
} RunState;


// 外部可访问的控制状态
//extern volatile RunState run_state;
//extern volatile uint8_t white_stop_threshold;
// 在 control.h 中添加以下宏（放在合适位置）
// 角度 PID 参数
#define ANGLE_KP          0.1f
#define ANGLE_KI          0.0f
#define ANGLE_KD          0.0f
#define ANGLE_OUT_MAX     0.5f        // 最大修正量 (rps)
#define ANGLE_DEADBAND    2.0f        // 角度误差死区（度）
#define ANGLE_SETTLE_COUNT 5          // 稳定周期数（50ms）
// 开环转向速度（用于转向阶段）
#define OPENLOOP_TURN_SPEED 0.5f
// // 目标角度宏
// #define TARGET_ANGLE_0   0.0f
// #define TARGET_ANGLE_1   0.0f
#endif  