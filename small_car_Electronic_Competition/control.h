#ifndef CONTROL_H
#define CONTROL_H
#include <stdint.h>
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

/* ===== 修改部分：模式1检测到黑色区域后继续循迹的时间 =====
 * 单位为毫秒。例如设为500U，表示首次检测到黑色区域后继续循迹500ms再停车。
 */
#define MODE1_BLACK_CONTINUE_MS  500U
/* ===== 修改部分结束：模式1黑区继续循迹时间 ===== */

/* ===== 修改部分：按键切换延时及模式1五次S曲线平滑启动参数 =====
 * 切换到模式1、2、3后先保持停车1秒，再正式启动控制程序。
 * 模式2和模式3不使用S曲线，改用下方分段线性速度规划。
 */
#define MODE_SWITCH_START_DELAY_MS 1000U
#define MODE1_START_RAMP_MS     1500U  // 模式1从静止平滑加速到设定速度的时间
/* ===== 修改部分结束：按键切换延时及模式1 S曲线参数 ===== */

/* ===== 修改部分：模式2分段匀加速/匀减速时间参数 =====
 * 0~MODE2_ACCEL_END_MS：从0匀加速到模式2预设速度；
 * MODE2_DECEL_START_MS后保持循迹并匀减速，MODE2_STOP_TIME_MS时速度为0并停车。
 */
#define MODE2_ACCEL_END_MS       5000U
#define MODE2_DECEL_START_MS    20000U
#define MODE2_STOP_TIME_MS      27600U
#if (MODE2_ACCEL_END_MS == 0U) || (MODE2_ACCEL_END_MS > MODE2_DECEL_START_MS) || \
    (MODE2_DECEL_START_MS >= MODE2_STOP_TIME_MS)
#error "Invalid mode2 speed-profile time settings"
#endif
/* ===== 修改部分结束：模式2速度规划参数 ===== */

/* ===== 修改部分：模式3分段匀加速/匀减速时间参数 =====
 * 模式3与模式2使用完全相同的线性速度规划算法，但时间参数可以独立修改。
 * 当前设置：0~2s匀加速，2~6s匀速，6~8s匀减速到0并停车。
 */
#define MODE3_ACCEL_END_MS       2000U
#define MODE3_DECEL_START_MS     7000U
#define MODE3_STOP_TIME_MS       8000U
#if (MODE3_ACCEL_END_MS == 0U) || (MODE3_ACCEL_END_MS > MODE3_DECEL_START_MS) || \
    (MODE3_DECEL_START_MS >= MODE3_STOP_TIME_MS)
#error "Invalid mode3 speed-profile time settings"
#endif
/* ===== 修改部分结束：模式3速度规划参数 ===== */

/* ===== 修改部分：模式1/2/3默认循迹速度，可自行修改 ===== */
#define MODE1_DEFAULT_SPEED_RPS  2.2f
#define MODE2_DEFAULT_SPEED_RPS  1.77f
#define MODE3_DEFAULT_SPEED_RPS  1.7f
/* ===== 修改部分结束：三种模式默认速度 ===== */

// 模式速度（运行时可通过 MODE 按键调整）
extern volatile float MODE1_SPEED_RPS;  // 模式1 循迹速度 (rps)
extern volatile float MODE2_SPEED_RPS;  // 模式2 循迹速度 (rps)
extern volatile float MODE3_SPEED_RPS;  // 模式3 线性速度规划的预设速度 (rps)

// 模式运行时间（OLED 显示用，单位 ms）
extern volatile uint32_t mode1_elapsed_ms;
extern volatile uint32_t mode2_elapsed_ms;
extern volatile uint32_t mode3_elapsed_ms;

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
