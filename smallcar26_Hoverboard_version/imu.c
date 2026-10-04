#include "imu.h"
#include "axis6.h"
#include "delay.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / M_PI)
#define DEG_TO_RAD (M_PI / 180.0f)

IMU_State imu_state;

/* ========== 初始化 ========== */
bool IMU_Init(float comp_alpha)
{
    imu_state.pitch              = 0.0f;
    imu_state.roll               = 0.0f;
    imu_state.yaw                = 0.0f;
    imu_state.gyro_offset_x      = 0.0f;
    imu_state.gyro_offset_y      = 0.0f;
    imu_state.gyro_offset_z      = 0.0f;
    imu_state.comp_filter_alpha  = comp_alpha;
    imu_state.last_update_tick   = 0;
    imu_state.initialized        = true;

    return true;
}

/* ========== 陀螺仪校准 (静止状态下采集零偏) ========== */
bool IMU_CalibrateGyro(uint16_t samples)
{
    if (samples == 0) samples = 100;

    float    sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    uint16_t valid = 0;
    AXIS6_RawData raw;
    uint32_t t_start = Tick;  /* 记录开始时间 (ms), 用于整体超时 */

    for (uint16_t i = 0; i < samples; i++) {

        /* 整体超时保护: 最长等 3 秒 */
        if (Tick - t_start > 3000)
            break;

        /* 读取一次, 失败则跳过该样本 */
        if (!AXIS6_ReadRawData(&raw))
            continue;

        sum_x += (float)raw.gyro_x;
        sum_y += (float)raw.gyro_y;
        sum_z += (float)raw.gyro_z;
        valid++;

        /* 500μs 间隔 (AXIS6 输出率足够快) */
        delay_us(500);
    }

    /* 至少需要 30 个有效样本 (100次采集中成功率>30%即可) */
    if (valid < 30)
        return false;

    imu_state.gyro_offset_x = sum_x / (float)valid;
    imu_state.gyro_offset_y = sum_y / (float)valid;
    imu_state.gyro_offset_z = sum_z / (float)valid;

    return true;
}

/* ========== 姿态更新 ==========
 *
 * AXIS6 模块内部已完成传感器融合 (加速度计+陀螺仪),
 * 直接使用预计算的角度。同时使用陀螺仪原始数据作为角速度。
 *
 * 注意事项:
 *   AXIS6 Pitch 前倾为正还是负取决于模块的安装方向.
 *   当前代码沿用原始约定: pitch_rate 前倾=正值.
 *   如果实际安装方向不同, 调整下方的符号即可.
 * ========== */
bool IMU_Update(IMU_Attitude *att, float dt_s)
{
    AXIS6_RawData raw;

    if (!AXIS6_ReadRawData(&raw))
        return false;

    /* ---- 1. 转换为物理单位 ---- */
    float gyro_x  = ((float)raw.gyro_x - imu_state.gyro_offset_x) * axis6_gyro_scale;  /* °/s */
    float gyro_y  = ((float)raw.gyro_y - imu_state.gyro_offset_y) * axis6_gyro_scale;
    float gyro_z  = ((float)raw.gyro_z - imu_state.gyro_offset_z) * axis6_gyro_scale;

    /* AXIS6 预计算角度 (内部已做融合滤波) */
    float axis6_pitch = (float)raw.pitch_raw * axis6_angle_scale;  /* ° */
    float axis6_roll  = (float)raw.roll_raw  * axis6_angle_scale;
    float axis6_yaw   = (float)raw.yaw_raw   * axis6_angle_scale;

    /* ---- 2. 互补滤波融合 (可选, 对 AXIS6 已融合数据做轻微平滑) ----
     *
     * AXIS6 的 Pitch 已经过内部融合, 这里用陀螺仪积分做短期预测,
     * 与 AXIS6 Pitch 做互补滤波, 系数可用 alpha 调节:
     *   alpha=1.0 → 纯陀螺仪积分 (无 AXIS6 角度校正)
     *   alpha=0.98 → 轻微融合 (推荐, 保留陀螺响应速度 + AXIS6 稳定性)
     *   alpha=0.0 → 纯 AXIS6 角度 (最平滑但有延迟)
     */
    float alpha = imu_state.comp_filter_alpha;

    imu_state.pitch = alpha * (imu_state.pitch + gyro_x * dt_s)
                    + (1.0f - alpha) * axis6_pitch;

    imu_state.roll  = alpha * (imu_state.roll  + gyro_y * dt_s)
                    + (1.0f - alpha) * axis6_roll;

    /* 偏航角仅靠陀螺仪积分 (AXIS6 Yaw 可能有漂移, 以陀螺积分为主) */
    imu_state.yaw   = imu_state.yaw + gyro_z * dt_s;

    /* ---- 3. 输出 ----
     *
     * pitch:       融合后的俯仰角 (前倾=正值)
     * pitch_rate:  gyro_x 角速度 (前倾旋转=正值)
     *
     * 如果实际安装后符号相反, 修改:
     *   att->pitch      = -imu_state.pitch;
     *   att->pitch_rate = -gyro_x;
     */
    att->pitch      = imu_state.pitch;
    att->pitch_rate = gyro_x;
    att->roll       = imu_state.roll;
    att->yaw        = imu_state.yaw;
    att->yaw_rate   = gyro_z;      /* 转向 PD 用 gyro_z 角速度, 非积分角度 */

    imu_state.last_update_tick = Tick;

    return true;
}

/* ========== 角度复位 ========== */
void IMU_Reset(void)
{
    imu_state.pitch = 0.0f;
    imu_state.roll  = 0.0f;
    imu_state.yaw   = 0.0f;
}
