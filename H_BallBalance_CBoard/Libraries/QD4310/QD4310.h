#ifndef QD4310_H
#define QD4310_H

#include <atomic>
#include <cstdint>

#include "can.h"

/*
 * QD4310 的 CAN 驱动。
 *
 * 对上层提供“使能、失能、角度、速度、电流”等直观接口，
 * 在内部完成 CAN 帧 ID、命令码和数值缩放。
 */
class QD4310 {
public:
    // 一帧反馈解析后的完整快照，单位已经换算成人能直接理解的 A/rpm/rad。
    struct Snapshot {
        bool hasFeedback{};            // 启动后是否至少收到过一帧反馈。
        bool enabled{};                // 电机反馈的使能状态。
        float currentA{};              // 实际电流，单位 A。
        float speedRpm{};              // 实际转速，单位 rpm。
        float angleRad{};              // 绝对角度，范围约 [0, 2π] rad。
        std::uint32_t feedbackCount{};  // 有效反馈帧累计数量。
        std::uint32_t lastFeedbackMs{}; // 最近反馈到达时的 HAL 毫秒时钟。
    };

    // hcan 指向所使用的 CAN 外设；id 必须与电机上位机 can.id 一致。
    explicit QD4310(CAN_HandleTypeDef *hcan, const std::uint8_t id) :
        id(id), hcan(hcan) {}

    // 使能/失能只改变电机工作状态，不等同于切断电源。
    void enable() { SendCommand(Command::ENABLE, 0x0000); }
    void disable() { SendCommand(Command::DISABLE, 0x0000); }

    // 在 CAN 接收中断里调用：解析固定 8 字节反馈并更新内部数据。
    void update(const std::uint8_t feedback[8]);

    // update() 在 CAN 中断写数据，snapshot() 在任务中读取；
    // snapshot() 会短暂关中断，确保读到的所有字段来自同一份完整反馈。
    [[nodiscard]] Snapshot snapshot() const;

    // 设置绝对角度，单位 rad；超出 [0, 2π] 时会被限制到边界。
    void setAngle(float angleRad);

    // 设置相对角度增量，单位 rad；范围限制为 [-2π, 2π]。
    void setStepAngle(float stepAngleRad);

    // 设置速度模式目标，单位 rpm；范围限制为 [-1000, 1000]。
    void setSpeed(float speedRpm);

    // 设置电机固件的低速模式目标，单位 rpm；范围限制为 [-1000, 1000]。
    void setLowSpeed(float speedRpm);

    // 设置电流模式目标，单位 A；范围限制为 [-10, 10]。
    // 若传入 NaN/Inf，会记录一次错误并改发 0 A。
    void setCurrent(float currentA);

    // 累计发送失败次数；上层状态机用它判断 CAN 命令通路是否异常。
    [[nodiscard]] std::uint32_t txErrorCount() const {
        return tx_error_count_.load(std::memory_order_relaxed);
    }

    // 只读的电机 CAN ID。
    const std::uint8_t id;

private:
    // QD4310 手册规定的第 1 字节命令码。
    enum class Command : std::uint8_t {
        NOP = 0x00,
        ENABLE = 0x01,
        DISABLE = 0x02,
        CURRENT = 0x03,
        SPEED = 0x04,
        ANGLE = 0x05,
        LOW_SPEED = 0x06,
        STEP_ANGLE = 0x07
    };

    CAN_HandleTypeDef *hcan{};                // 使用的 STM32 HAL CAN 句柄。
    std::atomic_uint32_t tx_error_count_{0U}; // 中断/任务均可安全更新。

    // 以下字段只由 CAN 接收中断写入，由 snapshot() 成组读取。
    volatile bool enabled_{};
    volatile bool has_feedback_{};
    volatile float current_a_{};
    volatile float speed_rpm_{};
    volatile float angle_rad_{};
    volatile std::uint32_t feedback_count_{};
    volatile std::uint32_t last_feedback_ms_{};

    // 组成 0x400 + id 的 3 字节控制帧并送进 CAN 邮箱。
    void SendCommand(Command cmd, std::int16_t value);
    bool rejectNonFinite(float value);
};

#endif
