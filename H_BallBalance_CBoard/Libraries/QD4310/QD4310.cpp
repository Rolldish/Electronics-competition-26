#include "QD4310.h"

#include <algorithm>
#include <bit>
#include <climits>
#include <cmath>
#include <numbers>

namespace {

// 等待 CAN 发送邮箱的最大轮询次数；超限后记错并返回，不让控制任务永久卡死。
constexpr std::uint32_t kMailboxWaitIterations = 10000U;

// QD4310 多字节数据采用小端序：低字节在前，高字节在后。
std::uint16_t decodeUnsignedLe16(const std::uint8_t *bytes) {
    return static_cast<std::uint16_t>(bytes[0]) |
           (static_cast<std::uint16_t>(bytes[1]) << 8U);
}

std::int16_t decodeSignedLe16(const std::uint8_t *bytes) {
    return std::bit_cast<std::int16_t>(decodeUnsignedLe16(bytes));
}

} // namespace

static_assert(std::atomic_uint32_t::is_always_lock_free);

bool QD4310::rejectNonFinite(const float value) {
    if (std::isfinite(value)) {
        return false;
    }
    tx_error_count_.fetch_add(1U, std::memory_order_relaxed);
    return true;
}

void QD4310::SendCommand(
    const Command cmd,
    const std::int16_t value) {
    // 控制帧共 3 字节：[命令码][数值低字节][数值高字节]。
    const std::uint16_t rawValue = std::bit_cast<std::uint16_t>(value);
    std::uint8_t txBuffer[3]{
        static_cast<std::uint8_t>(cmd),
        static_cast<std::uint8_t>(rawValue & 0xFFU),
        static_cast<std::uint8_t>((rawValue >> 8U) & 0xFFU),
    };

    CAN_TxHeaderTypeDef txHeader{};
    // 电机 ID 为 0 时控制帧 StdId = 0x400。
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.StdId = 0x400U + id;
    txHeader.ExtId = 0U;
    txHeader.TransmitGlobalTime = DISABLE;
    txHeader.DLC = sizeof(txBuffer);

    // 等空邮箱时保持中断开启；真正写邮箱的极短临界区才关中断，
    // 防止不同上下文同时争用同一个 HAL CAN 发送邮箱。
    for (std::uint32_t remaining = kMailboxWaitIterations;
         remaining > 0U;
         --remaining) {
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0U) {
            continue;
        }

        const std::uint32_t previousPrimask = __get_PRIMASK();
        __disable_irq();

        HAL_StatusTypeDef status = HAL_BUSY;
        if (HAL_CAN_GetTxMailboxesFreeLevel(hcan) > 0U) {
            std::uint32_t txMailbox = 0U;
            status = HAL_CAN_AddTxMessage(
                hcan, &txHeader, txBuffer, &txMailbox);
        }

        if (previousPrimask == 0U) {
            __enable_irq();
        }

        if (status == HAL_OK) {
            return;
        }
        if (status != HAL_BUSY) {
            break;
        }
    }

    // CAN 异常时不永久等待，让控制循环继续运行并留下诊断计数。
    tx_error_count_.fetch_add(1U, std::memory_order_relaxed);
}

void QD4310::update(const std::uint8_t feedback[8]) {
    // 反馈格式：
    // byte 0      -> 状态位（bit0 为使能）
    // byte 2..3   -> 有符号电流，映射到 ±10 A
    // byte 4..5   -> 有符号速度，映射到 ±1000 rpm
    // byte 6..7   -> 无符号绝对角度，映射到 0..2π
    enabled_ = (feedback[0] & 0x01U) != 0U;
    current_a_ =
        static_cast<float>(decodeSignedLe16(feedback + 2)) *
        10.0f / static_cast<float>(INT16_MAX);
    speed_rpm_ =
        static_cast<float>(decodeSignedLe16(feedback + 4)) *
        1000.0f / static_cast<float>(INT16_MAX);
    angle_rad_ =
        static_cast<float>(decodeUnsignedLe16(feedback + 6)) *
        2.0f * std::numbers::pi_v<float> /
        static_cast<float>(UINT16_MAX);
    last_feedback_ms_ = HAL_GetTick();
    feedback_count_ = feedback_count_ + 1U;
    has_feedback_ = true;
}

QD4310::Snapshot QD4310::snapshot() const {
    // 单核 Cortex-M4 上，update() 只在 CAN 中断写入，snapshot() 只在任务读取。
    // 短暂屏蔽中断可防止“读到一半时新反馈插入”，保证各字段属于同一帧。
    const std::uint32_t previousPrimask = __get_PRIMASK();
    __disable_irq();

    const Snapshot result{
        has_feedback_,
        enabled_,
        current_a_,
        speed_rpm_,
        angle_rad_,
        feedback_count_,
        last_feedback_ms_,
    };

    if (previousPrimask == 0U) {
        __enable_irq();
    }
    return result;
}

void QD4310::setAngle(const float angleRad) {
    if (rejectNonFinite(angleRad)) {
        return;
    }
    // 把 0..2π rad 线性映射到 0..65535。
    const float clamped = std::clamp(
        angleRad, 0.0f, 2.0f * std::numbers::pi_v<float>);
    const auto encoded = static_cast<std::uint16_t>(
        clamped / (2.0f * std::numbers::pi_v<float>) *
        static_cast<float>(UINT16_MAX));
    SendCommand(Command::ANGLE, std::bit_cast<std::int16_t>(encoded));
}

void QD4310::setStepAngle(const float stepAngleRad) {
    if (rejectNonFinite(stepAngleRad)) {
        return;
    }
    // 把 -2π..2π rad 线性映射到 int16_t。
    const float clamped = std::clamp(
        stepAngleRad,
        -2.0f * std::numbers::pi_v<float>,
        2.0f * std::numbers::pi_v<float>);
    SendCommand(
        Command::STEP_ANGLE,
        static_cast<std::int16_t>(
            clamped / (2.0f * std::numbers::pi_v<float>) *
            static_cast<float>(INT16_MAX)));
}

void QD4310::setSpeed(const float speedRpm) {
    if (rejectNonFinite(speedRpm)) {
        return;
    }
    // 把 -1000..1000 rpm 线性映射到 int16_t。
    const float clamped = std::clamp(speedRpm, -1000.0f, 1000.0f);
    SendCommand(
        Command::SPEED,
        static_cast<std::int16_t>(
            clamped / 1000.0f * static_cast<float>(INT16_MAX)));
}

void QD4310::setLowSpeed(const float speedRpm) {
    if (rejectNonFinite(speedRpm)) {
        return;
    }
    const float clamped = std::clamp(speedRpm, -1000.0f, 1000.0f);
    SendCommand(
        Command::LOW_SPEED,
        static_cast<std::int16_t>(
            clamped / 1000.0f * static_cast<float>(INT16_MAX)));
}

void QD4310::setCurrent(const float currentA) {
    // 非法浮点数绝不能直接编码发给电机，改发 0 A 并让上层看到错误计数。
    if (rejectNonFinite(currentA)) {
        SendCommand(Command::CURRENT, 0);
        return;
    }

    // 把 -10..10 A 线性映射到 int16_t。
    const float clamped = std::clamp(currentA, -10.0f, 10.0f);
    SendCommand(
        Command::CURRENT,
        static_cast<std::int16_t>(
            clamped / 10.0f * static_cast<float>(INT16_MAX)));
}
