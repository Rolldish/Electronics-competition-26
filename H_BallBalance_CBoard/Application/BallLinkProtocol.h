#ifndef APPLICATION_BALL_LINK_PROTOCOL_H
#define APPLICATION_BALL_LINK_PROTOCOL_H

#include <array>
#include <cstddef>
#include <cstdint>

constexpr std::uint8_t kBallLinkMagic0 = 0xA5U;
constexpr std::uint8_t kBallLinkMagic1 = 0x5AU;
constexpr std::uint8_t kBallLinkVersion = 0x02U;
constexpr std::size_t kBallLinkMaxPayload = 32U;
constexpr std::size_t kBallLinkFixedSize = 8U;
constexpr std::size_t kBallLinkMaxFrameSize =
    kBallLinkFixedSize + kBallLinkMaxPayload;
constexpr std::uint8_t kBallLinkVisionPayloadSize = 24U;
constexpr std::size_t kBallLinkVisionFrameSize =
    kBallLinkFixedSize + kBallLinkVisionPayloadSize;

enum class BallLinkMessageType : std::uint8_t {
    VisionControlSnapshot = 0x10U,
};

enum class BallLinkVisionState : std::uint8_t {
    NoLink = 0U,
    MeasurementInvalid = 1U,
    MeasurementValid = 2U,
    Stale = 3U,
};

enum BallLinkVisionFlags : std::uint8_t {
    BallLinkBallValid = 1U << 0U,
    BallLinkVelocityValid = 1U << 1U,
    BallLinkCameraCalibrated = 1U << 2U,
    BallLinkProcessingDegraded = 1U << 3U,
};

constexpr std::uint8_t kBallLinkKnownVisionFlags =
    BallLinkBallValid | BallLinkVelocityValid |
    BallLinkCameraCalibrated | BallLinkProcessingDegraded;

enum BallLinkControlFlags : std::uint8_t {
    BallLinkControlEnabled = 1U << 0U,
    BallLinkRunActive = 1U << 1U,
    BallLinkTaskDone = 1U << 2U,
    BallLinkTaskTimeout = 1U << 3U,
    BallLinkTargetLatched = 1U << 4U,
};

constexpr std::uint8_t kBallLinkKnownControlFlags =
    BallLinkControlEnabled | BallLinkRunActive |
    BallLinkTaskDone | BallLinkTaskTimeout |
    BallLinkTargetLatched;

struct BallLinkFrame {
    std::uint8_t type = 0U;
    std::uint8_t sequence = 0U;
    std::uint8_t length = 0U;
    std::array<std::uint8_t, kBallLinkMaxPayload> payload{};
};

struct BallLinkVisionMeasurement {
    std::uint32_t piSessionId = 0U;
    std::uint32_t captureTimestampMs = 0U;
    std::uint32_t captureToSendDelayUs = 0U;
    std::int16_t position0p1Mm = 0;
    std::int16_t velocityMmps = 0;
    std::uint8_t confidence = 0U;
    std::uint8_t visionFlags = 0U;
    std::int16_t targetPosition0p1Mm = 0;
    std::uint8_t taskId = 0U;
    std::uint8_t controlFlags = 0U;
    std::uint16_t runId = 0U;
};

std::uint16_t ballLinkCrc16CcittFalse(
    const std::uint8_t *data, std::size_t length);

std::size_t ballLinkEncodeFrame(
    BallLinkMessageType type, std::uint8_t sequence,
    const std::uint8_t *payload, std::uint8_t payloadLength,
    std::uint8_t *output, std::size_t outputCapacity);

bool ballLinkDecodeVisionMeasurement(
    const BallLinkFrame &frame,
    BallLinkVisionMeasurement &measurement);

#endif /* APPLICATION_BALL_LINK_PROTOCOL_H */
