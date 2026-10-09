#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

enum class VehicleLaunchFeedforwardState : std::uint8_t {
    Disabled,
    Armed,
    Qualifying,
    Active,
    Complete,
    Cancelled,
};

struct VehicleLaunchFeedforwardConfig {
    bool enabled{true};
    float angleOffsetDeg{-2.0F};
    float accelerationThresholdMps2{0.06F};
    std::uint32_t requiredNewSamples{2U};
    std::uint32_t holdMs{100U};
    std::uint32_t releaseMs{700U};
    float maxTotalOffsetDeg{4.0F};
};

struct VehicleLaunchFeedforwardInput {
    std::uint32_t nowMs{};
    std::uint32_t piSessionId{};
    std::uint16_t runId{};
    std::uint8_t taskId{};
    std::uint8_t controlFlags{};
    bool taskContextValid{};
    bool balancing{};
    bool imuUsable{};
    std::uint32_t imuSampleCount{};
    float accelerationMps2{};
};

struct VehicleLaunchFeedforwardOutput {
    VehicleLaunchFeedforwardState state{
        VehicleLaunchFeedforwardState::Disabled};
    bool active{};
    bool runChanged{};
    float angleOffsetDeg{};
    std::uint32_t elapsedMs{};
    std::uint32_t triggerCount{};
};

// RUN_ACTIVE arms detection; it is deliberately not a launch trigger because
// the Pi START and the vehicle's physical start button are independent.
class VehicleLaunchFeedforwardController {
public:
    explicit VehicleLaunchFeedforwardController(
        const VehicleLaunchFeedforwardConfig& config);
    VehicleLaunchFeedforwardOutput update(
        const VehicleLaunchFeedforwardInput& input);

private:
    struct ConsumedRun {
        std::uint32_t piSessionId{};
        std::uint16_t runId{};
        std::uint8_t taskId{};
        bool valid{};
    };

    VehicleLaunchFeedforwardConfig config_{};
    bool configValid_{};
    VehicleLaunchFeedforwardState state_{
        VehicleLaunchFeedforwardState::Disabled};
    bool identitySeen_{};
    std::uint32_t piSessionId_{};
    std::uint16_t runId_{};
    std::uint8_t taskId_{};
    bool sampleSeen_{};
    std::uint32_t lastSampleCount_{};
    std::uint32_t consecutiveLaunchSamples_{};
    std::uint32_t launchStartedMs_{};
    std::uint32_t triggerCount_{};
    // Reject recent run-identity rollbacks without heap allocation in the
    // 200 Hz task. Normal Pi runs receive a new random runId each time.
    std::array<ConsumedRun, 8U> consumedRuns_{};
    std::size_t nextConsumedRun_{};
};
