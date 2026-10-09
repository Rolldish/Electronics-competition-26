#pragma once

#include "ImuFeedforwardController.h"
#include "ImuVehicleSnapshotStore.h"
#include "VehicleLaunchFeedforward.h"

#include <cstdint>

enum class CompetitionImuFeedforwardGate : std::uint8_t {
    Applied,
    Disabled,
    InvalidTaskContext,
    UnsupportedTask,
    ControlNotEnabled,
    RunNotActive,
    NotBalancing,
    ImuNotReady,
    ImuInvalid,
    ImuNotFinite,
    ImuStale,
    InvalidControlOutput,
};

struct CompetitionImuFeedforwardConfig {
    bool enabled{true};
    std::uint32_t staleTimeoutMs{100U};
    ImuFeedforwardConfig feedforward{};
    float task4StartAccelerationGain{1.0F};
    float task4BrakeAccelerationGain{1.0F};
    float task4MaxOffsetDeg{3.0F};
    float task4BrakeDetectAccelerationMps2{-0.15F};
    float task4StopDetectAccelerationMps2{-0.05F};
    float task4PostStopOffsetDeg{4.0F};
    std::uint32_t task4PostStopHoldMs{300U};
    std::uint32_t task4PostStopReleaseMs{700U};
    VehicleLaunchFeedforwardConfig launch{};
};

struct CompetitionImuFeedforwardInput {
    std::uint32_t nowMs{};
    bool taskContextValid{};
    std::uint8_t taskId{};
    std::uint8_t controlFlags{};
    bool balancing{};
    ImuVehicleControlSnapshot imu{};
    float pdOffsetDeg{};
    float activeAngleLimitDeg{};
    std::uint32_t piSessionId{};
    std::uint16_t runId{};
};

struct CompetitionImuFeedforwardOutput {
    CompetitionImuFeedforwardGate gate{
        CompetitionImuFeedforwardGate::Disabled};
    bool applied{};
    bool sampleFresh{};
    std::uint32_t sampleAgeMs{};
    float accelerationMps2{};
    float feedforwardOffsetDeg{};
    float pdOffsetDeg{};
    float totalOffsetDeg{};
    std::uint32_t invalidImuCycleCount{};
    std::uint32_t staleImuCycleCount{};
    VehicleLaunchFeedforwardOutput launch{};
};

class CompetitionImuFeedforwardController {
public:
    explicit CompetitionImuFeedforwardController(
        const CompetitionImuFeedforwardConfig& config);
    CompetitionImuFeedforwardOutput update(
        const CompetitionImuFeedforwardInput& input);

private:
    enum class Task4PostStopState : std::uint8_t {
        WaitingForBrake,
        WaitingForStop,
        Active,
        Complete,
    };

    void resetTask4PostStopCompensation();

    CompetitionImuFeedforwardOutput reject(
        CompetitionImuFeedforwardGate gate,
        const CompetitionImuFeedforwardInput& input,
        bool sampleFresh,
        std::uint32_t sampleAgeMs);

    CompetitionImuFeedforwardConfig config_{};
    ImuFeedforwardController feedforwardController_;
    ImuFeedforwardController task4StartFeedforwardController_;
    ImuFeedforwardController task4BrakeFeedforwardController_;
    VehicleLaunchFeedforwardController launchController_;
    VehicleLaunchFeedforwardOutput launchOutput_{};
    Task4PostStopState task4PostStopState_{
        Task4PostStopState::WaitingForBrake};
    std::uint32_t task4PostStopStartMs_{};
    bool sampleSeen_{};
    std::uint32_t lastSampleCount_{};
    std::uint32_t lastNewSampleMs_{};
    std::uint32_t invalidImuCycleCount_{};
    std::uint32_t staleImuCycleCount_{};
};
