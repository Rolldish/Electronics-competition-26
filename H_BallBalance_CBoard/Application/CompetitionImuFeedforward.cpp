#include "CompetitionImuFeedforward.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr std::uint8_t kControlEnabled = 1U << 0U;
constexpr std::uint8_t kRunActive = 1U << 1U;
constexpr std::uint32_t kImuReadyStatus = 1U;

bool has(const std::uint8_t flags, const std::uint8_t mask)
{
    return (flags & mask) != 0U;
}

bool taskSupportsFeedforward(const std::uint8_t taskId)
{
    return taskId == 4U || taskId == 5U || taskId == 6U;
}

ImuFeedforwardConfig task4FeedforwardConfig(
    const CompetitionImuFeedforwardConfig& config,
    const float accelerationGain)
{
    ImuFeedforwardConfig task4Config = config.feedforward;
    task4Config.accelerationGain = accelerationGain;
    task4Config.maxOffsetDeg = config.task4MaxOffsetDeg;
    return task4Config;
}
}

CompetitionImuFeedforwardController::CompetitionImuFeedforwardController(
    const CompetitionImuFeedforwardConfig& config)
    : config_(config),
      feedforwardController_(config.feedforward),
      task4StartFeedforwardController_(
          task4FeedforwardConfig(
              config, config.task4StartAccelerationGain)),
      task4BrakeFeedforwardController_(
          task4FeedforwardConfig(
              config, config.task4BrakeAccelerationGain)),
      launchController_(config.launch)
{
}

CompetitionImuFeedforwardOutput CompetitionImuFeedforwardController::update(
    const CompetitionImuFeedforwardInput& input)
{
    if (!sampleSeen_ || input.imu.sampleCount != lastSampleCount_) {
        sampleSeen_ = input.imu.sampleCount != 0U;
        lastSampleCount_ = input.imu.sampleCount;
        lastNewSampleMs_ = input.nowMs;
    }
    const std::uint32_t sampleAgeMs = sampleSeen_
        ? input.nowMs - lastNewSampleMs_
        : config_.staleTimeoutMs + 1U;
    const bool sampleFresh =
        sampleSeen_ && sampleAgeMs <= config_.staleTimeoutMs;

    const bool controlOutputValid = std::isfinite(input.pdOffsetDeg)
        && std::isfinite(input.activeAngleLimitDeg)
        && input.activeAngleLimitDeg > 0.0F;
    launchOutput_ = launchController_.update(VehicleLaunchFeedforwardInput{
        .nowMs = input.nowMs,
        .piSessionId = input.piSessionId,
        .runId = input.runId,
        .taskId = input.taskId,
        .controlFlags = input.controlFlags,
        .taskContextValid = config_.enabled && input.taskContextValid,
        .balancing = input.balancing && controlOutputValid,
        .imuUsable = input.imu.status == kImuReadyStatus
            && input.imu.calibrated && input.imu.valid && sampleFresh,
        .imuSampleCount = input.imu.sampleCount,
        .accelerationMps2 = input.imu.forwardAccelerationMps2,
    });
    if (launchOutput_.runChanged) {
        resetTask4PostStopCompensation();
    }

    if (!config_.enabled) {
        return reject(CompetitionImuFeedforwardGate::Disabled,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!input.taskContextValid) {
        return reject(CompetitionImuFeedforwardGate::InvalidTaskContext,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!taskSupportsFeedforward(input.taskId)) {
        return reject(CompetitionImuFeedforwardGate::UnsupportedTask,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!has(input.controlFlags, kControlEnabled)) {
        return reject(CompetitionImuFeedforwardGate::ControlNotEnabled,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!has(input.controlFlags, kRunActive)) {
        return reject(CompetitionImuFeedforwardGate::RunNotActive,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!input.balancing) {
        return reject(CompetitionImuFeedforwardGate::NotBalancing,
                      input, sampleFresh, sampleAgeMs);
    }
    if (input.imu.status != kImuReadyStatus || !input.imu.calibrated) {
        return reject(CompetitionImuFeedforwardGate::ImuNotReady,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!input.imu.valid) {
        return reject(CompetitionImuFeedforwardGate::ImuInvalid,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!std::isfinite(input.imu.forwardAccelerationMps2)) {
        return reject(CompetitionImuFeedforwardGate::ImuNotFinite,
                      input, sampleFresh, sampleAgeMs);
    }
    if (!sampleFresh) {
        return reject(CompetitionImuFeedforwardGate::ImuStale,
                      input, false, sampleAgeMs);
    }
    if (!controlOutputValid) {
        return reject(CompetitionImuFeedforwardGate::InvalidControlOutput,
                      input, true, sampleAgeMs);
    }

    if (input.taskId != 4U) {
        resetTask4PostStopCompensation();
    }

    bool task4PostStopActive = false;
    float task4PostStopOffsetDeg = 0.0F;
    if (input.taskId == 4U) {
        const bool task4PostStopConfigValid =
            std::isfinite(config_.task4BrakeDetectAccelerationMps2)
            && std::isfinite(config_.task4StopDetectAccelerationMps2)
            && config_.task4BrakeDetectAccelerationMps2 <
                   config_.task4StopDetectAccelerationMps2
            && config_.task4StopDetectAccelerationMps2 <= 0.0F
            && std::isfinite(config_.task4PostStopOffsetDeg)
            && config_.task4PostStopOffsetDeg >= 0.0F
            && config_.task4PostStopReleaseMs > 0U;
        if (!task4PostStopConfigValid) {
            resetTask4PostStopCompensation();
        } else {
            const float accelerationMps2 =
                input.imu.forwardAccelerationMps2;
            if (task4PostStopState_ ==
                    Task4PostStopState::WaitingForBrake
                && accelerationMps2 <=
                       config_.task4BrakeDetectAccelerationMps2) {
                task4PostStopState_ =
                    Task4PostStopState::WaitingForStop;
            }
            if (task4PostStopState_ ==
                    Task4PostStopState::WaitingForStop
                && accelerationMps2 >=
                       config_.task4StopDetectAccelerationMps2) {
                task4PostStopState_ = Task4PostStopState::Active;
                task4PostStopStartMs_ = input.nowMs;
            }
            if (task4PostStopState_ == Task4PostStopState::Active) {
                const std::uint32_t elapsedMs =
                    input.nowMs - task4PostStopStartMs_;
                if (elapsedMs < config_.task4PostStopHoldMs) {
                    task4PostStopActive = true;
                    task4PostStopOffsetDeg =
                        config_.task4PostStopOffsetDeg;
                } else {
                    const std::uint32_t releaseElapsedMs =
                        elapsedMs - config_.task4PostStopHoldMs;
                    if (releaseElapsedMs <
                        config_.task4PostStopReleaseMs) {
                        const float releaseRatio =
                            static_cast<float>(releaseElapsedMs)
                            / static_cast<float>(
                                config_.task4PostStopReleaseMs);
                        task4PostStopActive = true;
                        task4PostStopOffsetDeg =
                            config_.task4PostStopOffsetDeg
                            * (1.0F - releaseRatio);
                    } else {
                        task4PostStopState_ =
                            Task4PostStopState::Complete;
                    }
                }
            }
            if (task4PostStopState_ == Task4PostStopState::Complete
                && accelerationMps2 >=
                       -config_.task4BrakeDetectAccelerationMps2) {
                resetTask4PostStopCompensation();
            }
        }
    }

    const ImuFeedforwardController* activeFeedforwardController =
        &feedforwardController_;
    if (input.taskId == 4U) {
        activeFeedforwardController =
            input.imu.forwardAccelerationMps2 >= 0.0F
            ? &task4StartFeedforwardController_
            : &task4BrakeFeedforwardController_;
    }
    const ImuFeedforwardOutput feedforward =
        activeFeedforwardController->update(
            true, input.imu.forwardAccelerationMps2);
    if (!feedforward.valid || !std::isfinite(feedforward.angleOffsetDeg)) {
        return reject(CompetitionImuFeedforwardGate::ImuNotFinite,
                      input, true, sampleAgeMs);
    }
    const float appliedFeedforwardOffsetDeg = task4PostStopActive
        ? task4PostStopOffsetDeg
        : feedforward.angleOffsetDeg;
    float totalAngleLimitDeg = task4PostStopActive
        ? std::max(input.activeAngleLimitDeg,
                   config_.task4PostStopOffsetDeg)
        : input.activeAngleLimitDeg;
    if (launchOutput_.active) {
        totalAngleLimitDeg = std::max(
            totalAngleLimitDeg, config_.launch.maxTotalOffsetDeg);
    }
    const float totalOffsetDeg = std::clamp(
        input.pdOffsetDeg + appliedFeedforwardOffsetDeg
            + launchOutput_.angleOffsetDeg,
        -totalAngleLimitDeg,
        totalAngleLimitDeg);
    return CompetitionImuFeedforwardOutput{
        .gate = CompetitionImuFeedforwardGate::Applied,
        .applied = true,
        .sampleFresh = true,
        .sampleAgeMs = sampleAgeMs,
        .accelerationMps2 = input.imu.forwardAccelerationMps2,
        .feedforwardOffsetDeg = appliedFeedforwardOffsetDeg,
        .pdOffsetDeg = input.pdOffsetDeg,
        .totalOffsetDeg = totalOffsetDeg,
        .invalidImuCycleCount = invalidImuCycleCount_,
        .staleImuCycleCount = staleImuCycleCount_,
        .launch = launchOutput_,
    };
}

void CompetitionImuFeedforwardController::resetTask4PostStopCompensation()
{
    task4PostStopState_ = Task4PostStopState::WaitingForBrake;
    task4PostStopStartMs_ = 0U;
}

CompetitionImuFeedforwardOutput CompetitionImuFeedforwardController::reject(
    const CompetitionImuFeedforwardGate gate,
    const CompetitionImuFeedforwardInput& input,
    const bool sampleFresh,
    const std::uint32_t sampleAgeMs)
{
    resetTask4PostStopCompensation();
    if (gate == CompetitionImuFeedforwardGate::ImuNotReady
        || gate == CompetitionImuFeedforwardGate::ImuInvalid
        || gate == CompetitionImuFeedforwardGate::ImuNotFinite) {
        ++invalidImuCycleCount_;
    }
    if (gate == CompetitionImuFeedforwardGate::ImuStale) {
        ++staleImuCycleCount_;
    }
    return CompetitionImuFeedforwardOutput{
        .gate = gate,
        .applied = false,
        .sampleFresh = sampleFresh,
        .sampleAgeMs = sampleAgeMs,
        .accelerationMps2 = input.imu.forwardAccelerationMps2,
        .feedforwardOffsetDeg = 0.0F,
        .pdOffsetDeg = input.pdOffsetDeg,
        .totalOffsetDeg = input.pdOffsetDeg,
        .invalidImuCycleCount = invalidImuCycleCount_,
        .staleImuCycleCount = staleImuCycleCount_,
        .launch = launchOutput_,
    };
}
