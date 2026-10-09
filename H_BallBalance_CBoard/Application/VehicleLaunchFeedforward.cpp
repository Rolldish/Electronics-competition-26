#include "VehicleLaunchFeedforward.h"

#include "BallControlConfig.h"

#include <cmath>

namespace {
constexpr std::uint8_t kControlEnabled = 1U << 0U;
constexpr std::uint8_t kRunActive = 1U << 1U;
constexpr std::uint8_t kTerminalFlags = (1U << 2U) | (1U << 3U);

bool supportsLaunch(const std::uint8_t taskId)
{
    return taskId == 4U || taskId == 5U || taskId == 6U;
}
}

VehicleLaunchFeedforwardController::VehicleLaunchFeedforwardController(
    const VehicleLaunchFeedforwardConfig& config)
    : config_(config),
      configValid_(std::isfinite(config.angleOffsetDeg)
          && config.angleOffsetDeg != 0.0F
          && std::isfinite(config.accelerationThresholdMps2)
          && config.accelerationThresholdMps2 > 0.0F
          && config.requiredNewSamples > 0U
          && config.releaseMs > 0U
          && std::isfinite(config.maxTotalOffsetDeg)
          && config.maxTotalOffsetDeg >= std::fabs(config.angleOffsetDeg)
          && config.maxTotalOffsetDeg <=
                 ball_control_config::kMaximumTunableBreakawayAngleDeg)
{
}

VehicleLaunchFeedforwardOutput VehicleLaunchFeedforwardController::update(
    const VehicleLaunchFeedforwardInput& input)
{
    const bool newSample = input.imuSampleCount != 0U
        && (!sampleSeen_ || input.imuSampleCount != lastSampleCount_);
    sampleSeen_ = input.imuSampleCount != 0U;
    lastSampleCount_ = input.imuSampleCount;

    VehicleLaunchFeedforwardOutput output{};
    output.triggerCount = triggerCount_;
    if (!config_.enabled || !configValid_) {
        state_ = VehicleLaunchFeedforwardState::Disabled;
        return output;
    }

    const bool validTask = input.taskContextValid
        && supportsLaunch(input.taskId);
    if (validTask && (!identitySeen_
        || input.piSessionId != piSessionId_
        || input.runId != runId_ || input.taskId != taskId_)) {
        identitySeen_ = true;
        piSessionId_ = input.piSessionId;
        runId_ = input.runId;
        taskId_ = input.taskId;
        bool alreadyConsumed = false;
        for (const auto& run : consumedRuns_) {
            if (run.valid && run.piSessionId == piSessionId_
                && run.runId == runId_ && run.taskId == taskId_) {
                alreadyConsumed = true;
                break;
            }
        }
        state_ = alreadyConsumed
            ? VehicleLaunchFeedforwardState::Complete
            : VehicleLaunchFeedforwardState::Armed;
        consecutiveLaunchSamples_ = 0U;
        launchStartedMs_ = 0U;
        output.runChanged = true;
    }

    const bool terminal = (input.controlFlags & kTerminalFlags) != 0U;
    const bool allowed = validTask
        && (input.controlFlags & kControlEnabled) != 0U
        && (input.controlFlags & kRunActive) != 0U
        && !terminal && input.balancing && input.imuUsable
        && std::isfinite(input.accelerationMps2);
    if (!allowed) {
        consecutiveLaunchSamples_ = 0U;
        if (state_ == VehicleLaunchFeedforwardState::Active || terminal) {
            state_ = VehicleLaunchFeedforwardState::Cancelled;
        } else if (state_ == VehicleLaunchFeedforwardState::Qualifying) {
            state_ = VehicleLaunchFeedforwardState::Armed;
        }
        output.state = state_;
        return output;
    }

    if ((state_ == VehicleLaunchFeedforwardState::Armed
         || state_ == VehicleLaunchFeedforwardState::Qualifying)
        && newSample) {
        if (input.accelerationMps2 >= config_.accelerationThresholdMps2) {
            state_ = VehicleLaunchFeedforwardState::Qualifying;
            ++consecutiveLaunchSamples_;
            if (consecutiveLaunchSamples_ >= config_.requiredNewSamples) {
                state_ = VehicleLaunchFeedforwardState::Active;
                launchStartedMs_ = input.nowMs;
                ++triggerCount_;
                consumedRuns_[nextConsumedRun_] = ConsumedRun{
                    .piSessionId = piSessionId_,
                    .runId = runId_,
                    .taskId = taskId_,
                    .valid = true,
                };
                nextConsumedRun_ = (nextConsumedRun_ + 1U) % consumedRuns_.size();
            }
        } else {
            consecutiveLaunchSamples_ = 0U;
            state_ = VehicleLaunchFeedforwardState::Armed;
        }
    }

    if (state_ == VehicleLaunchFeedforwardState::Active) {
        // A launch bias must never oppose the existing brake compensation.
        if (input.accelerationMps2 < 0.0F) {
            state_ = VehicleLaunchFeedforwardState::Cancelled;
        } else {
            output.elapsedMs = input.nowMs - launchStartedMs_;
            if (output.elapsedMs < config_.holdMs) {
                output.active = true;
                output.angleOffsetDeg = config_.angleOffsetDeg;
            } else {
                const auto releaseElapsedMs = output.elapsedMs - config_.holdMs;
                if (releaseElapsedMs < config_.releaseMs) {
                    output.active = true;
                    output.angleOffsetDeg = config_.angleOffsetDeg
                        * (1.0F - static_cast<float>(releaseElapsedMs)
                            / static_cast<float>(config_.releaseMs));
                } else {
                    state_ = VehicleLaunchFeedforwardState::Complete;
                }
            }
        }
    }
    output.state = state_;
    output.triggerCount = triggerCount_;
    return output;
}
