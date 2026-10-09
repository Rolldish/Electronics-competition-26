#include "BallBalanceController.h"

#include "BallControlConfig.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kRadiansToDegrees = 180.0F / kPi;
constexpr float kDegreesToRadians = kPi / 180.0F;
constexpr float kTwoPi = 2.0F * kPi;

float clampValue(const float value, const float minimum, const float maximum)
{
    return std::max(minimum, std::min(value, maximum));
}

float normalizeAbsoluteAngle(float radians)
{
    radians = std::fmod(radians, kTwoPi);
    if (radians < 0.0F) {
        radians += kTwoPi;
    }
    return radians;
}

float wrappedAngleDifference(const float angle, const float reference)
{
    return std::remainder(angle - reference, kTwoPi);
}

} // namespace

BallBalanceController::BallBalanceController()
    : BallBalanceController(BallControlRuntimeConfig{
          ball_control_config::kMechanicalLevelAngleRad,
          ball_control_config::kAngleSign,
          ball_control_config::kMaxAngleOffsetDeg,
          ball_control_config::kMechanicalAngleLimitDeg,
          ball_control_config::kMaxMotorSpeedRpm,
          ball_control_config::kMaxMotorCurrentA})
{
}

BallBalanceController::BallBalanceController(
    const BallControlRuntimeConfig& config)
    : positionKpDegPerMm_(ball_control_config::kKpDegPerMm),
      positionKdDegPerMmps_(ball_control_config::kKdDegPerMmps),
      breakawayAngleDeg_(ball_control_config::kBreakawayAngleDeg),
      stuckErrorMm_(ball_control_config::kStuckErrorThresholdMm),
      stuckSpeedMmps_(ball_control_config::kStuckSpeedThresholdMmps),
      stuckTimeMs_(ball_control_config::kStuckQualificationMs),
      breakawayDurationMs_(ball_control_config::kBreakawayDurationMs),
      activeAngleLimitDeg_(config.maxAngleOffsetDeg),
      config_(config)
{
}

bool BallBalanceController::setMechanicalLevelAngleRad(const float angleRad)
{
    if (!std::isfinite(angleRad)
        || motorReady_
        || state_ == BallControlState::EnablePending
        || state_ == BallControlState::LevelHold
        || state_ == BallControlState::Balancing
        || state_ == BallControlState::VisionRecovery
        || state_ == BallControlState::Fault) {
        return false;
    }

    config_.mechanicalLevelAngleRad = normalizeAbsoluteAngle(angleRad);
    commandedOffsetDeg_ = 0.0F;
    return true;
}

bool BallBalanceController::setPositionGains(const float kpDegPerMm,
                                             const float kdDegPerMmps)
{
    if (!std::isfinite(kpDegPerMm)
        || !std::isfinite(kdDegPerMmps)
        || kpDegPerMm < ball_control_config::kMinimumTunableKpDegPerMm
        || kpDegPerMm > ball_control_config::kMaximumTunableKpDegPerMm
        || kdDegPerMmps
               < ball_control_config::kMinimumTunableKdDegPerMmps
        || kdDegPerMmps
               > ball_control_config::kMaximumTunableKdDegPerMmps) {
        return false;
    }

    positionKpDegPerMm_ = kpDegPerMm;
    positionKdDegPerMmps_ = kdDegPerMmps;
    return true;
}

bool BallBalanceController::setStictionParameters(
    const float breakawayAngleDeg,
    const float stuckErrorMm,
    const float stuckSpeedMmps,
    const std::uint32_t stuckTimeMs,
    const std::uint32_t breakawayDurationMs)
{
    if (!std::isfinite(breakawayAngleDeg)
        || !std::isfinite(stuckErrorMm)
        || !std::isfinite(stuckSpeedMmps)
        || breakawayAngleDeg < config_.maxAngleOffsetDeg
        || breakawayAngleDeg
               > ball_control_config::kMaximumTunableBreakawayAngleDeg
        || stuckErrorMm
               < ball_control_config::kMinimumTunableStuckErrorMm
        || stuckErrorMm
               > ball_control_config::kMaximumTunableStuckErrorMm
        || stuckSpeedMmps
               < ball_control_config::kMinimumTunableStuckSpeedMmps
        || stuckSpeedMmps
               > ball_control_config::kMaximumTunableStuckSpeedMmps
        || stuckTimeMs
               < ball_control_config::kMinimumTunableStuckTimeMs
        || stuckTimeMs
               > ball_control_config::kMaximumTunableStuckTimeMs
        || breakawayDurationMs
               < ball_control_config::kMinimumTunableBreakawayDurationMs
        || breakawayDurationMs
               > ball_control_config::kMaximumTunableBreakawayDurationMs) {
        return false;
    }

    breakawayAngleDeg_ = breakawayAngleDeg;
    stuckErrorMm_ = stuckErrorMm;
    stuckSpeedMmps_ = stuckSpeedMmps;
    stuckTimeMs_ = stuckTimeMs;
    breakawayDurationMs_ = breakawayDurationMs;
    if (stictionState_ == BallStictionState::Breakaway) {
        activeAngleLimitDeg_ = breakawayAngleDeg_;
    }
    return true;
}

BallControlOutput BallBalanceController::makeOutput(
    const BallControlInput& input) const
{
    BallControlOutput output{};
    output.state = state_;
    output.fault = fault_;
    output.mechanicalLevelAngleRad = config_.mechanicalLevelAngleRad;
    output.actualAngleRad = input.motorAngleRad;
    output.targetAngleRad = normalizeAbsoluteAngle(
        config_.mechanicalLevelAngleRad
        + commandedOffsetDeg_ * kDegreesToRadians);
    output.angleOffsetDeg = commandedOffsetDeg_;
    output.targetPosition0p1mm = input.vision.target_position_0p1mm;
    output.position0p1mm = input.vision.position_0p1mm;
    output.velocityMmps = input.vision.velocity_valid
                              ? static_cast<float>(input.vision.velocity_mmps)
                              : 0.0F;
    output.motorSpeedRpm = input.motorSpeedRpm;
    output.motorCurrentA = input.motorCurrentA;
    output.motorEnableConfirmations = motorEnableConfirmations_;
    output.controlElapsedMs =
        updateTimeInitialized_ ? input.nowMs - lastUpdateMs_ : 0U;
    output.consecutiveValidMeasurements = consecutiveValidMeasurements_;
    output.measurementUpdateCount = input.vision.measurement_update_count;
    output.stictionState = stictionState_;
    output.stuckElapsedMs = stuckElapsedMs_;
    output.activeAngleLimitDeg = activeAngleLimitDeg_;
    output.centerHoldActive = centerHoldActive_;
    output.taskId = input.vision.task_id;
    output.runId = input.vision.run_id;
    return output;
}

BallControlOutput BallBalanceController::safeShutdownOutput(
    const BallControlInput& input,
    const bool force)
{
    BallControlOutput output = makeOutput(input);
    const bool retryDue =
        !shutdownCommandInitialized_
        || (input.nowMs - lastShutdownCommandMs_)
               >= ball_control_config::kSafeShutdownRetryMs;
    if (force || retryDue) {
        output.zeroOutput = true;
        output.disableMotor = true;
        shutdownCommandInitialized_ = true;
        lastShutdownCommandMs_ = input.nowMs;
    }
    output.sendAngleCommand = false;
    return output;
}

BallControlOutput BallBalanceController::angleHoldOutput(
    const BallControlInput& input,
    const float desiredOffsetDeg,
    const bool requestEnable)
{
    std::uint32_t elapsedMs = 0U;
    if (updateTimeInitialized_) {
        elapsedMs = input.nowMs - lastUpdateMs_;
    }
    elapsedMs = std::min(
        elapsedMs, ball_control_config::kMaxSlewElapsedMs);

    const float maxStep =
        ball_control_config::kTargetSlewRateDegPerSecond
        * static_cast<float>(elapsedMs) / 1000.0F;
    const float delta = clampValue(
        desiredOffsetDeg - commandedOffsetDeg_, -maxStep, maxStep);
    commandedOffsetDeg_ += delta;

    BallControlOutput output = makeOutput(input);
    output.enableMotor = requestEnable;
    output.sendAngleCommand = true;
    lastUpdateMs_ = input.nowMs;
    updateTimeInitialized_ = true;
    return output;
}

BallControlOutput BallBalanceController::enterFault(
    const BallControlInput& input,
    const BallControlFault fault)
{
    if (state_ != BallControlState::Fault) {
        state_ = BallControlState::Fault;
        fault_ = fault;
    }
    return safeShutdownOutput(input, true);
}

bool BallBalanceController::visionQualityIsUsable(
    const BallControlInput& input) const
{
    const BallVisionSnapshot& vision = input.vision;
    const bool controlEnabled =
        (vision.control_flags & ball_control_config::kControlEnabled) != 0U;
    const bool ballValid =
        (vision.vision_flags & ball_control_config::kVisionBallValid) != 0U;
    const bool calibrated =
        (vision.vision_flags
         & ball_control_config::kVisionCameraCalibrated) != 0U;
    const bool degraded =
        (vision.vision_flags
         & ball_control_config::kVisionProcessingDegraded) != 0U;
    const bool linkFresh =
        (input.nowMs - vision.local_receive_ms)
        <= ball_control_config::kVisionSilenceTimeoutMs;

    return controlEnabled
        && ballValid
        && calibrated
        && !degraded
        && vision.confidence >= ball_control_config::kMinimumVisionConfidence
        && linkFresh;
}

bool BallBalanceController::visionIsFreshForControl(
    const BallControlInput& input) const
{
    return input.vision.valid
        && visionQualityIsUsable(input)
        && input.vision.measurement_age_ms
               <= ball_control_config::kVisionMaxMeasurementAgeMs;
}

bool BallBalanceController::observeNewVisionMeasurement(
    const BallControlInput& input,
    bool& identityChanged)
{
    const BallVisionSnapshot& vision = input.vision;
    identityChanged = visionIdentityInitialized_
        && (vision.pi_session_id != lastPiSessionId_
            || vision.run_id != lastRunId_);
    const bool initializeIdentity = !visionIdentityInitialized_;

    if (initializeIdentity || identityChanged) {
        visionIdentityInitialized_ = true;
        lastPiSessionId_ = vision.pi_session_id;
        lastRunId_ = vision.run_id;
        lastMeasurementUpdateCount_ = vision.measurement_update_count;
        consecutiveValidMeasurements_ = 0U;
        return vision.measurement_update_count != 0U;
    }

    if (vision.measurement_update_count == lastMeasurementUpdateCount_) {
        return false;
    }

    lastMeasurementUpdateCount_ = vision.measurement_update_count;
    return true;
}

void BallBalanceController::startVisionDropout(
    const BallControlInput& input)
{
    if (visionDropoutActive_) {
        return;
    }

    visionDropoutActive_ = true;
    const std::uint32_t ageBeyondNormal =
        input.vision.measurement_age_ms
                > ball_control_config::kVisionMaxMeasurementAgeMs
            ? input.vision.measurement_age_ms
                  - ball_control_config::kVisionMaxMeasurementAgeMs
            : 0U;
    visionDropoutStartMs_ = input.nowMs - ageBeyondNormal;
}

bool BallBalanceController::visionDropoutExceeded(
    const BallControlInput& input) const
{
    return visionDropoutActive_
        && (input.nowMs - visionDropoutStartMs_)
               > ball_control_config::kFastVisionRecoveryMaxDropoutMs;
}

void BallBalanceController::resetCenterHoldGate()
{
    centerHoldActive_ = false;
    consecutiveCenterEntryMeasurements_ = 0U;
    consecutiveCenterExitMeasurements_ = 0U;
}

void BallBalanceController::resetStictionControl()
{
    stictionState_ = BallStictionState::Monitoring;
    stictionCaptureInitialized_ = false;
    lastStictionCaptureTimestampMs_ = 0U;
    stuckElapsedMs_ = 0U;
    breakawayElapsedMs_ = 0U;
    breakawayStartAbsoluteErrorMm_ = 0.0F;
    breakawayLockoutAbsoluteErrorMm_ = 0.0F;
    activeAngleLimitDeg_ = config_.maxAngleOffsetDeg;
}

float BallBalanceController::updateStictionControl(
    const BallControlInput& input,
    const float absoluteErrorMm,
    const float absoluteVelocityMmps,
    const float requestedOffsetDeg,
    const bool newMeasurement)
{
    const bool task3TuningActive = input.vision.task_id == 3U;
    const float activeStuckErrorMm = task3TuningActive
        ? ball_control_config::kTask3StuckErrorThresholdMm
        : stuckErrorMm_;
    const std::uint32_t activeStuckTimeMs = task3TuningActive
        ? ball_control_config::kTask3StuckQualificationMs
        : stuckTimeMs_;
    const float minimumBreakawayCommandDeg = task3TuningActive
        ? ball_control_config::kTask3MinimumBreakawayCommandDeg
        : config_.maxAngleOffsetDeg;

    if (!newMeasurement) {
        return activeAngleLimitDeg_;
    }

    std::uint32_t captureElapsedMs = 0U;
    if (stictionCaptureInitialized_) {
        captureElapsedMs = std::min(
            input.vision.capture_timestamp_ms
                - lastStictionCaptureTimestampMs_,
            ball_control_config::kVisionMaxMeasurementAgeMs);
    }
    lastStictionCaptureTimestampMs_ =
        input.vision.capture_timestamp_ms;
    stictionCaptureInitialized_ = true;

    if (stictionState_ == BallStictionState::LockedOut) {
        const bool reachedDeadband =
            absoluteErrorMm
            <= ball_control_config::kCenterHoldEnterErrorMm;
        const bool ballMoved =
            input.vision.velocity_valid
            && absoluteVelocityMmps
                   >= ball_control_config::kBreakawayExitSpeedMmps;
        const bool positionChanged =
            std::fabs(absoluteErrorMm
                      - breakawayLockoutAbsoluteErrorMm_)
            >= ball_control_config::kBreakawayExitProgressMm;
        if (reachedDeadband || (ballMoved && positionChanged)) {
            resetStictionControl();
        }
        return activeAngleLimitDeg_;
    }

    if (stictionState_ == BallStictionState::Breakaway) {
        breakawayElapsedMs_ += captureElapsedMs;
        const bool moving =
            absoluteVelocityMmps
            >= ball_control_config::kBreakawayExitSpeedMmps;
        const bool madeProgress =
            breakawayStartAbsoluteErrorMm_ - absoluteErrorMm
            >= ball_control_config::kBreakawayExitProgressMm;
        const bool timedOut =
            breakawayElapsedMs_ >= breakawayDurationMs_;
        if (!input.vision.velocity_valid || moving || madeProgress) {
            resetStictionControl();
            return activeAngleLimitDeg_;
        }
        if (timedOut) {
            stictionState_ = BallStictionState::LockedOut;
            breakawayLockoutAbsoluteErrorMm_ = absoluteErrorMm;
            stuckElapsedMs_ = 0U;
            breakawayElapsedMs_ = 0U;
            activeAngleLimitDeg_ = config_.maxAngleOffsetDeg;
            return activeAngleLimitDeg_;
        }
        return activeAngleLimitDeg_;
    }

    const bool stationaryCandidate =
        input.vision.velocity_valid
        && absoluteErrorMm >= activeStuckErrorMm
        && absoluteVelocityMmps <= stuckSpeedMmps_
        && std::fabs(requestedOffsetDeg)
               >= minimumBreakawayCommandDeg;
    if (!stationaryCandidate) {
        resetStictionControl();
        return activeAngleLimitDeg_;
    }

    stictionState_ = BallStictionState::Qualifying;
    stuckElapsedMs_ += captureElapsedMs;
    if (stuckElapsedMs_ >= activeStuckTimeMs) {
        stictionState_ = BallStictionState::Breakaway;
        breakawayElapsedMs_ = 0U;
        breakawayStartAbsoluteErrorMm_ = absoluteErrorMm;
        activeAngleLimitDeg_ = breakawayAngleDeg_;
    }
    return activeAngleLimitDeg_;
}

void BallBalanceController::updateCenterHoldGate(
    const float absoluteErrorMm,
    const float absoluteVelocityMmps,
    const bool velocityValid,
    const bool newMeasurement)
{
    if (!newMeasurement) {
        return;
    }

    if (centerHoldActive_) {
        consecutiveCenterEntryMeasurements_ = 0U;
        if (absoluteErrorMm
            > ball_control_config::kCenterHoldExitErrorMm) {
            if (consecutiveCenterExitMeasurements_
                < ball_control_config::kCenterHoldExitMeasurements) {
                ++consecutiveCenterExitMeasurements_;
            }
            if (consecutiveCenterExitMeasurements_
                >= ball_control_config::kCenterHoldExitMeasurements) {
                centerHoldActive_ = false;
                consecutiveCenterExitMeasurements_ = 0U;
            }
        } else {
            consecutiveCenterExitMeasurements_ = 0U;
        }
        return;
    }

    consecutiveCenterExitMeasurements_ = 0U;
    if (velocityValid
        && absoluteErrorMm
               <= ball_control_config::kCenterHoldEnterErrorMm
        && absoluteVelocityMmps
               <= ball_control_config::kCenterHoldEnterMaxSpeedMmps) {
        if (consecutiveCenterEntryMeasurements_
            < ball_control_config::kCenterHoldEntryMeasurements) {
            ++consecutiveCenterEntryMeasurements_;
        }
        if (consecutiveCenterEntryMeasurements_
            >= ball_control_config::kCenterHoldEntryMeasurements) {
            centerHoldActive_ = true;
            consecutiveCenterEntryMeasurements_ = 0U;
        }
    } else {
        consecutiveCenterEntryMeasurements_ = 0U;
    }
}

void BallBalanceController::updateMotorFeedbackGate(
    const BallControlInput& input)
{
    if (!input.motorFeedbackFresh || !input.motorFeedbackFinite) {
        consecutiveValidMotorFeedback_ = 0U;
        lastMotorFeedbackCount_ = input.motorFeedbackCount;
        motorFeedbackCounterInitialized_ = true;
        return;
    }

    if (!motorFeedbackCounterInitialized_) {
        motorFeedbackCounterInitialized_ = true;
        lastMotorFeedbackCount_ = input.motorFeedbackCount;
        consecutiveValidMotorFeedback_ =
            input.motorFeedbackCount == 0U ? 0U : 1U;
        return;
    }

    if (input.motorFeedbackCount == lastMotorFeedbackCount_) {
        return;
    }

    lastMotorFeedbackCount_ = input.motorFeedbackCount;
    if (consecutiveValidMotorFeedback_
        < ball_control_config::kMotorFeedbackRequiredFrames) {
        ++consecutiveValidMotorFeedback_;
    }
}

BallControlFault BallBalanceController::updateMotorLimitGate(
    const BallControlInput& input)
{
    if (!input.motorFeedbackFresh || !input.motorFeedbackFinite) {
        consecutiveOverspeedFeedback_ = 0U;
        consecutiveOvercurrentFeedback_ = 0U;
        lastMotorLimitFeedbackCount_ = input.motorFeedbackCount;
        motorLimitCounterInitialized_ = true;
        return BallControlFault::None;
    }

    if (!motorLimitCounterInitialized_) {
        motorLimitCounterInitialized_ = true;
        lastMotorLimitFeedbackCount_ = input.motorFeedbackCount;
        if (input.motorFeedbackCount == 0U) {
            return BallControlFault::None;
        }
    } else {
        if (input.motorFeedbackCount == lastMotorLimitFeedbackCount_) {
            return BallControlFault::None;
        }
        lastMotorLimitFeedbackCount_ = input.motorFeedbackCount;
    }

    if (std::fabs(input.motorSpeedRpm)
        > config_.maxMotorSpeedRpm) {
        ++consecutiveOverspeedFeedback_;
    } else {
        consecutiveOverspeedFeedback_ = 0U;
    }
    if (std::fabs(input.motorCurrentA)
        > config_.maxMotorCurrentA) {
        ++consecutiveOvercurrentFeedback_;
    } else {
        consecutiveOvercurrentFeedback_ = 0U;
    }

    if (consecutiveOvercurrentFeedback_
        >= ball_control_config::kMotorLimitViolationFrames) {
        return BallControlFault::MotorOvercurrent;
    }
    if (consecutiveOverspeedFeedback_
        >= ball_control_config::kMotorLimitViolationFrames) {
        return BallControlFault::MotorOverspeed;
    }
    return BallControlFault::None;
}

BallControlOutput BallBalanceController::update(
    const BallControlInput& input)
{
    if (!txCounterInitialized_) {
        txCounterInitialized_ = true;
        lastTxErrorCount_ = input.txErrorCount;
        if (input.txErrorCount != 0U) {
            firstUpdate_ = false;
            lastUpdateMs_ = input.nowMs;
            updateTimeInitialized_ = true;
            return enterFault(input, BallControlFault::CanTransmit);
        }
    }

    if (firstUpdate_) {
        firstUpdate_ = false;
        updateMotorFeedbackGate(input);
        const BallControlFault limitFault = updateMotorLimitGate(input);
        if (limitFault != BallControlFault::None) {
            return enterFault(input, limitFault);
        }

        if (input.commissioned
            && input.motorFeedbackFresh
            && input.motorFeedbackFinite) {
            const float relativeAngleDeg = std::fabs(
                wrappedAngleDifference(
                    input.motorAngleRad,
                    config_.mechanicalLevelAngleRad)
                * kRadiansToDegrees);
            if (!std::isfinite(relativeAngleDeg)) {
                return enterFault(
                    input, BallControlFault::InvalidMotorFeedback);
            }
            if (relativeAngleDeg
                > config_.mechanicalAngleLimitDeg) {
                return enterFault(
                    input, BallControlFault::MechanicalAngleLimit);
            }
        }

        state_ = input.commissioned
                     ? BallControlState::WaitMotor
                     : BallControlState::ConfigRequired;
        lastUpdateMs_ = input.nowMs;
        updateTimeInitialized_ = true;

        BallControlOutput output = safeShutdownOutput(input, true);
        output.state = BallControlState::BootSafe;
        return output;
    }

    if (state_ == BallControlState::Fault) {
        return safeShutdownOutput(input);
    }
    if (input.keyEmergencyStop) {
        return enterFault(input, BallControlFault::EmergencyStop);
    }
    if (!input.commissioned) {
        state_ = BallControlState::ConfigRequired;
        commandedOffsetDeg_ = 0.0F;
        return safeShutdownOutput(input);
    }
    if (input.txErrorCount != lastTxErrorCount_) {
        lastTxErrorCount_ = input.txErrorCount;
        return enterFault(input, BallControlFault::CanTransmit);
    }
    const bool timingCriticalState =
        state_ == BallControlState::EnablePending
        || state_ == BallControlState::LevelHold
        || state_ == BallControlState::Balancing
        || state_ == BallControlState::VisionRecovery;
    if (timingCriticalState
        && updateTimeInitialized_
        && (input.nowMs - lastUpdateMs_)
               > ball_control_config::kSevereControlLapseMs) {
        return enterFault(input, BallControlFault::ControlLoopTiming);
    }
    if (motorReady_ && !input.motorFeedbackFresh) {
        return enterFault(input, BallControlFault::MotorFeedbackTimeout);
    }
    if (motorReady_ && !input.motorFeedbackFinite) {
        return enterFault(input, BallControlFault::InvalidMotorFeedback);
    }
    if (input.motorFeedbackFresh && input.motorFeedbackFinite) {
        const float relativeAngleDeg = std::fabs(
            wrappedAngleDifference(
                input.motorAngleRad,
                config_.mechanicalLevelAngleRad)
            * kRadiansToDegrees);
        if (!std::isfinite(relativeAngleDeg)) {
            return enterFault(input, BallControlFault::InvalidMotorFeedback);
        }
        if (relativeAngleDeg
            > config_.mechanicalAngleLimitDeg) {
            return enterFault(input, BallControlFault::MechanicalAngleLimit);
        }
    }
    const BallControlFault limitFault = updateMotorLimitGate(input);
    if (limitFault != BallControlFault::None) {
        return enterFault(input, limitFault);
    }

    if (!motorReady_) {
        updateMotorFeedbackGate(input);
        if (consecutiveValidMotorFeedback_
            < ball_control_config::kMotorFeedbackRequiredFrames) {
            state_ = BallControlState::WaitMotor;
            return safeShutdownOutput(input);
        }
        motorReady_ = true;
        state_ = BallControlState::EnablePending;
        enableCommandSent_ = false;
        enablePendingStartedMs_ = input.nowMs;
        lastEnableFeedbackCount_ = input.motorFeedbackCount;
        motorEnableConfirmations_ = 0U;
        const BallControlOutput output =
            angleHoldOutput(input, 0.0F, false);
        return output;
    }

    if (state_ == BallControlState::EnablePending) {
        if ((input.nowMs - enablePendingStartedMs_)
            > ball_control_config::kEnableTimeoutMs) {
            return enterFault(
                input, BallControlFault::MotorEnableTimeout);
        }

        if (enableCommandSent_
            && input.motorFeedbackCount != lastEnableFeedbackCount_) {
            lastEnableFeedbackCount_ = input.motorFeedbackCount;
            if (input.motorEnabled) {
                if (motorEnableConfirmations_
                    < ball_control_config::kMotorEnableConfirmFrames) {
                    ++motorEnableConfirmations_;
                }
            } else {
                motorEnableConfirmations_ = 0U;
            }
        }

        if (motorEnableConfirmations_
            >= ball_control_config::kMotorEnableConfirmFrames) {
            state_ = BallControlState::LevelHold;
            return angleHoldOutput(input, 0.0F, false);
        }

        const bool enableDue =
            !enableCommandSent_
            || (input.nowMs - lastEnableCommandMs_)
                   >= ball_control_config::kEnableRetryPeriodMs;
        BallControlOutput output =
            angleHoldOutput(input, 0.0F, enableDue);
        if (enableDue) {
            enableCommandSent_ = true;
            lastEnableCommandMs_ = input.nowMs;
        }
        return output;
    }

    if (!input.motorEnabled) {
        return enterFault(
            input, BallControlFault::MotorUnexpectedlyDisabled);
    }

    bool visionIdentityChanged = false;
    const bool newVisionMeasurement = observeNewVisionMeasurement(
        input, visionIdentityChanged);
    const bool visionQualityUsable = visionQualityIsUsable(input);
    const bool visionFresh = visionIsFreshForControl(input);
    const bool inReturnToLevelAgeBand =
        visionQualityUsable
        && input.vision.measurement_age_ms
               > ball_control_config::kVisionMaxMeasurementAgeMs
        && input.vision.measurement_age_ms
               <= ball_control_config::kVisionReturnToLevelAgeMs;

    if (visionIdentityChanged) {
        fullVisionGateRequired_ = true;
        consecutiveValidMeasurements_ = 0U;
        startVisionDropout(input);
        if (state_ == BallControlState::Balancing) {
            state_ = BallControlState::VisionRecovery;
        }
    }

    if (state_ == BallControlState::Balancing) {
        if (visionFresh) {
            visionDropoutActive_ = false;
            fullVisionGateRequired_ = false;
        } else if (inReturnToLevelAgeBand) {
            startVisionDropout(input);
        } else {
            startVisionDropout(input);
            consecutiveValidMeasurements_ = 0U;
            state_ = BallControlState::VisionRecovery;
            if (!visionQualityUsable
                || input.vision.measurement_age_ms
                       <= ball_control_config::kVisionMaxMeasurementAgeMs) {
                fullVisionGateRequired_ = true;
            }
        }
    }

    if (state_ == BallControlState::VisionRecovery
        && visionDropoutExceeded(input)) {
        fullVisionGateRequired_ = true;
    }

    if (state_ == BallControlState::LevelHold
        || state_ == BallControlState::VisionRecovery) {
        const std::uint32_t requiredMeasurements =
            state_ == BallControlState::LevelHold
                    || fullVisionGateRequired_
                ? ball_control_config::kVisionRequiredNewMeasurements
                : ball_control_config::kVisionRecoveryNewMeasurements;

        if (!visionFresh) {
            consecutiveValidMeasurements_ = 0U;
            if (state_ == BallControlState::VisionRecovery) {
                startVisionDropout(input);
                if (!visionQualityUsable
                    || input.vision.measurement_age_ms
                           <= ball_control_config::kVisionMaxMeasurementAgeMs) {
                    fullVisionGateRequired_ = true;
                }
            }
        } else if (newVisionMeasurement
                   && consecutiveValidMeasurements_
                          < requiredMeasurements) {
            ++consecutiveValidMeasurements_;
        }

        if (visionFresh
            && consecutiveValidMeasurements_ >= requiredMeasurements) {
            state_ = BallControlState::Balancing;
            visionDropoutActive_ = false;
            fullVisionGateRequired_ = false;
        }
    }

    float desiredOffsetDeg = 0.0F;
    if (state_ == BallControlState::Balancing && visionFresh) {
        const float errorMm =
            static_cast<float>(
                input.vision.target_position_0p1mm
                - input.vision.position_0p1mm)
            * 0.1F;
        const float velocityMmps =
            input.vision.velocity_valid
                ? static_cast<float>(input.vision.velocity_mmps)
                : 0.0F;
        updateCenterHoldGate(
            std::fabs(errorMm),
            std::fabs(velocityMmps),
            input.vision.velocity_valid,
            newVisionMeasurement);
        if (!centerHoldActive_) {
            const float requestedOffsetDeg =
                config_.angleSign
                * (positionKpDegPerMm_ * errorMm
                   - positionKdDegPerMmps_ * velocityMmps);
            float angleLimitDeg = updateStictionControl(
                input,
                std::fabs(errorMm),
                std::fabs(velocityMmps),
                requestedOffsetDeg,
                newVisionMeasurement);
            const bool task3FastTravel = input.vision.task_id == 3U
                && std::fabs(errorMm)
                       >= ball_control_config::kTask3FastTravelErrorThresholdMm;
            if (task3FastTravel) {
                angleLimitDeg = std::max(
                    angleLimitDeg,
                    ball_control_config::kTask3MotionAngleLimitDeg);
                activeAngleLimitDeg_ = angleLimitDeg;
            } else if (input.vision.task_id == 3U
                       && stictionState_ != BallStictionState::Breakaway) {
                angleLimitDeg = std::min(
                    angleLimitDeg,
                    config_.maxAngleOffsetDeg);
                activeAngleLimitDeg_ = angleLimitDeg;
            }
            desiredOffsetDeg = requestedOffsetDeg;
            if (input.vision.task_id == 3U
                && stictionState_ == BallStictionState::Breakaway
                && std::fabs(desiredOffsetDeg) < angleLimitDeg) {
                desiredOffsetDeg = std::copysign(
                    angleLimitDeg,
                    desiredOffsetDeg);
            }
            desiredOffsetDeg = clampValue(
                desiredOffsetDeg,
                -angleLimitDeg,
                angleLimitDeg);
        } else {
            resetStictionControl();
        }
    } else {
        resetCenterHoldGate();
        resetStictionControl();
    }

    return angleHoldOutput(input, desiredOffsetDeg, false);
}
