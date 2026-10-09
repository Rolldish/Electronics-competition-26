#include "BallLinkReceiver.h"

#include <limits>

namespace {

constexpr std::int32_t kMaximumPosition0p1Mm = 1300;
constexpr std::int32_t kMaximumVelocityMmps = 5000;
constexpr std::int32_t kMaximumTargetPosition0p1Mm = 1200;
constexpr std::uint32_t kMaximumProcessingDelayUs = 200000U;
constexpr std::uint32_t kEstimatedSerialTimeMs = 3U;

bool outsideSymmetricRange(
    const std::int16_t value, const std::int32_t limit) {
    const std::int32_t widened = value;
    return widened < -limit || widened > limit;
}

} // namespace

BallLinkReceiveResult BallLinkReceiver::process(
    const BallLinkFrame &frame, const std::uint32_t nowMs) {
    BallLinkVisionMeasurement measurement{};
    if (!ballLinkDecodeVisionMeasurement(frame, measurement)) {
        return BallLinkReceiveResult::ProtocolError;
    }
    if (!fieldsAreInRange(measurement)) {
        return BallLinkReceiveResult::RangeError;
    }

    const bool newSession =
        !have_session_ || measurement.piSessionId != session_id_;
    if (newSession) {
        if (have_session_) {
            session_change_count_ += 1U;
        }
        resetSession(measurement.piSessionId);
    }

    missing_frame_count_ = 0U;
    if (have_sequence_) {
        const std::uint8_t sequenceDelta =
            static_cast<std::uint8_t>(
                frame.sequence - last_sequence_);
        if (sequenceDelta == 0U) {
            return BallLinkReceiveResult::DuplicateSequence;
        }
        if (sequenceDelta >= 128U) {
            return BallLinkReceiveResult::OutOfOrderSequence;
        }
        if (sequenceDelta > 1U) {
            missing_frame_count_ =
                static_cast<std::uint8_t>(sequenceDelta - 1U);
        }
    }

    const bool newRun =
        !have_run_ || measurement.runId != run_id_;
    if (newRun) {
        if (have_run_) {
            run_change_count_ += 1U;
            clearMeasurementHistory();
        }
        run_id_ = measurement.runId;
        have_run_ = true;
        have_capture_timestamp_ = false;
    }

    const bool repeatedTimestamp =
        have_capture_timestamp_ &&
        measurement.captureTimestampMs ==
            last_capture_timestamp_ms_;
    if (have_capture_timestamp_ && !repeatedTimestamp &&
        static_cast<std::int32_t>(
            measurement.captureTimestampMs -
            last_capture_timestamp_ms_) < 0) {
        return BallLinkReceiveResult::TimestampError;
    }

    have_sequence_ = true;
    last_sequence_ = frame.sequence;
    have_legal_frame_ = true;
    last_legal_frame_ms_ = nowMs;
    have_capture_timestamp_ = true;
    last_capture_timestamp_ms_ =
        measurement.captureTimestampMs;

    snapshot_.pi_session_id = measurement.piSessionId;
    snapshot_.target_position_0p1mm =
        measurement.targetPosition0p1Mm;
    snapshot_.task_id = measurement.taskId;
    snapshot_.control_flags = measurement.controlFlags;
    snapshot_.run_id = measurement.runId;
    snapshot_.sequence = frame.sequence;

    const std::uint32_t baseAgeMs =
        measurementBaseAgeMs(measurement);
    if (repeatedTimestamp) {
        if (have_measurement_ &&
            snapshot_.capture_timestamp_ms ==
                measurement.captureTimestampMs) {
            const std::uint32_t currentAgeMs =
                measurementAgeMs(nowMs);
            measurement_base_age_ms_ =
                baseAgeMs > currentAgeMs
                    ? baseAgeMs
                    : currentAgeMs;
            measurement_receive_ms_ = nowMs;
            snapshot_.local_receive_ms = nowMs;
        }
        updateState(nowMs);
        return BallLinkReceiveResult::LegalFrame;
    }

    if (!measurementIsAcceptable(measurement, baseAgeMs)) {
        updateState(nowMs);
        return BallLinkReceiveResult::LegalFrame;
    }

    have_measurement_ = true;
    measurement_base_age_ms_ = baseAgeMs;
    measurement_receive_ms_ = nowMs;
    measurement_update_count_ += 1U;
    snapshot_.capture_timestamp_ms =
        measurement.captureTimestampMs;
    snapshot_.local_receive_ms = nowMs;
    snapshot_.position_0p1mm = measurement.position0p1Mm;
    snapshot_.velocity_mmps = measurement.velocityMmps;
    snapshot_.confidence = measurement.confidence;
    snapshot_.flags = measurement.visionFlags;
    snapshot_.velocity_valid =
        (measurement.visionFlags & BallLinkVelocityValid) != 0U
            ? 1U
            : 0U;
    snapshot_.measurement_update_count =
        measurement_update_count_;
    updateState(nowMs);
    return BallLinkReceiveResult::NewMeasurement;
}

void BallLinkReceiver::refresh(const std::uint32_t nowMs) {
    updateState(nowMs);
}

void BallLinkReceiver::invalidateLink() {
    clearMeasurementHistory();
    have_legal_frame_ = false;
    have_sequence_ = false;
    missing_frame_count_ = 0U;
    state_ = BallLinkVisionState::NoLink;
}

VisionLinkSnapshot BallLinkReceiver::snapshot(
    const std::uint32_t nowMs) const {
    VisionLinkSnapshot result = snapshot_;
    result.measurement_update_count =
        measurement_update_count_;
    result.measurement_age_ms = have_measurement_
                                    ? measurementAgeMs(nowMs)
                                    : std::numeric_limits<
                                          std::uint32_t>::max();

    const bool linkValid =
        have_legal_frame_ &&
        nowMs - last_legal_frame_ms_ <= kBallLinkTimeoutMs;
    const bool controlEnabled =
        (snapshot_.control_flags &
         BallLinkControlEnabled) != 0U;
    result.valid =
        linkValid && controlEnabled && have_measurement_ &&
                result.measurement_age_ms <=
                    kBallLinkFreshMeasurementAgeMs
            ? 1U
            : 0U;
    if (result.valid == 0U) {
        result.velocity_valid = 0U;
    }
    return result;
}

bool BallLinkReceiver::fieldsAreInRange(
    const BallLinkVisionMeasurement &measurement) {
    if (outsideSymmetricRange(
            measurement.position0p1Mm,
            kMaximumPosition0p1Mm) ||
        outsideSymmetricRange(
            measurement.velocityMmps,
            kMaximumVelocityMmps) ||
        outsideSymmetricRange(
            measurement.targetPosition0p1Mm,
            kMaximumTargetPosition0p1Mm) ||
        measurement.captureToSendDelayUs >
            kMaximumProcessingDelayUs ||
        (measurement.visionFlags &
         static_cast<std::uint8_t>(
             ~kBallLinkKnownVisionFlags)) != 0U ||
        (measurement.controlFlags &
         static_cast<std::uint8_t>(
             ~kBallLinkKnownControlFlags)) != 0U) {
        return false;
    }

    const bool ballValid =
        (measurement.visionFlags & BallLinkBallValid) != 0U;
    const bool velocityValid =
        (measurement.visionFlags & BallLinkVelocityValid) != 0U;
    if (!velocityValid && measurement.velocityMmps != 0) {
        return false;
    }
    if (!ballValid) {
        return !velocityValid &&
               measurement.position0p1Mm == 0 &&
               measurement.velocityMmps == 0 &&
               measurement.confidence == 0U;
    }
    return true;
}

bool BallLinkReceiver::measurementIsAcceptable(
    const BallLinkVisionMeasurement &measurement,
    const std::uint32_t baseAgeMs) {
    const bool controlEnabled =
        (measurement.controlFlags &
         BallLinkControlEnabled) != 0U;
    const bool ballValid =
        (measurement.visionFlags & BallLinkBallValid) != 0U;
    const bool cameraCalibrated =
        (measurement.visionFlags &
         BallLinkCameraCalibrated) != 0U;
    const bool processingDegraded =
        (measurement.visionFlags &
         BallLinkProcessingDegraded) != 0U;
    return controlEnabled && ballValid && cameraCalibrated &&
           !processingDegraded &&
           measurement.confidence >= kBallLinkMinimumConfidence &&
           baseAgeMs <= kBallLinkFreshMeasurementAgeMs;
}

std::uint32_t BallLinkReceiver::measurementBaseAgeMs(
    const BallLinkVisionMeasurement &measurement) {
    const std::uint32_t processingAgeMs =
        (measurement.captureToSendDelayUs + 999U) / 1000U;
    return saturatedAdd(
        processingAgeMs, kEstimatedSerialTimeMs);
}

std::uint32_t BallLinkReceiver::saturatedAdd(
    const std::uint32_t left, const std::uint32_t right) {
    return right >
                   std::numeric_limits<std::uint32_t>::max() -
                       left
               ? std::numeric_limits<std::uint32_t>::max()
               : left + right;
}

void BallLinkReceiver::clearMeasurementHistory() {
    have_measurement_ = false;
    measurement_receive_ms_ = 0U;
    measurement_base_age_ms_ = 0U;
    snapshot_.capture_timestamp_ms = 0U;
    snapshot_.local_receive_ms = 0U;
    snapshot_.measurement_age_ms =
        std::numeric_limits<std::uint32_t>::max();
    snapshot_.position_0p1mm = 0;
    snapshot_.velocity_mmps = 0;
    snapshot_.confidence = 0U;
    snapshot_.flags = 0U;
    snapshot_.valid = 0U;
    snapshot_.velocity_valid = 0U;
}

void BallLinkReceiver::resetSession(
    const std::uint32_t sessionId) {
    clearMeasurementHistory();
    session_id_ = sessionId;
    have_session_ = true;
    have_run_ = false;
    have_sequence_ = false;
    have_capture_timestamp_ = false;
    have_legal_frame_ = false;
    last_sequence_ = 0U;
    missing_frame_count_ = 0U;
    snapshot_.pi_session_id = sessionId;
    snapshot_.target_position_0p1mm = 0;
    snapshot_.task_id = 0U;
    snapshot_.control_flags = 0U;
    snapshot_.run_id = 0U;
    snapshot_.sequence = 0U;
    state_ = BallLinkVisionState::MeasurementInvalid;
}

void BallLinkReceiver::updateState(
    const std::uint32_t nowMs) {
    if (!have_legal_frame_ ||
        nowMs - last_legal_frame_ms_ > kBallLinkTimeoutMs) {
        if (have_legal_frame_) {
            have_sequence_ = false;
            missing_frame_count_ = 0U;
        }
        state_ = BallLinkVisionState::NoLink;
        return;
    }
    if ((snapshot_.control_flags &
         BallLinkControlEnabled) == 0U ||
        !have_measurement_) {
        state_ = BallLinkVisionState::MeasurementInvalid;
        return;
    }
    state_ = measurementAgeMs(nowMs) <=
                     kBallLinkFreshMeasurementAgeMs
                 ? BallLinkVisionState::MeasurementValid
                 : BallLinkVisionState::Stale;
}

std::uint32_t BallLinkReceiver::measurementAgeMs(
    const std::uint32_t nowMs) const {
    return saturatedAdd(
        measurement_base_age_ms_,
        nowMs - measurement_receive_ms_);
}
