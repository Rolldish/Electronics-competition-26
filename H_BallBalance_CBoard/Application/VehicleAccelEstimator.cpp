#include "VehicleAccelEstimator.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kMinimumAxisNorm = 0.0001F;

bool isConfigValid(const VehicleAccelConfig &config) {
    const float axisNormSquared =
        config.forwardAxisX * config.forwardAxisX +
        config.forwardAxisY * config.forwardAxisY +
        config.forwardAxisZ * config.forwardAxisZ;

    return config.samplesPerOutput > 0U &&
           config.calibrationSamples >= 2U &&
           std::isfinite(axisNormSquared) &&
           axisNormSquared >
               kMinimumAxisNorm * kMinimumAxisNorm &&
           std::isfinite(config.lowPassAlpha) &&
           config.lowPassAlpha > 0.0F &&
           config.lowPassAlpha <= 1.0F &&
           std::isfinite(config.minimumDeadbandMps2) &&
           config.minimumDeadbandMps2 >= 0.0F &&
           std::isfinite(config.deadbandSigmaMultiplier) &&
           config.deadbandSigmaMultiplier >= 0.0F &&
           std::isfinite(config.maximumCalibrationNoiseMps2) &&
           config.maximumCalibrationNoiseMps2 >= 0.0F &&
           std::isfinite(config.minimumGravityMps2) &&
           std::isfinite(config.maximumGravityMps2) &&
           config.minimumGravityMps2 > 0.0F &&
           config.maximumGravityMps2 >
               config.minimumGravityMps2 &&
           std::isfinite(config.maximumAbsOutputMps2) &&
           config.maximumAbsOutputMps2 > 0.0F;
}

float clampSymmetric(const float value, const float magnitude) {
    return std::clamp(value, -magnitude, magnitude);
}

} // namespace

VehicleAccelEstimator::VehicleAccelEstimator(
    const VehicleAccelConfig &config)
    : config_(config), configValid_(isConfigValid(config)) {
    if (configValid_) {
        const float axisNorm = std::sqrt(
            config_.forwardAxisX * config_.forwardAxisX +
            config_.forwardAxisY * config_.forwardAxisY +
            config_.forwardAxisZ * config_.forwardAxisZ);
        forwardAxisX_ = config_.forwardAxisX / axisNorm;
        forwardAxisY_ = config_.forwardAxisY / axisNorm;
        forwardAxisZ_ = config_.forwardAxisZ / axisNorm;
    }
    reset();
}

void VehicleAccelEstimator::reset() {
    snapshot_ = VehicleAccelSnapshot{};
    calibrationSumX_ = 0.0F;
    calibrationSumY_ = 0.0F;
    calibrationSumZ_ = 0.0F;
    calibrationForwardMean_ = 0.0F;
    calibrationForwardM2_ = 0.0F;
    groupSumX_ = 0.0F;
    groupSumY_ = 0.0F;
    groupSumZ_ = 0.0F;
    groupSampleCount_ = 0U;

    if (!configValid_) {
        snapshot_.status = VehicleAccelStatus::InvalidConfig;
    }
}

bool VehicleAccelEstimator::update(
    const float accelXMps2, const float accelYMps2,
    const float accelZMps2) {
    if (!configValid_ || !std::isfinite(accelXMps2) ||
        !std::isfinite(accelYMps2) ||
        !std::isfinite(accelZMps2)) {
        snapshot_.valid = false;
        return false;
    }

    if (snapshot_.status == VehicleAccelStatus::Calibrating) {
        const std::uint32_t sampleNumber =
            snapshot_.calibrationSampleCount + 1U;
        const float forwardSample =
            accelXMps2 * forwardAxisX_ +
            accelYMps2 * forwardAxisY_ +
            accelZMps2 * forwardAxisZ_;

        calibrationSumX_ += accelXMps2;
        calibrationSumY_ += accelYMps2;
        calibrationSumZ_ += accelZMps2;

        const float delta =
            forwardSample - calibrationForwardMean_;
        calibrationForwardMean_ +=
            delta / static_cast<float>(sampleNumber);
        const float deltaAfterMean =
            forwardSample - calibrationForwardMean_;
        calibrationForwardM2_ += delta * deltaAfterMean;
        snapshot_.calibrationSampleCount = sampleNumber;

        if (sampleNumber < config_.calibrationSamples) {
            return false;
        }

        const float inverseSampleCount =
            1.0F / static_cast<float>(sampleNumber);
        snapshot_.baselineXMps2 =
            calibrationSumX_ * inverseSampleCount;
        snapshot_.baselineYMps2 =
            calibrationSumY_ * inverseSampleCount;
        snapshot_.baselineZMps2 =
            calibrationSumZ_ * inverseSampleCount;
        snapshot_.calibrationNoiseMps2 = std::sqrt(
            calibrationForwardM2_ /
            static_cast<float>(sampleNumber - 1U));
        snapshot_.deadbandMps2 = std::max(
            config_.minimumDeadbandMps2,
            config_.deadbandSigmaMultiplier *
                snapshot_.calibrationNoiseMps2);

        const float gravityMagnitude = std::sqrt(
            snapshot_.baselineXMps2 *
                snapshot_.baselineXMps2 +
            snapshot_.baselineYMps2 *
                snapshot_.baselineYMps2 +
            snapshot_.baselineZMps2 *
                snapshot_.baselineZMps2);
        const bool gravityAccepted =
            gravityMagnitude >= config_.minimumGravityMps2 &&
            gravityMagnitude <= config_.maximumGravityMps2;
        const bool noiseAccepted =
            snapshot_.calibrationNoiseMps2 <=
            config_.maximumCalibrationNoiseMps2;

        snapshot_.status =
            gravityAccepted && noiseAccepted
                ? VehicleAccelStatus::Ready
                : VehicleAccelStatus::CalibrationRejected;
        return false;
    }

    if (snapshot_.status != VehicleAccelStatus::Ready) {
        snapshot_.valid = false;
        return false;
    }

    groupSumX_ += accelXMps2;
    groupSumY_ += accelYMps2;
    groupSumZ_ += accelZMps2;
    ++groupSampleCount_;
    if (groupSampleCount_ < config_.samplesPerOutput) {
        return false;
    }

    const float inverseGroupSize =
        1.0F / static_cast<float>(groupSampleCount_);
    const float dynamicX =
        groupSumX_ * inverseGroupSize -
        snapshot_.baselineXMps2;
    const float dynamicY =
        groupSumY_ * inverseGroupSize -
        snapshot_.baselineYMps2;
    const float dynamicZ =
        groupSumZ_ * inverseGroupSize -
        snapshot_.baselineZMps2;

    snapshot_.forwardRawMps2 =
        dynamicX * forwardAxisX_ +
        dynamicY * forwardAxisY_ +
        dynamicZ * forwardAxisZ_;
    const float deadbanded =
        std::fabs(snapshot_.forwardRawMps2) <=
                snapshot_.deadbandMps2
            ? 0.0F
            : snapshot_.forwardRawMps2;
    const float filtered =
        snapshot_.forwardFilteredMps2 +
        config_.lowPassAlpha *
            (deadbanded -
             snapshot_.forwardFilteredMps2);
    snapshot_.forwardFilteredMps2 = clampSymmetric(
        filtered, config_.maximumAbsOutputMps2);
    snapshot_.outputCount += 1U;
    snapshot_.valid = true;

    groupSumX_ = 0.0F;
    groupSumY_ = 0.0F;
    groupSumZ_ = 0.0F;
    groupSampleCount_ = 0U;
    return true;
}

VehicleAccelSnapshot VehicleAccelEstimator::snapshot() const {
    return snapshot_;
}
