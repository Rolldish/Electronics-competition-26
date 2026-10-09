#include "ImuFeedforwardController.h"

#include <algorithm>
#include <cmath>
#include <numbers>

ImuFeedforwardController::ImuFeedforwardController(
    const ImuFeedforwardConfig& config)
    : config_(config)
{
}

ImuFeedforwardOutput ImuFeedforwardController::update(
    const bool imuValid,
    const float forwardAccelerationMps2) const
{
    ImuFeedforwardOutput output{};
    output.targetAngleRad = config_.levelAngleRad;

    const bool configValid =
        std::isfinite(config_.levelAngleRad)
        && std::isfinite(config_.gravityMps2)
        && config_.gravityMps2 > 0.0F
        && std::isfinite(config_.accelerationGain)
        && config_.accelerationGain >= 0.0F
        && std::isfinite(config_.maxOffsetDeg)
        && config_.maxOffsetDeg >= 0.0F
        && std::isfinite(config_.minimumTargetAngleRad)
        && std::isfinite(config_.maximumTargetAngleRad)
        && config_.minimumTargetAngleRad
               <= config_.maximumTargetAngleRad;

    if (!configValid || !imuValid
        || !std::isfinite(forwardAccelerationMps2)) {
        output.targetAngleRad = std::clamp(
            output.targetAngleRad,
            config_.minimumTargetAngleRad,
            config_.maximumTargetAngleRad);
        return output;
    }

    output.accelerationMps2 = forwardAccelerationMps2;
    const float physicalOffsetDeg =
        -std::atan(
            config_.accelerationGain
            * forwardAccelerationMps2
            / config_.gravityMps2)
        * 180.0F / std::numbers::pi_v<float>;
    output.angleOffsetDeg = std::clamp(
        physicalOffsetDeg,
        -config_.maxOffsetDeg,
        config_.maxOffsetDeg);
    output.targetAngleRad = std::clamp(
        config_.levelAngleRad
            + output.angleOffsetDeg
                  * std::numbers::pi_v<float> / 180.0F,
        config_.minimumTargetAngleRad,
        config_.maximumTargetAngleRad);
    output.valid = true;
    return output;
}

ImuFeedforwardPeakHold::ImuFeedforwardPeakHold(
    const float initialTargetAngleRad)
{
    snapshot_.minimumTargetAngleRad = initialTargetAngleRad;
    snapshot_.maximumTargetAngleRad = initialTargetAngleRad;
}

void ImuFeedforwardPeakHold::observe(
    const ImuFeedforwardOutput& output)
{
    if (!output.valid
        || !std::isfinite(output.accelerationMps2)
        || !std::isfinite(output.angleOffsetDeg)
        || !std::isfinite(output.targetAngleRad)) {
        return;
    }

    ++snapshot_.validSampleCount;
    snapshot_.maximumAccelerationMps2 = std::max(
        snapshot_.maximumAccelerationMps2,
        output.accelerationMps2);
    snapshot_.minimumAccelerationMps2 = std::min(
        snapshot_.minimumAccelerationMps2,
        output.accelerationMps2);
    snapshot_.maximumAngleOffsetDeg = std::max(
        snapshot_.maximumAngleOffsetDeg,
        output.angleOffsetDeg);
    snapshot_.minimumAngleOffsetDeg = std::min(
        snapshot_.minimumAngleOffsetDeg,
        output.angleOffsetDeg);
    snapshot_.minimumTargetAngleRad = std::min(
        snapshot_.minimumTargetAngleRad,
        output.targetAngleRad);
    snapshot_.maximumTargetAngleRad = std::max(
        snapshot_.maximumTargetAngleRad,
        output.targetAngleRad);
}

const ImuFeedforwardPeakSnapshot& ImuFeedforwardPeakHold::snapshot() const
{
    return snapshot_;
}
