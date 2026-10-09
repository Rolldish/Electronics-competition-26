#pragma once

#include <cstdint>

struct ImuFeedforwardConfig {
    float levelAngleRad = 0.0F;
    float gravityMps2 = 9.80665F;
    float accelerationGain = 1.0F;
    float maxOffsetDeg = 3.0F;
    float minimumTargetAngleRad = 0.0F;
    float maximumTargetAngleRad = 0.0F;
};

struct ImuFeedforwardOutput {
    float accelerationMps2 = 0.0F;
    float angleOffsetDeg = 0.0F;
    float targetAngleRad = 0.0F;
    bool valid = false;
};

class ImuFeedforwardController {
public:
    explicit ImuFeedforwardController(
        const ImuFeedforwardConfig& config);

    [[nodiscard]] ImuFeedforwardOutput update(
        bool imuValid, float forwardAccelerationMps2) const;

private:
    ImuFeedforwardConfig config_{};
};

struct ImuFeedforwardPeakSnapshot {
    std::uint32_t validSampleCount = 0U;
    float maximumAccelerationMps2 = 0.0F;
    float minimumAccelerationMps2 = 0.0F;
    float maximumAngleOffsetDeg = 0.0F;
    float minimumAngleOffsetDeg = 0.0F;
    float minimumTargetAngleRad = 0.0F;
    float maximumTargetAngleRad = 0.0F;
};

class ImuFeedforwardPeakHold {
public:
    explicit ImuFeedforwardPeakHold(float initialTargetAngleRad);

    void observe(const ImuFeedforwardOutput& output);
    [[nodiscard]] const ImuFeedforwardPeakSnapshot& snapshot() const;

private:
    ImuFeedforwardPeakSnapshot snapshot_{};
};
