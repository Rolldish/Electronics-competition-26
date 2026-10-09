#ifndef APPLICATION_VEHICLE_ACCEL_ESTIMATOR_H
#define APPLICATION_VEHICLE_ACCEL_ESTIMATOR_H

#include <cstdint>

enum class VehicleAccelStatus : std::uint32_t {
    Calibrating = 0U,
    Ready = 1U,
    CalibrationRejected = 2U,
    InvalidConfig = 3U,
};

struct VehicleAccelConfig {
    std::uint32_t samplesPerOutput = 4U;
    std::uint32_t calibrationSamples = 1600U;
    float forwardAxisX = 1.0F;
    float forwardAxisY = 0.0F;
    float forwardAxisZ = 0.0F;
    float lowPassAlpha = 0.376F;
    float minimumDeadbandMps2 = 0.03F;
    float deadbandSigmaMultiplier = 3.0F;
    float maximumCalibrationNoiseMps2 = 0.20F;
    float minimumGravityMps2 = 7.84532F;
    float maximumGravityMps2 = 11.76798F;
    float maximumAbsOutputMps2 = 8.0F;
};

struct VehicleAccelSnapshot {
    VehicleAccelStatus status = VehicleAccelStatus::Calibrating;
    std::uint32_t calibrationSampleCount = 0U;
    std::uint32_t outputCount = 0U;
    float baselineXMps2 = 0.0F;
    float baselineYMps2 = 0.0F;
    float baselineZMps2 = 0.0F;
    float calibrationNoiseMps2 = 0.0F;
    float deadbandMps2 = 0.0F;
    float forwardRawMps2 = 0.0F;
    float forwardFilteredMps2 = 0.0F;
    bool valid = false;
};

class VehicleAccelEstimator {
public:
    explicit VehicleAccelEstimator(
        const VehicleAccelConfig &config = VehicleAccelConfig{});

    void reset();
    bool update(float accelXMps2, float accelYMps2, float accelZMps2);
    VehicleAccelSnapshot snapshot() const;

private:
    VehicleAccelConfig config_{};
    VehicleAccelSnapshot snapshot_{};
    float forwardAxisX_ = 1.0F;
    float forwardAxisY_ = 0.0F;
    float forwardAxisZ_ = 0.0F;
    float calibrationSumX_ = 0.0F;
    float calibrationSumY_ = 0.0F;
    float calibrationSumZ_ = 0.0F;
    float calibrationForwardMean_ = 0.0F;
    float calibrationForwardM2_ = 0.0F;
    float groupSumX_ = 0.0F;
    float groupSumY_ = 0.0F;
    float groupSumZ_ = 0.0F;
    std::uint32_t groupSampleCount_ = 0U;
    bool configValid_ = false;
};

#endif /* APPLICATION_VEHICLE_ACCEL_ESTIMATOR_H */
