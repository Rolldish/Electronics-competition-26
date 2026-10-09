#include "VehicleAccelEstimator.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

constexpr float kGravityMps2 = 9.80665F;

void require(const bool condition, const std::string &message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(const float actual, const float expected,
                 const float tolerance, const std::string &message) {
    if (!std::isfinite(actual) ||
        std::fabs(actual - expected) > tolerance) {
        throw std::runtime_error(
            message + ": expected " + std::to_string(expected) +
            ", got " + std::to_string(actual));
    }
}

VehicleAccelConfig testConfig() {
    VehicleAccelConfig config{};
    config.calibrationSamples = 8U;
    config.samplesPerOutput = 4U;
    config.lowPassAlpha = 0.376F;
    return config;
}

void calibrateLevel(VehicleAccelEstimator &estimator) {
    for (std::uint32_t index = 0U; index < 8U; ++index) {
        require(!estimator.update(0.0F, 0.0F, kGravityMps2),
                "calibration unexpectedly produced a control sample");
    }
    require(estimator.snapshot().status == VehicleAccelStatus::Ready,
            "quiet level calibration was not accepted");
}

void testFourSamplesProduceOneOutput() {
    VehicleAccelEstimator estimator(testConfig());
    calibrateLevel(estimator);

    for (std::uint32_t index = 0U; index < 3U; ++index) {
        require(!estimator.update(1.0F, 0.0F, kGravityMps2),
                "output was produced before four input samples");
    }
    require(estimator.update(1.0F, 0.0F, kGravityMps2),
            "fourth input sample did not produce an output");

    const VehicleAccelSnapshot snapshot = estimator.snapshot();
    require(snapshot.valid, "ready estimator output was not valid");
    require(snapshot.outputCount == 1U, "output count was not one");
    requireNear(snapshot.forwardRawMps2, 1.0F, 0.0001F,
                "four-sample mean was incorrect");
    requireNear(snapshot.forwardFilteredMps2, 0.376F, 0.0001F,
                "first low-pass output was incorrect");
}

void testStaticBaselineAndDeadband() {
    VehicleAccelEstimator estimator(testConfig());
    calibrateLevel(estimator);

    const VehicleAccelSnapshot calibrated = estimator.snapshot();
    requireNear(calibrated.baselineXMps2, 0.0F, 0.0001F,
                "X baseline was incorrect");
    requireNear(calibrated.baselineZMps2, kGravityMps2, 0.0001F,
                "Z baseline was incorrect");
    requireNear(calibrated.calibrationNoiseMps2, 0.0F, 0.0001F,
                "quiet calibration noise was not zero");
    requireNear(calibrated.deadbandMps2, 0.03F, 0.0001F,
                "minimum deadband was not selected");

    for (std::uint32_t index = 0U; index < 4U; ++index) {
        estimator.update(0.02F, 0.0F, kGravityMps2);
    }
    requireNear(estimator.snapshot().forwardFilteredMps2, 0.0F,
                0.0001F, "deadband did not suppress static noise");
}

void testForwardAxisCanBeConfigured() {
    VehicleAccelConfig config = testConfig();
    config.forwardAxisX = 0.0F;
    config.forwardAxisY = -2.0F;
    config.forwardAxisZ = 0.0F;
    VehicleAccelEstimator estimator(config);
    calibrateLevel(estimator);

    for (std::uint32_t index = 0U; index < 4U; ++index) {
        estimator.update(0.0F, -1.0F, kGravityMps2);
    }
    requireNear(estimator.snapshot().forwardRawMps2, 1.0F, 0.0001F,
                "configured forward axis was not normalized/applied");
}

void testNoisyCalibrationIsRejected() {
    VehicleAccelEstimator estimator(testConfig());
    for (std::uint32_t index = 0U; index < 8U; ++index) {
        const float noise = (index % 2U == 0U) ? 0.5F : -0.5F;
        estimator.update(noise, 0.0F, kGravityMps2);
    }

    const VehicleAccelSnapshot snapshot = estimator.snapshot();
    require(snapshot.status ==
                VehicleAccelStatus::CalibrationRejected,
            "noisy calibration was accepted");
    require(!snapshot.valid, "rejected calibration marked output valid");
}

void testInvalidAxisIsRejected() {
    VehicleAccelConfig config = testConfig();
    config.forwardAxisX = 0.0F;
    config.forwardAxisY = 0.0F;
    config.forwardAxisZ = 0.0F;
    VehicleAccelEstimator estimator(config);

    require(estimator.snapshot().status ==
                VehicleAccelStatus::InvalidConfig,
            "zero forward axis was accepted");
    require(!estimator.update(0.0F, 0.0F, kGravityMps2),
            "invalid estimator produced an output");
}

void testOutputIsLimited() {
    VehicleAccelConfig config = testConfig();
    config.lowPassAlpha = 1.0F;
    config.maximumAbsOutputMps2 = 2.0F;
    VehicleAccelEstimator estimator(config);
    calibrateLevel(estimator);

    for (std::uint32_t index = 0U; index < 4U; ++index) {
        estimator.update(20.0F, 0.0F, kGravityMps2);
    }
    requireNear(estimator.snapshot().forwardFilteredMps2, 2.0F,
                0.0001F, "positive output limit was not applied");
}

void testNonFiniteSampleInvalidatesOutput()
{
    VehicleAccelEstimator estimator(testConfig());
    calibrateLevel(estimator);
    for (std::uint32_t index = 0U; index < 4U; ++index) {
        estimator.update(1.0F, 0.0F, kGravityMps2);
    }
    require(estimator.snapshot().valid,
            "test setup did not produce a valid output");

    require(!estimator.update(
                std::numeric_limits<float>::quiet_NaN(),
                0.0F,
                kGravityMps2),
            "nonfinite sample unexpectedly produced output");
    require(!estimator.snapshot().valid,
            "nonfinite sample left the previous output marked valid");
}

void testResetClearsPartialOutputGroup()
{
    VehicleAccelEstimator estimator(testConfig());
    calibrateLevel(estimator);
    estimator.update(4.0F, 0.0F, kGravityMps2);
    estimator.update(4.0F, 0.0F, kGravityMps2);

    estimator.reset();
    calibrateLevel(estimator);
    for (std::uint32_t index = 0U; index < 3U; ++index) {
        require(!estimator.update(1.0F, 0.0F, kGravityMps2),
                "reset retained samples from an old output group");
    }
    require(estimator.update(1.0F, 0.0F, kGravityMps2),
            "fresh fourth sample did not complete the group after reset");
}

} // namespace

int main() {
    try {
        testFourSamplesProduceOneOutput();
        testStaticBaselineAndDeadband();
        testForwardAxisCanBeConfigured();
        testNoisyCalibrationIsRejected();
        testInvalidAxisIsRejected();
        testOutputIsLimited();
        testNonFiniteSampleInvalidatesOutput();
        testResetClearsPartialOutputGroup();
        std::cout << "PASS: vehicle acceleration estimator tests\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
