#include "ImuFeedforwardController.h"

#include <cmath>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void requireNear(const float actual,
                 const float expected,
                 const float tolerance,
                 const std::string& message)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        throw std::runtime_error(
            message + ": expected " + std::to_string(expected)
            + ", got " + std::to_string(actual));
    }
}

ImuFeedforwardConfig testConfig()
{
    ImuFeedforwardConfig config{};
    config.levelAngleRad = 1.2396F;
    config.gravityMps2 = 9.80665F;
    config.accelerationGain = 1.0F;
    config.maxOffsetDeg = 3.0F;
    config.minimumTargetAngleRad = 1.10F;
    config.maximumTargetAngleRad = 1.40F;
    return config;
}

void testInvalidImuReturnsLevel()
{
    const ImuFeedforwardController controller(testConfig());
    const ImuFeedforwardOutput output =
        controller.update(false, 5.0F);

    requireNear(output.angleOffsetDeg, 0.0F, 0.0001F,
                "invalid IMU did not clear feedforward");
    requireNear(output.targetAngleRad, 1.2396F, 0.0001F,
                "invalid IMU did not return to level");
}

void testForwardAccelerationDecreasesAngle()
{
    const ImuFeedforwardController controller(testConfig());
    const ImuFeedforwardOutput output =
        controller.update(true, 0.197923586F);
    const float expectedOffsetDeg =
        -std::atan(0.197923586F / 9.80665F)
        * 180.0F / 3.14159265358979323846F;

    requireNear(output.angleOffsetDeg, expectedOffsetDeg, 0.0001F,
                "positive forward acceleration used the wrong sign");
    requireNear(output.targetAngleRad,
                1.2396F
                    + expectedOffsetDeg
                          * 3.14159265358979323846F / 180.0F,
                0.0001F,
                "positive acceleration target was incorrect");
}

void testRearwardAccelerationIncreasesAngle()
{
    const ImuFeedforwardController controller(testConfig());
    const ImuFeedforwardOutput output =
        controller.update(true, -0.25F);

    if (!(output.angleOffsetDeg > 0.0F)
        || !(output.targetAngleRad > 1.2396F)) {
        throw std::runtime_error(
            "negative acceleration did not increase motor angle");
    }
}

void testOffsetAndAbsoluteTargetAreLimited()
{
    const ImuFeedforwardController controller(testConfig());
    const ImuFeedforwardOutput positive =
        controller.update(true, 100.0F);
    const ImuFeedforwardOutput negative =
        controller.update(true, -100.0F);

    requireNear(positive.angleOffsetDeg, -3.0F, 0.0001F,
                "positive acceleration exceeded offset limit");
    requireNear(negative.angleOffsetDeg, 3.0F, 0.0001F,
                "negative acceleration exceeded offset limit");
    if (positive.targetAngleRad < 1.10F
        || negative.targetAngleRad > 1.40F) {
        throw std::runtime_error(
            "feedforward target exceeded absolute command limits");
    }
}

void testPeakHoldIgnoresInvalidSamples()
{
    ImuFeedforwardPeakHold peakHold(1.2396F);
    ImuFeedforwardOutput invalid{};
    invalid.accelerationMps2 = 8.0F;
    invalid.angleOffsetDeg = -3.0F;
    invalid.targetAngleRad = 1.10F;

    peakHold.observe(invalid);
    const ImuFeedforwardPeakSnapshot& snapshot = peakHold.snapshot();

    if (snapshot.validSampleCount != 0U) {
        throw std::runtime_error("invalid sample changed peak count");
    }
    requireNear(snapshot.minimumTargetAngleRad, 1.2396F, 0.0001F,
                "invalid sample changed minimum target");
    requireNear(snapshot.maximumTargetAngleRad, 1.2396F, 0.0001F,
                "invalid sample changed maximum target");
}

void testPeakHoldCapturesBothDirections()
{
    ImuFeedforwardPeakHold peakHold(1.2396F);
    peakHold.observe(ImuFeedforwardOutput{
        0.42F, -2.1F, 1.2029F, true});
    peakHold.observe(ImuFeedforwardOutput{
        -0.31F, 1.7F, 1.2693F, true});
    peakHold.observe(ImuFeedforwardOutput{
        0.10F, -0.5F, 1.2309F, true});

    const ImuFeedforwardPeakSnapshot& snapshot = peakHold.snapshot();
    if (snapshot.validSampleCount != 3U) {
        throw std::runtime_error("valid samples were not counted");
    }
    requireNear(snapshot.maximumAccelerationMps2, 0.42F, 0.0001F,
                "maximum acceleration was not held");
    requireNear(snapshot.minimumAccelerationMps2, -0.31F, 0.0001F,
                "minimum acceleration was not held");
    requireNear(snapshot.maximumAngleOffsetDeg, 1.7F, 0.0001F,
                "maximum angle offset was not held");
    requireNear(snapshot.minimumAngleOffsetDeg, -2.1F, 0.0001F,
                "minimum angle offset was not held");
    requireNear(snapshot.minimumTargetAngleRad, 1.2029F, 0.0001F,
                "minimum target angle was not held");
    requireNear(snapshot.maximumTargetAngleRad, 1.2693F, 0.0001F,
                "maximum target angle was not held");
}

} // namespace

int main()
{
    try {
        testInvalidImuReturnsLevel();
        testForwardAccelerationDecreasesAngle();
        testRearwardAccelerationIncreasesAngle();
        testOffsetAndAbsoluteTargetAreLimited();
        testPeakHoldIgnoresInvalidSamples();
        testPeakHoldCapturesBothDirections();
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }

    std::cout << "PASS: 6 IMU feedforward controller tests\n";
    return 0;
}
