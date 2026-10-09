#include "CompetitionImuFeedforward.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void requireNear(const float actual, const float expected,
                 const std::string& message)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > 0.0001F) {
        throw std::runtime_error(message + ": expected "
            + std::to_string(expected) + ", got " + std::to_string(actual));
    }
}

CompetitionImuFeedforwardConfig isolatedLaunchConfig()
{
    CompetitionImuFeedforwardConfig config{};
    config.feedforward = ImuFeedforwardConfig{
        .levelAngleRad = 1.2461F,
        .gravityMps2 = 9.80665F,
        .accelerationGain = 0.0F,
        .maxOffsetDeg = 3.0F,
        .minimumTargetAngleRad = 1.10F,
        .maximumTargetAngleRad = 1.40F,
    };
    config.task4StartAccelerationGain = 0.0F;
    config.task4BrakeAccelerationGain = 0.0F;
    return config;
}

CompetitionImuFeedforwardInput activeInput()
{
    return CompetitionImuFeedforwardInput{
        .nowMs = 1000U,
        .taskContextValid = true,
        .taskId = 4U,
        .controlFlags = 0x03U,
        .balancing = true,
        .imu = ImuVehicleControlSnapshot{
            .status = 1U,
            .calibrated = true,
            .valid = true,
            .sampleCount = 10U,
            .forwardAccelerationMps2 = 0.0F,
        },
        .pdOffsetDeg = 0.0F,
        .activeAngleLimitDeg = 3.0F,
    };
}

void freshAcceleration(CompetitionImuFeedforwardInput& input,
                       const std::uint32_t nowMs, const float acceleration)
{
    input.nowMs = nowMs;
    ++input.imu.sampleCount;
    input.imu.forwardAccelerationMps2 = acceleration;
}

void triggerLaunch(CompetitionImuFeedforwardController& controller,
                   CompetitionImuFeedforwardInput& input,
                   const std::uint32_t startMs = 1000U)
{
    freshAcceleration(input, startMs, 0.25F);
    (void)controller.update(input);
    freshAcceleration(input, startMs + 5U, 0.25F);
    (void)controller.update(input);
}

void testPiStartWaitsForActualVehicleAcceleration()
{
    CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
    auto input = activeInput();
    input.pdOffsetDeg = 0.6F;
    requireNear(controller.update(input).totalOffsetDeg, 0.6F,
                "Pi START moved a stationary vehicle's pipe");
    freshAcceleration(input, 6000U, 0.0F);
    requireNear(controller.update(input).totalOffsetDeg, 0.6F,
                "waiting for the separate car button changed PD");
    freshAcceleration(input, 6010U, 0.25F);
    (void)controller.update(input);
    freshAcceleration(input, 6015U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, -1.4F,
                "actual launch did not add the -2 degree compensation to PD");
}

void testRepeatedImuSnapshotCannotQualifyLaunch()
{
    CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
    auto input = activeInput();
    input.imu.forwardAccelerationMps2 = 0.25F;
    requireNear(controller.update(input).totalOffsetDeg, 0.0F,
                "one acceleration sample triggered launch");
    input.nowMs += 5U;
    requireNear(controller.update(input).totalOffsetDeg, 0.0F,
                "repeated snapshot counted as a second acceleration sample");
    freshAcceleration(input, 1010U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, -2.0F,
                "two independent acceleration samples did not trigger launch");
}

void testLaunchHoldsThenDecaysAndDoesNotReplay()
{
    CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
    auto input = activeInput();
    triggerLaunch(controller, input);
    freshAcceleration(input, 1105U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, -2.0F,
                "launch compensation did not hold for 100 ms");
    freshAcceleration(input, 1455U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, -1.0F,
                "launch compensation did not decay linearly");
    freshAcceleration(input, 1805U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, 0.0F,
                "launch compensation did not end after 800 ms");
    freshAcceleration(input, 2000U, 0.4F);
    requireNear(controller.update(input).totalOffsetDeg, 0.0F,
                "continued acceleration replayed launch compensation");
}

void testLaunchGetsFourDegreeHeadroomBeforeMechanicalSlew()
{
    auto config = isolatedLaunchConfig();
    config.task4StartAccelerationGain = 10.0F;
    config.task4MaxOffsetDeg = 20.0F;
    CompetitionImuFeedforwardController controller(config);
    auto input = activeInput();
    triggerLaunch(controller, input);
    freshAcceleration(input, 1010U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, -4.0F,
                "launch compensation was still clipped to the normal 3 degrees");
    freshAcceleration(input, 1805U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, -3.0F,
                "launch did not restore the normal total angle limit");
}

void testAllThreeVehicleTasksUseLaunchCompensation()
{
    for (const std::uint8_t taskId : std::array<std::uint8_t, 3>{4U, 5U, 6U}) {
        CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
        auto input = activeInput();
        input.taskId = taskId;
        input.controlFlags = taskId == 6U ? 0x13U : 0x03U;
        input.piSessionId = 42U;
        input.runId = 7U;
        triggerLaunch(controller, input);
        const auto output = controller.update(input);
        requireNear(output.totalOffsetDeg, -2.0F,
                    "a supported vehicle task missed launch compensation");
        require(output.launch.active && output.launch.triggerCount == 1U,
                "launch diagnostics did not record exactly one trigger");
    }
}

void testStaticTasksPreparingAndTerminalRunsNeverLaunch()
{
    const std::array<std::uint8_t, 4> tasks{2U, 3U, 4U, 6U};
    for (const auto taskId : tasks) {
        for (const auto flags : std::array<std::uint8_t, 3>{0x01U, 0x07U, 0x0BU}) {
            CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
            auto input = activeInput();
            input.taskId = taskId;
            input.controlFlags = flags;
            triggerLaunch(controller, input);
            const auto output = controller.update(input);
            require(!output.launch.active && output.launch.triggerCount == 0U,
                    "preparing, terminal, or static task triggered launch");
            requireNear(output.totalOffsetDeg, input.pdOffsetDeg,
                        "inactive launch context changed visual PD");
        }
    }
    for (const auto taskId : std::array<std::uint8_t, 2>{2U, 3U}) {
        CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
        auto input = activeInput();
        input.taskId = taskId;
        triggerLaunch(controller, input);
        require(!controller.update(input).launch.active,
                "active static task triggered launch");
    }
}

void testBelowThresholdSampleResetsQualification()
{
    auto config = isolatedLaunchConfig();
    config.launch.accelerationThresholdMps2 = 0.10F;
    CompetitionImuFeedforwardController controller(config);
    auto input = activeInput();
    freshAcceleration(input, 1000U, 0.10F);
    (void)controller.update(input);
    freshAcceleration(input, 1005U, 0.099F);
    (void)controller.update(input);
    freshAcceleration(input, 1010U, 0.10F);
    require(!controller.update(input).launch.active,
            "nonconsecutive acceleration samples triggered launch");
    freshAcceleration(input, 1015U, 0.10F);
    require(controller.update(input).launch.active,
            "threshold equality failed to qualify two fresh samples");
}

void testStaleImuCancelsWithoutReplayingOnRecovery()
{
    CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
    auto input = activeInput();
    input.pdOffsetDeg = 0.5F;
    triggerLaunch(controller, input);
    input.nowMs = 1105U;
    require(controller.update(input).launch.active,
            "launch expired at the inclusive 100 ms freshness boundary");
    input.nowMs = 1106U;
    const auto stale = controller.update(input);
    require(!stale.applied && !stale.launch.active,
            "stale IMU kept launch compensation active");
    requireNear(stale.totalOffsetDeg, 0.5F, "stale launch changed pure PD");
    freshAcceleration(input, 1110U, 0.25F);
    const auto recovered = controller.update(input);
    require(recovered.applied && !recovered.launch.active
                && recovered.launch.triggerCount == 1U,
            "IMU recovery replayed cancelled launch compensation");
    requireNear(recovered.totalOffsetDeg, 0.5F,
                "IMU recovery retained a cancelled launch bias");
}

void testInvalidImuCancelsWithoutSuppressingPd()
{
    for (unsigned failure = 0U; failure < 5U; ++failure) {
        CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
        auto input = activeInput();
        input.pdOffsetDeg = 0.5F;
        triggerLaunch(controller, input);
        auto invalid = input;
        if (failure == 0U) invalid.imu.valid = false;
        if (failure == 1U) invalid.imu.calibrated = false;
        if (failure == 2U) invalid.imu.sampleCount = 0U;
        if (failure == 3U) invalid.imu.forwardAccelerationMps2 =
            std::numeric_limits<float>::quiet_NaN();
        if (failure == 4U) invalid.imu.forwardAccelerationMps2 =
            std::numeric_limits<float>::infinity();
        requireNear(controller.update(invalid).totalOffsetDeg, 0.5F,
                    "invalid IMU launch changed visual PD");
        freshAcceleration(input, 1020U, 0.25F);
        require(!controller.update(input).launch.active,
                "invalid IMU recovery replayed launch compensation");
    }
}

void testLostControlContextCancelsWithoutReplaying()
{
    for (unsigned failure = 0U; failure < 6U; ++failure) {
        CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
        auto input = activeInput();
        input.pdOffsetDeg = 0.5F;
        triggerLaunch(controller, input);
        auto invalid = input;
        if (failure == 0U) invalid.taskContextValid = false;
        if (failure == 1U) invalid.balancing = false;
        if (failure == 2U) invalid.controlFlags = 0x02U;
        if (failure == 3U) invalid.controlFlags = 0x01U;
        if (failure == 4U) invalid.controlFlags = 0x07U;
        if (failure == 5U) invalid.controlFlags = 0x0BU;
        const auto rejected = controller.update(invalid);
        require(!rejected.launch.active, "lost context kept launch active");
        requireNear(rejected.totalOffsetDeg, 0.5F,
                    "lost launch context changed visual PD");
        freshAcceleration(input, 1020U, 0.25F);
        require(!controller.update(input).launch.active,
                "same-run context recovery replayed launch compensation");
    }
}

void testBrakingCancelsLaunchAndPreservesTask4PostStop()
{
    auto config = isolatedLaunchConfig();
    config.task4BrakeAccelerationGain = 1.0F;
    CompetitionImuFeedforwardController controller(config);
    auto input = activeInput();
    triggerLaunch(controller, input);
    freshAcceleration(input, 1010U, -0.25F);
    const auto braking = controller.update(input);
    require(!braking.launch.active && braking.feedforwardOffsetDeg > 0.0F,
            "launch bias opposed Task4 braking compensation");
    requireNear(braking.totalOffsetDeg, braking.feedforwardOffsetDeg,
                "cancelled launch still contributed during braking");
    freshAcceleration(input, 1020U, 0.0F);
    requireNear(controller.update(input).totalOffsetDeg, 4.0F,
                "launch integration broke Task4 post-stop hold");
    freshAcceleration(input, 1320U, 0.0F);
    requireNear(controller.update(input).totalOffsetDeg, 4.0F,
                "Task4 post-stop compensation did not hold 300 ms");
    freshAcceleration(input, 1670U, 0.0F);
    requireNear(controller.update(input).totalOffsetDeg, 2.0F,
                "Task4 post-stop compensation did not decay linearly");
    freshAcceleration(input, 2020U, 0.0F);
    requireNear(controller.update(input).totalOffsetDeg, 0.0F,
                "Task4 post-stop compensation did not finish");
}

void testNewRunOrPiSessionRearmsExactlyOnce()
{
    CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
    auto input = activeInput();
    input.piSessionId = 42U;
    input.runId = 7U;
    triggerLaunch(controller, input);
    freshAcceleration(input, 1805U, 0.25F);
    require(!controller.update(input).launch.active,
            "first run did not complete its launch pulse");
    ++input.runId;
    triggerLaunch(controller, input, 2000U);
    auto output = controller.update(input);
    require(output.launch.active && output.launch.triggerCount == 2U,
            "new run did not rearm launch detection");
    ++input.piSessionId;
    freshAcceleration(input, 2010U, 0.25F);
    require(!controller.update(input).launch.active,
            "Pi session change carried the previous run's active bias");
    freshAcceleration(input, 2015U, 0.25F);
    output = controller.update(input);
    require(output.launch.active && output.launch.triggerCount == 3U,
            "new Pi session did not rearm launch detection");
}

void testLaunchTimerHandlesMillisecondWraparound()
{
    CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
    auto input = activeInput();
    const auto startMs = std::numeric_limits<std::uint32_t>::max() - 20U;
    triggerLaunch(controller, input, startMs);
    freshAcceleration(input, startMs + 455U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, -1.0F,
                "millisecond wrap broke launch decay");
    freshAcceleration(input, startMs + 805U, 0.25F);
    requireNear(controller.update(input).totalOffsetDeg, 0.0F,
                "millisecond wrap prevented launch completion");
}

void testDisabledOrInvalidLaunchConfigurationKeepsPurePd()
{
    for (unsigned failure = 0U; failure < 7U; ++failure) {
        auto config = isolatedLaunchConfig();
        if (failure == 0U) config.launch.enabled = false;
        if (failure == 1U) config.launch.angleOffsetDeg =
            std::numeric_limits<float>::quiet_NaN();
        if (failure == 2U) config.launch.accelerationThresholdMps2 = 0.0F;
        if (failure == 3U) config.launch.requiredNewSamples = 0U;
        if (failure == 4U) config.launch.releaseMs = 0U;
        if (failure == 5U) config.launch.maxTotalOffsetDeg = 5.0F;
        if (failure == 6U) config.launch.maxTotalOffsetDeg = 1.0F;
        CompetitionImuFeedforwardController controller(config);
        auto input = activeInput();
        input.pdOffsetDeg = 0.5F;
        triggerLaunch(controller, input);
        const auto output = controller.update(input);
        require(!output.launch.active && output.launch.triggerCount == 0U,
                "disabled or invalid launch configuration triggered a pulse");
        requireNear(output.totalOffsetDeg, 0.5F,
                    "invalid launch configuration changed visual PD");
    }
}

void testLaunchSumIsLimitedInBothDirections()
{
    for (const float direction : std::array<float, 2>{-1.0F, 1.0F}) {
        auto config = isolatedLaunchConfig();
        config.launch.angleOffsetDeg = 2.0F * direction;
        CompetitionImuFeedforwardController controller(config);
        auto input = activeInput();
        input.pdOffsetDeg = 2.9F * direction;
        triggerLaunch(controller, input);
        requireNear(controller.update(input).totalOffsetDeg, 4.0F * direction,
                    "PD plus launch exceeded the temporary total angle ceiling");
        freshAcceleration(input, 1805U, 0.25F);
        requireNear(controller.update(input).totalOffsetDeg, 2.9F * direction,
                    "launch completion did not hand control back to PD");
    }
}

void testMildBrakingImmediatelyCancelsLaunchForT5T6()
{
    for (const auto taskId : std::array<std::uint8_t, 2>{5U, 6U}) {
        auto config = isolatedLaunchConfig();
        config.feedforward.accelerationGain = 1.0F;
        CompetitionImuFeedforwardController controller(config);
        auto input = activeInput();
        input.taskId = taskId;
        triggerLaunch(controller, input);
        freshAcceleration(input, 1010U, -0.05F);
        const auto output = controller.update(input);
        require(!output.launch.active,
                "mild braking kept the launch bias active on T5/T6");
        require(output.totalOffsetDeg > 0.0F,
                "launch bias reversed the normal brake compensation");
        requireNear(output.totalOffsetDeg, output.feedforwardOffsetDeg,
                    "mild braking retained a negative launch bias");
    }
}

void testReturningToRecentlyConsumedRunDoesNotReplay()
{
    CompetitionImuFeedforwardController controller(isolatedLaunchConfig());
    auto input = activeInput();
    input.piSessionId = 42U;
    input.runId = 7U;
    triggerLaunch(controller, input);
    ++input.runId;
    triggerLaunch(controller, input, 1020U);
    require(controller.update(input).launch.triggerCount == 2U,
            "second independent run did not trigger launch");
    --input.runId;
    freshAcceleration(input, 1040U, 0.25F);
    (void)controller.update(input);
    freshAcceleration(input, 1045U, 0.25F);
    const auto returned = controller.update(input);
    require(!returned.launch.active && returned.launch.triggerCount == 2U,
            "returning A->B->A replayed an already consumed launch");
    requireNear(returned.totalOffsetDeg, 0.0F,
                "previous run retained an active launch bias");
}

} // namespace

int main()
{
    try {
        testPiStartWaitsForActualVehicleAcceleration();
        testRepeatedImuSnapshotCannotQualifyLaunch();
        testLaunchHoldsThenDecaysAndDoesNotReplay();
        testLaunchGetsFourDegreeHeadroomBeforeMechanicalSlew();
        testAllThreeVehicleTasksUseLaunchCompensation();
        testStaticTasksPreparingAndTerminalRunsNeverLaunch();
        testBelowThresholdSampleResetsQualification();
        testStaleImuCancelsWithoutReplayingOnRecovery();
        testInvalidImuCancelsWithoutSuppressingPd();
        testLostControlContextCancelsWithoutReplaying();
        testBrakingCancelsLaunchAndPreservesTask4PostStop();
        testNewRunOrPiSessionRearmsExactlyOnce();
        testLaunchTimerHandlesMillisecondWraparound();
        testDisabledOrInvalidLaunchConfigurationKeepsPurePd();
        testLaunchSumIsLimitedInBothDirections();
        testMildBrakingImmediatelyCancelsLaunchForT5T6();
        testReturningToRecentlyConsumedRunDoesNotReplay();
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "PASS: 17 competition launch feedforward tests\n";
    return 0;
}
