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

CompetitionImuFeedforwardConfig validConfig()
{
    return CompetitionImuFeedforwardConfig{
        .enabled = true,
        .staleTimeoutMs = 100U,
        .feedforward = ImuFeedforwardConfig{
            .levelAngleRad = 1.2461F,
            .gravityMps2 = 9.80665F,
            .accelerationGain = 1.0F,
            .maxOffsetDeg = 3.0F,
            .minimumTargetAngleRad = 1.10F,
            .maximumTargetAngleRad = 1.40F,
        },
        .launch = VehicleLaunchFeedforwardConfig{.enabled = false},
    };
}

CompetitionImuFeedforwardInput validInput()
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
            .forwardAccelerationMps2 = 0.25F,
        },
        .pdOffsetDeg = 1.0F,
        .activeAngleLimitDeg = 3.0F,
    };
}

void requireRejectedWithoutChangingPd(
    const CompetitionImuFeedforwardOutput& output,
    const CompetitionImuFeedforwardGate gate,
    const float expectedPd,
    const char* message)
{
    require(!output.applied, message);
    require(output.gate == gate, "wrong feedforward gate reason");
    requireNear(output.feedforwardOffsetDeg, 0.0F, 0.0001F,
                "rejected feedforward was not zero");
    requireNear(output.totalOffsetDeg, expectedPd, 0.0001F,
                "rejected feedforward changed visual PD");
}

void testT4T5T6ApplyFeedforward()
{
    for (const std::uint8_t taskId : std::array<std::uint8_t, 3>{4U, 5U, 6U}) {
        CompetitionImuFeedforwardController controller(validConfig());
        CompetitionImuFeedforwardInput input = validInput();
        input.taskId = taskId;
        const CompetitionImuFeedforwardOutput output = controller.update(input);
        require(output.applied, "supported active task did not apply feedforward");
        require(output.gate == CompetitionImuFeedforwardGate::Applied,
                "supported active task returned wrong gate");
        require(output.sampleFresh, "new IMU sample was not fresh");
    }
}

void testUnsupportedAndInactiveContextsKeepPurePd()
{
    struct GateCase {
        CompetitionImuFeedforwardInput input;
        CompetitionImuFeedforwardGate gate;
    };
    CompetitionImuFeedforwardInput t2 = validInput();
    t2.taskId = 2U;
    CompetitionImuFeedforwardInput t3 = validInput();
    t3.taskId = 3U;
    CompetitionImuFeedforwardInput ready = validInput();
    ready.controlFlags = 0x01U;
    CompetitionImuFeedforwardInput invalidContext = validInput();
    invalidContext.taskContextValid = false;
    CompetitionImuFeedforwardInput disabledControl = validInput();
    disabledControl.controlFlags = 0x02U;
    CompetitionImuFeedforwardInput recovery = validInput();
    recovery.balancing = false;

    const std::array<GateCase, 6> cases{
        GateCase{t2, CompetitionImuFeedforwardGate::UnsupportedTask},
        GateCase{t3, CompetitionImuFeedforwardGate::UnsupportedTask},
        GateCase{ready, CompetitionImuFeedforwardGate::RunNotActive},
        GateCase{invalidContext,
                 CompetitionImuFeedforwardGate::InvalidTaskContext},
        GateCase{disabledControl,
                 CompetitionImuFeedforwardGate::ControlNotEnabled},
        GateCase{recovery, CompetitionImuFeedforwardGate::NotBalancing},
    };

    for (const GateCase& testCase : cases) {
        CompetitionImuFeedforwardController controller(validConfig());
        const CompetitionImuFeedforwardOutput output =
            controller.update(testCase.input);
        requireRejectedWithoutChangingPd(
            output, testCase.gate, testCase.input.pdOffsetDeg,
            "inactive context applied feedforward");
    }
}

void testAccelerationSignMatchesInstalledMechanism()
{
    CompetitionImuFeedforwardController forwardController(validConfig());
    CompetitionImuFeedforwardInput forward = validInput();
    forward.imu.forwardAccelerationMps2 = 0.4F;
    const CompetitionImuFeedforwardOutput forwardOutput =
        forwardController.update(forward);
    require(forwardOutput.feedforwardOffsetDeg < 0.0F,
            "positive forward acceleration did not reduce angle");

    CompetitionImuFeedforwardController rearwardController(validConfig());
    CompetitionImuFeedforwardInput rearward = validInput();
    rearward.imu.forwardAccelerationMps2 = -0.4F;
    const CompetitionImuFeedforwardOutput rearwardOutput =
        rearwardController.update(rearward);
    require(rearwardOutput.feedforwardOffsetDeg > 0.0F,
            "negative acceleration did not increase angle");
}

void testPdPlusFeedforwardUsesDynamicAngleLimit()
{
    CompetitionImuFeedforwardController upperController(validConfig());
    CompetitionImuFeedforwardInput upper = validInput();
    upper.pdOffsetDeg = 2.9F;
    upper.imu.forwardAccelerationMps2 = -1.0F;
    requireNear(upperController.update(upper).totalOffsetDeg,
                3.0F, 0.0001F,
                "positive total exceeded normal angle limit");

    CompetitionImuFeedforwardController lowerController(validConfig());
    CompetitionImuFeedforwardInput lower = validInput();
    lower.pdOffsetDeg = -2.9F;
    lower.imu.forwardAccelerationMps2 = 1.0F;
    requireNear(lowerController.update(lower).totalOffsetDeg,
                -3.0F, 0.0001F,
                "negative total exceeded normal angle limit");

    CompetitionImuFeedforwardController breakawayController(validConfig());
    CompetitionImuFeedforwardInput breakaway = validInput();
    breakaway.pdOffsetDeg = 3.4F;
    breakaway.activeAngleLimitDeg = 3.5F;
    breakaway.imu.forwardAccelerationMps2 = -1.0F;
    requireNear(breakawayController.update(breakaway).totalOffsetDeg,
                3.5F, 0.0001F,
                "feedforward ignored the active breakaway limit");
}

void testFreshnessExpiresAtMoreThanOneHundredMsAndRecovers()
{
    CompetitionImuFeedforwardController controller(validConfig());
    CompetitionImuFeedforwardInput input = validInput();
    require(controller.update(input).applied, "new sample was not applied");

    input.nowMs = 1100U;
    require(controller.update(input).applied,
            "sample was stale at the inclusive 100 ms boundary");

    input.nowMs = 1101U;
    const CompetitionImuFeedforwardOutput stale = controller.update(input);
    requireRejectedWithoutChangingPd(
        stale, CompetitionImuFeedforwardGate::ImuStale,
        input.pdOffsetDeg, "101 ms sample applied feedforward");
    require(stale.staleImuCycleCount == 1U,
            "stale cycle counter did not increment");

    input.imu.sampleCount = 11U;
    const CompetitionImuFeedforwardOutput recovered = controller.update(input);
    require(recovered.applied && recovered.sampleAgeMs == 0U,
            "new sample did not restore feedforward");
}

void testInvalidImuOnlySuppressesFeedforward()
{
    struct InvalidCase {
        CompetitionImuFeedforwardInput input;
        CompetitionImuFeedforwardGate gate;
    };
    CompetitionImuFeedforwardInput uncalibrated = validInput();
    uncalibrated.imu.calibrated = false;
    CompetitionImuFeedforwardInput invalid = validInput();
    invalid.imu.valid = false;
    CompetitionImuFeedforwardInput nonFinite = validInput();
    nonFinite.imu.forwardAccelerationMps2 =
        std::numeric_limits<float>::quiet_NaN();

    const std::array<InvalidCase, 3> cases{
        InvalidCase{uncalibrated, CompetitionImuFeedforwardGate::ImuNotReady},
        InvalidCase{invalid, CompetitionImuFeedforwardGate::ImuInvalid},
        InvalidCase{nonFinite, CompetitionImuFeedforwardGate::ImuNotFinite},
    };
    for (const InvalidCase& testCase : cases) {
        CompetitionImuFeedforwardController controller(validConfig());
        const CompetitionImuFeedforwardOutput output =
            controller.update(testCase.input);
        requireRejectedWithoutChangingPd(
            output, testCase.gate, testCase.input.pdOffsetDeg,
            "invalid IMU applied feedforward");
        require(output.invalidImuCycleCount == 1U,
                "invalid IMU counter did not increment");
    }
}

void testInvalidControlOutputIsRejected()
{
    CompetitionImuFeedforwardController controller(validConfig());
    CompetitionImuFeedforwardInput input = validInput();
    input.activeAngleLimitDeg = 0.0F;
    requireRejectedWithoutChangingPd(
        controller.update(input),
        CompetitionImuFeedforwardGate::InvalidControlOutput,
        input.pdOffsetDeg, "invalid control output applied feedforward");
}

void testDisabledConfigurationAlwaysReportsDisabled()
{
    CompetitionImuFeedforwardConfig config = validConfig();
    config.enabled = false;
    CompetitionImuFeedforwardController controller(config);
    const CompetitionImuFeedforwardInput input = validInput();
    requireRejectedWithoutChangingPd(
        controller.update(input), CompetitionImuFeedforwardGate::Disabled,
        input.pdOffsetDeg, "disabled configuration applied feedforward");
}

} // namespace

int main()
{
    try {
        testT4T5T6ApplyFeedforward();
        testUnsupportedAndInactiveContextsKeepPurePd();
        testAccelerationSignMatchesInstalledMechanism();
        testPdPlusFeedforwardUsesDynamicAngleLimit();
        testFreshnessExpiresAtMoreThanOneHundredMsAndRecovers();
        testInvalidImuOnlySuppressesFeedforward();
        testInvalidControlOutputIsRejected();
        testDisabledConfigurationAlwaysReportsDisabled();
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "PASS: 8 competition IMU feedforward tests\n";
    return 0;
}
