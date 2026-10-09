#include "BallBalanceController.h"
#include "BallControlConfig.h"
#include "CompetitionTaskGuard.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr std::uint8_t kRunActive = 0x02U;

[[noreturn]] void fail(const std::string& message)
{
    throw std::runtime_error(message);
}

void require(const bool condition, const std::string& message)
{
    if (!condition) {
        fail(message);
    }
}

void requireNear(const float actual,
                 const float expected,
                 const float tolerance,
                 const std::string& message)
{
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        fail(message + ": expected " + std::to_string(expected)
             + ", got " + std::to_string(actual));
    }
}

BallControlInput baseInput(const std::uint32_t nowMs = 0U)
{
    BallControlInput input{};
    input.nowMs = nowMs;
    input.commissioned = true;
    input.motorFeedbackFresh = true;
    input.motorFeedbackFinite = true;
    input.motorEnabled = false;
    input.motorFeedbackCount =
        ball_control_config::kMotorFeedbackRequiredFrames;
    input.motorSpeedRpm = 0.0F;
    input.motorCurrentA = 0.0F;
    input.motorAngleRad = ball_control_config::kMechanicalLevelAngleRad;
    return input;
}

BallVisionSnapshot healthyVision(const std::uint32_t updateCount,
                                 const std::uint8_t taskId = 3U)
{
    BallVisionSnapshot vision{};
    vision.pi_session_id = 7U;
    vision.capture_timestamp_ms = updateCount * 10U;
    vision.local_receive_ms = updateCount * 10U;
    vision.measurement_age_ms = 10U;
    vision.position_0p1mm = 0;
    vision.velocity_mmps = 0;
    vision.target_position_0p1mm = 0;
    vision.measurement_update_count = updateCount;
    vision.confidence = 220U;
    vision.vision_flags =
        ball_control_config::kVisionBallValid
        | ball_control_config::kVisionCameraCalibrated;
    vision.control_flags = ball_control_config::kControlEnabled;
    vision.task_id = taskId;
    vision.run_id = 42U;
    vision.sequence = static_cast<std::uint8_t>(updateCount);
    vision.valid = true;
    vision.velocity_valid = true;
    return vision;
}

CompetitionTaskOutput applyCompetitionContext(
    CompetitionTaskGuard& guard,
    BallVisionSnapshot& vision)
{
    const CompetitionTaskOutput context = guard.update(
        CompetitionTaskInput{
            .piSessionId = vision.pi_session_id,
            .runId = vision.run_id,
            .targetPosition0p1mm = vision.target_position_0p1mm,
            .taskId = vision.task_id,
            .controlFlags = vision.control_flags,
            .sequence = vision.sequence,
        });
    vision.control_flags = context.effectiveControlFlags;
    return context;
}

BallControlOutput leaveBootSafe(BallBalanceController& controller,
                                BallControlInput& input)
{
    const BallControlOutput boot = controller.update(input);
    require(boot.state == BallControlState::BootSafe,
            "first update did not report BootSafe");
    require(boot.zeroOutput && boot.disableMotor,
            "BootSafe did not request zero output and disable");
    input.nowMs += ball_control_config::kControlPeriodMs;
    return controller.update(input);
}

void reachLevelHold(BallBalanceController& controller,
                    BallControlInput& input)
{
    input.motorFeedbackCount = 0U;
    BallControlOutput output = leaveBootSafe(controller, input);
    require(output.state == BallControlState::WaitMotor,
            "controller did not wait for motor feedback");

    for (std::uint32_t count = 1U;
         count <= ball_control_config::kMotorFeedbackRequiredFrames;
         ++count) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.motorFeedbackCount = count;
        output = controller.update(input);
    }

    require(output.state == BallControlState::EnablePending,
            "five motor feedback frames did not enter EnablePending");
    require(!output.enableMotor && output.sendAngleCommand,
            "EnablePending did not prime level angle before enable");

    input.nowMs += ball_control_config::kControlPeriodMs;
    output = controller.update(input);
    require(output.state == BallControlState::EnablePending,
            "enable command did not remain EnablePending");
    require(output.enableMotor && output.sendAngleCommand,
            "EnablePending did not send the first rate-limited enable");

    input.motorEnabled = true;
    for (std::uint32_t confirmation = 1U;
         confirmation <= ball_control_config::kMotorEnableConfirmFrames;
         ++confirmation) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        ++input.motorFeedbackCount;
        output = controller.update(input);
    }

    require(output.state == BallControlState::LevelHold,
            "enabled feedback confirmation did not enter LevelHold");
    require(!output.enableMotor && output.sendAngleCommand,
            "LevelHold entry emitted an extra enable or lost angle hold");
    requireNear(output.targetAngleRad,
                ball_control_config::kMechanicalLevelAngleRad,
                0.00001F,
                "LevelHold did not command mechanical level");
}

void enterBalancing(BallBalanceController& controller,
                    BallControlInput& input,
                    const std::uint8_t taskId = 3U)
{
    reachLevelHold(controller, input);
    BallControlOutput output{};
    for (std::uint32_t update = 1U;
         update <= ball_control_config::kVisionRequiredNewMeasurements;
         ++update) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(update, taskId);
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }
    require(output.state == BallControlState::Balancing,
            "ten healthy new observations did not enter Balancing");
}

void testInstalledConfigIsCommissioned()
{
    require(ball_control_config::kBallControlCommissioned,
            "installed build did not enable commissioned control");
    requireNear(ball_control_config::kMechanicalLevelAngleRad,
                1.2461F,
                0.0001F,
                "installed level angle changed unexpectedly");
    require(ball_control_config::kControlPeriodMs == 5U,
            "ball control period is not fixed at 5 milliseconds");
}

void testUncommissionedControllerStaysDisabled()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.commissioned = false;

    BallControlOutput output = leaveBootSafe(controller, input);
    require(output.state == BallControlState::ConfigRequired,
            "uncommissioned controller did not report ConfigRequired");
    require(!output.enableMotor && !output.sendAngleCommand,
            "uncommissioned controller attempted active motor control");
    require(!output.zeroOutput && !output.disableMotor,
            "ConfigRequired immediately repeated BootSafe shutdown");

    input.nowMs += ball_control_config::kControlPeriodMs;
    output = controller.update(input);
    require(output.state == BallControlState::ConfigRequired,
            "uncommissioned controller left ConfigRequired");
    require(!output.zeroOutput && !output.disableMotor,
            "ConfigRequired repeated shutdown commands at 200 Hz");

    input.nowMs += ball_control_config::kSafeShutdownRetryMs;
    output = controller.update(input);
    require(output.zeroOutput && output.disableMotor,
            "ConfigRequired did not retry shutdown at the low-rate interval");
}

void testExactlyFiveMotorFramesAreRequired()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.motorFeedbackCount = 0U;
    BallControlOutput output = leaveBootSafe(controller, input);

    for (std::uint32_t count = 1U; count < 5U; ++count) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.motorFeedbackCount = count;
        output = controller.update(input);
        require(output.state == BallControlState::WaitMotor,
                "controller entered LevelHold before five feedback frames");
        require(!output.enableMotor, "motor enabled before feedback gate");
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorFeedbackCount = 5U;
    output = controller.update(input);
    require(output.state == BallControlState::EnablePending,
            "fifth feedback frame did not enter EnablePending");
    require(!output.enableMotor && output.sendAngleCommand,
            "fifth feedback did not prime angle before enable");
}

void testPreGateMechanicalAngleViolationFaultsImmediately()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.motorFeedbackCount = 0U;
    leaveBootSafe(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorFeedbackCount = 1U;
    input.motorAngleRad =
        ball_control_config::kMechanicalLevelAngleRad
        + (ball_control_config::kMechanicalAngleLimitDeg + 0.1F)
              * kPi / 180.0F;
    const BallControlOutput output = controller.update(input);

    require(output.state == BallControlState::Fault,
            "pre-gate mechanical angle violation did not fault");
    require(output.fault == BallControlFault::MechanicalAngleLimit,
            "pre-gate angle violation reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "pre-gate angle violation did not request shutdown");
    require(!output.enableMotor && !output.sendAngleCommand,
            "pre-gate angle violation enabled or commanded the motor");
}

void testFifthFeedbackOutsideMechanicalLimitCannotEnable()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.motorFeedbackCount = 0U;
    leaveBootSafe(controller, input);

    BallControlOutput output{};
    for (std::uint32_t count = 1U; count <= 4U; ++count) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.motorFeedbackCount = count;
        output = controller.update(input);
        require(output.state == BallControlState::WaitMotor,
                "controller enabled before the fifth feedback");
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorFeedbackCount = 5U;
    input.motorAngleRad =
        ball_control_config::kMechanicalLevelAngleRad
        - (ball_control_config::kMechanicalAngleLimitDeg + 0.1F)
              * kPi / 180.0F;
    output = controller.update(input);

    require(output.state == BallControlState::Fault,
            "out-of-limit fifth feedback entered LevelHold");
    require(output.fault == BallControlFault::MechanicalAngleLimit,
            "out-of-limit fifth feedback reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "out-of-limit fifth feedback did not request shutdown");
    require(!output.enableMotor && !output.sendAngleCommand,
            "out-of-limit fifth feedback enabled or commanded the motor");
}

void testMotorFeedbackGateRequiresConsecutiveDistinctFrames()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.motorFeedbackCount = 0U;
    leaveBootSafe(controller, input);

    BallControlOutput output{};
    for (std::uint32_t count = 1U; count <= 4U; ++count) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.motorFeedbackCount = count;
        output = controller.update(input);
    }
    require(output.state == BallControlState::WaitMotor,
            "four feedback frames unexpectedly enabled the motor");

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorFeedbackFresh = false;
    output = controller.update(input);
    require(output.state == BallControlState::WaitMotor,
            "pre-gate stale feedback did not remain WaitMotor");

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorFeedbackFresh = true;
    input.motorFeedbackCount = 5U;
    output = controller.update(input);
    require(output.state == BallControlState::WaitMotor,
            "one frame after a stale gap incorrectly completed the gate");
    require(!output.enableMotor && !output.sendAngleCommand,
            "one frame after a stale gap enabled or commanded the motor");
}

void testEnablePendingNeedsDistinctEnabledConfirmations()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.motorFeedbackCount = 0U;
    leaveBootSafe(controller, input);

    BallControlOutput output{};
    for (std::uint32_t count = 1U; count <= 5U; ++count) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.motorFeedbackCount = count;
        output = controller.update(input);
    }
    require(output.state == BallControlState::EnablePending,
            "feedback gate did not enter EnablePending");
    require(output.sendAngleCommand && !output.enableMotor,
            "first EnablePending cycle did not prime level angle only");

    input.nowMs += ball_control_config::kControlPeriodMs;
    output = controller.update(input);
    require(output.enableMotor,
            "cycle after angle prime did not send enable");

    input.motorEnabled = true;
    for (std::uint32_t repeat = 0U; repeat < 5U; ++repeat) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        output = controller.update(input);
        require(output.state == BallControlState::EnablePending,
                "repeated enabled snapshot counted as new confirmation");
    }

    for (std::uint32_t confirmation = 1U;
         confirmation <= ball_control_config::kMotorEnableConfirmFrames;
         ++confirmation) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        ++input.motorFeedbackCount;
        output = controller.update(input);
        if (confirmation < ball_control_config::kMotorEnableConfirmFrames) {
            require(output.state == BallControlState::EnablePending,
                    "enabled confirmation gate completed too early");
        }
    }
    require(output.state == BallControlState::LevelHold,
            "final distinct enabled confirmation did not enter LevelHold");
}

void testEnableRetriesAreRateLimitedAndTimeoutFaults()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.motorFeedbackCount = 0U;
    leaveBootSafe(controller, input);

    BallControlOutput output{};
    for (std::uint32_t count = 1U; count <= 5U; ++count) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.motorFeedbackCount = count;
        output = controller.update(input);
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    output = controller.update(input);
    require(output.enableMotor, "first enable command was not sent");

    input.nowMs += ball_control_config::kEnableRetryPeriodMs - 1U;
    output = controller.update(input);
    require(!output.enableMotor,
            "enable command retried before the retry interval");

    input.nowMs += 1U;
    output = controller.update(input);
    require(output.enableMotor,
            "enable command was not retried at the retry interval");

    while ((input.nowMs - 30U)
           <= ball_control_config::kEnableTimeoutMs) {
        input.nowMs += ball_control_config::kEnableRetryPeriodMs;
        output = controller.update(input);
        if (output.state == BallControlState::Fault) {
            break;
        }
    }
    require(output.state == BallControlState::Fault,
            "enable wait did not time out");
    require(output.fault == BallControlFault::MotorEnableTimeout,
            "enable timeout reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "enable timeout did not request hard shutdown");
}

void testRunningMotorDisableIsHardFault()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    ++input.motorFeedbackCount;
    input.motorEnabled = false;
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "running motor disable did not enter Fault");
    require(output.fault == BallControlFault::MotorUnexpectedlyDisabled,
            "running motor disable reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "running motor disable did not request hard shutdown");
}

void testTenNewMeasurementsAreRequiredAndDuplicatesDoNotCount()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    BallControlOutput output{};
    for (std::uint32_t update = 1U; update <= 9U; ++update) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(update);
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
        require(output.state == BallControlState::LevelHold,
                "controller entered Balancing before ten observations");
        require(output.consecutiveValidMeasurements == update,
                "new observation did not increment valid count");

        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision.sequence++;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
        require(output.consecutiveValidMeasurements == update,
                "duplicate image incremented valid count");
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(10U);
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    require(output.state == BallControlState::Balancing,
            "tenth new observation did not enter Balancing");
    require(output.consecutiveValidMeasurements == 10U,
            "Balancing entry reported the wrong valid count");
}

void testPdTracksTargetMinusPosition()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.vision = healthyVision(11U);
    input.vision.target_position_0p1mm = 100;
    input.vision.position_0p1mm = 0;
    input.vision.velocity_mmps = 20;
    input.vision.velocity_valid = true;
    BallControlOutput output{};
    for (std::uint32_t step = 0U; step < 3U; ++step) {
        input.nowMs += ball_control_config::kMaxSlewElapsedMs;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }

    const float expected =
        ball_control_config::kAngleSign
        * (ball_control_config::kKpDegPerMm * 10.0F
           - ball_control_config::kKdDegPerMmps * 20.0F);
    requireNear(output.angleOffsetDeg, expected, 0.0001F,
                "PD output did not track target minus position");
    require(output.targetPosition0p1mm == 100
                && output.position0p1mm == 0,
            "PD diagnostics did not preserve target and position");
}

void testVisionAgeGraceBandReturnsCommandToLevelBeforeRecovery()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    BallControlOutput output{};
    for (std::uint32_t update = 1U;
         update <= ball_control_config::kVisionRequiredNewMeasurements;
         ++update) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(update);
        input.vision.measurement_age_ms = 80U;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }
    require(output.state == BallControlState::Balancing,
            "80 ms vision measurements did not enter Balancing");

    input.vision.position_0p1mm = 100;
    for (std::uint32_t step = 0U; step < 4U; ++step) {
        input.nowMs += ball_control_config::kMaxSlewElapsedMs;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }
    require(std::fabs(output.angleOffsetDeg) > 0.5F,
            "test setup did not create a nonzero control angle");
    const float controlledOffset = output.angleOffsetDeg;

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(11U);
    input.vision.measurement_age_ms = 81U;
    input.vision.valid = false;
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    require(output.state == BallControlState::Balancing,
            "81 ms vision measurement left Balancing during grace band");
    require(std::fabs(output.angleOffsetDeg) < std::fabs(controlledOffset),
            "81 ms grace band did not slew the command toward level");

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision.measurement_age_ms = 110U;
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    require(output.state == BallControlState::Balancing,
            "110 ms boundary did not remain in the level-return grace band");

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision.measurement_age_ms = 111U;
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    require(output.state == BallControlState::VisionRecovery,
            "111 ms vision measurement did not enter VisionRecovery");
}

void testRuntimePositionGainsCanBeChangedWithinSafeBounds()
{
    BallBalanceController controller;
    require(controller.setPositionGains(0.24F, 0.02F),
            "safe runtime position gains were rejected");
    require(!controller.setPositionGains(
                std::numeric_limits<float>::quiet_NaN(), 0.02F),
            "non-finite runtime gain was accepted");

    BallControlInput input = baseInput();
    enterBalancing(controller, input);
    input.vision = healthyVision(11U);
    input.vision.target_position_0p1mm = 100;
    input.vision.velocity_mmps = 20;
    input.vision.velocity_valid = true;

    BallControlOutput output{};
    for (std::uint32_t step = 0U; step < 5U; ++step) {
        input.nowMs += ball_control_config::kMaxSlewElapsedMs;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }

    const float expected =
        ball_control_config::kAngleSign * (0.24F * 10.0F - 0.02F * 20.0F);
    requireNear(output.angleOffsetDeg, expected, 0.0001F,
                "runtime Kp/Kd were not used by the position loop");
    require(!controller.setPositionGains(
                ball_control_config::kMaximumTunableKpDegPerMm + 0.01F,
                0.02F),
            "out-of-range runtime Kp was accepted");
}

void testTask3NearTargetStictionKickIsIsolated()
{
    auto stationaryOutput = [](const std::uint8_t taskId) {
        BallBalanceController controller;
        BallControlInput input = baseInput();
        enterBalancing(controller, input, taskId);

        BallControlOutput output{};
        for (std::uint32_t update = 11U; update <= 26U; ++update) {
            input.nowMs += 10U;
            input.vision = healthyVision(update, taskId);
            input.vision.local_receive_ms = input.nowMs;
            input.vision.target_position_0p1mm = 80;
            input.vision.position_0p1mm = 0;
            input.vision.velocity_mmps = 0;
            output = controller.update(input);
        }
        return output;
    };

    const BallControlOutput task3 = stationaryOutput(3U);
    require(task3.stictionState == BallStictionState::Breakaway,
            "task 3 did not enter breakaway for a stationary 8 mm error");
    require(task3.activeAngleLimitDeg >
                ball_control_config::kMaxAngleOffsetDeg,
            "task-3 breakaway did not enable the qualified hard ceiling");
    require(std::fabs(task3.angleOffsetDeg) > 2.0F,
            "task-3 breakaway did not command a kick beyond ordinary PD");

    const BallControlOutput task2 = stationaryOutput(2U);
    require(task2.stictionState == BallStictionState::Monitoring,
            "task-3 stiction calibration leaked into another task");
    requireNear(task2.activeAngleLimitDeg,
                ball_control_config::kMaxAngleOffsetDeg,
                0.0001F,
                "another task inherited the task-3 breakaway ceiling");
}

void testTask3UsesDedicatedMotionAngleLimitOnly()
{
    auto saturatedOutput = [](const std::uint8_t taskId) {
        BallBalanceController controller;
        BallControlInput input = baseInput();
        enterBalancing(controller, input, taskId);

        BallControlOutput output{};
        for (std::uint32_t update = 11U; update <= 30U; ++update) {
            input.nowMs += ball_control_config::kMaxSlewElapsedMs;
            input.vision = healthyVision(update, taskId);
            input.vision.local_receive_ms = input.nowMs;
            input.vision.target_position_0p1mm = 500;
            input.vision.position_0p1mm = 0;
            input.vision.velocity_mmps = 6;
            output = controller.update(input);
        }
        return output;
    };

    const BallControlOutput task3 = saturatedOutput(3U);
    requireNear(std::fabs(task3.angleOffsetDeg),
                4.0F,
                0.0001F,
                "task 3 did not use its dedicated motion angle limit");

    const BallControlOutput task2 = saturatedOutput(2U);
    requireNear(std::fabs(task2.angleOffsetDeg),
                ball_control_config::kMaxAngleOffsetDeg,
                0.0001F,
                "task-3 motion angle limit leaked into another task");
}

void testCenterHoldUsesThreeMmBandAndDistinctMeasurements()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    auto updateAt = [&](const std::uint32_t updateCount,
                        const std::int16_t position0p1mm) {
        input.nowMs += ball_control_config::kMaxSlewElapsedMs;
        input.vision = healthyVision(updateCount);
        input.vision.local_receive_ms = input.nowMs;
        input.vision.position_0p1mm = position0p1mm;
        return controller.update(input);
    };

    BallControlOutput output{};
    for (std::uint32_t step = 0U; step < 5U; ++step) {
        output = updateAt(11U, 100);
    }
    requireNear(output.angleOffsetDeg,
                -ball_control_config::kKpDegPerMm * 10.0F,
                0.0001F,
                "outside-band PD setup did not settle");

    output = updateAt(12U, 30);
    for (std::uint32_t duplicate = 0U; duplicate < 3U; ++duplicate) {
        output = updateAt(12U, 30);
    }
    require(!output.centerHoldActive,
            "duplicate in-band frames confirmed center hold");
    requireNear(output.angleOffsetDeg,
                -ball_control_config::kKpDegPerMm * 3.0F,
                0.0001F,
                "duplicate in-band frames changed the PD target");

    output = updateAt(13U, 30);
    output = updateAt(14U, 30);
    require(output.centerHoldActive,
                "three distinct 3 mm measurements did not enter center hold");

    output = updateAt(15U, 50);
    require(!output.centerHoldActive,
            "first fresh measurement outside 3 mm did not exit center hold");
    require(output.angleOffsetDeg < -0.1F,
            "first fresh measurement outside 3 mm did not resume PD control");
}

void testVelocityTermIsIgnoredWhenInvalid()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.vision = healthyVision(11U);
    input.vision.target_position_0p1mm = 100;
    input.vision.velocity_mmps = 300;
    input.vision.velocity_valid = false;
    BallControlOutput output{};
    for (std::uint32_t step = 0U; step < 5U; ++step) {
        input.nowMs += ball_control_config::kMaxSlewElapsedMs;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }

    const float expected =
        ball_control_config::kAngleSign
        * ball_control_config::kKpDegPerMm * 10.0F;
    requireNear(output.angleOffsetDeg, expected, 0.0001F,
                "invalid velocity still affected PD output");
    requireNear(output.velocityMmps, 0.0F, 0.0001F,
                "invalid velocity was not zeroed in diagnostics");
}

void testAngleOffsetIsLimitedToThreeDegrees()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input, 2U);

    input.vision = healthyVision(11U, 2U);
    input.vision.target_position_0p1mm = 1200;
    input.vision.position_0p1mm = -1200;
    BallControlOutput output{};
    for (std::uint32_t step = 0U; step < 8U; ++step) {
        input.nowMs += ball_control_config::kMaxSlewElapsedMs;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }

    requireNear(std::fabs(output.angleOffsetDeg),
                ball_control_config::kMaxAngleOffsetDeg,
                0.0001F,
                "large ball error was not limited to three degrees");
}

void testTargetAngleSlewRateIsLimited()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(11U);
    input.vision.local_receive_ms = input.nowMs;
    input.vision.target_position_0p1mm = 1200;
    input.vision.position_0p1mm = -1200;
    const BallControlOutput output = controller.update(input);

    const float maxStep =
        ball_control_config::kTargetSlewRateDegPerSecond
        * static_cast<float>(ball_control_config::kControlPeriodMs)
        / 1000.0F;
    requireNear(std::fabs(output.angleOffsetDeg), maxStep, 0.0001F,
                "target angle exceeded one-cycle slew limit");
}

void testControlDisabledReturnsToLevelWithSlewLimit()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.vision = healthyVision(11U);
    input.vision.target_position_0p1mm = 1200;
    input.vision.position_0p1mm = -1200;
    BallControlOutput output{};
    for (std::uint32_t step = 0U; step < 8U; ++step) {
        input.nowMs += ball_control_config::kMaxSlewElapsedMs;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }
    require(std::fabs(output.angleOffsetDeg) > 0.5F,
            "test setup did not create a nonzero balancing angle");
    const float previousOffset = output.angleOffsetDeg;

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision.control_flags = 0U;
    output = controller.update(input);
    require(output.state == BallControlState::VisionRecovery,
            "CONTROL_ENABLED=0 did not enter VisionRecovery");
    require(output.sendAngleCommand && !output.disableMotor,
            "vision soft failure disabled the motor");
    require(std::fabs(output.angleOffsetDeg) < std::fabs(previousOffset),
            "VisionRecovery did not slew toward level");
}

void testMotorSpeedAndCurrentRequireConsecutiveViolations()
{
    {
        BallBalanceController controller;
        BallControlInput input = baseInput();
        reachLevelHold(controller, input);
        BallControlOutput output{};
        for (std::uint32_t frame = 1U;
             frame <= ball_control_config::kMotorLimitViolationFrames;
             ++frame) {
            input.nowMs += ball_control_config::kControlPeriodMs;
            ++input.motorFeedbackCount;
            input.motorSpeedRpm =
                ball_control_config::kMaxMotorSpeedRpm + 1.0F;
            output = controller.update(input);
        }
        require(output.state == BallControlState::Fault,
                "consecutive overspeed feedback did not fault");
        require(output.fault == BallControlFault::MotorOverspeed,
                "overspeed reported the wrong fault");
    }

    {
        BallBalanceController controller;
        BallControlInput input = baseInput();
        reachLevelHold(controller, input);
        BallControlOutput output{};
        for (std::uint32_t frame = 1U;
             frame <= ball_control_config::kMotorLimitViolationFrames;
             ++frame) {
            input.nowMs += ball_control_config::kControlPeriodMs;
            ++input.motorFeedbackCount;
            input.motorCurrentA =
                ball_control_config::kMaxMotorCurrentA + 0.1F;
            output = controller.update(input);
        }
        require(output.state == BallControlState::Fault,
                "consecutive overcurrent feedback did not fault");
        require(output.fault == BallControlFault::MotorOvercurrent,
                "overcurrent reported the wrong fault");
    }
}

void testSlewDoesNotAdvanceAtZeroElapsedAndCapsSingleStep()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.vision = healthyVision(11U);
    input.vision.local_receive_ms = input.nowMs;
    input.vision.target_position_0p1mm = 1200;
    input.vision.position_0p1mm = -1200;
    BallControlOutput output = controller.update(input);
    requireNear(output.angleOffsetDeg, 0.0F, 0.0001F,
                "zero elapsed time advanced the slew limiter");

    input.nowMs += ball_control_config::kMaxSlewElapsedMs + 30U;
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    const float cappedStep =
        ball_control_config::kTargetSlewRateDegPerSecond
        * static_cast<float>(ball_control_config::kMaxSlewElapsedMs)
        / 1000.0F;
    requireNear(std::fabs(output.angleOffsetDeg), cappedStep, 0.0001F,
                "single delayed update was not capped");
}

void testSevereControlLoopDelayIsHardFault()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    input.nowMs += ball_control_config::kSevereControlLapseMs + 1U;
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "severe control-loop delay did not enter Fault");
    require(output.fault == BallControlFault::ControlLoopTiming,
            "severe control-loop delay reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "control-loop timing fault did not request hard shutdown");
}

void testShortAgeDropoutRecoversAfterThreeDistinctNewMeasurements()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision.measurement_age_ms = 111U;
    input.vision.valid = false;
    input.vision.local_receive_ms = input.nowMs;
    BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::VisionRecovery,
            "short age dropout did not enter VisionRecovery");

    for (std::uint32_t offset = 1U;
         offset < ball_control_config::kVisionRecoveryNewMeasurements;
         ++offset) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(10U + offset);
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
        require(output.state == BallControlState::VisionRecovery,
                "short recovery accepted fewer than three new images");
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(
        10U + ball_control_config::kVisionRecoveryNewMeasurements);
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    require(output.state == BallControlState::Balancing,
            "third distinct image did not complete short recovery");
}

void testLongAgeDropoutRequiresTenNewMeasurementsAgain()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    BallControlOutput output{};
    for (std::uint32_t elapsed = 5U;
         elapsed <= ball_control_config::kFastVisionRecoveryMaxDropoutMs + 5U;
         elapsed += ball_control_config::kControlPeriodMs) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision.measurement_age_ms =
            ball_control_config::kVisionMaxMeasurementAgeMs + elapsed;
        input.vision.valid = false;
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
    }
    require(output.state == BallControlState::VisionRecovery,
            "long age dropout did not remain in VisionRecovery");

    for (std::uint32_t offset = 1U;
         offset < ball_control_config::kVisionRequiredNewMeasurements;
         ++offset) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(10U + offset);
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
        require(output.state == BallControlState::VisionRecovery,
                "long recovery ended before ten distinct new images");
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(
        10U + ball_control_config::kVisionRequiredNewMeasurements);
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    require(output.state == BallControlState::Balancing,
            "tenth distinct image did not complete long recovery");
}

void testSystemFailureRequiresTenNewMeasurementsAgain()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision.control_flags = 0U;
    BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::VisionRecovery,
            "test setup did not enter VisionRecovery");

    const std::uint32_t firstRecoveryUpdate =
        input.vision.measurement_update_count + 1U;
    for (std::uint32_t offset = 0U; offset < 9U; ++offset) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(firstRecoveryUpdate + offset);
        input.vision.local_receive_ms = input.nowMs;
        output = controller.update(input);
        require(output.state == BallControlState::VisionRecovery,
                "VisionRecovery ended before ten new observations");
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(firstRecoveryUpdate + 9U);
    input.vision.local_receive_ms = input.nowMs;
    output = controller.update(input);
    require(output.state == BallControlState::Balancing,
            "tenth recovery observation did not restore Balancing");
}

void testInvalidCompetitionContextUsesSoftRecoveryAndTenFrameReentry()
{
    BallBalanceController controller;
    CompetitionTaskGuard guard;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    BallControlOutput output{};
    for (std::uint32_t update = 1U;
         update <= ball_control_config::kVisionRequiredNewMeasurements;
         ++update) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(update);
        input.vision.local_receive_ms = input.nowMs;
        input.vision.target_position_0p1mm = 500;
        input.vision.control_flags = static_cast<std::uint8_t>(
            ball_control_config::kControlEnabled | kRunActive);
        const CompetitionTaskOutput context =
            applyCompetitionContext(guard, input.vision);
        require(context.contextValid, "legal active T3 context was rejected");
        output = controller.update(input);
    }
    require(output.state == BallControlState::Balancing,
            "legal active T3 did not enter Balancing");
    require(std::fabs(output.angleOffsetDeg) > 0.0F,
            "legal active T3 did not create a nonzero balancing angle");
    const float balancingOffset = output.angleOffsetDeg;

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(11U);
    input.vision.local_receive_ms = input.nowMs;
    input.vision.target_position_0p1mm = 0;
    input.vision.control_flags = static_cast<std::uint8_t>(
        ball_control_config::kControlEnabled | kRunActive);
    const CompetitionTaskOutput invalidContext =
        applyCompetitionContext(guard, input.vision);
    require(!invalidContext.contextValid,
            "invalid active T3 target=0 context was accepted");
    require((invalidContext.effectiveControlFlags
             & ball_control_config::kControlEnabled) == 0U,
            "invalid task context retained CONTROL_ENABLED");
    output = controller.update(input);
    require(output.state == BallControlState::VisionRecovery,
            "invalid task context did not enter VisionRecovery");
    require(output.sendAngleCommand && !output.zeroOutput
                && !output.disableMotor,
            "task soft failure disabled the motor");
    const float maxRecoveryStep =
        ball_control_config::kTargetSlewRateDegPerSecond
        * static_cast<float>(ball_control_config::kControlPeriodMs)
        / 1000.0F;
    require(std::fabs(output.angleOffsetDeg) < std::fabs(balancingOffset),
            "task soft recovery did not move toward level");
    require(std::fabs(output.angleOffsetDeg - balancingOffset)
                <= maxRecoveryStep + 0.0001F,
            "task soft recovery exceeded the angle slew limit");

    for (std::uint32_t offset = 0U; offset < 9U; ++offset) {
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(12U + offset);
        input.vision.local_receive_ms = input.nowMs;
        input.vision.target_position_0p1mm = -500;
        input.vision.control_flags = static_cast<std::uint8_t>(
            ball_control_config::kControlEnabled | kRunActive);
        const CompetitionTaskOutput context =
            applyCompetitionContext(guard, input.vision);
        require(context.contextValid,
                "restored legal T3 context was rejected");
        output = controller.update(input);
        require(output.state == BallControlState::VisionRecovery,
                "task recovery ended before ten new observations");
    }

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.vision = healthyVision(21U);
    input.vision.local_receive_ms = input.nowMs;
    input.vision.target_position_0p1mm = -500;
    input.vision.control_flags = static_cast<std::uint8_t>(
        ball_control_config::kControlEnabled | kRunActive);
    const CompetitionTaskOutput restoredContext =
        applyCompetitionContext(guard, input.vision);
    require(restoredContext.contextValid,
            "tenth restored T3 context was rejected");
    output = controller.update(input);
    require(output.state == BallControlState::Balancing,
            "tenth restored observation did not enter Balancing");
}

void testVisionSilenceReturnsToLevelHold()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    enterBalancing(controller, input);

    input.nowMs += ball_control_config::kVisionSilenceTimeoutMs + 1U;
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::VisionRecovery,
            "vision silence did not enter VisionRecovery");
    require(output.sendAngleCommand && !output.zeroOutput
                && !output.disableMotor,
            "vision silence was treated as a hard motor fault");
}

void testVisionHealthFailuresUseSoftRecovery()
{
    enum class FailureCase : std::uint8_t {
        InvalidSnapshot,
        NoBall,
        CameraUncalibrated,
        ProcessingDegraded,
        LowConfidence,
    };

    constexpr std::array<FailureCase, 5> failureCases{
        FailureCase::InvalidSnapshot,
        FailureCase::NoBall,
        FailureCase::CameraUncalibrated,
        FailureCase::ProcessingDegraded,
        FailureCase::LowConfidence,
    };

    for (const FailureCase failure : failureCases) {
        BallBalanceController controller;
        BallControlInput input = baseInput();
        enterBalancing(controller, input);

        input.nowMs += ball_control_config::kControlPeriodMs;
        input.vision = healthyVision(11U);
        input.vision.local_receive_ms = input.nowMs;
        switch (failure) {
        case FailureCase::InvalidSnapshot:
            input.vision.valid = false;
            break;
        case FailureCase::NoBall:
            input.vision.vision_flags = static_cast<std::uint8_t>(
                input.vision.vision_flags
                & ~ball_control_config::kVisionBallValid);
            break;
        case FailureCase::CameraUncalibrated:
            input.vision.vision_flags = static_cast<std::uint8_t>(
                input.vision.vision_flags
                & ~ball_control_config::kVisionCameraCalibrated);
            break;
        case FailureCase::ProcessingDegraded:
            input.vision.vision_flags = static_cast<std::uint8_t>(
                input.vision.vision_flags
                | ball_control_config::kVisionProcessingDegraded);
            break;
        case FailureCase::LowConfidence:
            input.vision.confidence = static_cast<std::uint8_t>(
                ball_control_config::kMinimumVisionConfidence - 1U);
            break;
        }

        const BallControlOutput output = controller.update(input);
        require(output.state == BallControlState::VisionRecovery,
                "vision health failure did not enter VisionRecovery");
        require(output.sendAngleCommand && !output.zeroOutput
                    && !output.disableMotor,
                "vision health failure was treated as a hard motor fault");
    }
}

void testCanErrorIsHardFault()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.txErrorCount = 1U;
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "CAN error did not enter Fault");
    require(output.fault == BallControlFault::CanTransmit,
            "CAN error reported the wrong fault");
    require(output.zeroOutput && output.disableMotor
                && !output.sendAngleCommand,
            "CAN hard fault did not zero and disable");
}

void testStartupCanErrorCannotBeBaselined()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    input.txErrorCount = 1U;

    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "startup CAN error was silently accepted as a baseline");
    require(output.fault == BallControlFault::CanTransmit,
            "startup CAN error reported the wrong fault");
    require(output.zeroOutput && output.disableMotor
                && !output.enableMotor && !output.sendAngleCommand,
            "startup CAN error did not force an immediate safe shutdown");
}

void testWaitMotorAndFaultShutdownRetriesAreRateLimited()
{
    {
        BallBalanceController controller;
        BallControlInput input = baseInput();
        input.motorFeedbackCount = 0U;
        const BallControlOutput boot = controller.update(input);
        require(boot.zeroOutput && boot.disableMotor,
                "BootSafe did not send immediate shutdown");

        input.nowMs += ball_control_config::kControlPeriodMs;
        BallControlOutput output = controller.update(input);
        require(output.state == BallControlState::WaitMotor,
                "test setup did not enter WaitMotor");
        require(!output.zeroOutput && !output.disableMotor,
                "WaitMotor repeated shutdown at 200 Hz");

        input.nowMs += ball_control_config::kSafeShutdownRetryMs;
        output = controller.update(input);
        require(output.zeroOutput && output.disableMotor,
                "WaitMotor did not perform low-rate shutdown retry");
    }

    {
        BallBalanceController controller;
        BallControlInput input = baseInput();
        reachLevelHold(controller, input);
        input.nowMs += ball_control_config::kControlPeriodMs;
        input.txErrorCount = 1U;
        BallControlOutput output = controller.update(input);
        require(output.state == BallControlState::Fault
                    && output.zeroOutput && output.disableMotor,
                "Fault entry did not send immediate shutdown");

        input.nowMs += ball_control_config::kControlPeriodMs;
        output = controller.update(input);
        require(!output.zeroOutput && !output.disableMotor,
                "Fault repeated shutdown at 200 Hz");

        input.nowMs += ball_control_config::kSafeShutdownRetryMs;
        output = controller.update(input);
        require(output.zeroOutput && output.disableMotor,
                "Fault did not perform low-rate shutdown retry");
    }
}

void testStaleMotorFeedbackIsHardFault()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorFeedbackFresh = false;
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "stale motor feedback did not enter Fault");
    require(output.fault == BallControlFault::MotorFeedbackTimeout,
            "stale motor feedback reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "stale feedback fault did not zero and disable");
}

void testInvalidMotorFeedbackIsHardFault()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorFeedbackFinite = false;
    input.motorAngleRad = std::numeric_limits<float>::quiet_NaN();
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "invalid motor feedback did not enter Fault");
    require(output.fault == BallControlFault::InvalidMotorFeedback,
            "invalid motor feedback reported the wrong fault");
}

void testKeyIsEmergencyStopOnly()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.keyEmergencyStop = true;
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "KEY emergency stop did not enter Fault");
    require(output.fault == BallControlFault::EmergencyStop,
            "KEY emergency stop reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "KEY emergency stop did not zero and disable");
}

void testMechanicalAngleLimitIsHardFault()
{
    BallBalanceController controller;
    BallControlInput input = baseInput();
    reachLevelHold(controller, input);

    input.nowMs += ball_control_config::kControlPeriodMs;
    input.motorAngleRad =
        ball_control_config::kMechanicalLevelAngleRad
        + (ball_control_config::kMechanicalAngleLimitDeg + 0.1F)
              * kPi / 180.0F;
    const BallControlOutput output = controller.update(input);
    require(output.state == BallControlState::Fault,
            "mechanical angle limit did not enter Fault");
    require(output.fault == BallControlFault::MechanicalAngleLimit,
            "mechanical angle limit reported the wrong fault");
    require(output.zeroOutput && output.disableMotor,
            "mechanical angle fault did not zero and disable");
}

} // namespace

int main()
{
    try {
        testInstalledConfigIsCommissioned();
        testUncommissionedControllerStaysDisabled();
        testExactlyFiveMotorFramesAreRequired();
        testPreGateMechanicalAngleViolationFaultsImmediately();
        testFifthFeedbackOutsideMechanicalLimitCannotEnable();
        testMotorFeedbackGateRequiresConsecutiveDistinctFrames();
        testEnablePendingNeedsDistinctEnabledConfirmations();
        testEnableRetriesAreRateLimitedAndTimeoutFaults();
        testRunningMotorDisableIsHardFault();
        testTenNewMeasurementsAreRequiredAndDuplicatesDoNotCount();
        testPdTracksTargetMinusPosition();
        testVisionAgeGraceBandReturnsCommandToLevelBeforeRecovery();
        testRuntimePositionGainsCanBeChangedWithinSafeBounds();
        testTask3NearTargetStictionKickIsIsolated();
        testTask3UsesDedicatedMotionAngleLimitOnly();
        testCenterHoldUsesThreeMmBandAndDistinctMeasurements();
        testVelocityTermIsIgnoredWhenInvalid();
        testAngleOffsetIsLimitedToThreeDegrees();
        testTargetAngleSlewRateIsLimited();
        testControlDisabledReturnsToLevelWithSlewLimit();
        testMotorSpeedAndCurrentRequireConsecutiveViolations();
        testSlewDoesNotAdvanceAtZeroElapsedAndCapsSingleStep();
        testSevereControlLoopDelayIsHardFault();
        testShortAgeDropoutRecoversAfterThreeDistinctNewMeasurements();
        testLongAgeDropoutRequiresTenNewMeasurementsAgain();
        testSystemFailureRequiresTenNewMeasurementsAgain();
        testInvalidCompetitionContextUsesSoftRecoveryAndTenFrameReentry();
        testVisionSilenceReturnsToLevelHold();
        testVisionHealthFailuresUseSoftRecovery();
        testCanErrorIsHardFault();
        testStartupCanErrorCannotBeBaselined();
        testWaitMotorAndFaultShutdownRetriesAreRateLimited();
        testStaleMotorFeedbackIsHardFault();
        testInvalidMotorFeedbackIsHardFault();
        testKeyIsEmergencyStopOnly();
        testMechanicalAngleLimitIsHardFault();
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }

    std::cout << "PASS: 35 ball balance controller host tests\n";
    return 0;
}
