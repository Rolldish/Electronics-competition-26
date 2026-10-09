#include "task_public.h"

#include "BallBalanceController.h"
#include "BallControlConfig.h"
#include "BoardIo.h"
#include "CompetitionImuFeedforward.h"
#include "CompetitionTaskGuard.h"
#include "ImuFeedforwardController.h"
#include "ImuVehicleSnapshotStore.h"
#include "QD4310.h"
#include "SafetyWatchdog.h"
#include "VisionLink.h"
#include "VehicleLaunchTrace.h"
#include "FreeRTOS.h"
#include "can.h"
#include "main.h"
#include "task.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <numbers>

/*
 * 本文件是 QD4310 命令和 CAN 回调的唯一拥有者。
 * 通信任务只发布视觉快照；所有电机动作都在本 200 Hz 任务中完成。
 */
extern "C" {

volatile std::uint8_t ball_control_debug_state = 0U;
volatile std::uint8_t ball_control_debug_fault = 0U;
volatile float ball_control_debug_mechanical_level_rad = 0.0F;
volatile float ball_control_debug_actual_angle_rad = 0.0F;
volatile float ball_control_debug_target_angle_rad = 0.0F;
volatile float ball_control_debug_angle_offset_deg = 0.0F;
volatile std::int16_t ball_control_debug_target_position_0p1mm = 0;
volatile std::int16_t ball_control_debug_position_0p1mm = 0;
volatile float ball_control_debug_velocity_mmps = 0.0F;
volatile std::uint32_t ball_control_debug_valid_measurements = 0U;
volatile std::uint32_t ball_control_debug_measurement_update_count = 0U;
volatile std::uint32_t ball_control_debug_motor_feedback_count = 0U;
volatile std::uint8_t ball_control_debug_motor_enabled = 0U;
volatile float ball_control_debug_motor_speed_rpm = 0.0F;
volatile float ball_control_debug_motor_current_a = 0.0F;
volatile float ball_control_debug_peak_abs_motor_speed_rpm = 0.0F;
volatile std::uint8_t ball_control_debug_fault_snapshot_valid = 0U;
volatile float ball_control_debug_fault_motor_speed_rpm = 0.0F;
volatile float ball_control_debug_fault_motor_current_a = 0.0F;
volatile float ball_control_debug_fault_actual_angle_rad = 0.0F;
volatile float ball_control_debug_fault_target_angle_rad = 0.0F;
volatile float ball_control_debug_fault_last_sent_angle_rad = 0.0F;
volatile float ball_control_debug_last_sent_angle_rad = 0.0F;
volatile std::uint32_t ball_control_debug_enable_confirmations = 0U;
volatile std::uint32_t ball_control_debug_control_elapsed_ms = 0U;
volatile std::uint32_t ball_control_debug_tx_error_count = 0U;
volatile std::uint8_t ball_control_debug_task_id = 0U;
volatile std::uint16_t ball_control_debug_run_id = 0U;
volatile float ball_control_tune_kp_deg_per_mm =
    ball_control_config::kKpDegPerMm;
volatile float ball_control_tune_kd_deg_per_mmps =
    ball_control_config::kKdDegPerMmps;
volatile float ball_control_tune_breakaway_angle_deg =
    ball_control_config::kBreakawayAngleDeg;
volatile float ball_control_tune_stuck_error_mm =
    ball_control_config::kStuckErrorThresholdMm;
volatile float ball_control_tune_stuck_speed_mmps =
    ball_control_config::kStuckSpeedThresholdMmps;
volatile std::uint32_t ball_control_tune_stuck_time_ms =
    ball_control_config::kStuckQualificationMs;
volatile std::uint32_t ball_control_tune_breakaway_duration_ms =
    ball_control_config::kBreakawayDurationMs;
volatile std::uint8_t ball_control_debug_stiction_state = 0U;
volatile std::uint32_t ball_control_debug_stuck_elapsed_ms = 0U;
volatile float ball_control_debug_active_angle_limit_deg =
    ball_control_config::kMaxAngleOffsetDeg;
volatile std::uint8_t ball_control_debug_center_hold_active = 0U;
volatile std::uint8_t competition_task_debug_state = 0U;
volatile std::uint8_t competition_task_debug_error = 0U;
volatile std::uint8_t competition_task_debug_context_valid = 0U;
volatile std::uint8_t competition_task_debug_task_id = 0U;
volatile std::uint16_t competition_task_debug_run_id = 0U;
volatile std::int16_t competition_task_debug_target_0p1mm = 0;
volatile std::uint8_t competition_task_debug_raw_control_flags = 0U;
volatile std::uint8_t competition_task_debug_effective_control_flags = 0U;
volatile std::uint32_t competition_task_debug_reject_count = 0U;
volatile std::uint8_t task3_final_debug_valid = 0U;
volatile std::uint32_t task3_final_debug_capture_count = 0U;
volatile std::uint32_t task3_final_debug_pi_session_id = 0U;
volatile std::uint8_t task3_final_debug_task_id = 0U;
volatile std::uint16_t task3_final_debug_run_id = 0U;
volatile std::uint8_t task3_final_debug_terminal_state = 0U;
volatile std::uint32_t task3_final_debug_vision_status = 0U;
volatile std::int16_t task3_final_debug_target_0p1mm = 0;
volatile std::uint8_t task3_final_debug_raw_control_flags = 0U;
volatile std::int16_t task3_final_debug_x_0p1mm = 0;
volatile std::int16_t task3_final_debug_v_mmps = 0;
volatile std::uint32_t task3_final_debug_measurement_update_count = 0U;
volatile std::uint32_t task3_final_debug_measurement_age_ms = 0U;
volatile std::uint32_t task3_final_debug_accepted_frame_count = 0U;
volatile std::uint32_t task3_final_debug_crc_error_count = 0U;
volatile std::uint32_t task3_final_debug_uart_error_count = 0U;
volatile std::uint8_t task3_final_debug_guard_state = 0U;
volatile std::uint8_t task3_final_debug_guard_error = 0U;
volatile std::uint8_t task3_final_debug_context_valid = 0U;
volatile std::uint8_t task3_final_debug_effective_control_flags = 0U;
volatile std::uint32_t task3_final_debug_reject_count = 0U;
volatile std::uint8_t task3_final_debug_ball_state = 0U;
volatile std::uint8_t task3_final_debug_ball_fault = 0U;
volatile std::int16_t task3_final_debug_ball_target_0p1mm = 0;
volatile std::int16_t task3_final_debug_ball_position_0p1mm = 0;
volatile float task3_final_debug_ball_velocity_mmps = 0.0F;
volatile std::uint32_t task3_final_debug_ball_measurement_update_count = 0U;
volatile std::uint8_t task3_final_debug_center_hold_active = 0U;
volatile float task3_final_debug_angle_offset_deg = 0.0F;
volatile std::uint8_t task3_final_debug_motor_enabled = 0U;
volatile std::uint8_t task3_trace_valid_mask = 0U;
volatile std::uint32_t task3_trace_pi_session_id = 0U;
volatile std::uint16_t task3_trace_run_id = 0U;
volatile std::uint8_t task3_trace_sequence[4]{};
volatile std::int16_t task3_trace_target_0p1mm[4]{};
volatile std::uint8_t task3_trace_control_flags[4]{};
volatile std::int16_t task3_trace_x_0p1mm[4]{};
volatile std::int16_t task3_trace_v_mmps[4]{};
volatile std::uint32_t task3_trace_measurement_update_count[4]{};
volatile std::uint32_t task3_trace_measurement_age_ms[4]{};
volatile std::uint32_t task3_trace_accepted_frame_count[4]{};
volatile std::uint8_t task3_trace_guard_state[4]{};
volatile std::uint8_t task3_trace_guard_error[4]{};
volatile std::uint8_t task3_trace_effective_control_flags[4]{};
volatile std::uint32_t task3_trace_reject_count[4]{};
volatile std::uint8_t task3_trace_ball_state[4]{};
volatile std::uint8_t task3_trace_ball_fault[4]{};
volatile std::int16_t task3_trace_ball_position_0p1mm[4]{};
volatile float task3_trace_ball_velocity_mmps[4]{};
volatile std::uint8_t task3_trace_center_hold_active[4]{};
volatile float task3_trace_angle_offset_deg[4]{};
volatile std::uint8_t task3_trace_motor_enabled[4]{};
#if defined(HBALL_SIM_VISION_MOTOR_TEST)
volatile std::uint8_t ball_control_debug_sim_mode = 1U;
#else
volatile std::uint8_t ball_control_debug_sim_mode = 0U;
#endif
#if defined(HBALL_PIPE_ANGLE_TEST)
volatile std::uint8_t ball_control_debug_pipe_angle_mode = 1U;
#else
volatile std::uint8_t ball_control_debug_pipe_angle_mode = 0U;
#endif
#if defined(HBALL_PIPE_SWEEP_TEST)
volatile std::uint8_t ball_control_debug_pipe_sweep_mode = 1U;
#else
volatile std::uint8_t ball_control_debug_pipe_sweep_mode = 0U;
#endif
volatile std::uint8_t ball_control_debug_pipe_sweep_phase = 0U;
volatile std::uint8_t ball_control_debug_sim_level_captured = 0U;
volatile std::uint32_t ball_control_debug_sim_level_stable_frames = 0U;
#if defined(HBALL_SIM_VISION_MOTOR_TEST)
volatile std::uint8_t ball_control_debug_sim_absolute_angle_safe = 0U;
#else
volatile std::uint8_t ball_control_debug_sim_absolute_angle_safe = 0U;
#endif
#if defined(HBALL_IMU_FEEDFORWARD_TEST)
volatile std::uint8_t ball_control_debug_imu_feedforward_mode = 1U;
#else
volatile std::uint8_t ball_control_debug_imu_feedforward_mode = 0U;
#endif
volatile std::uint8_t imu_feedforward_debug_configured_enabled =
    ball_control_config::kCompetitionImuFeedforwardEnabled ? 1U : 0U;
volatile std::uint8_t imu_feedforward_debug_ready = 0U;
volatile std::uint8_t imu_feedforward_debug_valid = 0U;
volatile float imu_feedforward_debug_accel_mps2 = 0.0F;
volatile float imu_feedforward_debug_angle_offset_deg = 0.0F;
volatile float imu_feedforward_debug_target_angle_rad =
    ball_control_config::kInstalledLevelCommandAngleRad;
volatile std::uint8_t imu_feedforward_debug_gate_reason = 0U;
volatile std::uint8_t imu_feedforward_debug_sample_fresh = 0U;
volatile std::uint32_t imu_feedforward_debug_sample_age_ms = 0U;
volatile float imu_feedforward_debug_pd_offset_deg = 0.0F;
volatile float imu_feedforward_debug_total_offset_deg = 0.0F;
volatile std::uint32_t imu_feedforward_debug_invalid_cycle_count = 0U;
volatile std::uint32_t imu_feedforward_debug_stale_cycle_count = 0U;
volatile std::uint8_t vehicle_launch_debug_state = 0U;
volatile std::uint8_t vehicle_launch_debug_active = 0U;
volatile float vehicle_launch_debug_offset_deg = 0.0F;
volatile std::uint32_t vehicle_launch_debug_elapsed_ms = 0U;
volatile std::uint32_t vehicle_launch_debug_trigger_count = 0U;
volatile std::uint32_t imu_feedforward_debug_valid_sample_count = 0U;
volatile float imu_feedforward_debug_max_accel_mps2 = 0.0F;
volatile float imu_feedforward_debug_min_accel_mps2 = 0.0F;
volatile float imu_feedforward_debug_max_angle_offset_deg = 0.0F;
volatile float imu_feedforward_debug_min_angle_offset_deg = 0.0F;
volatile float imu_feedforward_debug_min_target_angle_rad =
    ball_control_config::kInstalledLevelCommandAngleRad;
volatile float imu_feedforward_debug_max_target_angle_rad =
    ball_control_config::kInstalledLevelCommandAngleRad;

}

namespace {

constexpr std::uint8_t kMotorId = 0U;
constexpr std::uint32_t kFeedbackStandardId = 0x500U + kMotorId;
constexpr std::uint32_t kFeedbackFreshnessMs = 100U;
constexpr std::uint32_t kStartupSafetyDelayMs = 100U;

constexpr std::uint32_t kCanNotifications =
    CAN_IT_RX_FIFO0_MSG_PENDING
    | CAN_IT_TX_MAILBOX_EMPTY
    | CAN_IT_ERROR
    | CAN_IT_BUSOFF
    | CAN_IT_LAST_ERROR_CODE
    | CAN_IT_ERROR_WARNING
    | CAN_IT_ERROR_PASSIVE
    | CAN_IT_RX_FIFO0_FULL
    | CAN_IT_RX_FIFO0_OVERRUN;

QD4310 motor(&hcan1, kMotorId);

std::atomic_uint32_t canErrorCount{0U};
std::atomic_uint32_t canAbortCount{0U};
std::atomic_uint32_t canRxOverrunCount{0U};
std::atomic_uint32_t canRxFullCount{0U};
std::atomic_uint32_t canTxCompleteCount{0U};

static_assert(std::atomic_uint32_t::is_always_lock_free);

#if !defined(HBALL_PIPE_SWEEP_TEST)
BallControlRuntimeConfig makeControllerConfig()
{
#if defined(HBALL_PIPE_ANGLE_TEST)
    return BallControlRuntimeConfig{
        ball_control_config::kPipeTestMechanicalLevelAngleRad,
        ball_control_config::kPipeTestAngleSign,
        ball_control_config::kPipeTestMaxAngleOffsetDeg,
        ball_control_config::kPipeTestMechanicalAngleLimitDeg,
        ball_control_config::kPipeTestMaxMotorSpeedRpm,
        ball_control_config::kPipeTestMaxMotorCurrentA,
    };
#else
    BallControlRuntimeConfig config{
        ball_control_config::kMechanicalLevelAngleRad,
        ball_control_config::kAngleSign,
        ball_control_config::kMaxAngleOffsetDeg,
        ball_control_config::kMechanicalAngleLimitDeg,
        ball_control_config::kMaxMotorSpeedRpm,
        ball_control_config::kMaxMotorCurrentA,
    };
#if defined(HBALL_SIM_VISION_MOTOR_TEST) \
    || defined(HBALL_IMU_FEEDFORWARD_TEST)
    config.mechanicalLevelAngleRad =
        ball_control_config::kInstalledLevelCommandAngleRad;
    config.maxAngleOffsetDeg =
        ball_control_config::kSimMaxAngleOffsetDeg;
    config.mechanicalAngleLimitDeg =
        ball_control_config::kSimMechanicalAngleLimitDeg;
    config.maxMotorSpeedRpm =
        ball_control_config::kSimMaxMotorSpeedRpm;
    config.maxMotorCurrentA =
        ball_control_config::kSimMaxMotorCurrentA;
#endif
    return config;
#endif
}
#endif

#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST)
bool installedAngleIsAbsolutelySafe(const float angleRad)
{
    return std::isfinite(angleRad)
           && angleRad
                  >= ball_control_config::kInstalledPhysicalLowerAngleRad
           && angleRad
                  <= ball_control_config::kInstalledPhysicalUpperAngleRad;
}

float clampInstalledTargetAngle(const float angleRad)
{
    return std::clamp(
        angleRad,
        ball_control_config::kInstalledCommandLowerAngleRad,
        ball_control_config::kInstalledCommandUpperAngleRad);
}

class InstalledCommandSlew {
public:
    void trackActual(const bool angleSafe,
                     const float actualAngleRad,
                     const std::uint32_t nowMs)
    {
        if (!angleSafe) {
            initialized_ = false;
            return;
        }
        commandedAngleRad_ = actualAngleRad;
        lastUpdateMs_ = nowMs;
        initialized_ = true;
    }

    [[nodiscard]] float update(const bool angleSafe,
                               const float actualAngleRad,
                               const float motorSpeedRpm,
                               const float desiredAngleRad,
                               const std::uint32_t nowMs)
    {
        if (!angleSafe) {
            initialized_ = false;
            return clampInstalledTargetAngle(desiredAngleRad);
        }
        if (!initialized_) {
            trackActual(true, actualAngleRad, nowMs);
            return commandedAngleRad_;
        }
        if (std::fabs(motorSpeedRpm)
            > ball_control_config::kInstalledTrackingPauseSpeedRpm) {
            trackActual(true, actualAngleRad, nowMs);
            return commandedAngleRad_;
        }

        const std::uint32_t elapsedMs = std::min(
            nowMs - lastUpdateMs_,
            ball_control_config::kMaxSlewElapsedMs);
        lastUpdateMs_ = nowMs;
        const float maxStepRad =
            ball_control_config::kInstalledCommandSlewRateDegPerSecond
            * std::numbers::pi_v<float> / 180.0F
            * static_cast<float>(elapsedMs) / 1000.0F;
        const float destination =
            clampInstalledTargetAngle(desiredAngleRad);
        const float remaining = destination - commandedAngleRad_;
        commandedAngleRad_ += std::clamp(
            remaining, -maxStepRad, maxStepRad);
        const float maxFollowingErrorRad =
            ball_control_config::kInstalledMaxFollowingErrorDeg
            * std::numbers::pi_v<float> / 180.0F;
        commandedAngleRad_ = std::clamp(
            commandedAngleRad_,
            actualAngleRad - maxFollowingErrorRad,
            actualAngleRad + maxFollowingErrorRad);
        commandedAngleRad_ = std::clamp(
            commandedAngleRad_,
            ball_control_config::kInstalledPhysicalLowerAngleRad,
            ball_control_config::kInstalledPhysicalUpperAngleRad);
        return commandedAngleRad_;
    }

private:
    bool initialized_{};
    std::uint32_t lastUpdateMs_{};
    float commandedAngleRad_{};
};

#endif

#if defined(HBALL_PIPE_SWEEP_TEST)
enum class PipeSweepPhase : std::uint8_t {
    WaitMotor,
    EnablePending,
    MoveCenter,
    HoldCenter,
    Fault,
};

class PipeSweepSequence {
public:
    BallControlOutput update(const BallControlInput& input)
    {
        if (phase_ == PipeSweepPhase::Fault) {
            return makeOutput(input);
        }
        if (input.keyEmergencyStop) {
            return enterFault(input, BallControlFault::EmergencyStop);
        }
        if (!txCounterInitialized_) {
            txCounterInitialized_ = true;
            lastTxErrorCount_ = input.txErrorCount;
        } else if (input.txErrorCount != lastTxErrorCount_) {
            return enterFault(input, BallControlFault::CanTransmit);
        }

        if (phase_ == PipeSweepPhase::WaitMotor) {
            updateFeedbackGate(input);
            if (input.motorFeedbackFresh
                && input.motorFeedbackFinite
                && !angleIsSafe(input.motorAngleRad)) {
                return enterFault(
                    input, BallControlFault::MechanicalAngleLimit);
            }
            if (consecutiveFeedback_
                >= ball_control_config::kMotorFeedbackRequiredFrames) {
                phase_ = PipeSweepPhase::EnablePending;
                phaseStartedMs_ = input.nowMs;
                lastEnableCommandMs_ =
                    input.nowMs
                    - ball_control_config::kEnableRetryPeriodMs;
                commandedAngleRad_ = input.motorAngleRad;
                lastUpdateMs_ = input.nowMs;
                updateTimeInitialized_ = true;
            }
        }

        if (phase_ == PipeSweepPhase::EnablePending) {
            if (!input.motorFeedbackFresh) {
                return enterFault(
                    input, BallControlFault::MotorFeedbackTimeout);
            }
            if (!input.motorFeedbackFinite) {
                return enterFault(
                    input, BallControlFault::InvalidMotorFeedback);
            }
            if (!angleIsSafe(input.motorAngleRad)) {
                return enterFault(
                    input, BallControlFault::MechanicalAngleLimit);
            }
            if ((input.nowMs - phaseStartedMs_)
                > ball_control_config::kEnableTimeoutMs) {
                return enterFault(
                    input, BallControlFault::MotorEnableTimeout);
            }

            updateEnableGate(input);
            if (enableConfirmations_
                >= ball_control_config::kMotorEnableConfirmFrames) {
                phase_ = PipeSweepPhase::MoveCenter;
                commandedAngleRad_ = input.motorAngleRad;
                lastUpdateMs_ = input.nowMs;
            }
        } else if (phaseIsActive()) {
            if (const BallControlFault safetyFault =
                    activeSafetyFault(input);
                safetyFault != BallControlFault::None) {
                return enterFault(input, safetyFault);
            }
            updateCommand(input);
        }

        return makeOutput(input);
    }

    [[nodiscard]] std::uint8_t phaseValue() const
    {
        return static_cast<std::uint8_t>(phase_);
    }

private:
    static bool angleIsSafe(const float angleRad)
    {
        return angleRad
                   >= ball_control_config::kPipeSweepSafetyLowerAngleRad
               && angleRad
                   <= ball_control_config::kPipeSweepSafetyUpperAngleRad;
    }

    [[nodiscard]] bool phaseIsActive() const
    {
        return phase_ >= PipeSweepPhase::MoveCenter
               && phase_ <= PipeSweepPhase::HoldCenter;
    }

    void updateFeedbackGate(const BallControlInput& input)
    {
        if (!input.motorFeedbackFresh || !input.motorFeedbackFinite) {
            feedbackCounterInitialized_ = false;
            consecutiveFeedback_ = 0U;
            return;
        }
        if (!feedbackCounterInitialized_) {
            feedbackCounterInitialized_ = true;
            lastFeedbackCount_ = input.motorFeedbackCount;
            consecutiveFeedback_ =
                input.motorFeedbackCount == 0U ? 0U : 1U;
        } else if (input.motorFeedbackCount != lastFeedbackCount_) {
            lastFeedbackCount_ = input.motorFeedbackCount;
            if (consecutiveFeedback_
                < ball_control_config::kMotorFeedbackRequiredFrames) {
                ++consecutiveFeedback_;
            }
        }
    }

    void updateEnableGate(const BallControlInput& input)
    {
        if (input.motorFeedbackCount == lastEnableFeedbackCount_) {
            return;
        }
        lastEnableFeedbackCount_ = input.motorFeedbackCount;
        if (input.motorEnabled) {
            if (enableConfirmations_
                < ball_control_config::kMotorEnableConfirmFrames) {
                ++enableConfirmations_;
            }
        } else {
            enableConfirmations_ = 0U;
        }
    }

    BallControlFault activeSafetyFault(const BallControlInput& input)
    {
        if (!input.motorFeedbackFresh) {
            return BallControlFault::MotorFeedbackTimeout;
        }
        if (!input.motorFeedbackFinite) {
            return BallControlFault::InvalidMotorFeedback;
        }
        if (!input.motorEnabled) {
            return BallControlFault::MotorUnexpectedlyDisabled;
        }
        if (!angleIsSafe(input.motorAngleRad)) {
            return BallControlFault::MechanicalAngleLimit;
        }

        if (input.motorFeedbackCount != lastLimitFeedbackCount_) {
            lastLimitFeedbackCount_ = input.motorFeedbackCount;
            consecutiveOverspeed_ =
                std::fabs(input.motorSpeedRpm)
                    > ball_control_config::kPipeSweepMaxMotorSpeedRpm
                ? consecutiveOverspeed_ + 1U
                : 0U;
            consecutiveOvercurrent_ =
                std::fabs(input.motorCurrentA)
                    > ball_control_config::kPipeSweepMaxMotorCurrentA
                ? consecutiveOvercurrent_ + 1U
                : 0U;
        }
        if (consecutiveOverspeed_
            >= ball_control_config::kMotorLimitViolationFrames) {
            return BallControlFault::MotorOverspeed;
        }
        if (consecutiveOvercurrent_
            >= ball_control_config::kMotorLimitViolationFrames) {
            return BallControlFault::MotorOvercurrent;
        }
        if (updateTimeInitialized_
            && (input.nowMs - lastUpdateMs_)
                   > ball_control_config::kSevereControlLapseMs) {
            return BallControlFault::ControlLoopTiming;
        }
        return BallControlFault::None;
    }

    void updateCommand(const BallControlInput& input)
    {
        const std::uint32_t elapsedMs =
            updateTimeInitialized_
            ? input.nowMs - lastUpdateMs_
            : ball_control_config::kControlPeriodMs;
        lastUpdateMs_ = input.nowMs;
        updateTimeInitialized_ = true;
        const std::uint32_t limitedElapsedMs =
            elapsedMs > ball_control_config::kMaxSlewElapsedMs
            ? ball_control_config::kMaxSlewElapsedMs
            : elapsedMs;
        const float slewRateRadPerSecond =
            ball_control_config::kPipeSweepSlewRateDegPerSecond
            * std::numbers::pi_v<float> / 180.0F;
        const float maxStepRad =
            slewRateRadPerSecond
            * static_cast<float>(limitedElapsedMs) / 1000.0F;
        const float destination =
            ball_control_config::kPipeSweepCenterAngleRad;
        const float remaining = destination - commandedAngleRad_;
        if (remaining > maxStepRad) {
            commandedAngleRad_ += maxStepRad;
        } else if (remaining < -maxStepRad) {
            commandedAngleRad_ -= maxStepRad;
        } else {
            commandedAngleRad_ = destination;
        }

        const bool arrived =
            std::fabs(input.motorAngleRad - destination)
                <= ball_control_config::kPipeSweepArrivalToleranceRad
            && std::fabs(input.motorSpeedRpm)
                <= ball_control_config::kPipeSweepArrivalMaxSpeedRpm;
        if (phase_ == PipeSweepPhase::MoveCenter
            && arrived
            && commandedAngleRad_ == destination) {
            phase_ = PipeSweepPhase::HoldCenter;
        }
    }

    BallControlOutput enterFault(const BallControlInput& input,
                                 const BallControlFault fault)
    {
        phase_ = PipeSweepPhase::Fault;
        fault_ = fault;
        shutdownCommandInitialized_ = false;
        return makeOutput(input);
    }

    BallControlOutput makeOutput(const BallControlInput& input) const
    {
        BallControlOutput output{};
        output.fault = fault_;
        output.mechanicalLevelAngleRad =
            ball_control_config::kPipeSweepCenterAngleRad;
        output.actualAngleRad = input.motorAngleRad;
        output.targetAngleRad = commandedAngleRad_;
        output.angleOffsetDeg =
            (commandedAngleRad_
             - ball_control_config::kPipeSweepCenterAngleRad)
            * 180.0F / std::numbers::pi_v<float>;
        output.motorSpeedRpm = input.motorSpeedRpm;
        output.motorCurrentA = input.motorCurrentA;
        output.motorEnableConfirmations = enableConfirmations_;

        if (phase_ == PipeSweepPhase::WaitMotor) {
            output.state = BallControlState::WaitMotor;
            applyRateLimitedSafeShutdown(input, output);
        } else if (phase_ == PipeSweepPhase::EnablePending) {
            output.state = BallControlState::EnablePending;
            output.sendAngleCommand = true;
            if (!input.motorEnabled
                && (input.nowMs - lastEnableCommandMs_)
                       >= ball_control_config::kEnableRetryPeriodMs) {
                output.enableMotor = true;
                lastEnableCommandMs_ = input.nowMs;
            }
        } else if (phase_ == PipeSweepPhase::Fault) {
            output.state = BallControlState::Fault;
            applyRateLimitedSafeShutdown(input, output);
        } else {
            output.state = BallControlState::LevelHold;
            output.sendAngleCommand = true;
        }
        return output;
    }

    void applyRateLimitedSafeShutdown(
        const BallControlInput& input,
        BallControlOutput& output) const
    {
        const bool retryDue =
            !shutdownCommandInitialized_
            || (input.nowMs - lastShutdownCommandMs_)
                   >= ball_control_config::kSafeShutdownRetryMs;
        if (retryDue) {
            output.zeroOutput = true;
            output.disableMotor = true;
            shutdownCommandInitialized_ = true;
            lastShutdownCommandMs_ = input.nowMs;
        }
    }

    PipeSweepPhase phase_{PipeSweepPhase::WaitMotor};
    BallControlFault fault_{BallControlFault::None};
    float commandedAngleRad_{ball_control_config::kPipeSweepCenterAngleRad};
    bool feedbackCounterInitialized_{};
    std::uint32_t lastFeedbackCount_{};
    std::uint32_t consecutiveFeedback_{};
    bool txCounterInitialized_{};
    std::uint32_t lastTxErrorCount_{};
    std::uint32_t lastEnableFeedbackCount_{};
    mutable std::uint32_t lastEnableCommandMs_{};
    std::uint32_t enableConfirmations_{};
    std::uint32_t lastLimitFeedbackCount_{};
    std::uint32_t consecutiveOverspeed_{};
    std::uint32_t consecutiveOvercurrent_{};
    bool updateTimeInitialized_{};
    std::uint32_t lastUpdateMs_{};
    std::uint32_t phaseStartedMs_{};
    mutable bool shutdownCommandInitialized_{};
    mutable std::uint32_t lastShutdownCommandMs_{};
};
#endif

[[noreturn]] void failCanInitialization(const bool canStarted)
{
    if (canStarted) {
        motor.setSpeed(0.0F);
        motor.setCurrent(0.0F);
        motor.disable();
    }
    BoardIo_SetLed(BoardLedStatus::Fault, HAL_GetTick());
    Error_Handler();
    for (;;) {
    }
}

void initializeCan()
{
    CAN_FilterTypeDef filter{};
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh =
        static_cast<std::uint16_t>(kFeedbackStandardId << 5U);
    filter.FilterIdLow = 0U;
    filter.FilterMaskIdHigh = 0xFFE0U;
    filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14U;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        failCanInitialization(false);
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        failCanInitialization(false);
    }
    if (HAL_CAN_ActivateNotification(
            &hcan1, kCanNotifications) != HAL_OK) {
        failCanInitialization(true);
    }
}

void recordTxComplete(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        canTxCompleteCount.fetch_add(1U, std::memory_order_relaxed);
    }
}

void recordTxAbort(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        canAbortCount.fetch_add(1U, std::memory_order_relaxed);
    }
}

BallVisionSnapshot copyVisionSnapshot(const VisionLinkSnapshot& source)
{
    BallVisionSnapshot result{};
    result.pi_session_id = source.pi_session_id;
    result.capture_timestamp_ms = source.capture_timestamp_ms;
    result.local_receive_ms = source.local_receive_ms;
    result.measurement_age_ms = source.measurement_age_ms;
    result.position_0p1mm = source.position_0p1mm;
    result.velocity_mmps = source.velocity_mmps;
    result.target_position_0p1mm = source.target_position_0p1mm;
    result.measurement_update_count = source.measurement_update_count;
    result.confidence = source.confidence;
    result.vision_flags = source.flags;
    result.control_flags = source.control_flags;
    result.task_id = source.task_id;
    result.run_id = source.run_id;
    result.sequence = source.sequence;
    result.valid = source.valid != 0U;
    result.velocity_valid = source.velocity_valid != 0U;
    return result;
}

BoardLedStatus ledStatusFor(const BallControlState state)
{
    switch (state) {
    case BallControlState::LevelHold:
        return BoardLedStatus::Ready;
    case BallControlState::Balancing:
        return BoardLedStatus::Running;
    case BallControlState::BootSafe:
    case BallControlState::WaitMotor:
    case BallControlState::EnablePending:
    case BallControlState::VisionRecovery:
        return BoardLedStatus::WaitingForFeedback;
    case BallControlState::ConfigRequired:
    case BallControlState::Fault:
    default:
        return BoardLedStatus::Fault;
    }
}

void publishDebug(const BallControlInput& input,
                  const BallControlOutput& output)
{
    ball_control_debug_state = static_cast<std::uint8_t>(output.state);
    ball_control_debug_fault = static_cast<std::uint8_t>(output.fault);
    ball_control_debug_mechanical_level_rad =
        output.mechanicalLevelAngleRad;
    ball_control_debug_actual_angle_rad = output.actualAngleRad;
    ball_control_debug_target_angle_rad = output.targetAngleRad;
    ball_control_debug_angle_offset_deg = output.angleOffsetDeg;
    ball_control_debug_target_position_0p1mm =
        output.targetPosition0p1mm;
    ball_control_debug_position_0p1mm = output.position0p1mm;
    ball_control_debug_velocity_mmps = output.velocityMmps;
    ball_control_debug_valid_measurements =
        output.consecutiveValidMeasurements;
    ball_control_debug_measurement_update_count =
        output.measurementUpdateCount;
    ball_control_debug_stiction_state =
        static_cast<std::uint8_t>(output.stictionState);
    ball_control_debug_stuck_elapsed_ms = output.stuckElapsedMs;
    ball_control_debug_active_angle_limit_deg =
        output.activeAngleLimitDeg;
    ball_control_debug_center_hold_active =
        output.centerHoldActive ? 1U : 0U;
    ball_control_debug_motor_feedback_count = input.motorFeedbackCount;
    ball_control_debug_motor_enabled = input.motorEnabled ? 1U : 0U;
    ball_control_debug_motor_speed_rpm = output.motorSpeedRpm;
    ball_control_debug_motor_current_a = output.motorCurrentA;
    const float absoluteMotorSpeedRpm = std::fabs(input.motorSpeedRpm);
    if (absoluteMotorSpeedRpm
        > ball_control_debug_peak_abs_motor_speed_rpm) {
        ball_control_debug_peak_abs_motor_speed_rpm =
            absoluteMotorSpeedRpm;
    }
    if (output.fault != BallControlFault::None
        && ball_control_debug_fault_snapshot_valid == 0U) {
        ball_control_debug_fault_snapshot_valid = 1U;
        ball_control_debug_fault_motor_speed_rpm =
            input.motorSpeedRpm;
        ball_control_debug_fault_motor_current_a =
            input.motorCurrentA;
        ball_control_debug_fault_actual_angle_rad =
            input.motorAngleRad;
        ball_control_debug_fault_target_angle_rad =
            output.targetAngleRad;
        ball_control_debug_fault_last_sent_angle_rad =
            ball_control_debug_last_sent_angle_rad;
    }
    ball_control_debug_enable_confirmations =
        output.motorEnableConfirmations;
    ball_control_debug_control_elapsed_ms = output.controlElapsedMs;
    ball_control_debug_tx_error_count = input.txErrorCount;
    ball_control_debug_task_id = output.taskId;
    ball_control_debug_run_id = output.runId;
}

void publishCompetitionTaskDebug(const CompetitionTaskOutput& taskContext)
{
    competition_task_debug_state =
        static_cast<std::uint8_t>(taskContext.state);
    competition_task_debug_error =
        static_cast<std::uint8_t>(taskContext.error);
    competition_task_debug_context_valid = taskContext.contextValid ? 1U : 0U;
    competition_task_debug_task_id = taskContext.taskId;
    competition_task_debug_run_id = taskContext.runId;
    competition_task_debug_target_0p1mm = taskContext.targetPosition0p1mm;
    competition_task_debug_raw_control_flags = taskContext.rawControlFlags;
    competition_task_debug_effective_control_flags =
        taskContext.effectiveControlFlags;
    competition_task_debug_reject_count = taskContext.rejectCount;
}

void publishTask3FinalDebug(const VisionLinkSnapshot& vision,
                            const CompetitionTaskOutput& taskContext,
                            const BallControlInput& input,
                            const BallControlOutput& output)
{
    static bool identityInitialized = false;
    static std::uint32_t activeSessionId = 0U;
    static std::uint16_t activeRunId = 0U;
    static bool terminalCaptured = false;

    if (!taskContext.contextValid || taskContext.taskId != 3U) {
        return;
    }

    if (!identityInitialized
        || vision.pi_session_id != activeSessionId
        || taskContext.runId != activeRunId) {
        identityInitialized = true;
        activeSessionId = vision.pi_session_id;
        activeRunId = taskContext.runId;
        terminalCaptured = false;
        task3_final_debug_valid = 0U;
        task3_trace_valid_mask = 0U;
        task3_trace_pi_session_id = vision.pi_session_id;
        task3_trace_run_id = taskContext.runId;
    }

    const bool terminal = taskContext.state == CompetitionTaskState::Done
        || taskContext.state == CompetitionTaskState::Timeout;

    std::uint8_t traceIndex = 0xFFU;
    if (taskContext.rawControlFlags == 0x01U
        && taskContext.targetPosition0p1mm == 0) {
        traceIndex = 0U;
    } else if (taskContext.rawControlFlags == 0x03U
               && taskContext.targetPosition0p1mm == 500) {
        traceIndex = 1U;
    } else if (taskContext.rawControlFlags == 0x03U
               && taskContext.targetPosition0p1mm == -500) {
        traceIndex = 2U;
    } else if (terminal) {
        traceIndex = 3U;
    }

    if (traceIndex < 4U) {
        const std::uint8_t traceBit =
            static_cast<std::uint8_t>(1U << traceIndex);
        if ((task3_trace_valid_mask & traceBit) == 0U) {
            task3_trace_sequence[traceIndex] = vision.sequence;
            task3_trace_target_0p1mm[traceIndex] =
                vision.target_position_0p1mm;
            task3_trace_control_flags[traceIndex] = vision.control_flags;
            task3_trace_x_0p1mm[traceIndex] = vision.position_0p1mm;
            task3_trace_v_mmps[traceIndex] = vision.velocity_mmps;
            task3_trace_measurement_update_count[traceIndex] =
                vision.measurement_update_count;
            task3_trace_measurement_age_ms[traceIndex] =
                vision.measurement_age_ms;
            task3_trace_accepted_frame_count[traceIndex] =
                vision_debug_accepted_frame_count;
            task3_trace_guard_state[traceIndex] =
                static_cast<std::uint8_t>(taskContext.state);
            task3_trace_guard_error[traceIndex] =
                static_cast<std::uint8_t>(taskContext.error);
            task3_trace_effective_control_flags[traceIndex] =
                taskContext.effectiveControlFlags;
            task3_trace_reject_count[traceIndex] = taskContext.rejectCount;
            task3_trace_ball_state[traceIndex] =
                static_cast<std::uint8_t>(output.state);
            task3_trace_ball_fault[traceIndex] =
                static_cast<std::uint8_t>(output.fault);
            task3_trace_ball_position_0p1mm[traceIndex] =
                output.position0p1mm;
            task3_trace_ball_velocity_mmps[traceIndex] =
                output.velocityMmps;
            task3_trace_center_hold_active[traceIndex] =
                output.centerHoldActive ? 1U : 0U;
            task3_trace_angle_offset_deg[traceIndex] = output.angleOffsetDeg;
            task3_trace_motor_enabled[traceIndex] =
                input.motorEnabled ? 1U : 0U;
            task3_trace_valid_mask = static_cast<std::uint8_t>(
                task3_trace_valid_mask | traceBit);
        }
    }

    if (!terminal || terminalCaptured) {
        return;
    }

    task3_final_debug_valid = 0U;
    task3_final_debug_pi_session_id = vision.pi_session_id;
    task3_final_debug_task_id = taskContext.taskId;
    task3_final_debug_run_id = taskContext.runId;
    task3_final_debug_terminal_state =
        static_cast<std::uint8_t>(taskContext.state);
    task3_final_debug_vision_status = vision_debug_status;
    task3_final_debug_target_0p1mm = vision.target_position_0p1mm;
    task3_final_debug_raw_control_flags = vision.control_flags;
    task3_final_debug_x_0p1mm = vision.position_0p1mm;
    task3_final_debug_v_mmps = vision.velocity_mmps;
    task3_final_debug_measurement_update_count =
        vision.measurement_update_count;
    task3_final_debug_measurement_age_ms = vision.measurement_age_ms;
    task3_final_debug_accepted_frame_count =
        vision_debug_accepted_frame_count;
    task3_final_debug_crc_error_count = vision_debug_crc_error_count;
    task3_final_debug_uart_error_count = vision_debug_uart_error_count;
    task3_final_debug_guard_state =
        static_cast<std::uint8_t>(taskContext.state);
    task3_final_debug_guard_error =
        static_cast<std::uint8_t>(taskContext.error);
    task3_final_debug_context_valid = taskContext.contextValid ? 1U : 0U;
    task3_final_debug_effective_control_flags =
        taskContext.effectiveControlFlags;
    task3_final_debug_reject_count = taskContext.rejectCount;
    task3_final_debug_ball_state =
        static_cast<std::uint8_t>(output.state);
    task3_final_debug_ball_fault =
        static_cast<std::uint8_t>(output.fault);
    task3_final_debug_ball_target_0p1mm = output.targetPosition0p1mm;
    task3_final_debug_ball_position_0p1mm = output.position0p1mm;
    task3_final_debug_ball_velocity_mmps = output.velocityMmps;
    task3_final_debug_ball_measurement_update_count =
        output.measurementUpdateCount;
    task3_final_debug_center_hold_active =
        output.centerHoldActive ? 1U : 0U;
    task3_final_debug_angle_offset_deg = output.angleOffsetDeg;
    task3_final_debug_motor_enabled = input.motorEnabled ? 1U : 0U;
    task3_final_debug_capture_count += 1U;
    task3_final_debug_valid = 1U;
    terminalCaptured = true;
}

} // namespace

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) {
        return;
    }

    for (std::uint32_t frameIndex = 0U; frameIndex < 3U; ++frameIndex) {
        if (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) == 0U) {
            break;
        }

        CAN_RxHeaderTypeDef header{};
        std::uint8_t data[8]{};
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data)
            != HAL_OK) {
            canErrorCount.fetch_add(1U, std::memory_order_relaxed);
            break;
        }

        if (header.IDE == CAN_ID_STD
            && header.RTR == CAN_RTR_DATA
            && header.StdId == kFeedbackStandardId
            && header.DLC == 8U) {
            motor.update(data);
        }
    }
}

extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
    recordTxComplete(hcan);
}

extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
    recordTxComplete(hcan);
}

extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
    recordTxComplete(hcan);
}

extern "C" void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *hcan)
{
    recordTxAbort(hcan);
}

extern "C" void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *hcan)
{
    recordTxAbort(hcan);
}

extern "C" void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *hcan)
{
    recordTxAbort(hcan);
}

extern "C" void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == &hcan1) {
        canRxFullCount.fetch_add(1U, std::memory_order_relaxed);
    }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan != &hcan1) {
        return;
    }

    const std::uint32_t error = HAL_CAN_GetError(hcan);
    canErrorCount.fetch_add(1U, std::memory_order_relaxed);
    if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U) {
        canRxOverrunCount.fetch_add(1U, std::memory_order_relaxed);
    }
}

extern "C" void BallControlTask_Entry(void *argument)
{
    (void)argument;

    initializeCan();

    motor.setSpeed(0.0F);
    motor.setCurrent(0.0F);
    motor.disable();
    vTaskDelay(pdMS_TO_TICKS(kStartupSafetyDelayMs));
    motor.setSpeed(0.0F);
    motor.setCurrent(0.0F);
    motor.disable();

#if defined(HBALL_PIPE_SWEEP_TEST)
    PipeSweepSequence pipeSweepSequence;
#else
    BallBalanceController controller(makeControllerConfig());
#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST)
    InstalledCommandSlew installedCommandSlew;
#endif
#if defined(HBALL_IMU_FEEDFORWARD_TEST)
    const ImuFeedforwardController imuFeedforwardController(
        ImuFeedforwardConfig{
            ball_control_config::kInstalledLevelCommandAngleRad,
            ball_control_config::kImuFeedforwardGravityMps2,
            ball_control_config::kImuFeedforwardGain,
            ball_control_config::kImuFeedforwardMaxAngleOffsetDeg,
            ball_control_config::kInstalledCommandLowerAngleRad,
            ball_control_config::kInstalledCommandUpperAngleRad,
        });
    ImuFeedforwardPeakHold imuFeedforwardPeakHold(
        ball_control_config::kInstalledLevelCommandAngleRad);
    bool imuCommissioningReadyLatched = false;
#endif
#endif
    CompetitionTaskGuard competitionTaskGuard;
#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST) \
    && !defined(HBALL_IMU_FEEDFORWARD_TEST)
    CompetitionImuFeedforwardController competitionImuFeedforward(
        CompetitionImuFeedforwardConfig{
            .enabled =
                ball_control_config::kCompetitionImuFeedforwardEnabled,
            .staleTimeoutMs =
                ball_control_config::kImuFeedforwardStaleTimeoutMs,
            .feedforward = ImuFeedforwardConfig{
                ball_control_config::kInstalledLevelCommandAngleRad,
                ball_control_config::kImuFeedforwardGravityMps2,
                ball_control_config::kImuFeedforwardGain,
                ball_control_config::kImuFeedforwardMaxAngleOffsetDeg,
                ball_control_config::kInstalledCommandLowerAngleRad,
                ball_control_config::kInstalledCommandUpperAngleRad,
            },
            .task4StartAccelerationGain =
                ball_control_config::kTask4StartFeedforwardGain,
            .task4BrakeAccelerationGain =
                ball_control_config::kTask4BrakeFeedforwardGain,
            .task4MaxOffsetDeg =
                ball_control_config::kTask4MaxFeedforwardOffsetDeg,
            .task4BrakeDetectAccelerationMps2 =
                ball_control_config::
                    kTask4BrakeDetectAccelerationMps2,
            .task4StopDetectAccelerationMps2 =
                ball_control_config::
                    kTask4StopDetectAccelerationMps2,
            .task4PostStopOffsetDeg =
                ball_control_config::kTask4PostStopOffsetDeg,
            .task4PostStopHoldMs =
                ball_control_config::kTask4PostStopHoldMs,
            .task4PostStopReleaseMs =
                ball_control_config::kTask4PostStopReleaseMs,
            .launch = VehicleLaunchFeedforwardConfig{
                .enabled = ball_control_config::kVehicleLaunchFeedforwardEnabled,
                .angleOffsetDeg = ball_control_config::kVehicleLaunchAngleOffsetDeg,
                .accelerationThresholdMps2 =
                    ball_control_config::kVehicleLaunchAccelerationThresholdMps2,
                .requiredNewSamples =
                    ball_control_config::kVehicleLaunchRequiredNewSamples,
                .holdMs = ball_control_config::kVehicleLaunchHoldMs,
                .releaseMs = ball_control_config::kVehicleLaunchReleaseMs,
                .maxTotalOffsetDeg =
                    ball_control_config::kVehicleLaunchMaxTotalOffsetDeg,
            },
        });
    ImuVehicleControlSnapshot lastImuControlSnapshot{};
    VehicleLaunchTraceRecorder launchTraceRecorder(vehicle_launch_trace);
#endif
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t controlPeriod =
        pdMS_TO_TICKS(ball_control_config::kControlPeriodMs);

    for (;;) {
        vTaskDelayUntil(&lastWakeTime, controlPeriod);
        BoardIo_UpdateKey();

        // Exactly one coherent snapshot from each producer per cycle.
        const QD4310::Snapshot motorSnapshot = motor.snapshot();
        VisionLinkSnapshot visionSnapshot{};
        VisionLink_GetSnapshot(&visionSnapshot);
#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST) \
    && !defined(HBALL_IMU_FEEDFORWARD_TEST)
        ImuVehicleControlSnapshot candidateImuSnapshot{};
        if (ImuVehicle_TryGetControlSnapshot(candidateImuSnapshot)) {
            lastImuControlSnapshot = candidateImuSnapshot;
        }
#endif

        BallControlInput input{};
        input.nowMs = HAL_GetTick();
        input.keyEmergencyStop = BoardIo_KeyPressed();
        input.motorFeedbackFresh =
            motorSnapshot.hasFeedback
            && (input.nowMs - motorSnapshot.lastFeedbackMs)
                   <= kFeedbackFreshnessMs;
        input.motorFeedbackFinite =
            std::isfinite(motorSnapshot.angleRad)
            && std::isfinite(motorSnapshot.speedRpm)
            && std::isfinite(motorSnapshot.currentA);
        input.motorEnabled = motorSnapshot.enabled;
        input.motorFeedbackCount = motorSnapshot.feedbackCount;
        const std::uint32_t canDiagnosticErrors =
            canErrorCount.load(std::memory_order_relaxed)
            + canAbortCount.load(std::memory_order_relaxed)
            + canRxOverrunCount.load(std::memory_order_relaxed)
            + canRxFullCount.load(std::memory_order_relaxed);
        input.txErrorCount =
            motor.txErrorCount() + canDiagnosticErrors;
        input.motorAngleRad = motorSnapshot.angleRad;
        input.motorSpeedRpm = motorSnapshot.speedRpm;
        input.motorCurrentA = motorSnapshot.currentA;
        input.vision = copyVisionSnapshot(visionSnapshot);
        const CompetitionTaskOutput taskContext = competitionTaskGuard.update(
            CompetitionTaskInput{
                .piSessionId = visionSnapshot.pi_session_id,
                .runId = visionSnapshot.run_id,
                .targetPosition0p1mm = visionSnapshot.target_position_0p1mm,
                .taskId = visionSnapshot.task_id,
                .controlFlags = visionSnapshot.control_flags,
                .sequence = visionSnapshot.sequence,
            });
        input.vision.control_flags = taskContext.effectiveControlFlags;
        publishCompetitionTaskDebug(taskContext);
#if defined(HBALL_IMU_FEEDFORWARD_TEST)
        // This mode is deliberately independent of Raspberry Pi/vision input.
        // Clearing the snapshot keeps the generic controller in LevelHold;
        // only the BMI088 feedforward target below can move the pipe.
        input.vision = BallVisionSnapshot{};
#endif

#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST)
        const bool installedAbsoluteAngleSafe =
            input.motorFeedbackFresh
            && input.motorFeedbackFinite
            && installedAngleIsAbsolutelySafe(input.motorAngleRad);
        ball_control_debug_sim_absolute_angle_safe =
            installedAbsoluteAngleSafe ? 1U : 0U;
        ball_control_debug_sim_level_captured =
            installedAbsoluteAngleSafe ? 1U : 0U;
        ball_control_debug_sim_level_stable_frames = 0U;
#if defined(HBALL_IMU_FEEDFORWARD_TEST)
        const bool imuReady =
            imu_vehicle_status == 1U
            && imu_vehicle_calibrated != 0U
            && imu_vehicle_valid != 0U
            && std::isfinite(imu_vehicle_forward_filtered_mps2);
        if (!imuCommissioningReadyLatched
            && installedAbsoluteAngleSafe && imuReady) {
            imuCommissioningReadyLatched = true;
        }
        imu_feedforward_debug_ready = imuReady ? 1U : 0U;
        input.commissioned =
            installedAbsoluteAngleSafe
            && imuCommissioningReadyLatched;
#elif defined(HBALL_SIM_VISION_MOTOR_TEST)
        input.commissioned = installedAbsoluteAngleSafe;
#else
        input.commissioned =
            ball_control_config::kBallControlCommissioned
            && installedAbsoluteAngleSafe;
#endif
#elif defined(HBALL_PIPE_ANGLE_TEST)
        input.commissioned = true;
#else
        input.commissioned = true;
#endif

#if defined(HBALL_PIPE_SWEEP_TEST)
        BallControlOutput output = pipeSweepSequence.update(input);
        ball_control_debug_pipe_sweep_phase =
            pipeSweepSequence.phaseValue();
#else
        float runtimeKp = ball_control_tune_kp_deg_per_mm;
        float runtimeKd = ball_control_tune_kd_deg_per_mmps;
        if (!std::isfinite(runtimeKp)) {
            runtimeKp = ball_control_config::kKpDegPerMm;
        }
        if (!std::isfinite(runtimeKd)) {
            runtimeKd = ball_control_config::kKdDegPerMmps;
        }
        runtimeKp = std::clamp(
            runtimeKp,
            ball_control_config::kMinimumTunableKpDegPerMm,
            ball_control_config::kMaximumTunableKpDegPerMm);
        runtimeKd = std::clamp(
            runtimeKd,
            ball_control_config::kMinimumTunableKdDegPerMmps,
            ball_control_config::kMaximumTunableKdDegPerMmps);
        ball_control_tune_kp_deg_per_mm = runtimeKp;
        ball_control_tune_kd_deg_per_mmps = runtimeKd;
        (void)controller.setPositionGains(runtimeKp, runtimeKd);

        float breakawayAngleDeg =
            ball_control_tune_breakaway_angle_deg;
        float stuckErrorMm = ball_control_tune_stuck_error_mm;
        float stuckSpeedMmps = ball_control_tune_stuck_speed_mmps;
        if (!std::isfinite(breakawayAngleDeg)) {
            breakawayAngleDeg = ball_control_config::kBreakawayAngleDeg;
        }
        if (!std::isfinite(stuckErrorMm)) {
            stuckErrorMm = ball_control_config::kStuckErrorThresholdMm;
        }
        if (!std::isfinite(stuckSpeedMmps)) {
            stuckSpeedMmps =
                ball_control_config::kStuckSpeedThresholdMmps;
        }
        breakawayAngleDeg = std::clamp(
            breakawayAngleDeg,
            ball_control_config::kMinimumTunableBreakawayAngleDeg,
            ball_control_config::kMaximumTunableBreakawayAngleDeg);
        stuckErrorMm = std::clamp(
            stuckErrorMm,
            ball_control_config::kMinimumTunableStuckErrorMm,
            ball_control_config::kMaximumTunableStuckErrorMm);
        stuckSpeedMmps = std::clamp(
            stuckSpeedMmps,
            ball_control_config::kMinimumTunableStuckSpeedMmps,
            ball_control_config::kMaximumTunableStuckSpeedMmps);
        std::uint32_t stuckTimeMs = std::clamp(
            static_cast<std::uint32_t>(ball_control_tune_stuck_time_ms),
            ball_control_config::kMinimumTunableStuckTimeMs,
            ball_control_config::kMaximumTunableStuckTimeMs);
        std::uint32_t breakawayDurationMs = std::clamp(
            static_cast<std::uint32_t>(
                ball_control_tune_breakaway_duration_ms),
            ball_control_config::kMinimumTunableBreakawayDurationMs,
            ball_control_config::kMaximumTunableBreakawayDurationMs);
        ball_control_tune_breakaway_angle_deg = breakawayAngleDeg;
        ball_control_tune_stuck_error_mm = stuckErrorMm;
        ball_control_tune_stuck_speed_mmps = stuckSpeedMmps;
        ball_control_tune_stuck_time_ms = stuckTimeMs;
        ball_control_tune_breakaway_duration_ms =
            breakawayDurationMs;
        (void)controller.setStictionParameters(
            breakawayAngleDeg,
            stuckErrorMm,
            stuckSpeedMmps,
            stuckTimeMs,
            breakawayDurationMs);
        BallControlOutput output = controller.update(input);
#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST) \
    && !defined(HBALL_IMU_FEEDFORWARD_TEST)
        const CompetitionImuFeedforwardOutput competitionFeedforwardOutput =
            competitionImuFeedforward.update(
                CompetitionImuFeedforwardInput{
                    .nowMs = input.nowMs,
                    .taskContextValid = taskContext.contextValid,
                    .taskId = taskContext.taskId,
                    .controlFlags = taskContext.effectiveControlFlags,
                    .balancing =
                        output.state == BallControlState::Balancing
                        && input.vision.valid
                        && input.vision.measurement_age_ms <=
                               ball_control_config::kVisionMaxMeasurementAgeMs,
                    .imu = lastImuControlSnapshot,
                    .pdOffsetDeg = output.angleOffsetDeg,
                    .activeAngleLimitDeg = output.activeAngleLimitDeg,
                    .piSessionId = visionSnapshot.pi_session_id,
                    .runId = taskContext.runId,
                });
        if (competitionFeedforwardOutput.applied) {
            output.angleOffsetDeg =
                competitionFeedforwardOutput.totalOffsetDeg;
            output.targetAngleRad = output.mechanicalLevelAngleRad
                + output.angleOffsetDeg
                    * std::numbers::pi_v<float> / 180.0F;
        }
        imu_feedforward_debug_ready =
            lastImuControlSnapshot.status == 1U
                && lastImuControlSnapshot.calibrated ? 1U : 0U;
        imu_feedforward_debug_valid =
            lastImuControlSnapshot.valid ? 1U : 0U;
        imu_feedforward_debug_gate_reason =
            static_cast<std::uint8_t>(competitionFeedforwardOutput.gate);
        imu_feedforward_debug_sample_fresh =
            competitionFeedforwardOutput.sampleFresh ? 1U : 0U;
        imu_feedforward_debug_sample_age_ms =
            competitionFeedforwardOutput.sampleAgeMs;
        imu_feedforward_debug_accel_mps2 =
            competitionFeedforwardOutput.accelerationMps2;
        imu_feedforward_debug_angle_offset_deg =
            competitionFeedforwardOutput.feedforwardOffsetDeg;
        imu_feedforward_debug_pd_offset_deg =
            competitionFeedforwardOutput.pdOffsetDeg;
        imu_feedforward_debug_total_offset_deg =
            competitionFeedforwardOutput.totalOffsetDeg;
        imu_feedforward_debug_target_angle_rad = output.targetAngleRad;
        imu_feedforward_debug_invalid_cycle_count =
            competitionFeedforwardOutput.invalidImuCycleCount;
        imu_feedforward_debug_stale_cycle_count =
            competitionFeedforwardOutput.staleImuCycleCount;
        vehicle_launch_debug_state = static_cast<std::uint8_t>(
            competitionFeedforwardOutput.launch.state);
        vehicle_launch_debug_active =
            competitionFeedforwardOutput.launch.active ? 1U : 0U;
        vehicle_launch_debug_offset_deg =
            competitionFeedforwardOutput.launch.angleOffsetDeg;
        vehicle_launch_debug_elapsed_ms =
            competitionFeedforwardOutput.launch.elapsedMs;
        vehicle_launch_debug_trigger_count =
            competitionFeedforwardOutput.launch.triggerCount;
        const float competitionDesiredAngleRad = output.targetAngleRad;
#endif
#if defined(HBALL_IMU_FEEDFORWARD_TEST)
        const ImuFeedforwardOutput imuFeedforwardOutput =
            imuFeedforwardController.update(
                imu_vehicle_valid != 0U,
                imu_vehicle_forward_filtered_mps2);
        imuFeedforwardPeakHold.observe(imuFeedforwardOutput);
        const ImuFeedforwardPeakSnapshot& imuFeedforwardPeaks =
            imuFeedforwardPeakHold.snapshot();
        float desiredAngleRad =
            imuFeedforwardOutput.targetAngleRad;
        output.targetAngleRad = desiredAngleRad;
        output.angleOffsetDeg =
            imuFeedforwardOutput.angleOffsetDeg;
        imu_feedforward_debug_valid =
            imuFeedforwardOutput.valid ? 1U : 0U;
        imu_feedforward_debug_accel_mps2 =
            imuFeedforwardOutput.accelerationMps2;
        imu_feedforward_debug_angle_offset_deg =
            imuFeedforwardOutput.angleOffsetDeg;
        imu_feedforward_debug_target_angle_rad =
            imuFeedforwardOutput.targetAngleRad;
        imu_feedforward_debug_valid_sample_count =
            imuFeedforwardPeaks.validSampleCount;
        imu_feedforward_debug_max_accel_mps2 =
            imuFeedforwardPeaks.maximumAccelerationMps2;
        imu_feedforward_debug_min_accel_mps2 =
            imuFeedforwardPeaks.minimumAccelerationMps2;
        imu_feedforward_debug_max_angle_offset_deg =
            imuFeedforwardPeaks.maximumAngleOffsetDeg;
        imu_feedforward_debug_min_angle_offset_deg =
            imuFeedforwardPeaks.minimumAngleOffsetDeg;
        imu_feedforward_debug_min_target_angle_rad =
            imuFeedforwardPeaks.minimumTargetAngleRad;
        imu_feedforward_debug_max_target_angle_rad =
            imuFeedforwardPeaks.maximumTargetAngleRad;
#endif
#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST)
        const bool installedCommandActive =
            output.sendAngleCommand
            && (input.motorEnabled || output.enableMotor);
        if (installedCommandActive) {
            output.targetAngleRad = installedCommandSlew.update(
                installedAbsoluteAngleSafe,
                input.motorAngleRad,
                input.motorSpeedRpm,
                output.targetAngleRad,
                input.nowMs);
        } else {
            installedCommandSlew.trackActual(
                installedAbsoluteAngleSafe,
                input.motorAngleRad,
                input.nowMs);
            // EnablePending keeps sending angle frames even between enable
            // retries.  Until enabled feedback is confirmed, every one of
            // those frames must hold the measured angle; otherwise the second
            // frame would reintroduce the full installed-angle step.
            output.targetAngleRad =
                output.sendAngleCommand && installedAbsoluteAngleSafe
                ? input.motorAngleRad
                : clampInstalledTargetAngle(output.targetAngleRad);
        }
#endif
#endif

        if (output.zeroOutput) {
            motor.setSpeed(0.0F);
            motor.setCurrent(0.0F);
        }
        if (output.disableMotor) {
            motor.disable();
        }
        if (output.sendAngleCommand && !output.zeroOutput) {
            ball_control_debug_last_sent_angle_rad =
                output.targetAngleRad;
            motor.setAngle(output.targetAngleRad);
        }
        // Preload the hold angle before enabling.  QD4310 may retain an old
        // angle target while disabled; enabling first can briefly apply that
        // stale target and create a large startup jerk.
        if (output.enableMotor) {
            motor.enable();
        }

        BoardIo_SetLed(ledStatusFor(output.state), input.nowMs);
        publishDebug(input, output);
        publishTask3FinalDebug(visionSnapshot, taskContext, input, output);
#if !defined(HBALL_PIPE_SWEEP_TEST) \
    && !defined(HBALL_PIPE_ANGLE_TEST) \
    && !defined(HBALL_IMU_FEEDFORWARD_TEST)
        launchTraceRecorder.update(VehicleLaunchTraceInput{
            .piSessionId = visionSnapshot.pi_session_id,
            .runId = visionSnapshot.run_id,
            .taskId = visionSnapshot.task_id,
            .levelAngleRad = output.mechanicalLevelAngleRad,
            .calibrationNoiseMps2 = imu_vehicle_noise_mps2,
            .deadbandMps2 = imu_vehicle_deadband_mps2,
            .record = VehicleLaunchTraceRecord{
                .nowMs = input.nowMs,
                .imuSampleCount = lastImuControlSnapshot.sampleCount,
                .launchTriggerCount = competitionFeedforwardOutput.launch.triggerCount,
                .accelerationMps2 = lastImuControlSnapshot.forwardAccelerationMps2,
                .rawAccelerationMps2 = imu_vehicle_forward_raw_mps2,
                .actualAngleRad = input.motorAngleRad,
                .desiredAngleRad = competitionDesiredAngleRad,
                .sentAngleRad = ball_control_debug_last_sent_angle_rad,
                .pdOffsetDeg = competitionFeedforwardOutput.pdOffsetDeg,
                .imuOffsetDeg = competitionFeedforwardOutput.feedforwardOffsetDeg,
                .launchOffsetDeg = competitionFeedforwardOutput.launch.angleOffsetDeg,
                .totalOffsetDeg = competitionFeedforwardOutput.totalOffsetDeg,
                .position0p1mm = input.vision.position_0p1mm,
                .velocityMmps = input.vision.velocity_mmps,
                .visionAgeMs = static_cast<std::uint16_t>(std::min<std::uint32_t>(
                    input.vision.measurement_age_ms, 65535U)),
                .imuAgeMs = static_cast<std::uint16_t>(std::min<std::uint32_t>(
                    competitionFeedforwardOutput.sampleAgeMs, 65535U)),
                .ballState = static_cast<std::uint8_t>(output.state),
                .ballFault = static_cast<std::uint8_t>(output.fault),
                .feedforwardGate = static_cast<std::uint8_t>(competitionFeedforwardOutput.gate),
                .launchState = static_cast<std::uint8_t>(competitionFeedforwardOutput.launch.state),
                .rawControlFlags = visionSnapshot.control_flags,
                .effectiveControlFlags = taskContext.effectiveControlFlags,
                .validityFlags = static_cast<std::uint8_t>(
                    (input.vision.valid ? 1U : 0U)
                    | (lastImuControlSnapshot.valid ? 2U : 0U)
                    | (lastImuControlSnapshot.calibrated ? 4U : 0U)
                    | (competitionFeedforwardOutput.sampleFresh ? 8U : 0U)
                    | (input.motorEnabled ? 16U : 0U)
                    | (output.sendAngleCommand && !output.zeroOutput ? 32U : 0U)
                    | (taskContext.contextValid ? 64U : 0U)
                    | (competitionFeedforwardOutput.launch.active ? 128U : 0U)),
            },
        });
#endif
        SafetyWatchdog_Refresh();
    }
}
