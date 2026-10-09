#pragma once

#include <cstdint>

namespace ball_control_config {

// Installed mechanism values verified during the vehicle-mounted bring-up.
// A larger motor angle drives the ball toward the vehicle rear; therefore a
// positive rearward position error must command a smaller motor angle.
inline constexpr bool kBallControlCommissioned = true;
inline constexpr float kMechanicalLevelAngleRad = 1.2461F;
inline constexpr float kAngleSign = 1.0F;
inline constexpr float kInstalledPhysicalLowerAngleRad = 0.45F;
inline constexpr float kInstalledPhysicalUpperAngleRad = 1.55F;
inline constexpr float kInstalledCommandLowerAngleRad = 1.10F;
inline constexpr float kInstalledCommandUpperAngleRad = 1.40F;
inline constexpr float kInstalledLevelCommandAngleRad =
    kMechanicalLevelAngleRad;
inline constexpr float kInstalledCommandSlewRateDegPerSecond = 25.0F;
inline constexpr float kInstalledMaxFollowingErrorDeg = 0.75F;
inline constexpr float kInstalledTrackingPauseSpeedRpm = 30.0F;

// QD4310 internal-loop parameters verified with the vendor upper computer.
// The current CAN protocol cannot read or write these settings. Until their
// non-volatile storage is verified, they must be checked after every power-on.
inline constexpr float kRequiredQd4310SpeedKp = 0.00400F;
inline constexpr float kRequiredQd4310SpeedKi = 0.00010F;
inline constexpr float kRequiredQd4310SpeedKd = 0.0F;
inline constexpr float kRequiredQd4310AngleKp = 1200.0F;
inline constexpr float kRequiredQd4310AngleKi = 0.0F;
inline constexpr float kRequiredQd4310AngleKd = 0.500F;
inline constexpr float kRequiredQd4310CurrentLimitA = 1.65F;
inline constexpr float kRequiredQd4310SpeedLimitRpm = 1000.0F;
inline constexpr std::uint32_t kRequiredQd4310CanBaud = 1000000U;

inline constexpr std::uint32_t kControlPeriodMs = 5U;
inline constexpr std::uint32_t kMotorFeedbackRequiredFrames = 5U;
inline constexpr std::uint32_t kMotorEnableConfirmFrames = 3U;
inline constexpr std::uint32_t kEnableRetryPeriodMs = 100U;
inline constexpr std::uint32_t kEnableTimeoutMs = 1000U;
inline constexpr std::uint32_t kSafeShutdownRetryMs = 100U;
inline constexpr std::uint32_t kVisionRequiredNewMeasurements = 10U;
inline constexpr std::uint32_t kVisionRecoveryNewMeasurements = 3U;
inline constexpr std::uint32_t kVisionMaxMeasurementAgeMs = 80U;
inline constexpr std::uint32_t kVisionReturnToLevelAgeMs = 110U;
inline constexpr std::uint32_t kFastVisionRecoveryMaxDropoutMs = 300U;
inline constexpr std::uint32_t kVisionSilenceTimeoutMs = 110U;
inline constexpr std::uint8_t kMinimumVisionConfidence = 180U;

// Hardware-accepted center hold: treat the inclusive interval from -3 mm
// through +3 mm around the current target as the center deadband. Entry also
// requires low speed and three distinct fresh measurements; the first fresh
// measurement outside the interval resumes position control immediately.
inline constexpr float kCenterHoldEnterErrorMm = 3.0F;
inline constexpr float kCenterHoldExitErrorMm = 3.0F;
inline constexpr float kCenterHoldEnterMaxSpeedMmps = 10.0F;
inline constexpr std::uint32_t kCenterHoldEntryMeasurements = 3U;
inline constexpr std::uint32_t kCenterHoldExitMeasurements = 1U;

// Hardware-accepted center-recovery gains (2026-08-01). Keep these as the
// default baseline unless a later hardware trial is explicitly accepted.
inline constexpr float kKpDegPerMm = 0.25F;
inline constexpr float kKdDegPerMmps = 0.15F;
inline constexpr float kMinimumTunableKpDegPerMm = 0.0F;
inline constexpr float kMaximumTunableKpDegPerMm = 0.30F;
inline constexpr float kMinimumTunableKdDegPerMmps = 0.0F;
inline constexpr float kMaximumTunableKdDegPerMmps = 0.2F;
inline constexpr float kMaxAngleOffsetDeg = 3.0F;

// Static-friction escape. Normal control remains limited to 3 degrees. Only a
// qualified stationary ball may temporarily use the 4-degree hard ceiling.
inline constexpr float kBreakawayAngleDeg = 3.5F;
inline constexpr float kMinimumTunableBreakawayAngleDeg = 3.0F;
inline constexpr float kMaximumTunableBreakawayAngleDeg = 4.0F;
inline constexpr float kStuckErrorThresholdMm = 10.0F;
inline constexpr float kMinimumTunableStuckErrorMm = 5.0F;
inline constexpr float kMaximumTunableStuckErrorMm = 100.0F;
inline constexpr float kStuckSpeedThresholdMmps = 5.0F;
inline constexpr float kMinimumTunableStuckSpeedMmps = 0.0F;
inline constexpr float kMaximumTunableStuckSpeedMmps = 30.0F;
inline constexpr std::uint32_t kStuckQualificationMs = 350U;
inline constexpr std::uint32_t kMinimumTunableStuckTimeMs = 100U;
inline constexpr std::uint32_t kMaximumTunableStuckTimeMs = 1000U;
inline constexpr std::uint32_t kBreakawayDurationMs = 150U;
inline constexpr std::uint32_t kMinimumTunableBreakawayDurationMs = 100U;
inline constexpr std::uint32_t kMaximumTunableBreakawayDurationMs = 1000U;
inline constexpr float kBreakawayExitSpeedMmps = 15.0F;
inline constexpr float kBreakawayExitProgressMm = 3.0F;
// Task 3 must traverse both 5 cm targets inside one 5 s Pi-side window.  Keep
// the accepted baseline for every other task, but qualify a short breakaway
// earlier when T3 is stationary outside its 3 mm completion band.
inline constexpr float kTask3StuckErrorThresholdMm = 5.0F;
inline constexpr std::uint32_t kTask3StuckQualificationMs = 150U;
inline constexpr float kTask3MinimumBreakawayCommandDeg = 1.25F;
inline constexpr float kTask3MotionAngleLimitDeg = 4.0F;
inline constexpr float kTask3FastTravelErrorThresholdMm = 20.0F;
inline constexpr float kTargetSlewRateDegPerSecond = 25.0F;
inline constexpr float kMechanicalAngleLimitDeg = 46.0F;
inline constexpr std::uint32_t kMaxSlewElapsedMs = 20U;
inline constexpr std::uint32_t kSevereControlLapseMs = 200U;

// Conservative placeholders for the installed mechanism. Confirm normal
// speed/current envelopes on hardware before commissioning automatic control.
inline constexpr float kMaxMotorSpeedRpm = 60.0F;
inline constexpr float kMaxMotorCurrentA = 1.5F;
inline constexpr std::uint32_t kMotorLimitViolationFrames = 3U;

// Dedicated short-arm bench limits. These values are selected only by the
// SimVisionMotorTest build and never commission the production Debug build.
inline constexpr float kSimMaxAngleOffsetDeg = 3.0F;
// The installed mechanism can be pushed anywhere inside the independently
// checked absolute range below.  Keep the commanded target narrow, but allow
// the controller to recover from either physical endpoint instead of latching
// a relative-to-level angle fault before it can enable the motor.
inline constexpr float kSimMechanicalAngleLimitDeg = 46.0F;
inline constexpr float kSimMaxMotorSpeedRpm = 60.0F;
inline constexpr float kSimMaxMotorCurrentA = 1.5F;

// Standalone vehicle-acceleration feedforward trial. The BMI088 estimator
// reports positive acceleration toward the vehicle front. Decreasing the
// installed motor angle drives the ball toward the front, so the feedforward
// mapping applies the opposite pipe angle to the measured acceleration.
inline constexpr float kImuFeedforwardGravityMps2 = 9.80665F;
inline constexpr float kImuFeedforwardGain = 1.0F;
inline constexpr float kImuFeedforwardMaxAngleOffsetDeg = 3.0F;
// Task 4 runs only in the vehicle-forward direction.  Tune launch and brake
// compensation independently by flashing and observing the ball motion.
inline constexpr float kTask4StartFeedforwardGain = 10.0F;
inline constexpr float kTask4BrakeFeedforwardGain =10.0F;
inline constexpr float kTask4MaxFeedforwardOffsetDeg = 20.0F;
// Detect the end of Task 4 braking from the filtered vehicle acceleration,
// hold the positive hard ceiling, then remove it linearly so PD takes over.
inline constexpr float kTask4BrakeDetectAccelerationMps2 = -0.15F;
inline constexpr float kTask4StopDetectAccelerationMps2 = -0.05F;
inline constexpr float kTask4PostStopOffsetDeg = 4.0F;
inline constexpr std::uint32_t kTask4PostStopHoldMs = 300U;
inline constexpr std::uint32_t kTask4PostStopReleaseMs = 700U;
inline constexpr bool kCompetitionImuFeedforwardEnabled = true;
inline constexpr std::uint32_t kImuFeedforwardStaleTimeoutMs = 100U;

// Pi START and the 3507 start key are independent. Arm in T4/T5/T6 ACTIVE,
// then confirm actual forward launch from two distinct fresh IMU outputs.
// These are initial trial values, not hardware-accepted competition gains.
// A negative installed-motor offset drives the ball toward the vehicle front.
inline constexpr bool kVehicleLaunchFeedforwardEnabled = true;
inline constexpr float kVehicleLaunchAngleOffsetDeg = -2.0F;
inline constexpr float kVehicleLaunchAccelerationThresholdMps2 = 0.06F;
inline constexpr std::uint32_t kVehicleLaunchRequiredNewSamples = 2U;
inline constexpr std::uint32_t kVehicleLaunchHoldMs = 100U;
inline constexpr std::uint32_t kVehicleLaunchReleaseMs = 700U;
// Only the launch window can use this previously established 4-degree ceiling.
// The normal PD/IMU total limit resumes when launch ends or is cancelled.
inline constexpr float kVehicleLaunchMaxTotalOffsetDeg = 4.0F;

// Installed-mechanism angle-loop trial. This is selected only by the
// PipeAngleTest build; it does not commission the production Debug build.
// The level angle is an initial upper-computer measurement and must be
// re-calibrated after the full mechanism is finalized.
inline constexpr float kPipeTestMechanicalLevelAngleRad = 1.27F;
inline constexpr float kPipeTestAngleSign = 1.0F;
inline constexpr float kPipeTestMaxAngleOffsetDeg = 3.0F;
inline constexpr float kPipeTestMechanicalAngleLimitDeg = 6.0F;
inline constexpr float kPipeTestMaxMotorSpeedRpm = 20.0F;
inline constexpr float kPipeTestMaxMotorCurrentA = 1.0F;

// Standalone installed-pipe center hold. It intentionally reuses the existing
// PipeSweepTest build/debug entry so the hardware operator has one stable
// flashing workflow. No Raspberry Pi or vision frames are required.
inline constexpr float kPipeSweepCenterAngleRad =
    kInstalledLevelCommandAngleRad;
inline constexpr float kPipeSweepSafetyLowerAngleRad =
    kInstalledPhysicalLowerAngleRad;
inline constexpr float kPipeSweepSafetyUpperAngleRad =
    kInstalledPhysicalUpperAngleRad;
inline constexpr float kPipeSweepSlewRateDegPerSecond = 25.0F;
inline constexpr float kPipeSweepArrivalToleranceRad = 0.01F;
inline constexpr float kPipeSweepArrivalMaxSpeedRpm = 2.0F;
inline constexpr float kPipeSweepMaxMotorSpeedRpm = 60.0F;
inline constexpr float kPipeSweepMaxMotorCurrentA = 1.5F;

// Frozen BallLink flag meanings, duplicated here so the pure controller has no
// protocol/parser dependency.
inline constexpr std::uint8_t kVisionBallValid = 1U << 0U;
inline constexpr std::uint8_t kVisionCameraCalibrated = 1U << 2U;
inline constexpr std::uint8_t kVisionProcessingDegraded = 1U << 3U;
inline constexpr std::uint8_t kControlEnabled = 1U << 0U;

} // namespace ball_control_config
