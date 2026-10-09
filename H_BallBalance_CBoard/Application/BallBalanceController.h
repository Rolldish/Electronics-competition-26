#pragma once

#include <cstdint>

enum class BallControlState : std::uint8_t {
    BootSafe,
    WaitMotor,
    ConfigRequired,
    EnablePending,
    LevelHold,
    Balancing,
    VisionRecovery,
    Fault,
};

enum class BallControlFault : std::uint8_t {
    None,
    EmergencyStop,
    CanTransmit,
    MotorFeedbackTimeout,
    InvalidMotorFeedback,
    MechanicalAngleLimit,
    MotorEnableTimeout,
    MotorUnexpectedlyDisabled,
    MotorOverspeed,
    MotorOvercurrent,
    ControlLoopTiming,
};

enum class BallStictionState : std::uint8_t {
    Monitoring,
    Qualifying,
    Breakaway,
    LockedOut,
};

// Protocol-independent copy of the fields consumed by the controller.
// BallControlTask is the adapter from VisionLinkSnapshot to this type.
struct BallVisionSnapshot {
    std::uint32_t pi_session_id{};
    std::uint32_t capture_timestamp_ms{};
    std::uint32_t local_receive_ms{};
    std::uint32_t measurement_age_ms{};
    std::int16_t position_0p1mm{};
    std::int16_t velocity_mmps{};
    std::int16_t target_position_0p1mm{};
    std::uint32_t measurement_update_count{};
    std::uint8_t confidence{};
    std::uint8_t vision_flags{};
    std::uint8_t control_flags{};
    std::uint8_t task_id{};
    std::uint16_t run_id{};
    std::uint8_t sequence{};
    bool valid{};
    bool velocity_valid{};
};

struct BallControlInput {
    std::uint32_t nowMs{};
    bool commissioned{};
    bool keyEmergencyStop{};
    bool motorFeedbackFresh{};
    bool motorFeedbackFinite{};
    bool motorEnabled{};
    std::uint32_t motorFeedbackCount{};
    std::uint32_t txErrorCount{};
    float motorAngleRad{};
    float motorSpeedRpm{};
    float motorCurrentA{};
    BallVisionSnapshot vision{};
};

struct BallControlOutput {
    BallControlState state{BallControlState::BootSafe};
    BallControlFault fault{BallControlFault::None};
    bool enableMotor{};
    bool zeroOutput{};
    bool disableMotor{};
    bool sendAngleCommand{};
    float mechanicalLevelAngleRad{};
    float actualAngleRad{};
    float targetAngleRad{};
    float angleOffsetDeg{};
    std::int16_t targetPosition0p1mm{};
    std::int16_t position0p1mm{};
    float velocityMmps{};
    float motorSpeedRpm{};
    float motorCurrentA{};
    std::uint32_t motorEnableConfirmations{};
    std::uint32_t controlElapsedMs{};
    std::uint32_t consecutiveValidMeasurements{};
    std::uint32_t measurementUpdateCount{};
    BallStictionState stictionState{BallStictionState::Monitoring};
    std::uint32_t stuckElapsedMs{};
    float activeAngleLimitDeg{};
    bool centerHoldActive{};
    std::uint8_t taskId{};
    std::uint16_t runId{};
};

struct BallControlRuntimeConfig {
    float mechanicalLevelAngleRad{};
    float angleSign{};
    float maxAngleOffsetDeg{};
    float mechanicalAngleLimitDeg{};
    float maxMotorSpeedRpm{};
    float maxMotorCurrentA{};
};

class BallBalanceController {
public:
    BallBalanceController();
    explicit BallBalanceController(const BallControlRuntimeConfig& config);

    bool setMechanicalLevelAngleRad(float angleRad);
    bool setPositionGains(float kpDegPerMm, float kdDegPerMmps);
    bool setStictionParameters(float breakawayAngleDeg,
                               float stuckErrorMm,
                               float stuckSpeedMmps,
                               std::uint32_t stuckTimeMs,
                               std::uint32_t breakawayDurationMs);
    BallControlOutput update(const BallControlInput& input);

private:
    BallControlOutput makeOutput(const BallControlInput& input) const;
    BallControlOutput safeShutdownOutput(const BallControlInput& input,
                                         bool force = false);
    BallControlOutput angleHoldOutput(const BallControlInput& input,
                                      float desiredOffsetDeg,
                                      bool requestEnable);
    BallControlOutput enterFault(const BallControlInput& input,
                                 BallControlFault fault);

    bool visionQualityIsUsable(const BallControlInput& input) const;
    bool visionIsFreshForControl(const BallControlInput& input) const;
    bool observeNewVisionMeasurement(const BallControlInput& input,
                                     bool& identityChanged);
    void startVisionDropout(const BallControlInput& input);
    bool visionDropoutExceeded(const BallControlInput& input) const;
    void updateCenterHoldGate(float absoluteErrorMm,
                              float absoluteVelocityMmps,
                              bool velocityValid,
                              bool newMeasurement);
    void resetCenterHoldGate();
    void resetStictionControl();
    float updateStictionControl(const BallControlInput& input,
                                float absoluteErrorMm,
                                float absoluteVelocityMmps,
                                float requestedOffsetDeg,
                                bool newMeasurement);
    void updateMotorFeedbackGate(const BallControlInput& input);
    BallControlFault updateMotorLimitGate(const BallControlInput& input);

    BallControlState state_{BallControlState::BootSafe};
    BallControlFault fault_{BallControlFault::None};
    bool firstUpdate_{true};
    bool motorReady_{};
    bool motorFeedbackCounterInitialized_{};
    std::uint32_t lastMotorFeedbackCount_{};
    std::uint32_t consecutiveValidMotorFeedback_{};
    bool motorLimitCounterInitialized_{};
    std::uint32_t lastMotorLimitFeedbackCount_{};
    std::uint32_t consecutiveOverspeedFeedback_{};
    std::uint32_t consecutiveOvercurrentFeedback_{};
    bool enableCommandSent_{};
    std::uint32_t enablePendingStartedMs_{};
    std::uint32_t lastEnableCommandMs_{};
    std::uint32_t lastEnableFeedbackCount_{};
    std::uint32_t motorEnableConfirmations_{};
    bool txCounterInitialized_{};
    std::uint32_t lastTxErrorCount_{};
    bool visionIdentityInitialized_{};
    std::uint32_t lastPiSessionId_{};
    std::uint16_t lastRunId_{};
    std::uint32_t lastMeasurementUpdateCount_{};
    std::uint32_t consecutiveValidMeasurements_{};
    bool fullVisionGateRequired_{true};
    bool visionDropoutActive_{};
    std::uint32_t visionDropoutStartMs_{};
    bool centerHoldActive_{};
    std::uint32_t consecutiveCenterEntryMeasurements_{};
    std::uint32_t consecutiveCenterExitMeasurements_{};
    float commandedOffsetDeg_{};
    bool updateTimeInitialized_{};
    std::uint32_t lastUpdateMs_{};
    bool shutdownCommandInitialized_{};
    std::uint32_t lastShutdownCommandMs_{};
    float positionKpDegPerMm_{};
    float positionKdDegPerMmps_{};
    float breakawayAngleDeg_{};
    float stuckErrorMm_{};
    float stuckSpeedMmps_{};
    std::uint32_t stuckTimeMs_{};
    std::uint32_t breakawayDurationMs_{};
    BallStictionState stictionState_{BallStictionState::Monitoring};
    bool stictionCaptureInitialized_{};
    std::uint32_t lastStictionCaptureTimestampMs_{};
    std::uint32_t stuckElapsedMs_{};
    std::uint32_t breakawayElapsedMs_{};
    float breakawayStartAbsoluteErrorMm_{};
    float breakawayLockoutAbsoluteErrorMm_{};
    float activeAngleLimitDeg_{};
    BallControlRuntimeConfig config_{};
};
