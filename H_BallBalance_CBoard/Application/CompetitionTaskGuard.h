#pragma once

#include <cstdint>

enum class CompetitionTaskState : std::uint8_t {
    Idle,
    Preparing,
    Active,
    Done,
    Timeout,
    Invalid,
};

enum class CompetitionTaskError : std::uint8_t {
    None,
    UnknownTask,
    InvalidRunId,
    InvalidIdleContext,
    MissingControlEnable,
    ContradictoryTerminalFlags,
    TerminalWithoutRunActive,
    TaskTargetMismatch,
    MissingTargetLatch,
    RunIdentityMismatch,
};

struct CompetitionTaskInput {
    std::uint32_t piSessionId{};
    std::uint16_t runId{};
    std::int16_t targetPosition0p1mm{};
    std::uint8_t taskId{};
    std::uint8_t controlFlags{};
    std::uint8_t sequence{};
};

struct CompetitionTaskOutput {
    CompetitionTaskState state{CompetitionTaskState::Invalid};
    CompetitionTaskError error{CompetitionTaskError::None};
    bool contextValid{};
    std::uint8_t effectiveControlFlags{};
    std::uint8_t taskId{};
    std::uint16_t runId{};
    std::int16_t targetPosition0p1mm{};
    std::uint8_t rawControlFlags{};
    std::uint32_t rejectCount{};
    std::uint32_t identityResetCount{};
};

class CompetitionTaskGuard {
public:
    CompetitionTaskOutput update(const CompetitionTaskInput& input);

private:
    CompetitionTaskError validate(const CompetitionTaskInput& input) const;
    CompetitionTaskState deriveState(const CompetitionTaskInput& input) const;
    bool isNewObservation(const CompetitionTaskInput& input);

    bool identityInitialized_{};
    std::uint32_t lastPiSessionId_{};
    std::uint16_t lastRunId_{};
    std::uint8_t lastTaskId_{};
    std::int16_t lastTargetPosition0p1mm_{};
    bool observationInitialized_{};
    std::uint32_t lastObservedSessionId_{};
    std::uint8_t lastObservedSequence_{};
    std::uint32_t rejectCount_{};
    std::uint32_t identityResetCount_{};
};
