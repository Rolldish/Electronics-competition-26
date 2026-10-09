#include "CompetitionTaskGuard.h"

namespace {
constexpr std::uint8_t kControlEnabled = 1U << 0U;
constexpr std::uint8_t kRunActive = 1U << 1U;
constexpr std::uint8_t kTaskDone = 1U << 2U;
constexpr std::uint8_t kTaskTimeout = 1U << 3U;
constexpr std::uint8_t kTargetLatched = 1U << 4U;

bool has(std::uint8_t flags, std::uint8_t mask)
{
    return (flags & mask) != 0U;
}
}

CompetitionTaskOutput CompetitionTaskGuard::update(
    const CompetitionTaskInput& input)
{
    const CompetitionTaskError error = validate(input);
    const bool valid = error == CompetitionTaskError::None;
    if (!valid && isNewObservation(input)) {
        ++rejectCount_;
    }
    if (valid) {
        if (identityInitialized_
            && (input.piSessionId != lastPiSessionId_
                || input.runId != lastRunId_)) {
            ++identityResetCount_;
        }
        identityInitialized_ = true;
        lastPiSessionId_ = input.piSessionId;
        lastRunId_ = input.runId;
        lastTaskId_ = input.taskId;
        lastTargetPosition0p1mm_ = input.targetPosition0p1mm;
    }
    return CompetitionTaskOutput{
        .state = valid ? deriveState(input) : CompetitionTaskState::Invalid,
        .error = error,
        .contextValid = valid,
        .effectiveControlFlags = static_cast<std::uint8_t>(
            valid ? input.controlFlags : input.controlFlags & ~kControlEnabled),
        .taskId = input.taskId,
        .runId = input.runId,
        .targetPosition0p1mm = input.targetPosition0p1mm,
        .rawControlFlags = input.controlFlags,
        .rejectCount = rejectCount_,
        .identityResetCount = identityResetCount_,
    };
}

CompetitionTaskError CompetitionTaskGuard::validate(
    const CompetitionTaskInput& input) const
{
    if (input.taskId != 0U && input.taskId != 2U && input.taskId != 3U
        && input.taskId != 4U && input.taskId != 5U && input.taskId != 6U) {
        return CompetitionTaskError::UnknownTask;
    }

    const std::uint8_t flags = input.controlFlags;
    const bool done = has(flags, kTaskDone);
    const bool timeout = has(flags, kTaskTimeout);
    const bool terminal = done || timeout;
    if (done && timeout) {
        return CompetitionTaskError::ContradictoryTerminalFlags;
    }
    if (terminal && !has(flags, kRunActive)) {
        return CompetitionTaskError::TerminalWithoutRunActive;
    }
    if (input.taskId != 0U) {
        if (input.runId == 0U) {
            return CompetitionTaskError::InvalidRunId;
        }
        if (!has(flags, kControlEnabled)) {
            return CompetitionTaskError::MissingControlEnable;
        }
    }
    if (identityInitialized_
        && input.piSessionId == lastPiSessionId_
        && input.runId == lastRunId_
        && input.taskId != lastTaskId_) {
        return CompetitionTaskError::RunIdentityMismatch;
    }

    if (input.taskId == 0U) {
        return input.targetPosition0p1mm == 0 && flags == 0U
            ? CompetitionTaskError::None
            : CompetitionTaskError::InvalidIdleContext;
    }

    if (input.taskId == 3U) {
        if (flags == kControlEnabled) {
            return input.targetPosition0p1mm == 0
                ? CompetitionTaskError::None
                : CompetitionTaskError::TaskTargetMismatch;
        }
        if (flags == static_cast<std::uint8_t>(
                         kControlEnabled | kRunActive)) {
            return input.targetPosition0p1mm == 500
                    || input.targetPosition0p1mm == -500
                ? CompetitionTaskError::None
                : CompetitionTaskError::TaskTargetMismatch;
        }
        if (flags == static_cast<std::uint8_t>(
                         kControlEnabled | kRunActive | kTaskDone)
            || flags == static_cast<std::uint8_t>(
                            kControlEnabled | kRunActive | kTaskTimeout)) {
            return input.targetPosition0p1mm == -500
                ? CompetitionTaskError::None
                : CompetitionTaskError::TaskTargetMismatch;
        }
        return CompetitionTaskError::TaskTargetMismatch;
    }

    if (input.taskId == 2U) {
        return input.targetPosition0p1mm == 0
            ? CompetitionTaskError::None
            : CompetitionTaskError::TaskTargetMismatch;
    }

    if (input.taskId == 4U || input.taskId == 5U) {
        const bool flagsValid = flags == kControlEnabled
            || flags == static_cast<std::uint8_t>(
                            kControlEnabled | kRunActive)
            || flags == static_cast<std::uint8_t>(
                            kControlEnabled | kRunActive | kTaskDone)
            || flags == static_cast<std::uint8_t>(
                            kControlEnabled | kRunActive | kTaskTimeout);
        return input.targetPosition0p1mm == 0 && flagsValid
            ? CompetitionTaskError::None
            : CompetitionTaskError::TaskTargetMismatch;
    }

    if (input.taskId == 6U) {
        if (!has(flags, kTargetLatched)) {
            return CompetitionTaskError::MissingTargetLatch;
        }
        if (input.targetPosition0p1mm < -1000
            || input.targetPosition0p1mm > 1000) {
            return CompetitionTaskError::TaskTargetMismatch;
        }
        if (identityInitialized_
            && input.piSessionId == lastPiSessionId_
            && input.runId == lastRunId_
            && input.targetPosition0p1mm != lastTargetPosition0p1mm_) {
            return CompetitionTaskError::TaskTargetMismatch;
        }
        return CompetitionTaskError::None;
    }

    return CompetitionTaskError::UnknownTask;
}

CompetitionTaskState CompetitionTaskGuard::deriveState(
    const CompetitionTaskInput& input) const
{
    if (input.taskId == 0U) {
        return CompetitionTaskState::Idle;
    }
    if (has(input.controlFlags, kTaskTimeout)) {
        return CompetitionTaskState::Timeout;
    }
    if (has(input.controlFlags, kTaskDone)) {
        return CompetitionTaskState::Done;
    }
    if (has(input.controlFlags, kRunActive)) {
        return CompetitionTaskState::Active;
    }
    return CompetitionTaskState::Preparing;
}

bool CompetitionTaskGuard::isNewObservation(
    const CompetitionTaskInput& input)
{
    const bool fresh = !observationInitialized_
        || input.piSessionId != lastObservedSessionId_
        || input.sequence != lastObservedSequence_;
    observationInitialized_ = true;
    lastObservedSessionId_ = input.piSessionId;
    lastObservedSequence_ = input.sequence;
    return fresh;
}
