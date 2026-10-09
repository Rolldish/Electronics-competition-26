#include "CompetitionTaskGuard.h"

#include <array>
#include <iostream>
#include <stdexcept>

namespace {
constexpr std::uint8_t kEnabled = 0x01U;
constexpr std::uint8_t kActive = 0x02U;
constexpr std::uint8_t kDone = 0x04U;
constexpr std::uint8_t kTimeout = 0x08U;
constexpr std::uint8_t kTargetLatched = 0x10U;

void require(bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

CompetitionTaskInput idleInput()
{
    return CompetitionTaskInput{
        .piSessionId = 7U,
        .runId = 9U,
        .targetPosition0p1mm = 0,
        .taskId = 0U,
        .controlFlags = 0U,
        .sequence = 1U,
    };
}

CompetitionTaskInput t3Input(std::int16_t target,
                             std::uint8_t flags,
                             std::uint8_t sequence)
{
    return CompetitionTaskInput{
        .piSessionId = 7U,
        .runId = 42U,
        .targetPosition0p1mm = target,
        .taskId = 3U,
        .controlFlags = flags,
        .sequence = sequence,
    };
}

CompetitionTaskInput t6Input(std::int16_t target,
                             std::uint8_t flags,
                             std::uint8_t sequence)
{
    return CompetitionTaskInput{
        .piSessionId = 9U,
        .runId = 106U,
        .targetPosition0p1mm = target,
        .taskId = 6U,
        .controlFlags = flags,
        .sequence = sequence,
    };
}

CompetitionTaskInput centerHoldInput(std::uint8_t taskId,
                                     std::int16_t target,
                                     std::uint8_t flags,
                                     std::uint8_t sequence)
{
    return CompetitionTaskInput{
        .piSessionId = 8U,
        .runId = static_cast<std::uint16_t>(100U + taskId),
        .targetPosition0p1mm = target,
        .taskId = taskId,
        .controlFlags = flags,
        .sequence = sequence,
    };
}

void testIdleContextIsSafeAndValid()
{
    CompetitionTaskGuard guard;
    const CompetitionTaskOutput output = guard.update(idleInput());
    require(output.contextValid, "IDLE context was rejected");
    require(output.state == CompetitionTaskState::Idle, "IDLE state mismatch");
    require(output.effectiveControlFlags == 0U, "IDLE enabled control");
}

void testIdleRejectsNonzeroTargetOrFlags()
{
    CompetitionTaskGuard targetGuard;
    CompetitionTaskInput input = idleInput();
    input.targetPosition0p1mm = 1;
    const CompetitionTaskOutput targetOutput = targetGuard.update(input);
    require(!targetOutput.contextValid, "IDLE accepted nonzero target");
    require(targetOutput.error == CompetitionTaskError::InvalidIdleContext,
            "IDLE target returned wrong error");

    CompetitionTaskGuard flagsGuard;
    input = idleInput();
    input.controlFlags = kEnabled;
    const CompetitionTaskOutput flagsOutput = flagsGuard.update(input);
    require(!flagsOutput.contextValid, "IDLE accepted control flags");
    require(flagsOutput.error == CompetitionTaskError::InvalidIdleContext,
            "IDLE flags returned wrong error");
}

void testUnknownTaskIsRejectedFirst()
{
    CompetitionTaskGuard guard;
    CompetitionTaskInput input = t3Input(-500,
        static_cast<std::uint8_t>(kEnabled | kDone | kTimeout), 1U);
    input.taskId = 1U;
    const CompetitionTaskOutput output = guard.update(input);
    require(!output.contextValid, "unknown task was accepted");
    require(output.error == CompetitionTaskError::UnknownTask,
            "unknown task did not have highest error priority");
}

void testCenterHoldTasksRequireZeroTargetInEveryPhase()
{
    struct PhaseCase {
        std::uint8_t flags;
        CompetitionTaskState expectedState;
    };
    constexpr std::array<std::uint8_t, 3> taskIds{2U, 4U, 5U};
    constexpr std::array<PhaseCase, 4> phases{
        PhaseCase{kEnabled, CompetitionTaskState::Preparing},
        PhaseCase{static_cast<std::uint8_t>(kEnabled | kActive),
                  CompetitionTaskState::Active},
        PhaseCase{static_cast<std::uint8_t>(kEnabled | kActive | kDone),
                  CompetitionTaskState::Done},
        PhaseCase{static_cast<std::uint8_t>(kEnabled | kActive | kTimeout),
                  CompetitionTaskState::Timeout},
    };
    constexpr std::array<std::int16_t, 2> nonzeroTargets{1, -1};

    for (const std::uint8_t taskId : taskIds) {
        for (const PhaseCase phase : phases) {
            CompetitionTaskInput input{
                .piSessionId = 8U,
                .runId = static_cast<std::uint16_t>(100U + taskId),
                .targetPosition0p1mm = 0,
                .taskId = taskId,
                .controlFlags = phase.flags,
                .sequence = 1U,
            };

            CompetitionTaskGuard validGuard;
            const CompetitionTaskOutput valid = validGuard.update(input);
            require(valid.error != CompetitionTaskError::UnknownTask,
                    "center-hold task returned UnknownTask");
            require(valid.contextValid,
                    "center-hold task rejected zero target");
            require(valid.state == phase.expectedState,
                    "center-hold task state mismatch");

            for (const std::int16_t target : nonzeroTargets) {
                CompetitionTaskGuard invalidGuard;
                input.targetPosition0p1mm = target;
                const CompetitionTaskOutput invalid = invalidGuard.update(input);
                require(!invalid.contextValid,
                        "center-hold task accepted nonzero target");
                require(invalid.error == CompetitionTaskError::TaskTargetMismatch,
                        "center-hold nonzero target returned wrong error");
            }
        }
    }
}

void testT4T5UseExactReadyAndActiveFramesAndRejectLatch()
{
    struct InvalidFrame {
        std::int16_t target;
        std::uint8_t flags;
    };
    constexpr std::array<std::uint8_t, 2> taskIds{4U, 5U};
    constexpr std::array<InvalidFrame, 4> invalidFrames{
        InvalidFrame{1, kEnabled},
        InvalidFrame{-1, static_cast<std::uint8_t>(kEnabled | kActive)},
        InvalidFrame{0,
                     static_cast<std::uint8_t>(kEnabled | kTargetLatched)},
        InvalidFrame{0,
                     static_cast<std::uint8_t>(
                         kEnabled | kActive | kTargetLatched)},
    };

    for (const std::uint8_t taskId : taskIds) {
        CompetitionTaskGuard guard;
        const CompetitionTaskOutput ready = guard.update(
            centerHoldInput(taskId, 0, kEnabled, 1U));
        require(ready.contextValid
                    && ready.state == CompetitionTaskState::Preparing,
                "T4/T5 exact READY frame was rejected");
        require(ready.effectiveControlFlags == kEnabled
                    && ready.rawControlFlags == kEnabled,
                "T4/T5 READY flags changed");
        require(ready.taskId == taskId
                    && ready.runId == static_cast<std::uint16_t>(100U + taskId)
                    && ready.targetPosition0p1mm == 0,
                "T4/T5 READY diagnostics changed");

        const std::uint8_t activeFlags = static_cast<std::uint8_t>(
            kEnabled | kActive);
        const CompetitionTaskOutput active = guard.update(
            centerHoldInput(taskId, 0, activeFlags, 2U));
        require(active.contextValid
                    && active.state == CompetitionTaskState::Active,
                "T4/T5 exact ACTIVE frame was rejected");
        require(active.effectiveControlFlags == activeFlags
                    && active.rawControlFlags == activeFlags,
                "T4/T5 ACTIVE flags changed");
        require(active.taskId == taskId
                    && active.runId == ready.runId
                    && active.targetPosition0p1mm == 0
                    && active.identityResetCount == 0U,
                "T4/T5 ACTIVE identity or diagnostics changed");

        for (const InvalidFrame frame : invalidFrames) {
            CompetitionTaskGuard invalidGuard;
            const CompetitionTaskOutput invalid = invalidGuard.update(
                centerHoldInput(taskId, frame.target, frame.flags, 3U));
            require(!invalid.contextValid,
                    "T4/T5 accepted nonzero target or TARGET_LATCHED");
            require(invalid.error == CompetitionTaskError::TaskTargetMismatch,
                    "T4/T5 invalid frame returned wrong error");
            require((invalid.effectiveControlFlags & kEnabled) == 0U,
                    "T4/T5 invalid frame retained CONTROL_ENABLED");
        }
    }
}

void testT3PreparationAndActiveTargets()
{
    CompetitionTaskGuard guard;
    const CompetitionTaskOutput preparing = guard.update(t3Input(0, kEnabled, 1U));
    require(preparing.contextValid, "T3 preparation was rejected");
    require(preparing.state == CompetitionTaskState::Preparing,
            "T3 preparation state mismatch");
    require(preparing.targetPosition0p1mm == 0,
            "guard changed T3 preparation target");

    const CompetitionTaskOutput positive = guard.update(
        t3Input(500, static_cast<std::uint8_t>(kEnabled | kActive), 2U));
    require(positive.contextValid, "T3 +500 was rejected");
    require(positive.state == CompetitionTaskState::Active,
            "T3 active state mismatch");
    require(positive.targetPosition0p1mm == 500,
            "guard changed T3 +500 target");

    const CompetitionTaskOutput negative = guard.update(
        t3Input(-500, static_cast<std::uint8_t>(kEnabled | kActive), 3U));
    require(negative.contextValid, "T3 -500 was rejected");
    require(negative.targetPosition0p1mm == -500,
            "guard changed T3 -500 target");
}

void testT3RejectsWrongPhaseTargets()
{
    CompetitionTaskGuard guard;
    const CompetitionTaskOutput preparing = guard.update(t3Input(500, kEnabled, 1U));
    require(!preparing.contextValid, "T3 preparation accepted nonzero target");
    require(preparing.error == CompetitionTaskError::TaskTargetMismatch,
            "T3 preparation target returned wrong error");

    const CompetitionTaskOutput active = guard.update(
        t3Input(0, static_cast<std::uint8_t>(kEnabled | kActive), 2U));
    require(!active.contextValid, "T3 active accepted zero target");
    require(active.error == CompetitionTaskError::TaskTargetMismatch,
            "T3 active target returned wrong error");
}

void testT3TerminalTargetsAndStates()
{
    CompetitionTaskGuard doneGuard;
    const CompetitionTaskOutput done = doneGuard.update(t3Input(
        -500, static_cast<std::uint8_t>(kEnabled | kActive | kDone), 1U));
    require(done.contextValid, "valid T3 DONE was rejected");
    require(done.state == CompetitionTaskState::Done, "T3 DONE state mismatch");

    CompetitionTaskGuard timeoutGuard;
    const CompetitionTaskOutput timeout = timeoutGuard.update(t3Input(
        -500, static_cast<std::uint8_t>(kEnabled | kActive | kTimeout), 1U));
    require(timeout.contextValid, "valid T3 TIMEOUT was rejected");
    require(timeout.state == CompetitionTaskState::Timeout,
            "T3 TIMEOUT state mismatch");

    CompetitionTaskGuard targetGuard;
    const CompetitionTaskOutput wrongTarget = targetGuard.update(t3Input(
        500, static_cast<std::uint8_t>(kEnabled | kActive | kDone), 1U));
    require(!wrongTarget.contextValid, "T3 terminal accepted +500 target");
    require(wrongTarget.error == CompetitionTaskError::TaskTargetMismatch,
            "T3 terminal target returned wrong error");
}

void testT3FrozenFrameSequenceKeepsOneRunIdentity()
{
    struct FrameCase {
        std::int16_t target;
        std::uint8_t flags;
        CompetitionTaskState state;
    };
    constexpr std::array<FrameCase, 4> successFrames{
        FrameCase{0, kEnabled, CompetitionTaskState::Preparing},
        FrameCase{500, static_cast<std::uint8_t>(kEnabled | kActive),
                  CompetitionTaskState::Active},
        FrameCase{-500, static_cast<std::uint8_t>(kEnabled | kActive),
                  CompetitionTaskState::Active},
        FrameCase{-500,
                  static_cast<std::uint8_t>(kEnabled | kActive | kDone),
                  CompetitionTaskState::Done},
    };

    CompetitionTaskGuard successGuard;
    std::uint8_t sequence = 1U;
    for (const FrameCase frame : successFrames) {
        const CompetitionTaskOutput output = successGuard.update(
            t3Input(frame.target, frame.flags, sequence++));
        require(output.contextValid, "frozen T3 success frame was rejected");
        require(output.state == frame.state,
                "frozen T3 success frame state mismatch");
        require(output.runId == 42U && output.identityResetCount == 0U,
                "T3 phase transition changed the run identity");
    }

    CompetitionTaskGuard timeoutGuard;
    require(timeoutGuard.update(t3Input(0, kEnabled, 1U)).contextValid,
            "T3 timeout preparation frame was rejected");
    require(timeoutGuard.update(t3Input(
                500, static_cast<std::uint8_t>(kEnabled | kActive), 2U))
                .contextValid,
            "T3 timeout positive frame was rejected");
    require(timeoutGuard.update(t3Input(
                -500, static_cast<std::uint8_t>(kEnabled | kActive), 3U))
                .contextValid,
            "T3 timeout negative frame was rejected");
    const CompetitionTaskOutput timeout = timeoutGuard.update(t3Input(
        -500,
        static_cast<std::uint8_t>(kEnabled | kActive | kTimeout),
        4U));
    require(timeout.contextValid
                && timeout.state == CompetitionTaskState::Timeout,
            "frozen T3 timeout frame was rejected");
    require(timeout.runId == 42U && timeout.identityResetCount == 0U,
            "T3 timeout transition changed the run identity");
}

void testT3RejectsNonFrozenControlFlags()
{
    struct InvalidFrame {
        std::int16_t target;
        std::uint8_t flags;
    };
    constexpr std::array<InvalidFrame, 4> invalidFrames{
        InvalidFrame{0,
                     static_cast<std::uint8_t>(kEnabled | kTargetLatched)},
        InvalidFrame{500,
                     static_cast<std::uint8_t>(
                         kEnabled | kActive | kTargetLatched)},
        InvalidFrame{-500,
                     static_cast<std::uint8_t>(
                         kEnabled | kActive | kDone | kTargetLatched)},
        InvalidFrame{-500,
                     static_cast<std::uint8_t>(
                         kEnabled | kActive | kTimeout | kTargetLatched)},
    };

    std::uint8_t sequence = 1U;
    for (const InvalidFrame frame : invalidFrames) {
        CompetitionTaskGuard guard;
        const CompetitionTaskOutput output = guard.update(
            t3Input(frame.target, frame.flags, sequence++));
        require(!output.contextValid,
                "T3 accepted a non-frozen control flag pattern");
        require(output.error == CompetitionTaskError::TaskTargetMismatch,
                "T3 non-frozen flags returned the wrong error");
        require((output.effectiveControlFlags & kEnabled) == 0U,
                "T3 non-frozen flags retained CONTROL_ENABLED");
    }
}

void testTerminalFlagCombinationsAreChecked()
{
    CompetitionTaskGuard conflictGuard;
    const CompetitionTaskOutput conflict = conflictGuard.update(t3Input(
        -500,
        static_cast<std::uint8_t>(kEnabled | kActive | kDone | kTimeout),
        1U));
    require(!conflict.contextValid, "DONE+TIMEOUT was accepted");
    require(conflict.error == CompetitionTaskError::ContradictoryTerminalFlags,
            "DONE+TIMEOUT returned wrong error");

    CompetitionTaskGuard activeGuard;
    const CompetitionTaskOutput withoutActive = activeGuard.update(t3Input(
        -500, static_cast<std::uint8_t>(kEnabled | kDone), 1U));
    require(!withoutActive.contextValid,
            "terminal flag without RUN_ACTIVE was accepted");
    require(withoutActive.error == CompetitionTaskError::TerminalWithoutRunActive,
            "terminal without RUN_ACTIVE returned wrong error");
}

void testNonIdleRequiresRunIdAndControlEnable()
{
    CompetitionTaskGuard runGuard;
    CompetitionTaskInput input = t3Input(0, kEnabled, 1U);
    input.runId = 0U;
    const CompetitionTaskOutput zeroRun = runGuard.update(input);
    require(!zeroRun.contextValid, "T3 accepted zero run_id");
    require(zeroRun.error == CompetitionTaskError::InvalidRunId,
            "zero run_id returned wrong error");

    CompetitionTaskGuard enableGuard;
    const CompetitionTaskOutput disabled = enableGuard.update(t3Input(0, 0U, 1U));
    require(!disabled.contextValid, "T3 accepted missing CONTROL_ENABLED");
    require(disabled.error == CompetitionTaskError::MissingControlEnable,
            "missing CONTROL_ENABLED returned wrong error");
}

void testSameRunRejectsTaskIdentityChange()
{
    CompetitionTaskGuard guard;
    require(guard.update(t3Input(0, kEnabled, 1U)).contextValid,
            "initial T3 identity was rejected");
    CompetitionTaskInput changed = idleInput();
    changed.runId = 42U;
    changed.sequence = 2U;
    const CompetitionTaskOutput output = guard.update(changed);
    require(!output.contextValid, "same run accepted changed task identity");
    require(output.error == CompetitionTaskError::RunIdentityMismatch,
            "changed task identity returned wrong error");
}

void testNewSessionOrRunResetsIdentity()
{
    CompetitionTaskGuard guard;
    require(guard.update(t3Input(0, kEnabled, 1U)).contextValid,
            "initial identity was rejected");

    CompetitionTaskInput newRun = t3Input(0, kEnabled, 2U);
    newRun.runId = 43U;
    const CompetitionTaskOutput runOutput = guard.update(newRun);
    require(runOutput.contextValid, "new run did not reset identity");
    require(runOutput.identityResetCount == 1U,
            "new run reset count mismatch");

    CompetitionTaskInput newSession = idleInput();
    newSession.piSessionId = 8U;
    newSession.runId = 43U;
    newSession.sequence = 3U;
    const CompetitionTaskOutput sessionOutput = guard.update(newSession);
    require(sessionOutput.contextValid, "new session did not allow task change");
    require(sessionOutput.identityResetCount == 2U,
            "new session reset count mismatch");
}

void testInvalidContextClearsOnlyControlEnable()
{
    CompetitionTaskGuard guard;
    const CompetitionTaskInput input = t3Input(
        0, static_cast<std::uint8_t>(kEnabled | kActive), 1U);
    const CompetitionTaskOutput output = guard.update(input);
    require(!output.contextValid, "invalid T3 context was accepted");
    require(output.state == CompetitionTaskState::Invalid,
            "invalid context state mismatch");
    require(output.effectiveControlFlags == kActive,
            "invalid context did not clear only CONTROL_ENABLED");
    require(output.rawControlFlags == input.controlFlags,
            "raw flags diagnostic changed");
    require(output.taskId == input.taskId && output.runId == input.runId
            && output.targetPosition0p1mm == input.targetPosition0p1mm,
            "context diagnostics changed input fields");
}

void testRepeatedBadFrameCountsOneRejection()
{
    CompetitionTaskGuard guard;
    CompetitionTaskInput badFrame = t3Input(
        0, static_cast<std::uint8_t>(kEnabled | kActive), 17U);
    require(guard.update(badFrame).rejectCount == 1U,
            "first bad frame was not counted");
    for (int read = 0; read < 200; ++read) {
        require(guard.update(badFrame).rejectCount == 1U,
                "repeated bad frame was counted again");
    }

    badFrame.sequence = 18U;
    require(guard.update(badFrame).rejectCount == 2U,
            "new bad sequence was not counted");
    badFrame.piSessionId = 8U;
    require(guard.update(badFrame).rejectCount == 3U,
            "same sequence in a new session was not counted");
}

void testT6RequiresLatchAndRange()
{
    constexpr std::array<std::int16_t, 3> boundaryTargets{-1000, 0, 1000};
    for (const std::int16_t target : boundaryTargets) {
        CompetitionTaskGuard guard;
        const CompetitionTaskOutput output = guard.update(t6Input(
            target, static_cast<std::uint8_t>(kEnabled | kTargetLatched), 1U));
        require(output.error != CompetitionTaskError::UnknownTask,
                "T6 returned UnknownTask");
        require(output.contextValid, "latched T6 boundary target was rejected");
        require(output.state == CompetitionTaskState::Preparing,
                "T6 READY state mismatch");
        require(output.targetPosition0p1mm == target,
                "guard changed a T6 boundary target");
    }

    CompetitionTaskGuard latchGuard;
    const CompetitionTaskOutput unlatched = latchGuard.update(
        t6Input(700, kEnabled, 1U));
    require(!unlatched.contextValid, "unlatched T6 was accepted");
    require(unlatched.error == CompetitionTaskError::MissingTargetLatch,
            "unlatched T6 returned wrong error");

    for (const std::int16_t target : {-1001, 1001}) {
        CompetitionTaskGuard rangeGuard;
        const CompetitionTaskOutput output = rangeGuard.update(t6Input(
            target, static_cast<std::uint8_t>(kEnabled | kTargetLatched), 1U));
        require(!output.contextValid, "out-of-range T6 was accepted");
        require(output.error == CompetitionTaskError::TaskTargetMismatch,
                "out-of-range T6 returned wrong error");
    }
}

void testT6KeepsLatchedTargetThroughAllPhases()
{
    constexpr std::int16_t target = 700;
    struct TerminalCase {
        std::uint8_t flag;
        CompetitionTaskState state;
    };
    constexpr std::array<TerminalCase, 2> terminalCases{
        TerminalCase{kDone, CompetitionTaskState::Done},
        TerminalCase{kTimeout, CompetitionTaskState::Timeout},
    };

    for (const TerminalCase terminal : terminalCases) {
        CompetitionTaskGuard guard;
        const CompetitionTaskOutput ready = guard.update(t6Input(
            target,
            static_cast<std::uint8_t>(kEnabled | kTargetLatched),
            1U));
        require(ready.contextValid
                    && ready.state == CompetitionTaskState::Preparing,
                "T6 READY phase rejected the latched target");

        const CompetitionTaskOutput active = guard.update(t6Input(
            target,
            static_cast<std::uint8_t>(
                kEnabled | kTargetLatched | kActive),
            2U));
        require(active.contextValid
                    && active.state == CompetitionTaskState::Active,
                "T6 ACTIVE phase rejected the latched target");

        const std::uint8_t terminalFlags = static_cast<std::uint8_t>(
            kEnabled | kTargetLatched | kActive | terminal.flag);
        const CompetitionTaskOutput finished = guard.update(t6Input(
            target, terminalFlags, 3U));
        require(finished.contextValid && finished.state == terminal.state,
                "T6 terminal phase rejected the latched target");
        require(finished.targetPosition0p1mm == target,
                "guard changed the T6 terminal target");

        const CompetitionTaskOutput changed = guard.update(t6Input(
            static_cast<std::int16_t>(target + 1), terminalFlags, 4U));
        require(!changed.contextValid
                    && changed.error == CompetitionTaskError::TaskTargetMismatch,
                "T6 terminal phase accepted a changed target");
    }
}

void testT6RejectsTargetChangeWithinRun()
{
    CompetitionTaskGuard guard;
    require(guard.update(t6Input(
        700, static_cast<std::uint8_t>(kEnabled | kTargetLatched), 1U)).contextValid,
        "initial T6 target was rejected");

    const CompetitionTaskOutput changed = guard.update(t6Input(
        701,
        static_cast<std::uint8_t>(kEnabled | kTargetLatched | kActive),
        2U));
    require(!changed.contextValid, "T6 target changed within one run");
    require(changed.error == CompetitionTaskError::TaskTargetMismatch,
            "changed T6 target returned wrong error");

    CompetitionTaskInput newRun = t6Input(
        701,
        static_cast<std::uint8_t>(kEnabled | kTargetLatched | kActive),
        3U);
    newRun.runId = 107U;
    require(guard.update(newRun).contextValid,
            "new T6 run could not latch a new target");
}

}

int main()
{
    using TestFunction = void (*)();
    constexpr std::array<TestFunction, 19> tests{
        testIdleContextIsSafeAndValid,
        testIdleRejectsNonzeroTargetOrFlags,
        testUnknownTaskIsRejectedFirst,
        testCenterHoldTasksRequireZeroTargetInEveryPhase,
        testT4T5UseExactReadyAndActiveFramesAndRejectLatch,
        testT3PreparationAndActiveTargets,
        testT3RejectsWrongPhaseTargets,
        testT3TerminalTargetsAndStates,
        testT3FrozenFrameSequenceKeepsOneRunIdentity,
        testT3RejectsNonFrozenControlFlags,
        testTerminalFlagCombinationsAreChecked,
        testNonIdleRequiresRunIdAndControlEnable,
        testSameRunRejectsTaskIdentityChange,
        testNewSessionOrRunResetsIdentity,
        testInvalidContextClearsOnlyControlEnable,
        testRepeatedBadFrameCountsOneRejection,
        testT6RequiresLatchAndRange,
        testT6KeepsLatchedTargetThroughAllPhases,
        testT6RejectsTargetChangeWithinRun,
    };
    try {
        for (const TestFunction test : tests) {
            test();
        }
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "PASS: " << tests.size()
              << " competition task guard tests\n";
    return 0;
}
