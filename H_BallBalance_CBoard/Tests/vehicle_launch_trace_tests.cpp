#include "VehicleLaunchTrace.h"

#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace {
VehicleLaunchTraceStorage storage{};

void require(const bool ok, const char* message)
{
    if (!ok) {
        throw std::runtime_error(message);
    }
}

VehicleLaunchTraceInput inputAt(const std::uint32_t nowMs = 1000U)
{
    VehicleLaunchTraceInput input{};
    input.piSessionId = 17U;
    input.runId = 9U;
    input.taskId = 4U;
    input.levelAngleRad = 1.2461F;
    input.calibrationNoiseMps2 = 0.03F;
    input.deadbandMps2 = 0.09F;
    input.record.nowMs = nowMs;
    input.record.rawControlFlags = 0x01U;
    input.record.effectiveControlFlags = 0x01U;
    input.record.accelerationMps2 = 0.25F;
    input.record.actualAngleRad = 1.24F;
    input.record.desiredAngleRad = 1.20F;
    input.record.sentAngleRad = 1.23F;
    return input;
}

void testCapturesBeforePiStart()
{
    VehicleLaunchTraceRecorder recorder(storage);
    const auto input = inputAt();
    recorder.update(input);
    require(storage.header.count == 1U, "READY data was not recorded");
    require(storage.header.taskId == 4U, "task identity was lost");
    require(storage.records[0].desiredAngleRad == 1.20F
        && storage.records[0].sentAngleRad == 1.23F,
        "desired and actually sent commands must remain distinct");
    require(storage.header.magic == kVehicleLaunchTraceMagic
        && storage.header.version == 1U
        && storage.header.headerBytes == 80U
        && storage.header.recordBytes == 64U,
        "trace binary format is not identifiable");
    require(storage.header.deadbandMps2 == input.deadbandMps2,
        "calibration context was not retained");
    require((storage.header.generation & 1U) == 0U,
        "trace publication was left incomplete");
}

void testUsesBoundedSamplingWithoutBackfilling()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    recorder.update(input);
    input.record.nowMs = 1039U;
    recorder.update(input);
    require(storage.header.count == 1U, "captured faster than configured");
    input.record.nowMs = 1040U;
    recorder.update(input);
    input.record.nowMs = 5000U;
    recorder.update(input);
    require(storage.header.count == 3U, "gap was filled with invented samples");
}

void testTerminalFreezesAndPreservesEvidence()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    recorder.update(input);
    input.record.nowMs = 1005U;
    input.record.rawControlFlags = 0x07U;
    input.record.launchTriggerCount = 1U;
    recorder.update(input);
    require(storage.header.count == 2U,
        "terminal transition inside sample period was missed");
    require(storage.header.state == static_cast<std::uint32_t>(
        VehicleLaunchTraceState::Frozen), "terminal did not freeze trace");
    require(storage.header.triggerCountAtEnd == 1U,
        "final cumulative trigger count was lost");
    const auto generation = storage.header.generation;
    input.record.nowMs = 2000U;
    input.record.launchTriggerCount = 5U;
    recorder.update(input);
    require(storage.header.count == 2U
        && storage.header.generation == generation,
        "finished trace was mutated while waiting to reconnect ST-Link");
}

void testTimeoutFreezes()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    recorder.update(input);
    input.record.rawControlFlags = 0x0bU;
    recorder.update(input);
    require(storage.header.freezeReason == static_cast<std::uint32_t>(
        VehicleLaunchTraceFreezeReason::Terminal), "timeout did not freeze");
}

void testCapacityFreezesWithoutOverwrite()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    for (std::uint32_t index = 0U; index < kVehicleLaunchTraceCapacity + 20U;
         ++index) {
        input.record.nowMs = 1000U + index * kVehicleLaunchTracePeriodMs;
        input.record.position0p1mm = static_cast<std::int16_t>(index);
        recorder.update(input);
    }
    require(storage.header.count == kVehicleLaunchTraceCapacity,
        "trace exceeded static capacity");
    require(storage.records.front().position0p1mm == 0
        && storage.records.back().position0p1mm == 999,
        "late reconnect overwrote launch data");
    require(storage.header.freezeReason == static_cast<std::uint32_t>(
        VehicleLaunchTraceFreezeReason::Capacity), "capacity did not freeze");
}

void testUnsupportedTaskKeepsPastTrace()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    recorder.update(input);
    input.taskId = 3U;
    input.record.nowMs = 1040U;
    recorder.update(input);
    require(storage.header.taskId == 4U && storage.header.count == 1U,
        "T3 erased the preceding moving-task trace");
    require(storage.header.freezeReason == static_cast<std::uint32_t>(
        VehicleLaunchTraceFreezeReason::ContextChanged),
        "different task did not preserve the previous run");
}

void testNewRunReplacesTraceWithoutStaleRows()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    recorder.update(input);
    input.runId = 10U;
    input.record.nowMs = 2000U;
    input.record.accelerationMps2 = 0.75F;
    recorder.update(input);
    require(storage.header.runId == 10U && storage.header.count == 1U
        && storage.records[0].accelerationMps2 == 0.75F,
        "new run mixed records with old identity");
    input.piSessionId = 18U;
    input.taskId = 6U;
    input.record.nowMs = 2040U;
    recorder.update(input);
    require(storage.header.piSessionId == 18U && storage.header.taskId == 6U
        && storage.header.count == 1U, "session/task change was ignored");
}

void testContextChangeRetainsTriggerBetweenSamples()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    recorder.update(input);
    input.record.nowMs = 1005U;
    input.record.launchTriggerCount = 1U;
    recorder.update(input);
    input.taskId = 3U;
    input.record.nowMs = 1010U;
    recorder.update(input);
    require(storage.header.triggerCountAtEnd == 1U
        && storage.header.endMs == 1010U,
        "context change lost a launch trigger between diagnostic samples");
    require(storage.header.count == 1U && storage.header.taskId == 4U,
        "context boundary mixed T3 data into T4 records");
}

void testRecordsRejectedControlAndMissingVision()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    input.record.effectiveControlFlags = 0U;
    input.record.feedforwardGate = 2U;
    input.record.ballState = 6U;
    input.record.visionAgeMs = 160U;
    recorder.update(input);
    require(storage.header.count == 1U
        && storage.records[0].feedforwardGate == 2U
        && storage.records[0].visionAgeMs == 160U,
        "the rejection that prevents launch was not captured");
    input.record.nowMs = 1040U;
    input.record.rawControlFlags = 0U;
    recorder.update(input);
    require(storage.header.count == 2U,
        "temporary control loss ended diagnostic history prematurely");
}

void testMillisWrap()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt(0xfffffff0U);
    recorder.update(input);
    input.record.nowMs = 23U;
    recorder.update(input);
    require(storage.header.count == 1U, "wrap produced a false interval");
    input.record.nowMs = 24U;
    recorder.update(input);
    require(storage.header.count == 2U, "wrap stopped trace sampling");
}

void testOtherTasksAndOldTerminalDoNotStart()
{
    VehicleLaunchTraceRecorder recorder(storage);
    auto input = inputAt();
    input.taskId = 3U;
    recorder.update(input);
    input.taskId = 4U;
    input.record.rawControlFlags = 0x07U;
    recorder.update(input);
    require(storage.header.count == 0U, "non-running task created a trace");
}
}

int main()
{
    const struct { const char* name; void (*run)(); } tests[] = {
        {"captures READY before Pi START", testCapturesBeforePiStart},
        {"bounded sampling", testUsesBoundedSamplingWithoutBackfilling},
        {"terminal preservation", testTerminalFreezesAndPreservesEvidence},
        {"timeout", testTimeoutFreezes},
        {"capacity without overwrite", testCapacityFreezesWithoutOverwrite},
        {"other task preserves evidence", testUnsupportedTaskKeepsPastTrace},
        {"new run identity", testNewRunReplacesTraceWithoutStaleRows},
        {"context boundary trigger", testContextChangeRetainsTriggerBetweenSamples},
        {"rejected control is recorded", testRecordsRejectedControlAndMissingVision},
        {"millisecond wrap", testMillisWrap},
        {"irrelevant/old terminal task", testOtherTasksAndOldTerminalDoNotStart},
    };
    for (const auto& test : tests) {
        try {
            test.run();
            std::cout << "PASS: " << test.name << '\n';
        } catch (const std::exception& error) {
            std::cerr << "FAIL: " << test.name << ": " << error.what() << '\n';
            return 1;
        }
    }
}
