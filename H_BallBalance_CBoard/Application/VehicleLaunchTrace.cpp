#include "VehicleLaunchTrace.h"

#include <atomic>

namespace {
constexpr std::uint8_t kControlEnabled = 1U;
constexpr std::uint8_t kTerminalFlags = (1U << 2U) | (1U << 3U);

bool supportsTrace(const std::uint8_t taskId)
{
    return taskId == 4U || taskId == 5U || taskId == 6U;
}

void beginWrite(VehicleLaunchTraceHeader& header)
{
    header.generation = header.generation + 1U;
    std::atomic_signal_fence(std::memory_order_seq_cst);
}

void endWrite(VehicleLaunchTraceHeader& header)
{
    std::atomic_signal_fence(std::memory_order_seq_cst);
    header.generation = header.generation + 1U;
}
}

extern "C" {
VehicleLaunchTraceStorage vehicle_launch_trace{};
}

VehicleLaunchTraceRecorder::VehicleLaunchTraceRecorder(
    VehicleLaunchTraceStorage& storage)
    : storage_(storage)
{
    // Reset only the 80-byte header. Old array entries outside count are never
    // published; clearing 64 KB in the 200 Hz loop is unnecessary.
    storage_.header = VehicleLaunchTraceHeader{
        .magic = kVehicleLaunchTraceMagic,
        .version = 1U,
        .headerBytes = sizeof(VehicleLaunchTraceHeader),
        .recordBytes = sizeof(VehicleLaunchTraceRecord),
        .capacity = kVehicleLaunchTraceCapacity,
        .samplePeriodMs = kVehicleLaunchTracePeriodMs,
    };
}

void VehicleLaunchTraceRecorder::update(const VehicleLaunchTraceInput& input)
{
    auto& header = storage_.header;
    const bool terminal = (input.record.rawControlFlags & kTerminalFlags) != 0U;
    const bool sameIdentity = header.piSessionId == input.piSessionId
        && header.runId == input.runId && header.taskId == input.taskId;
    const bool newRun = supportsTrace(input.taskId)
        && (input.record.rawControlFlags & kControlEnabled) != 0U
        && !terminal
        && (!sameIdentity || header.state == static_cast<std::uint32_t>(
                VehicleLaunchTraceState::Idle));

    if (!newRun && header.state != static_cast<std::uint32_t>(
            VehicleLaunchTraceState::Recording)) {
        return;
    }
    if (!newRun && !sameIdentity) {
        beginWrite(header);
        header.state = static_cast<std::uint32_t>(VehicleLaunchTraceState::Frozen);
        header.endMs = input.record.nowMs;
        header.triggerCountAtEnd = input.record.launchTriggerCount;
        header.freezeReason = static_cast<std::uint32_t>(
            VehicleLaunchTraceFreezeReason::ContextChanged);
        endWrite(header);
        return;
    }
    if (!newRun && !terminal && header.count != 0U
        && input.record.nowMs - lastSampleMs_ < kVehicleLaunchTracePeriodMs) {
        return;
    }

    beginWrite(header);
    if (newRun) {
        header.state = static_cast<std::uint32_t>(VehicleLaunchTraceState::Recording);
        header.count = 0U;
        header.piSessionId = input.piSessionId;
        header.runId = input.runId;
        header.taskId = input.taskId;
        header.startMs = input.record.nowMs;
        header.triggerCountAtStart = input.record.launchTriggerCount;
        header.levelAngleRad = input.levelAngleRad;
        header.freezeReason = static_cast<std::uint32_t>(
            VehicleLaunchTraceFreezeReason::None);
        header.initialRawControlFlags = input.record.rawControlFlags;
    }

    storage_.records[header.count] = input.record;
    header.count = header.count + 1U;
    lastSampleMs_ = input.record.nowMs;
    header.endMs = input.record.nowMs;
    header.triggerCountAtEnd = input.record.launchTriggerCount;
    header.calibrationNoiseMps2 = input.calibrationNoiseMps2;
    header.deadbandMps2 = input.deadbandMps2;
    if (terminal || header.count >= kVehicleLaunchTraceCapacity) {
        header.state = static_cast<std::uint32_t>(VehicleLaunchTraceState::Frozen);
        header.freezeReason = static_cast<std::uint32_t>(terminal
            ? VehicleLaunchTraceFreezeReason::Terminal
            : VehicleLaunchTraceFreezeReason::Capacity);
    }
    endWrite(header);
}
