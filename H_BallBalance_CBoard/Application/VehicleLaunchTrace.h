#pragma once

#include <array>
#include <cstdint>

// Fixed binary format read through SWD after a trial, without a probe attached
// during motion. No flash writes, heap allocation or serial traffic.
inline constexpr std::uint32_t kVehicleLaunchTraceMagic = 0x52544248U;
inline constexpr std::uint32_t kVehicleLaunchTraceCapacity = 1000U;
inline constexpr std::uint32_t kVehicleLaunchTracePeriodMs = 40U;

enum class VehicleLaunchTraceState : std::uint32_t {
    Idle,
    Recording,
    Frozen,
};

enum class VehicleLaunchTraceFreezeReason : std::uint32_t {
    None,
    Capacity,
    Terminal,
    ContextChanged,
};

struct VehicleLaunchTraceHeader {
    std::uint32_t magic{};
    std::uint16_t version{};
    std::uint16_t headerBytes{};
    std::uint32_t recordBytes{};
    std::uint32_t capacity{};
    std::uint32_t samplePeriodMs{};
    volatile std::uint32_t generation{};
    std::uint32_t state{};
    std::uint32_t count{};
    std::uint32_t piSessionId{};
    std::uint32_t runId{};
    std::uint32_t taskId{};
    std::uint32_t startMs{};
    std::uint32_t endMs{};
    std::uint32_t triggerCountAtStart{};
    std::uint32_t triggerCountAtEnd{};
    float levelAngleRad{};
    float calibrationNoiseMps2{};
    float deadbandMps2{};
    std::uint32_t freezeReason{};
    std::uint32_t initialRawControlFlags{};
};

struct VehicleLaunchTraceRecord {
    std::uint32_t nowMs{};
    std::uint32_t imuSampleCount{};
    std::uint32_t launchTriggerCount{};
    float accelerationMps2{};
    float rawAccelerationMps2{};
    float actualAngleRad{};
    float desiredAngleRad{};
    float sentAngleRad{};
    float pdOffsetDeg{};
    float imuOffsetDeg{};
    float launchOffsetDeg{};
    float totalOffsetDeg{};
    std::int16_t position0p1mm{};
    std::int16_t velocityMmps{};
    std::uint16_t visionAgeMs{};
    std::uint16_t imuAgeMs{};
    std::uint8_t ballState{};
    std::uint8_t ballFault{};
    std::uint8_t feedforwardGate{};
    std::uint8_t launchState{};
    std::uint8_t rawControlFlags{};
    std::uint8_t effectiveControlFlags{};
    // bits: vision valid, IMU valid, calibrated, fresh IMU, motor enabled,
    // angle sent this cycle, valid task context, active launch offset.
    std::uint8_t validityFlags{};
    std::uint8_t reserved{};
};

struct VehicleLaunchTraceStorage {
    VehicleLaunchTraceHeader header{};
    std::array<VehicleLaunchTraceRecord, kVehicleLaunchTraceCapacity> records{};
};

static_assert(sizeof(VehicleLaunchTraceHeader) == 80U);
static_assert(sizeof(VehicleLaunchTraceRecord) == 64U);
static_assert(sizeof(VehicleLaunchTraceStorage) == 64080U);

struct VehicleLaunchTraceInput {
    std::uint32_t piSessionId{};
    std::uint16_t runId{};
    std::uint8_t taskId{};
    float levelAngleRad{};
    float calibrationNoiseMps2{};
    float deadbandMps2{};
    VehicleLaunchTraceRecord record{};
};

class VehicleLaunchTraceRecorder {
public:
    explicit VehicleLaunchTraceRecorder(VehicleLaunchTraceStorage& storage);
    void update(const VehicleLaunchTraceInput& input);

private:
    VehicleLaunchTraceStorage& storage_;
    std::uint32_t lastSampleMs_{};
};

extern "C" VehicleLaunchTraceStorage vehicle_launch_trace;
