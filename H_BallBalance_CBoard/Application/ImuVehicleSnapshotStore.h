#pragma once

#include <atomic>
#include <cstdint>

struct ImuVehicleControlSnapshot {
    std::uint32_t status{};
    bool calibrated{};
    bool valid{};
    std::uint32_t sampleCount{};
    float forwardAccelerationMps2{};
};

class ImuVehicleSnapshotStore {
public:
    void publish(const ImuVehicleControlSnapshot& snapshot);
    [[nodiscard]] bool tryLoad(ImuVehicleControlSnapshot& snapshot) const;

private:
    std::atomic<std::uint32_t> generation_{0U};
    std::atomic<std::uint32_t> status_{0U};
    std::atomic<std::uint32_t> calibrated_{0U};
    std::atomic<std::uint32_t> valid_{0U};
    std::atomic<std::uint32_t> sampleCount_{0U};
    std::atomic<std::uint32_t> accelerationBits_{0U};
};

void ImuVehicle_PublishControlSnapshot(
    const ImuVehicleControlSnapshot& snapshot);
[[nodiscard]] bool ImuVehicle_TryGetControlSnapshot(
    ImuVehicleControlSnapshot& snapshot);
