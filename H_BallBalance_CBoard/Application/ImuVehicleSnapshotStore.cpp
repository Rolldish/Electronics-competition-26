#include "ImuVehicleSnapshotStore.h"

#include <bit>

namespace {
ImuVehicleSnapshotStore controlSnapshotStore;
}

static_assert(std::atomic<std::uint32_t>::is_always_lock_free);

void ImuVehicleSnapshotStore::publish(
    const ImuVehicleControlSnapshot& snapshot)
{
    generation_.fetch_add(1U, std::memory_order_acq_rel);
    status_.store(snapshot.status, std::memory_order_relaxed);
    calibrated_.store(snapshot.calibrated ? 1U : 0U,
                      std::memory_order_relaxed);
    valid_.store(snapshot.valid ? 1U : 0U, std::memory_order_relaxed);
    sampleCount_.store(snapshot.sampleCount, std::memory_order_relaxed);
    accelerationBits_.store(
        std::bit_cast<std::uint32_t>(snapshot.forwardAccelerationMps2),
        std::memory_order_relaxed);
    generation_.fetch_add(1U, std::memory_order_release);
}

bool ImuVehicleSnapshotStore::tryLoad(
    ImuVehicleControlSnapshot& snapshot) const
{
    const std::uint32_t before = generation_.load(std::memory_order_acquire);
    if ((before & 1U) != 0U) {
        return false;
    }

    ImuVehicleControlSnapshot candidate{};
    candidate.status = status_.load(std::memory_order_relaxed);
    candidate.calibrated = calibrated_.load(std::memory_order_relaxed) != 0U;
    candidate.valid = valid_.load(std::memory_order_relaxed) != 0U;
    candidate.sampleCount = sampleCount_.load(std::memory_order_relaxed);
    candidate.forwardAccelerationMps2 = std::bit_cast<float>(
        accelerationBits_.load(std::memory_order_relaxed));

    const std::uint32_t after = generation_.load(std::memory_order_acquire);
    if (before != after || (after & 1U) != 0U) {
        return false;
    }
    snapshot = candidate;
    return true;
}

void ImuVehicle_PublishControlSnapshot(
    const ImuVehicleControlSnapshot& snapshot)
{
    controlSnapshotStore.publish(snapshot);
}

bool ImuVehicle_TryGetControlSnapshot(ImuVehicleControlSnapshot& snapshot)
{
    return controlSnapshotStore.tryLoad(snapshot);
}
