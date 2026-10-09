#include "ImuVehicleSnapshotStore.h"

#include <atomic>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {

void require(const bool condition, const char* message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testDefaultSnapshotIsReadableAndSafe()
{
    ImuVehicleSnapshotStore store;
    ImuVehicleControlSnapshot snapshot{
        .status = 99U,
        .calibrated = true,
        .valid = true,
        .sampleCount = 99U,
        .forwardAccelerationMps2 = 99.0F,
    };
    require(store.tryLoad(snapshot), "default snapshot was not readable");
    require(snapshot.status == 0U, "default status was not zero");
    require(!snapshot.calibrated, "default snapshot was calibrated");
    require(!snapshot.valid, "default snapshot was valid");
    require(snapshot.sampleCount == 0U, "default sample count was not zero");
    require(snapshot.forwardAccelerationMps2 == 0.0F,
            "default acceleration was not zero");
}

void testPublishAndLoadExactSnapshot()
{
    ImuVehicleSnapshotStore store;
    const ImuVehicleControlSnapshot expected{
        .status = 1U,
        .calibrated = true,
        .valid = true,
        .sampleCount = 42U,
        .forwardAccelerationMps2 = -0.75F,
    };
    store.publish(expected);

    ImuVehicleControlSnapshot actual{};
    require(store.tryLoad(actual), "stable snapshot was not readable");
    require(actual.status == expected.status, "status was torn");
    require(actual.calibrated == expected.calibrated,
            "calibration was torn");
    require(actual.valid == expected.valid, "validity was torn");
    require(actual.sampleCount == expected.sampleCount, "count was torn");
    require(actual.forwardAccelerationMps2
                == expected.forwardAccelerationMps2,
            "acceleration was torn");
}

void testInvalidPublicationPreservesOneGeneration()
{
    ImuVehicleSnapshotStore store;
    store.publish(ImuVehicleControlSnapshot{
        .status = 2U,
        .calibrated = true,
        .valid = false,
        .sampleCount = 77U,
        .forwardAccelerationMps2 = 1.25F,
    });

    ImuVehicleControlSnapshot snapshot{};
    require(store.tryLoad(snapshot), "invalid snapshot was not readable");
    require(snapshot.status == 2U && snapshot.calibrated,
            "invalid publication changed status or calibration");
    require(!snapshot.valid, "invalid publication became valid");
    require(snapshot.sampleCount == 77U
                && snapshot.forwardAccelerationMps2 == 1.25F,
            "invalid publication mixed generations");
}

void testConcurrentSnapshotsAreCoherentAndReadDoesNotSpin()
{
    ImuVehicleSnapshotStore store;
    std::atomic<bool> writerDone{false};
    std::thread writer([&]() {
        for (std::uint32_t count = 1U; count <= 100000U; ++count) {
            store.publish(ImuVehicleControlSnapshot{
                .status = 1U,
                .calibrated = true,
                .valid = true,
                .sampleCount = count,
                .forwardAccelerationMps2 = static_cast<float>(count),
            });
        }
        writerDone.store(true, std::memory_order_release);
    });

    std::uint32_t readAttempts = 0U;
    while (!writerDone.load(std::memory_order_acquire)) {
        ImuVehicleControlSnapshot snapshot{};
        ++readAttempts;
        if (store.tryLoad(snapshot) && snapshot.sampleCount != 0U) {
            require(snapshot.forwardAccelerationMps2
                        == static_cast<float>(snapshot.sampleCount),
                    "reader accepted a mixed-generation snapshot");
        }
    }
    writer.join();
    require(readAttempts != 0U, "reader made no non-blocking attempts");
}

} // namespace

int main()
{
    try {
        testDefaultSnapshotIsReadableAndSafe();
        testPublishAndLoadExactSnapshot();
        testInvalidPublicationPreservesOneGeneration();
        testConcurrentSnapshotsAreCoherentAndReadDoesNotSpin();
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "PASS: 4 IMU vehicle snapshot store tests\n";
    return 0;
}
