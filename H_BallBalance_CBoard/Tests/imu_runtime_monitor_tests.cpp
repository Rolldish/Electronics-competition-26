#include "ImuRuntimeMonitor.h"

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {

void require(const bool condition, const char *message)
{
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void testHealthySamplesRemainOnline()
{
    ImuRuntimeMonitor monitor;
    monitor.reset(100U);
    monitor.sampleReceived(180U);
    require(monitor.poll(260U) == ImuRuntimeTimeout::None,
            "fresh IMU samples were reported as stale");
}

void testMissingSamplesTimeout()
{
    ImuRuntimeMonitor monitor;
    monitor.reset(100U);
    require(monitor.poll(201U) == ImuRuntimeTimeout::Sample,
            "missing IMU samples did not time out");
}

void testStuckTransferTakesPriority()
{
    ImuRuntimeMonitor monitor;
    monitor.reset(100U);
    monitor.transferStarted(150U);
    require(monitor.poll(171U) == ImuRuntimeTimeout::Transfer,
            "stuck DMA transfer did not time out");
    monitor.transferFinished();
    monitor.sampleReceived(171U);
    require(monitor.poll(171U) == ImuRuntimeTimeout::None,
            "completed transfer remained timed out");
}

void testClockRolloverIsHandled()
{
    ImuRuntimeMonitor monitor;
    constexpr std::uint32_t start =
        std::numeric_limits<std::uint32_t>::max() - 10U;
    monitor.reset(start);
    monitor.transferStarted(start);
    require(monitor.poll(11U) == ImuRuntimeTimeout::Transfer,
            "DMA timeout across HAL tick rollover was missed");
}

} // namespace

int main()
{
    try {
        testHealthySamplesRemainOnline();
        testMissingSamplesTimeout();
        testStuckTransferTakesPriority();
        testClockRolloverIsHandled();
    } catch (const std::exception& exception) {
        std::cerr << "FAIL: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "PASS: IMU runtime monitor tests\n";
    return 0;
}
