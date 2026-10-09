#pragma once

#include <cstdint>

enum class ImuRuntimeTimeout : std::uint8_t {
    None = 0U,
    Transfer = 1U,
    Sample = 2U,
};

class ImuRuntimeMonitor {
public:
    static constexpr std::uint32_t kTransferTimeoutMs = 20U;
    static constexpr std::uint32_t kSampleTimeoutMs = 100U;

    void reset(std::uint32_t nowMs);
    void transferStarted(std::uint32_t nowMs);
    void transferFinished();
    void sampleReceived(std::uint32_t nowMs);
    [[nodiscard]] ImuRuntimeTimeout poll(std::uint32_t nowMs) const;

private:
    volatile std::uint32_t lastSampleMs_{};
    volatile std::uint32_t transferStartedMs_{};
    volatile bool transferActive_{};
};
