#include "ImuRuntimeMonitor.h"

void ImuRuntimeMonitor::reset(const std::uint32_t nowMs)
{
    lastSampleMs_ = nowMs;
    transferStartedMs_ = nowMs;
    transferActive_ = false;
}

void ImuRuntimeMonitor::transferStarted(const std::uint32_t nowMs)
{
    transferStartedMs_ = nowMs;
    transferActive_ = true;
}

void ImuRuntimeMonitor::transferFinished()
{
    transferActive_ = false;
}

void ImuRuntimeMonitor::sampleReceived(const std::uint32_t nowMs)
{
    lastSampleMs_ = nowMs;
}

ImuRuntimeTimeout ImuRuntimeMonitor::poll(const std::uint32_t nowMs) const
{
    if (transferActive_
        && (nowMs - transferStartedMs_) > kTransferTimeoutMs) {
        return ImuRuntimeTimeout::Transfer;
    }
    if ((nowMs - lastSampleMs_) > kSampleTimeoutMs) {
        return ImuRuntimeTimeout::Sample;
    }
    return ImuRuntimeTimeout::None;
}
