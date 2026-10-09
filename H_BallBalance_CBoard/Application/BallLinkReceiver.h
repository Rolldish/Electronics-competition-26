#ifndef APPLICATION_BALL_LINK_RECEIVER_H
#define APPLICATION_BALL_LINK_RECEIVER_H

#include "BallLinkProtocol.h"
#include "VisionLink.h"

#include <cstdint>

inline constexpr std::uint32_t
    kBallLinkFreshMeasurementAgeMs = 80U;
inline constexpr std::uint32_t kBallLinkTimeoutMs = 110U;
inline constexpr std::uint8_t kBallLinkMinimumConfidence = 180U;

static_assert(
    kBallLinkFreshMeasurementAgeMs < kBallLinkTimeoutMs);

enum class BallLinkReceiveResult : std::uint8_t {
    ProtocolError,
    RangeError,
    TimestampError,
    DuplicateSequence,
    OutOfOrderSequence,
    LegalFrame,
    NewMeasurement,
};

class BallLinkReceiver {
public:
    BallLinkReceiveResult process(
        const BallLinkFrame &frame, std::uint32_t nowMs);
    void refresh(std::uint32_t nowMs);
    void invalidateLink();
    [[nodiscard]] VisionLinkSnapshot snapshot(
        std::uint32_t nowMs) const;

    [[nodiscard]] BallLinkVisionState state() const {
        return state_;
    }
    [[nodiscard]] std::uint8_t missingFrameCount() const {
        return missing_frame_count_;
    }
    [[nodiscard]] std::uint32_t measurementUpdateCount() const {
        return measurement_update_count_;
    }
    [[nodiscard]] std::uint32_t sessionChangeCount() const {
        return session_change_count_;
    }
    [[nodiscard]] std::uint32_t runChangeCount() const {
        return run_change_count_;
    }

private:
    static bool fieldsAreInRange(
        const BallLinkVisionMeasurement &measurement);
    static bool measurementIsAcceptable(
        const BallLinkVisionMeasurement &measurement,
        std::uint32_t baseAgeMs);
    static std::uint32_t measurementBaseAgeMs(
        const BallLinkVisionMeasurement &measurement);
    static std::uint32_t saturatedAdd(
        std::uint32_t left, std::uint32_t right);

    void clearMeasurementHistory();
    void resetSession(std::uint32_t sessionId);
    void updateState(std::uint32_t nowMs);
    [[nodiscard]] std::uint32_t measurementAgeMs(
        std::uint32_t nowMs) const;

    VisionLinkSnapshot snapshot_{};
    BallLinkVisionState state_{BallLinkVisionState::NoLink};
    std::uint32_t session_id_{0U};
    std::uint32_t last_capture_timestamp_ms_{0U};
    std::uint32_t last_legal_frame_ms_{0U};
    std::uint32_t measurement_receive_ms_{0U};
    std::uint32_t measurement_base_age_ms_{0U};
    std::uint32_t measurement_update_count_{0U};
    std::uint32_t session_change_count_{0U};
    std::uint32_t run_change_count_{0U};
    std::uint16_t run_id_{0U};
    std::uint8_t last_sequence_{0U};
    std::uint8_t missing_frame_count_{0U};
    bool have_session_{false};
    bool have_run_{false};
    bool have_sequence_{false};
    bool have_capture_timestamp_{false};
    bool have_legal_frame_{false};
    bool have_measurement_{false};
};

#endif /* APPLICATION_BALL_LINK_RECEIVER_H */
