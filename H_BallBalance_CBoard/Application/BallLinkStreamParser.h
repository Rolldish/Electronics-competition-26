#ifndef APPLICATION_BALL_LINK_STREAM_PARSER_H
#define APPLICATION_BALL_LINK_STREAM_PARSER_H

#include "BallLinkProtocol.h"

#include <array>
#include <cstddef>
#include <cstdint>

enum class BallLinkParseStatus : std::uint8_t {
    NeedMoreData = 0U,
    ByteDiscarded = 1U,
    CrcError = 2U,
    FrameReady = 3U,
};

class BallLinkStreamParser {
public:
    static constexpr std::size_t kCapacity = 160U;

    std::size_t append(
        const std::uint8_t *data, std::size_t length);
    std::size_t append(
        const std::uint8_t *data,
        const std::uint32_t *arrivalMs,
        std::size_t length);
    BallLinkParseStatus next(BallLinkFrame &output);
    BallLinkParseStatus next(
        BallLinkFrame &output, std::uint32_t &frameArrivalMs);
    std::size_t bufferedSize() const;
    void clear();

private:
    void consume(std::size_t count);

    std::array<std::uint8_t, kCapacity> buffer_{};
    std::array<std::uint32_t, kCapacity> arrival_ms_{};
    std::size_t size_ = 0U;
};

#endif /* APPLICATION_BALL_LINK_STREAM_PARSER_H */
