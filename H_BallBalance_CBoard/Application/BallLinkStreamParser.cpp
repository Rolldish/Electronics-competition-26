#include "BallLinkStreamParser.h"

#include <cstring>

std::size_t BallLinkStreamParser::append(
    const std::uint8_t *data, std::size_t length) {
    return append(data, nullptr, length);
}

std::size_t BallLinkStreamParser::append(
    const std::uint8_t *data,
    const std::uint32_t *arrivalMs,
    std::size_t length) {
    if (data == nullptr || length == 0U) {
        return 0U;
    }

    std::size_t dropped = 0U;
    if (length > buffer_.size()) {
        dropped += size_ + length - buffer_.size();
        if (arrivalMs != nullptr) {
            arrivalMs += length - buffer_.size();
        }
        data += length - buffer_.size();
        length = buffer_.size();
        size_ = 0U;
    } else if (size_ + length > buffer_.size()) {
        dropped = size_ + length - buffer_.size();
        consume(dropped);
    }

    std::memcpy(buffer_.data() + size_, data, length);
    if (arrivalMs != nullptr) {
        std::memcpy(
            arrival_ms_.data() + size_, arrivalMs,
            length * sizeof(arrival_ms_[0]));
    } else {
        std::memset(
            arrival_ms_.data() + size_, 0,
            length * sizeof(arrival_ms_[0]));
    }
    size_ += length;
    return dropped;
}

BallLinkParseStatus BallLinkStreamParser::next(
    BallLinkFrame &output) {
    std::uint32_t ignoredArrivalMs = 0U;
    return next(output, ignoredArrivalMs);
}

BallLinkParseStatus BallLinkStreamParser::next(
    BallLinkFrame &output, std::uint32_t &frameArrivalMs) {
    if (size_ == 0U) {
        return BallLinkParseStatus::NeedMoreData;
    }
    if (buffer_[0] != kBallLinkMagic0) {
        consume(1U);
        return BallLinkParseStatus::ByteDiscarded;
    }
    if (size_ < 2U) {
        return BallLinkParseStatus::NeedMoreData;
    }
    if (buffer_[1] != kBallLinkMagic1) {
        consume(1U);
        return BallLinkParseStatus::ByteDiscarded;
    }
    if (size_ < 6U) {
        return BallLinkParseStatus::NeedMoreData;
    }

    const std::uint8_t payloadLength = buffer_[5U];
    if (buffer_[2U] != kBallLinkVersion ||
        payloadLength > kBallLinkMaxPayload) {
        consume(1U);
        return BallLinkParseStatus::ByteDiscarded;
    }

    const std::size_t frameLength =
        kBallLinkFixedSize + payloadLength;
    if (size_ < frameLength) {
        return BallLinkParseStatus::NeedMoreData;
    }

    const std::size_t crcOffset = 6U + payloadLength;
    const std::uint16_t receivedCrc =
        static_cast<std::uint16_t>(buffer_[crcOffset]) |
        (static_cast<std::uint16_t>(
             buffer_[crcOffset + 1U])
         << 8U);
    const std::uint16_t calculatedCrc =
        ballLinkCrc16CcittFalse(
            buffer_.data() + 2U, 4U + payloadLength);
    if (receivedCrc != calculatedCrc) {
        consume(1U);
        return BallLinkParseStatus::CrcError;
    }

    output = {};
    output.type = buffer_[3U];
    output.sequence = buffer_[4U];
    output.length = payloadLength;
    if (payloadLength > 0U) {
        std::memcpy(
            output.payload.data(), buffer_.data() + 6U,
            payloadLength);
    }
    frameArrivalMs = arrival_ms_[frameLength - 1U];
    consume(frameLength);
    return BallLinkParseStatus::FrameReady;
}

std::size_t BallLinkStreamParser::bufferedSize() const {
    return size_;
}

void BallLinkStreamParser::clear() {
    size_ = 0U;
}

void BallLinkStreamParser::consume(const std::size_t count) {
    if (count >= size_) {
        size_ = 0U;
        return;
    }
    std::memmove(
        buffer_.data(), buffer_.data() + count,
        size_ - count);
    std::memmove(
        arrival_ms_.data(), arrival_ms_.data() + count,
        (size_ - count) * sizeof(arrival_ms_[0]));
    size_ -= count;
}
