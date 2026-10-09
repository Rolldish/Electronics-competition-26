#include "BallLinkProtocol.h"

#include <cstring>

namespace {

std::uint16_t decodeU16Le(const std::uint8_t *data) {
    return static_cast<std::uint16_t>(data[0]) |
           (static_cast<std::uint16_t>(data[1]) << 8U);
}

std::uint32_t decodeU32Le(const std::uint8_t *data) {
    return static_cast<std::uint32_t>(data[0]) |
           (static_cast<std::uint32_t>(data[1]) << 8U) |
           (static_cast<std::uint32_t>(data[2]) << 16U) |
           (static_cast<std::uint32_t>(data[3]) << 24U);
}

void encodeU16Le(
    const std::uint16_t value, std::uint8_t *output) {
    output[0] = static_cast<std::uint8_t>(value);
    output[1] = static_cast<std::uint8_t>(value >> 8U);
}

} // namespace

std::uint16_t ballLinkCrc16CcittFalse(
    const std::uint8_t *data, const std::size_t length) {
    std::uint16_t crc = 0xFFFFU;
    if (data == nullptr) {
        return crc;
    }

    for (std::size_t index = 0U; index < length; ++index) {
        crc ^= static_cast<std::uint16_t>(data[index]) << 8U;
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                      ? static_cast<std::uint16_t>(
                            (crc << 1U) ^ 0x1021U)
                      : static_cast<std::uint16_t>(crc << 1U);
        }
    }
    return crc;
}

std::size_t ballLinkEncodeFrame(
    const BallLinkMessageType type,
    const std::uint8_t sequence,
    const std::uint8_t *payload,
    const std::uint8_t payloadLength,
    std::uint8_t *output,
    const std::size_t outputCapacity) {
    const std::size_t frameLength =
        kBallLinkFixedSize + payloadLength;
    if (output == nullptr ||
        payloadLength > kBallLinkMaxPayload ||
        outputCapacity < frameLength ||
        (payloadLength > 0U && payload == nullptr)) {
        return 0U;
    }

    output[0] = kBallLinkMagic0;
    output[1] = kBallLinkMagic1;
    output[2] = kBallLinkVersion;
    output[3] = static_cast<std::uint8_t>(type);
    output[4] = sequence;
    output[5] = payloadLength;
    if (payloadLength > 0U) {
        std::memcpy(output + 6U, payload, payloadLength);
    }

    const std::uint16_t crc = ballLinkCrc16CcittFalse(
        output + 2U, 4U + payloadLength);
    encodeU16Le(crc, output + 6U + payloadLength);
    return frameLength;
}

bool ballLinkDecodeVisionMeasurement(
    const BallLinkFrame &frame,
    BallLinkVisionMeasurement &measurement) {
    if (frame.type != static_cast<std::uint8_t>(
                          BallLinkMessageType::
                              VisionControlSnapshot) ||
        frame.length != kBallLinkVisionPayloadSize) {
        return false;
    }

    measurement.piSessionId = decodeU32Le(frame.payload.data());
    measurement.captureTimestampMs =
        decodeU32Le(frame.payload.data() + 4U);
    measurement.captureToSendDelayUs =
        decodeU32Le(frame.payload.data() + 8U);
    measurement.position0p1Mm = static_cast<std::int16_t>(
        decodeU16Le(frame.payload.data() + 12U));
    measurement.velocityMmps = static_cast<std::int16_t>(
        decodeU16Le(frame.payload.data() + 14U));
    measurement.confidence = frame.payload[16U];
    measurement.visionFlags = frame.payload[17U];
    measurement.targetPosition0p1Mm =
        static_cast<std::int16_t>(
            decodeU16Le(frame.payload.data() + 18U));
    measurement.taskId = frame.payload[20U];
    measurement.controlFlags = frame.payload[21U];
    measurement.runId =
        decodeU16Le(frame.payload.data() + 22U);
    return true;
}
