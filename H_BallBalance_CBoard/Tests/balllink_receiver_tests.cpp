#include "BallLinkProtocol.h"
#include "BallLinkReceiver.h"
#include "BallLinkStreamParser.h"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {

void require(const bool condition, const char *message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

constexpr std::array<std::uint8_t, 32U> kFrozenVector{
    0xA5U, 0x5AU, 0x02U, 0x10U, 0x2AU, 0x18U,
    0x78U, 0x56U, 0x34U, 0x12U,
    0x04U, 0x03U, 0x02U, 0x01U,
    0x98U, 0x3AU, 0x00U, 0x00U,
    0xF4U, 0x01U, 0x06U, 0xFFU,
    0xDCU, 0x07U, 0xF4U, 0x01U,
    0x03U, 0x03U, 0x2AU, 0x00U,
    0xF0U, 0x68U,
};

void putU16(
    std::array<std::uint8_t, kBallLinkMaxPayload> &payload,
    const std::size_t offset, const std::uint16_t value) {
    payload[offset] = static_cast<std::uint8_t>(value);
    payload[offset + 1U] =
        static_cast<std::uint8_t>(value >> 8U);
}

void putU32(
    std::array<std::uint8_t, kBallLinkMaxPayload> &payload,
    const std::size_t offset, const std::uint32_t value) {
    payload[offset] = static_cast<std::uint8_t>(value);
    payload[offset + 1U] =
        static_cast<std::uint8_t>(value >> 8U);
    payload[offset + 2U] =
        static_cast<std::uint8_t>(value >> 16U);
    payload[offset + 3U] =
        static_cast<std::uint8_t>(value >> 24U);
}

struct FrameValues {
    std::uint32_t session = 1U;
    std::uint32_t timestamp = 1000U;
    std::uint32_t delayUs = 0U;
    std::int16_t position = 500;
    std::int16_t velocity = -250;
    std::uint8_t confidence = 220U;
    std::uint8_t visionFlags =
        BallLinkBallValid | BallLinkVelocityValid |
        BallLinkCameraCalibrated;
    std::int16_t target = 500;
    std::uint8_t taskId = 3U;
    std::uint8_t controlFlags =
        BallLinkControlEnabled | BallLinkRunActive;
    std::uint16_t runId = 42U;
    std::uint8_t sequence = 1U;
};

BallLinkFrame makeFrame(const FrameValues &values) {
    BallLinkFrame frame{};
    frame.type = static_cast<std::uint8_t>(
        BallLinkMessageType::VisionControlSnapshot);
    frame.sequence = values.sequence;
    frame.length = kBallLinkVisionPayloadSize;
    putU32(frame.payload, 0U, values.session);
    putU32(frame.payload, 4U, values.timestamp);
    putU32(frame.payload, 8U, values.delayUs);
    putU16(
        frame.payload, 12U,
        static_cast<std::uint16_t>(values.position));
    putU16(
        frame.payload, 14U,
        static_cast<std::uint16_t>(values.velocity));
    frame.payload[16U] = values.confidence;
    frame.payload[17U] = values.visionFlags;
    putU16(
        frame.payload, 18U,
        static_cast<std::uint16_t>(values.target));
    frame.payload[20U] = values.taskId;
    frame.payload[21U] = values.controlFlags;
    putU16(frame.payload, 22U, values.runId);
    return frame;
}

std::array<std::uint8_t, kBallLinkVisionFrameSize> encodeFrame(
    const FrameValues &values) {
    const BallLinkFrame frame = makeFrame(values);
    std::array<std::uint8_t, kBallLinkVisionFrameSize> encoded{};
    const std::size_t length = ballLinkEncodeFrame(
        BallLinkMessageType::VisionControlSnapshot,
        frame.sequence, frame.payload.data(), frame.length,
        encoded.data(), encoded.size());
    require(length == encoded.size(), "test frame encoding");
    return encoded;
}

void testFrozenProtocolVector() {
    require(kBallLinkVisionPayloadSize == 24U, "payload length");
    require(kBallLinkVisionFrameSize == 32U, "frame length");
    require(
        ballLinkCrc16CcittFalse(
            kFrozenVector.data() + 2U, 28U) == 0x68F0U,
        "frozen CRC vector");

    BallLinkStreamParser parser;
    require(
        parser.append(kFrozenVector.data(), kFrozenVector.size()) ==
            0U,
        "frozen frame append");

    BallLinkFrame frame{};
    require(
        parser.next(frame) == BallLinkParseStatus::FrameReady,
        "frozen frame parse");
    require(
        frame.type == static_cast<std::uint8_t>(
                          BallLinkMessageType::
                              VisionControlSnapshot),
        "snapshot type");
    require(frame.length == 24U, "decoded payload length");

    BallLinkVisionMeasurement decoded{};
    require(
        ballLinkDecodeVisionMeasurement(frame, decoded),
        "snapshot decode");
    require(decoded.piSessionId == 0x12345678U, "session");
    require(
        decoded.captureTimestampMs == 0x01020304U,
        "capture timestamp");
    require(
        decoded.captureToSendDelayUs == 15000U,
        "capture-to-send delay");
    require(decoded.position0p1Mm == 500, "x");
    require(decoded.velocityMmps == -250, "v");
    require(decoded.confidence == 220U, "confidence");
    require(decoded.visionFlags == 0x07U, "vision_flags");
    require(decoded.targetPosition0p1Mm == 500, "target_x");
    require(decoded.taskId == 3U, "task_id");
    require(decoded.controlFlags == 0x03U, "control_flags");
    require(decoded.runId == 42U, "run_id");
}

void testLegacyPayloadLengthIsRejected() {
    BallLinkFrame frame{};
    frame.type = static_cast<std::uint8_t>(
        BallLinkMessageType::VisionControlSnapshot);
    frame.length = 18U;
    BallLinkVisionMeasurement decoded{};
    require(
        !ballLinkDecodeVisionMeasurement(frame, decoded),
        "legacy 18-byte payload rejected");
}

void testLegalMeasurementPublishesCompleteSnapshot() {
    BallLinkReceiver receiver;
    FrameValues values;
    values.delayUs = 15000U;
    require(
        receiver.process(makeFrame(values), 25U) ==
            BallLinkReceiveResult::NewMeasurement,
        "legal measurement accepted");

    const VisionLinkSnapshot snapshot = receiver.snapshot(25U);
    require(snapshot.pi_session_id == 1U, "published session");
    require(snapshot.capture_timestamp_ms == 1000U,
            "published timestamp");
    require(snapshot.local_receive_ms == 25U,
            "published receive time");
    require(snapshot.measurement_age_ms == 18U,
            "published measurement age");
    require(snapshot.position_0p1mm == 500, "published x");
    require(snapshot.velocity_mmps == -250, "published v");
    require(snapshot.target_position_0p1mm == 500,
            "published target");
    require(snapshot.task_id == 3U, "published task");
    require(snapshot.control_flags == 0x03U,
            "published control flags");
    require(snapshot.run_id == 42U, "published run");
    require(snapshot.valid == 1U, "published valid");
    require(snapshot.velocity_valid == 1U,
            "published velocity valid");
    require(snapshot.measurement_update_count == 1U,
            "measurement update count");
}

void testControlDisabledImmediatelyInvalidatesSnapshot() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial control measurement");

    values.sequence = 2U;
    values.timestamp = 1020U;
    values.controlFlags = 0U;
    values.target = -500;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::LegalFrame,
        "control-disabled frame legal");
    const VisionLinkSnapshot snapshot = receiver.snapshot(20U);
    require(snapshot.valid == 0U, "control disable is immediate");
    require(snapshot.target_position_0p1mm == -500,
            "disabled frame still updates target");
    require(snapshot.control_flags == 0U,
            "disabled control flags published");
    require(snapshot.measurement_update_count == 1U,
            "disabled frame is not a measurement");
}

void testRepeatedTimestampRefreshesOnlyLink() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial repeated-image measurement");

    values.sequence = 2U;
    values.delayUs = 35000U;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::LegalFrame,
        "repeated image is a legal frame");
    require(receiver.measurementUpdateCount() == 1U,
            "repeated timestamp not a new measurement");
    require(receiver.snapshot(20U).measurement_age_ms == 38U,
            "duplicate uses updated true age");

    receiver.refresh(120U);
    require(receiver.state() != BallLinkVisionState::NoLink,
            "duplicate refreshes link at 100 ms");
    receiver.refresh(121U);
    require(receiver.state() == BallLinkVisionState::NoLink,
            "duplicate link times out after 100 ms");
}

void testRepeatedTimestampCannotMakeMeasurementYounger() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial monotonic-age measurement");

    values.sequence = 2U;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::LegalFrame,
        "same-delay duplicate frame legal");
    require(receiver.snapshot(20U).measurement_age_ms == 23U,
            "duplicate cannot reduce true measurement age");
}

void testShortNoBallRetainsMeasurementUntilTrueAgeExpires() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial no-ball retention measurement");

    values.sequence = 2U;
    values.timestamp = 1020U;
    values.position = 0;
    values.velocity = 0;
    values.confidence = 0U;
    values.visionFlags = BallLinkCameraCalibrated;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::LegalFrame,
        "legal no-ball frame");
    require(receiver.snapshot(40U).valid == 1U,
            "short no-ball retains prior measurement");
    require(receiver.snapshot(40U).position_0p1mm == 500,
            "short no-ball retains prior x");
    require(receiver.measurementUpdateCount() == 1U,
            "no-ball is not a measurement");

    receiver.refresh(77U);
    require(receiver.snapshot(77U).valid == 1U,
            "measurement is valid at 80 ms age");
    receiver.refresh(78U);
    require(receiver.state() == BallLinkVisionState::Stale,
            "measurement stale above 80 ms age");
    require(receiver.snapshot(78U).valid == 0U,
            "stale measurement invalid");
}

void testSessionAndRunChangesClearMeasurementHistory() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial history measurement");

    values.sequence = 2U;
    values.timestamp = 1020U;
    values.runId = 43U;
    values.position = 0;
    values.velocity = 0;
    values.confidence = 0U;
    values.visionFlags = BallLinkCameraCalibrated;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::LegalFrame,
        "new run no-ball frame");
    require(receiver.snapshot(20U).valid == 0U,
            "new run clears measurement");
    require(receiver.snapshot(20U).position_0p1mm == 0,
            "new run clears old x");
    require(receiver.runChangeCount() == 1U,
            "run change counted");

    values.session = 2U;
    values.sequence = 1U;
    values.timestamp = 10U;
    values.runId = 1U;
    require(
        receiver.process(makeFrame(values), 40U) ==
            BallLinkReceiveResult::LegalFrame,
        "new session no-ball frame");
    require(receiver.snapshot(40U).pi_session_id == 2U,
            "new session published");
    require(receiver.sessionChangeCount() == 1U,
            "session change counted");
    require(receiver.snapshot(40U).valid == 0U,
            "new session history cleared");
}

void testInvalidFieldsDoNotRefreshLink() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial range measurement");

    values.sequence = 2U;
    values.timestamp = 1020U;
    values.target = 1201;
    require(
        receiver.process(makeFrame(values), 90U) ==
            BallLinkReceiveResult::RangeError,
        "target range rejected");
    receiver.refresh(101U);
    require(receiver.state() == BallLinkVisionState::NoLink,
            "range error does not refresh link");

    BallLinkReceiver velocityReceiver;
    values = {};
    values.velocity = 1;
    values.visionFlags =
        BallLinkBallValid | BallLinkCameraCalibrated;
    require(
        velocityReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::RangeError,
        "velocity must be zero when invalid");

    BallLinkReceiver reservedVisionReceiver;
    values = {};
    values.visionFlags |= 0x80U;
    require(
        reservedVisionReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::RangeError,
        "reserved vision flag rejected");

    BallLinkReceiver reservedControlReceiver;
    values = {};
    values.controlFlags |= 0x80U;
    require(
        reservedControlReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::RangeError,
        "reserved control flag rejected");
}

void testMeasurementAcceptanceGates() {
    FrameValues values;

    BallLinkReceiver confidenceReceiver;
    values.confidence = 179U;
    require(
        confidenceReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::LegalFrame,
        "low confidence frame legal");
    require(confidenceReceiver.snapshot(0U).valid == 0U,
            "low confidence measurement rejected");

    BallLinkReceiver degradedReceiver;
    values = {};
    values.visionFlags |= BallLinkProcessingDegraded;
    require(
        degradedReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::LegalFrame,
        "degraded frame legal");
    require(degradedReceiver.snapshot(0U).valid == 0U,
            "degraded measurement rejected");

    BallLinkReceiver calibrationReceiver;
    values = {};
    values.visionFlags &= static_cast<std::uint8_t>(
        ~BallLinkCameraCalibrated);
    require(
        calibrationReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::LegalFrame,
        "uncalibrated frame legal");
    require(calibrationReceiver.snapshot(0U).valid == 0U,
            "uncalibrated measurement rejected");

    BallLinkReceiver ageReceiver;
    values = {};
    values.delayUs = 88000U;
    require(
        ageReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::LegalFrame,
        "over-age frame legal");
    require(ageReceiver.snapshot(0U).valid == 0U,
            "over-age measurement rejected");
}

void testSequenceDiagnostics() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial sequence measurement");

    values.timestamp = 1020U;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::DuplicateSequence,
        "duplicate sequence rejected");

    values.sequence = 0U;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::OutOfOrderSequence,
        "out-of-order sequence rejected");

    values.sequence = 4U;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::NewMeasurement,
        "forward sequence accepted");
    require(receiver.missingFrameCount() == 2U,
            "missing sequence count");
}

void testSameSessionReconnectAfterLongSilenceAcceptsFirstFrame() {
    BallLinkReceiver receiver;
    FrameValues values;
    values.sequence = 10U;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial long-silence measurement");

    receiver.refresh(101U);
    require(receiver.state() == BallLinkVisionState::NoLink,
            "long silence enters no-link");

    values.sequence = 140U;
    values.timestamp = 4000U;
    require(
        receiver.process(makeFrame(values), 3000U) ==
            BallLinkReceiveResult::NewMeasurement,
        "same-session reconnect accepts first frame");
    require(receiver.missingFrameCount() == 0U,
            "reconnect starts a new sequence baseline");
}

void testCaptureTimestampRegressionHasDistinctResult() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial timestamp measurement");

    values.sequence = 2U;
    values.timestamp = 999U;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::TimestampError,
        "timestamp regression has distinct result");
    receiver.refresh(101U);
    require(receiver.state() == BallLinkVisionState::NoLink,
            "timestamp error does not refresh link");
}

void testPositionRangeBoundaries() {
    for (const std::int16_t position : {
             static_cast<std::int16_t>(-1300),
             static_cast<std::int16_t>(1300)}) {
        BallLinkReceiver receiver;
        FrameValues values;
        values.position = position;
        require(
            receiver.process(makeFrame(values), 0U) ==
                BallLinkReceiveResult::NewMeasurement,
            "position boundary accepted");
    }
    for (const std::int16_t position : {
             static_cast<std::int16_t>(-1301),
             static_cast<std::int16_t>(1301)}) {
        BallLinkReceiver receiver;
        FrameValues values;
        values.position = position;
        require(
            receiver.process(makeFrame(values), 0U) ==
                BallLinkReceiveResult::RangeError,
            "position beyond boundary rejected");
    }
}

void testVelocityRangeBoundaries() {
    for (const std::int16_t velocity : {
             static_cast<std::int16_t>(-5000),
             static_cast<std::int16_t>(5000)}) {
        BallLinkReceiver receiver;
        FrameValues values;
        values.velocity = velocity;
        require(
            receiver.process(makeFrame(values), 0U) ==
                BallLinkReceiveResult::NewMeasurement,
            "velocity boundary accepted");
    }
    for (const std::int16_t velocity : {
             static_cast<std::int16_t>(-5001),
             static_cast<std::int16_t>(5001)}) {
        BallLinkReceiver receiver;
        FrameValues values;
        values.velocity = velocity;
        require(
            receiver.process(makeFrame(values), 0U) ==
                BallLinkReceiveResult::RangeError,
            "velocity beyond boundary rejected");
    }
}

void testProcessingDelayRangeBoundaries() {
    BallLinkReceiver boundaryReceiver;
    FrameValues values;
    values.delayUs = 200000U;
    require(
        boundaryReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::LegalFrame,
        "processing delay boundary is a legal frame");

    BallLinkReceiver beyondReceiver;
    values.delayUs = 200001U;
    require(
        beyondReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::RangeError,
        "processing delay beyond boundary rejected");
}

void testSequenceWrapFrom255ToZeroIsAccepted() {
    BallLinkReceiver receiver;
    FrameValues values;
    values.sequence = 255U;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "sequence 255 accepted");

    values.sequence = 0U;
    values.timestamp = 1020U;
    require(
        receiver.process(makeFrame(values), 20U) ==
            BallLinkReceiveResult::NewMeasurement,
        "sequence 255-to-zero wrap accepted");
    require(receiver.missingFrameCount() == 0U,
            "sequence wrap has no gap");
}

void testPolicyConstantsAreFrozen() {
    require(kBallLinkFreshMeasurementAgeMs == 80U,
            "measurement age policy");
    require(kBallLinkTimeoutMs == 100U, "link timeout policy");
    require(kBallLinkMinimumConfidence == 180U,
            "confidence policy");
}

void testTimedParserUsesFinalByteArrivalAcrossChunks() {
    FrameValues values;
    const auto encoded = encodeFrame(values);
    std::array<std::uint32_t, encoded.size()> arrivalMs{};
    arrivalMs.fill(10U);
    for (std::size_t index = 13U; index < arrivalMs.size();
         ++index) {
        arrivalMs[index] = 25U;
    }

    BallLinkStreamParser parser;
    require(
        parser.append(
            encoded.data(), arrivalMs.data(), 13U) == 0U,
        "timed first fragment append");
    BallLinkFrame frame{};
    std::uint32_t frameArrivalMs = 0U;
    require(
        parser.next(frame, frameArrivalMs) ==
            BallLinkParseStatus::NeedMoreData,
        "split frame waits for final bytes");
    require(
        parser.append(
            encoded.data() + 13U, arrivalMs.data() + 13U,
            encoded.size() - 13U) == 0U,
        "timed second fragment append");
    require(
        parser.next(frame, frameArrivalMs) ==
            BallLinkParseStatus::FrameReady,
        "split frame becomes ready");
    require(frameArrivalMs == 25U,
            "frame uses final-byte arrival tick");
}

void testTimedParserHandlesMultipleFramesInOneChunk() {
    FrameValues firstValues;
    FrameValues secondValues;
    secondValues.sequence = 2U;
    secondValues.timestamp = 1020U;
    const auto first = encodeFrame(firstValues);
    const auto second = encodeFrame(secondValues);
    std::array<std::uint8_t, 2U * kBallLinkVisionFrameSize>
        bytes{};
    std::array<std::uint32_t, bytes.size()> arrivalMs{};
    for (std::size_t index = 0U; index < first.size(); ++index) {
        bytes[index] = first[index];
        arrivalMs[index] = 10U;
        bytes[first.size() + index] = second[index];
        arrivalMs[first.size() + index] = 20U;
    }

    BallLinkStreamParser parser;
    require(
        parser.append(
            bytes.data(), arrivalMs.data(), bytes.size()) == 0U,
        "multi-frame timed append");
    BallLinkFrame frame{};
    std::uint32_t frameArrivalMs = 0U;
    require(
        parser.next(frame, frameArrivalMs) ==
            BallLinkParseStatus::FrameReady,
        "first sticky frame ready");
    require(frame.sequence == 1U && frameArrivalMs == 10U,
            "first sticky frame arrival");
    require(
        parser.next(frame, frameArrivalMs) ==
            BallLinkParseStatus::FrameReady,
        "second sticky frame ready");
    require(frame.sequence == 2U && frameArrivalMs == 20U,
            "second sticky frame arrival");
}

void testParserResynchronizesAfterNoiseAndCrcError() {
    FrameValues values;
    const auto valid = encodeFrame(values);
    auto damaged = valid;
    damaged[12U] ^= 0x01U;

    std::array<std::uint8_t, 3U + 2U * kBallLinkVisionFrameSize>
        bytes{};
    bytes[0U] = 0x00U;
    bytes[1U] = 0xA5U;
    bytes[2U] = 0x11U;
    for (std::size_t index = 0U; index < damaged.size(); ++index) {
        bytes[3U + index] = damaged[index];
        bytes[3U + damaged.size() + index] = valid[index];
    }

    BallLinkStreamParser parser;
    require(parser.append(bytes.data(), bytes.size()) == 0U,
            "noise and CRC stream append");
    BallLinkFrame frame{};
    bool sawCrcError = false;
    bool sawValidFrame = false;
    for (std::size_t step = 0U; step < bytes.size() + 4U;
         ++step) {
        const BallLinkParseStatus status = parser.next(frame);
        if (status == BallLinkParseStatus::CrcError) {
            sawCrcError = true;
        } else if (status == BallLinkParseStatus::FrameReady) {
            sawValidFrame = true;
            break;
        } else if (status == BallLinkParseStatus::NeedMoreData) {
            break;
        }
    }
    require(sawCrcError, "damaged frame reports CRC error");
    require(sawValidFrame, "parser resynchronizes after CRC error");
}

void testParserRejectsLengthAndClearDropsPartialFrame() {
    FrameValues values;
    const auto valid = encodeFrame(values);
    const std::array<std::uint8_t, 6U> invalidLength{
        kBallLinkMagic0, kBallLinkMagic1, kBallLinkVersion,
        static_cast<std::uint8_t>(
            BallLinkMessageType::VisionControlSnapshot),
        0U, 33U,
    };

    BallLinkStreamParser parser;
    require(
        parser.append(
            invalidLength.data(), invalidLength.size()) == 0U,
        "invalid length header append");
    require(parser.append(valid.data(), valid.size()) == 0U,
            "valid frame after invalid length append");
    BallLinkFrame frame{};
    bool sawValidFrame = false;
    for (std::size_t step = 0U;
         step < invalidLength.size() + valid.size(); ++step) {
        const BallLinkParseStatus status = parser.next(frame);
        if (status == BallLinkParseStatus::FrameReady) {
            sawValidFrame = true;
            break;
        }
        if (status == BallLinkParseStatus::NeedMoreData) {
            break;
        }
    }
    require(sawValidFrame,
            "parser resynchronizes after invalid length");

    parser.clear();
    require(parser.append(valid.data(), 15U) == 0U,
            "partial frame append before clear");
    parser.clear();
    require(parser.append(valid.data(), valid.size()) == 0U,
            "whole frame append after clear");
    require(
        parser.next(frame) == BallLinkParseStatus::FrameReady,
        "clear prevents cross-boundary frame splice");
}

void testCaptureTimestampAndLocalTickWrap() {
    BallLinkReceiver receiver;
    FrameValues values;
    values.timestamp = 0xFFFFFFF0U;
    require(
        receiver.process(makeFrame(values), 0xFFFFFFF0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "pre-wrap timestamps accepted");

    values.sequence = 2U;
    values.timestamp = 0x00000010U;
    require(
        receiver.process(makeFrame(values), 0x00000010U) ==
            BallLinkReceiveResult::NewMeasurement,
        "capture timestamp wrap accepted");
    require(receiver.snapshot(0x00000010U).measurement_age_ms ==
                3U,
            "measurement age at wrapped receive tick");

    receiver.refresh(0x00000074U);
    require(receiver.state() != BallLinkVisionState::NoLink,
            "local tick wrap preserves 100 ms link boundary");
    receiver.refresh(0x00000075U);
    require(receiver.state() == BallLinkVisionState::NoLink,
            "local tick wrap times out above 100 ms");
}

void testNegativeTargetBoundary() {
    BallLinkReceiver boundaryReceiver;
    FrameValues values;
    values.target = -1200;
    require(
        boundaryReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "negative target boundary accepted");

    BallLinkReceiver beyondReceiver;
    values.target = -1201;
    require(
        beyondReceiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::RangeError,
        "negative target beyond boundary rejected");
}

void testTransportFaultInvalidatesAndStartsFreshHistory() {
    BallLinkReceiver receiver;
    FrameValues values;
    require(
        receiver.process(makeFrame(values), 0U) ==
            BallLinkReceiveResult::NewMeasurement,
        "initial transport-fault measurement");

    receiver.invalidateLink();
    require(receiver.state() == BallLinkVisionState::NoLink,
            "transport fault enters no-link");
    const VisionLinkSnapshot invalid = receiver.snapshot(20U);
    require(invalid.valid == 0U,
            "transport fault invalidates published measurement");
    require(invalid.position_0p1mm == 0,
            "transport fault clears stale position");

    values.sequence = 200U;
    values.timestamp = 2000U;
    require(
        receiver.process(makeFrame(values), 40U) ==
            BallLinkReceiveResult::NewMeasurement,
        "first post-fault frame starts fresh history");
    require(receiver.missingFrameCount() == 0U,
            "post-fault frame starts sequence baseline");
}

void testQueuedFrameUsesWireArrivalForSafetyAges() {
    FrameValues values;
    const auto encoded = encodeFrame(values);
    std::array<std::uint32_t, encoded.size()> arrivalMs{};
    arrivalMs.fill(10U);

    BallLinkStreamParser parser;
    require(
        parser.append(
            encoded.data(), arrivalMs.data(), encoded.size()) ==
            0U,
        "queued frame append");
    BallLinkFrame frame{};
    std::uint32_t frameArrivalMs = 0U;
    require(
        parser.next(frame, frameArrivalMs) ==
            BallLinkParseStatus::FrameReady,
        "queued frame parse");

    BallLinkReceiver receiver;
    require(
        receiver.process(frame, frameArrivalMs) ==
            BallLinkReceiveResult::NewMeasurement,
        "queued frame accepted at wire arrival");
    receiver.refresh(98U);
    require(receiver.state() == BallLinkVisionState::Stale,
            "queued frame measurement age uses wire arrival");
    receiver.refresh(111U);
    require(receiver.state() == BallLinkVisionState::NoLink,
            "queued frame link age uses wire arrival");
}

} // namespace

int main() {
    testFrozenProtocolVector();
    testLegacyPayloadLengthIsRejected();
    testLegalMeasurementPublishesCompleteSnapshot();
    testControlDisabledImmediatelyInvalidatesSnapshot();
    testRepeatedTimestampRefreshesOnlyLink();
    testRepeatedTimestampCannotMakeMeasurementYounger();
    testShortNoBallRetainsMeasurementUntilTrueAgeExpires();
    testSessionAndRunChangesClearMeasurementHistory();
    testInvalidFieldsDoNotRefreshLink();
    testMeasurementAcceptanceGates();
    testSequenceDiagnostics();
    testSameSessionReconnectAfterLongSilenceAcceptsFirstFrame();
    testCaptureTimestampRegressionHasDistinctResult();
    testPositionRangeBoundaries();
    testVelocityRangeBoundaries();
    testProcessingDelayRangeBoundaries();
    testSequenceWrapFrom255ToZeroIsAccepted();
    testPolicyConstantsAreFrozen();
    testTimedParserUsesFinalByteArrivalAcrossChunks();
    testTimedParserHandlesMultipleFramesInOneChunk();
    testParserResynchronizesAfterNoiseAndCrcError();
    testParserRejectsLengthAndClearDropsPartialFrame();
    testCaptureTimestampAndLocalTickWrap();
    testNegativeTargetBoundary();
    testTransportFaultInvalidatesAndStartsFreshHistory();
    testQueuedFrameUsesWireArrivalForSafetyAges();
    std::cout
        << "PASS: BallLink one-way receiver protocol tests\n";
    return 0;
}
