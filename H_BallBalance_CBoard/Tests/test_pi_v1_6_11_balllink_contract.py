#!/usr/bin/env python3
"""Cross-check the authoritative Pi v1.6.11 BallLink wire contract."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pi-root", required=True, type=Path)
    arguments = parser.parse_args()

    pi_root = arguments.pi_root.resolve()
    require(pi_root.is_dir(), f"Pi root does not exist: {pi_root}")
    sys.path.insert(0, str(pi_root))

    from hball.drivers import protocol

    require(protocol.MAGIC_BYTES == b"\xA5\x5A", "magic bytes changed")
    require(protocol.VERSION == 0x02, "protocol version changed")
    require(
        protocol.VISION_CONTROL_SNAPSHOT == 0x10,
        "snapshot message type changed",
    )
    require(protocol.PAYLOAD_SIZE == 24, "payload size changed")
    require(protocol.PACKET_SIZE == 32, "packet size changed")
    require(
        protocol.MEASUREMENT_MAX_AGE_US == 70_000,
        "Pi measurement-valid age changed",
    )
    require(
        protocol.CAPTURE_DELAY_LIMIT_US == 200_000,
        "Pi legal-heartbeat age changed",
    )
    require(
        protocol.DEFAULT_CONFIDENCE_MIN_U8 == 180,
        "confidence threshold changed",
    )
    require(protocol.X_LIMIT_0P1MM == 1_300, "position limit changed")
    require(protocol.V_LIMIT_MMPS == 5_000, "velocity limit changed")
    require(
        protocol.TARGET_LIMIT_0P1MM == 1_200,
        "target position limit changed",
    )

    message = protocol.Telemetry(
        session_id=0x12345678,
        sequence=0x2A,
        capture_timestamp_ms=0x01020304,
        capture_to_send_delay_us=15_000,
        x_0p1mm=500,
        v_mmps=-250,
        confidence_u8=220,
        flags=(
            protocol.Flags.BALL_VALID
            | protocol.Flags.VELOCITY_VALID
            | protocol.Flags.CAMERA_CALIBRATED
        ),
        target_x_0p1mm=500,
        task_id=protocol.TaskId.T3,
        control_flags=(
            protocol.ControlFlags.CONTROL_ENABLED
            | protocol.ControlFlags.RUN_ACTIVE
        ),
        run_id=42,
    )
    expected = bytes.fromhex(
        "A55A02102A187856341204030201983A0000F40106FFDC07F40103032A00F068"
    )
    require(
        protocol.encode_packet(message) == expected,
        "Pi encoder no longer matches the C-board frozen vector",
    )

    capture_timestamp_ns = 1_000_000_000
    source = protocol.TxSnapshot(
        capture_timestamp_ns=capture_timestamp_ns,
        flags=(
            protocol.Flags.BALL_VALID
            | protocol.Flags.VELOCITY_VALID
            | protocol.Flags.CAMERA_CALIBRATED
        ),
        x_m=0.05,
        v_mps=-0.25,
        confidence=220.0 / 255.0,
        target_x_m=0.05,
        task_id=protocol.TaskId.T3,
        control_flags=(
            protocol.ControlFlags.CONTROL_ENABLED
            | protocol.ControlFlags.RUN_ACTIVE
        ),
        run_id=42,
    )

    def at_delay(delay_us: int):
        return protocol.make_telemetry(
            source,
            session_id=0x12345678,
            sequence=0x2A,
            send_timestamp_ns=capture_timestamp_ns + delay_us * 1_000,
        )

    age_70_ms = at_delay(70_000)
    require(age_70_ms.measurement_valid, "70 ms snapshot was invalidated")
    require(age_70_ms.velocity_valid, "70 ms velocity was invalidated")
    require(age_70_ms.x_0p1mm == 500, "70 ms position changed")
    require(age_70_ms.v_mmps == -250, "70 ms velocity changed")

    age_70_ms_plus = at_delay(70_001)
    require(
        not age_70_ms_plus.measurement_valid,
        "snapshot above 70 ms remained measurement-valid",
    )
    require(
        not age_70_ms_plus.velocity_valid,
        "snapshot above 70 ms retained valid velocity",
    )
    require(
        bool(age_70_ms_plus.flags & protocol.Flags.PROCESSING_DEGRADED),
        "snapshot above 70 ms did not set degraded",
    )
    require(
        (age_70_ms_plus.x_0p1mm,
         age_70_ms_plus.v_mmps,
         age_70_ms_plus.confidence_u8) == (0, 0, 0),
        "invalid heartbeat did not clear x/v/confidence",
    )

    age_200_ms = at_delay(200_000)
    encoded_200_ms = protocol.encode_packet(age_200_ms)
    require(len(encoded_200_ms) == 32, "200 ms heartbeat was not encoded")
    require(
        not age_200_ms.measurement_valid
        and bool(age_200_ms.flags & protocol.Flags.PROCESSING_DEGRADED),
        "200 ms heartbeat was not safely invalidated",
    )

    try:
        at_delay(200_001)
    except protocol.SnapshotTooOldError:
        pass
    else:
        raise AssertionError("snapshot above 200 ms did not raise SnapshotTooOldError")

    print("PASS: Pi v1.6.11 BallLink contract")


if __name__ == "__main__":
    main()
