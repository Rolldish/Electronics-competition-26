#!/usr/bin/env python3

import argparse
import struct
import sys
import time

import serial


MAGIC = b"\xA5\x5A"
VERSION = 0x02
MESSAGE_TYPE = 0x10
PAYLOAD_LENGTH = 24
FRAME_LENGTH = 32


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def build_frame(
    session_id: int,
    sequence: int,
    capture_timestamp_ms: int,
) -> bytes:
    payload = struct.pack(
        "<IIIhhBBhBBH",
        session_id,
        capture_timestamp_ms,
        15000,  # capture_to_send_delay_us
        500,    # x_0p1mm: 50.0 mm
        -250,   # v_mmps
        220,    # confidence
        0x07,   # ball/velocity/calibration valid
        500,    # target_x_0p1mm: 50.0 mm
        3,      # task_id
        0x03,   # control enabled + run active
        42,     # run_id
    )

    crc_input = struct.pack(
        "<BBBB",
        VERSION,
        MESSAGE_TYPE,
        sequence,
        PAYLOAD_LENGTH,
    ) + payload
    crc = crc16_ccitt_false(crc_input)
    frame = MAGIC + crc_input + struct.pack("<H", crc)

    if len(payload) != PAYLOAD_LENGTH or len(frame) != FRAME_LENGTH:
        raise RuntimeError("BallLink frame length is incorrect")
    return frame


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send continuous BallLink frames to the C board."
    )
    parser.add_argument(
        "--port",
        default="/dev/serial0",
        help="Raspberry Pi UART device (default: /dev/serial0)",
    )
    parser.add_argument(
        "--interval-ms",
        type=int,
        default=20,
        help="Frame interval in milliseconds (default: 20)",
    )
    parser.add_argument(
        "--duration-seconds",
        type=int,
        default=60,
        help="Duration in seconds; 0 runs until Ctrl+C (default: 60)",
    )
    args = parser.parse_args()

    if args.interval_ms <= 0:
        parser.error("--interval-ms must be greater than zero")
    if args.duration_seconds < 0:
        parser.error("--duration-seconds cannot be negative")
    return args


def main() -> int:
    args = parse_arguments()
    interval_ns = args.interval_ms * 1_000_000
    total_frames = (
        None
        if args.duration_seconds == 0
        else (args.duration_seconds * 1000) // args.interval_ms
    )

    session_id = time.time_ns() & 0xFFFFFFFF
    sequence = 0
    sent_frames = 0
    next_report_ns = 1_000_000_000

    uart = serial.Serial(
        port=args.port,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=1,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )

    try:
        start_ns = time.monotonic_ns()

        print("BallLink continuous sender started")
        print(
            f"Port: {args.port}, interval: {args.interval_ms} ms, "
            f"session: 0x{session_id:08X}"
        )
        print("Press Ctrl+C to stop.")

        while total_frames is None or sent_frames < total_frames:
            target_ns = start_ns + sent_frames * interval_ns
            remaining_ns = target_ns - time.monotonic_ns()
            if remaining_ns > 0:
                time.sleep(remaining_ns / 1_000_000_000)

            capture_timestamp_ms = (
                (time.monotonic_ns() - start_ns) // 1_000_000
            ) & 0xFFFFFFFF
            frame = build_frame(
                session_id,
                sequence,
                capture_timestamp_ms,
            )

            written = uart.write(frame)
            if written != FRAME_LENGTH:
                raise IOError(
                    f"short serial write: {written}/{FRAME_LENGTH} bytes"
                )

            sent_frames += 1
            sequence = (sequence + 1) & 0xFF

            elapsed_ns = time.monotonic_ns() - start_ns
            if elapsed_ns >= next_report_ns:
                print(
                    f"time={elapsed_ns / 1_000_000_000:.1f}s "
                    f"frames={sent_frames} "
                    f"bytes={sent_frames * FRAME_LENGTH}"
                )
                next_report_ns += 1_000_000_000

        uart.flush()
        print(
            f"Finished: frames={sent_frames} "
            f"bytes={sent_frames * FRAME_LENGTH}"
        )
        return 0
    except KeyboardInterrupt:
        uart.flush()
        print(
            f"\nStopped: frames={sent_frames} "
            f"bytes={sent_frames * FRAME_LENGTH}"
        )
        return 0
    finally:
        uart.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except serial.SerialException as error:
        print(f"Serial error: {error}", file=sys.stderr)
        raise SystemExit(1)
