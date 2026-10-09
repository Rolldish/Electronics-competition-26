#!/usr/bin/env python3
"""Simulate installed-pipe BallLink position data without a physical ball."""

from __future__ import annotations

import argparse
import math
import secrets
import select
import struct
import sys
import time
from dataclasses import dataclass
from typing import Callable

import serial


MAGIC = b"\xA5\x5A"
VERSION = 0x02
MESSAGE_TYPE = 0x10
PAYLOAD_LENGTH = 24
FRAME_LENGTH = 32

BALL_VALID = 1 << 0
VELOCITY_VALID = 1 << 1
CAMERA_CALIBRATED = 1 << 2

CONTROL_ENABLED = 1 << 0
RUN_ACTIVE = 1 << 1
TASK_DONE = 1 << 2
TARGET_LATCHED = 1 << 4

ACTIVE_CONTROL_FLAGS = CONTROL_ENABLED | RUN_ACTIVE | TARGET_LATCHED
VALID_BALL_FLAGS = BALL_VALID | CAMERA_CALIBRATED
VALID_MOVING_BALL_FLAGS = VALID_BALL_FLAGS | VELOCITY_VALID

DEFAULT_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
TX_HZ = 50
PERIOD_NS = 1_000_000_000 // TX_HZ
ARM_PHRASE = "ARM MOTOR TEST"


@dataclass(frozen=True)
class Snapshot:
    position_0p1mm: int
    velocity_mmps: int
    confidence: int
    vision_flags: int
    control_flags: int
    target_position_0p1mm: int = 0
    task_id: int = 1


SAFE_SNAPSHOT = Snapshot(
    position_0p1mm=0,
    velocity_mmps=0,
    confidence=0,
    vision_flags=CAMERA_CALIBRATED,
    control_flags=0,
)

END_SNAPSHOT = Snapshot(
    position_0p1mm=0,
    velocity_mmps=0,
    confidence=0,
    vision_flags=CAMERA_CALIBRATED,
    control_flags=TASK_DONE | TARGET_LATCHED,
)


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


def encode_frame(
    *,
    session_id: int,
    sequence: int,
    capture_timestamp_ms: int,
    run_id: int,
    snapshot: Snapshot,
    capture_to_send_delay_us: int = 0,
) -> bytes:
    payload = struct.pack(
        "<IIIhhBBhBBH",
        session_id & 0xFFFFFFFF,
        capture_timestamp_ms & 0xFFFFFFFF,
        capture_to_send_delay_us & 0xFFFFFFFF,
        snapshot.position_0p1mm,
        snapshot.velocity_mmps,
        snapshot.confidence,
        snapshot.vision_flags,
        snapshot.target_position_0p1mm,
        snapshot.task_id,
        snapshot.control_flags,
        run_id & 0xFFFF,
    )
    body = bytes(
        (VERSION, MESSAGE_TYPE, sequence & 0xFF, PAYLOAD_LENGTH)
    ) + payload
    frame = MAGIC + body + struct.pack("<H", crc16_ccitt_false(body))
    if len(payload) != PAYLOAD_LENGTH or len(frame) != FRAME_LENGTH:
        raise RuntimeError("internal BallLink frame length error")
    return frame


def verify_protocol_vector() -> None:
    vector_snapshot = Snapshot(
        position_0p1mm=500,
        velocity_mmps=-250,
        confidence=220,
        vision_flags=0x07,
        target_position_0p1mm=500,
        task_id=3,
        control_flags=0x03,
    )
    actual = encode_frame(
        session_id=0x12345678,
        sequence=0x2A,
        capture_timestamp_ms=0x01020304,
        capture_to_send_delay_us=15_000,
        run_id=42,
        snapshot=vector_snapshot,
    )
    expected = bytes.fromhex(
        "A5 5A 02 10 2A 18 "
        "78 56 34 12 04 03 02 01 98 3A 00 00 "
        "F4 01 06 FF DC 07 F4 01 03 03 2A 00 F0 68"
    )
    if actual != expected:
        raise RuntimeError("BallLink protocol self-check failed")


class BallLinkSender:
    def __init__(self, uart: serial.Serial) -> None:
        self.uart = uart
        self.session_id = secrets.randbits(32)
        self.run_id = secrets.randbelow(0xFFFF) + 1
        self.sequence = 0
        self.sent_frames = 0
        self.started_ns = time.monotonic_ns()
        self.next_send_ns = self.started_ns
        self.next_report_ns = self.started_ns + 1_000_000_000

    def send(self, snapshot: Snapshot) -> None:
        now_ns = time.monotonic_ns()
        if now_ns < self.next_send_ns:
            time.sleep((self.next_send_ns - now_ns) / 1_000_000_000)
            now_ns = time.monotonic_ns()
        if now_ns - self.next_send_ns > PERIOD_NS * 2:
            self.next_send_ns = now_ns

        frame = encode_frame(
            session_id=self.session_id,
            sequence=self.sequence,
            capture_timestamp_ms=(
                (now_ns - self.started_ns) // 1_000_000
            ),
            run_id=self.run_id,
            snapshot=snapshot,
        )
        written = self.uart.write(frame)
        if written != FRAME_LENGTH:
            raise RuntimeError(
                f"short UART write: expected {FRAME_LENGTH}, got {written}"
            )

        self.sent_frames += 1
        self.sequence = (self.sequence + 1) & 0xFF
        self.next_send_ns += PERIOD_NS
        if now_ns >= self.next_report_ns:
            elapsed_s = (now_ns - self.started_ns) / 1_000_000_000
            rate_hz = self.sent_frames / elapsed_s if elapsed_s > 0 else 0.0
            print(
                f"TX: frames={self.sent_frames} "
                f"rate={rate_hz:.2f}Hz next_seq={self.sequence}"
            )
            self.next_report_ns += 1_000_000_000

    def run_for(
        self,
        duration_s: float,
        snapshot_at: Callable[[float], Snapshot],
    ) -> None:
        stage_started_ns = time.monotonic_ns()
        while True:
            elapsed_s = (
                time.monotonic_ns() - stage_started_ns
            ) / 1_000_000_000
            if elapsed_s >= duration_s:
                return
            self.send(snapshot_at(elapsed_s))

    def rebase_after_silence(self) -> None:
        self.next_send_ns = time.monotonic_ns()


def constant_snapshot(
    *,
    position_0p1mm: int,
    confidence: int = 220,
    vision_flags: int = VALID_BALL_FLAGS,
    control_flags: int = ACTIVE_CONTROL_FLAGS,
) -> Snapshot:
    return Snapshot(
        position_0p1mm=position_0p1mm,
        velocity_mmps=0,
        confidence=confidence,
        vision_flags=vision_flags,
        control_flags=control_flags,
    )


def smooth_transition_snapshot(
    start_position_0p1mm: int,
    end_position_0p1mm: int,
    duration_s: float,
    elapsed_s: float,
) -> Snapshot:
    if duration_s <= 0.0:
        raise ValueError("smooth transition duration must be positive")

    progress = min(max(elapsed_s / duration_s, 0.0), 1.0)
    blend = 0.5 - 0.5 * math.cos(math.pi * progress)
    delta_0p1mm = end_position_0p1mm - start_position_0p1mm
    position_0p1mm = round(
        start_position_0p1mm + delta_0p1mm * blend
    )

    delta_mm = delta_0p1mm * 0.1
    velocity_mmps = round(
        delta_mm
        * math.pi
        * math.sin(math.pi * progress)
        / (2.0 * duration_s)
    )
    return Snapshot(
        position_0p1mm=position_0p1mm,
        velocity_mmps=velocity_mmps,
        confidence=220,
        vision_flags=VALID_MOVING_BALL_FLAGS,
        control_flags=ACTIVE_CONTROL_FLAGS,
    )


def run_constant_stage(
    sender: BallLinkSender,
    name: str,
    duration_s: float,
    snapshot: Snapshot,
) -> None:
    print(
        f"\nstage: {name}, x={snapshot.position_0p1mm / 10:+.1f}mm, "
        f"v={snapshot.velocity_mmps:+d}mm/s, "
        f"vision=0x{snapshot.vision_flags:02X}, "
        f"control=0x{snapshot.control_flags:02X}"
    )
    sender.run_for(duration_s, lambda _elapsed: snapshot)


def run_smooth_stage(
    sender: BallLinkSender,
    name: str,
    duration_s: float,
    start_position_0p1mm: int,
    end_position_0p1mm: int,
) -> None:
    print(
        f"\nstage: {name}, "
        f"x={start_position_0p1mm / 10:+.1f}"
        f"->{end_position_0p1mm / 10:+.1f}mm, "
        f"duration={duration_s:.1f}s, smooth position+velocity"
    )
    sender.run_for(
        duration_s,
        lambda elapsed: smooth_transition_snapshot(
            start_position_0p1mm,
            end_position_0p1mm,
            duration_s,
            elapsed,
        ),
    )


def wait_for_arm(sender: BallLinkSender) -> None:
    print(f"\nType exactly '{ARM_PHRASE}' and press Enter to continue.")
    while True:
        sender.send(SAFE_SNAPSHOT)
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            continue
        line = sys.stdin.readline()
        if line == "":
            raise RuntimeError("SSH terminal input closed before arming")
        if line.strip() == ARM_PHRASE:
            break
        print(f"Not armed. Type exactly '{ARM_PHRASE}'.")

    for remaining in range(3, 0, -1):
        print(f"Starting active snapshots in {remaining}...")
        sender.run_for(1.0, lambda _elapsed: SAFE_SNAPSHOT)


def run_sequence(sender: BallLinkSender, stage_seconds: float) -> None:
    run_constant_stage(
        sender, "safe wait / no ball", 3.0, SAFE_SNAPSHOT
    )
    wait_for_arm(sender)

    stages = (
        ("center and 10-frame gate", 0),
        ("direction check +10 mm", 100),
        ("return center", 0),
        ("direction check -10 mm", -100),
        ("return center", 0),
        ("installed-pipe check +20 mm", 200),
        ("return center", 0),
        ("installed-pipe check -20 mm", -200),
        ("return center", 0),
    )
    for name, position in stages:
        run_constant_stage(
            sender,
            name,
            stage_seconds,
            constant_snapshot(position_0p1mm=position),
        )

    run_constant_stage(
        sender,
        "final center recovery",
        stage_seconds,
        constant_snapshot(position_0p1mm=0),
    )


def run_continuous_sweep(
    sender: BallLinkSender,
    leg_seconds: float,
) -> None:
    run_constant_stage(
        sender, "safe wait / no ball", 3.0, SAFE_SNAPSHOT
    )
    wait_for_arm(sender)
    run_constant_stage(
        sender,
        "center and 10-frame gate",
        leg_seconds,
        constant_snapshot(position_0p1mm=0),
    )

    positions = (0, 100, 0, -100, 0)
    names = (
        "smooth center to +10 mm",
        "smooth +10 mm to center",
        "smooth center to -10 mm",
        "smooth -10 mm to center",
    )
    for name, start, end in zip(names, positions, positions[1:]):
        run_smooth_stage(sender, name, leg_seconds, start, end)

    run_constant_stage(
        sender,
        "final center recovery",
        leg_seconds,
        constant_snapshot(position_0p1mm=0),
    )


def run_limit_scan(sender: BallLinkSender, stage_seconds: float) -> None:
    run_constant_stage(
        sender, "safe wait / no ball", 3.0, SAFE_SNAPSHOT
    )
    wait_for_arm(sender)

    stages = (
        ("center and 10-frame gate", 0),
        ("positive 8.3 mm / expected 1.00 deg", 83),
        ("return center", 0),
        ("positive 16.7 mm / expected 2.00 deg", 167),
        ("return center", 0),
        ("positive 25.0 mm / expected 3.00 deg", 250),
        ("return center", 0),
        ("positive 30.0 mm / expected clamp at 3.00 deg", 300),
        ("return center", 0),
        ("negative 8.3 mm / expected 1.00 deg", -83),
        ("return center", 0),
        ("negative 16.7 mm / expected 2.00 deg", -167),
        ("return center", 0),
        ("negative 25.0 mm / expected 3.00 deg", -250),
        ("return center", 0),
        ("negative 30.0 mm / expected clamp at 3.00 deg", -300),
        ("final center", 0),
    )
    for name, position in stages:
        run_constant_stage(
            sender,
            name,
            stage_seconds,
            constant_snapshot(position_0p1mm=position),
        )


def send_safe_shutdown(sender: BallLinkSender) -> None:
    print("\nSending 1 second of CONTROL_ENABLED=0 shutdown frames...")
    try:
        sender.run_for(1.0, lambda _elapsed: END_SNAPSHOT)
        sender.uart.flush()
    except Exception as error:
        print(f"Could not complete shutdown frames: {error}", file=sys.stderr)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Simulate formal BallLink vision data for motor motion"
    )
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"USB-TTL device (default: {DEFAULT_PORT})",
    )
    parser.add_argument(
        "--stage-seconds",
        type=float,
        default=3.0,
        help="seconds per fixed stage or continuous leg (default: 3)",
    )
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument(
        "--limit-scan",
        action="store_true",
        help="run the stepped motor-angle limit scan instead of the formal sequence",
    )
    mode_group.add_argument(
        "--continuous-sweep",
        action="store_true",
        help="send a smooth 0,+10,0,-10,0 mm position and velocity sweep",
    )
    args = parser.parse_args()
    if args.stage_seconds <= 0:
        parser.error("--stage-seconds must be greater than zero")
    return args


def main() -> int:
    args = parse_arguments()
    verify_protocol_vector()

    uart = serial.Serial(
        port=args.port,
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=1.0,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    sender = BallLinkSender(uart)
    print("Simulated vision motor test sender opened")
    print(f"  port       = {args.port}")
    print("  format     = 115200-8-N-1")
    print(f"  tx_hz      = {TX_HZ}")
    print(f"  session_id = 0x{sender.session_id:08X}")
    print(f"  run_id     = {sender.run_id}")
    print("Press Ctrl+C at any time to return toward level.")

    try:
        if args.limit_scan:
            run_limit_scan(sender, args.stage_seconds)
        elif args.continuous_sweep:
            run_continuous_sweep(sender, args.stage_seconds)
        else:
            run_sequence(sender, args.stage_seconds)
        print("\nAll simulated stages completed.")
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C received.")
        return 130
    finally:
        send_safe_shutdown(sender)
        uart.close()
        print(
            f"UART closed after {sender.sent_frames} frames. "
            "Press the C-board KEY to disable the motor."
        )


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, serial.SerialException) as error:
        print(f"startup failed: {error}", file=sys.stderr)
        raise SystemExit(1)
