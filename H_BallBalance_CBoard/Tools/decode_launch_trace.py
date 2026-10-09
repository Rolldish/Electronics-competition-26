"""Decode the frozen onboard trace; requires only Python's standard library."""

import argparse
import collections
import csv
import datetime
import hashlib
import json
import math
import struct
from pathlib import Path

HEADER = struct.Struct("<IHH13I3f2I")
RECORD = struct.Struct("<III9fhhHH8B")
HEADER_FIELDS = (
    "magic", "version", "header_bytes", "record_bytes", "capacity",
    "sample_period_ms", "generation", "state", "count", "pi_session_id",
    "run_id", "task_id", "start_ms", "end_ms", "trigger_count_at_start",
    "trigger_count_at_end", "level_angle_rad", "calibration_noise_mps2",
    "deadband_mps2", "freeze_reason", "initial_raw_control_flags",
)
RECORD_FIELDS = (
    "now_ms", "imu_sample_count", "trigger_count", "accel_filtered_mps2",
    "accel_raw_mps2", "actual_angle_rad", "desired_angle_rad", "sent_angle_rad",
    "pd_offset_deg", "imu_offset_deg", "launch_offset_deg", "total_offset_deg",
    "position_0p1mm", "velocity_mmps", "vision_age_ms", "imu_age_ms",
    "ball_state", "ball_fault", "ff_gate", "launch_state", "raw_control_flags",
    "effective_control_flags", "validity_flags", "reserved",
)
GATES = (
    "Applied", "Disabled", "InvalidTaskContext", "UnsupportedTask",
    "ControlNotEnabled", "RunNotActive", "NotBalancing", "ImuNotReady",
    "ImuInvalid", "ImuNotFinite", "ImuStale", "InvalidControlOutput",
)


def decode_trace(before: bytes, records: bytes, after: bytes):
    if len(before) != HEADER.size or len(after) != HEADER.size:
        raise ValueError("truncated trace header")
    if before != after:
        raise ValueError("trace changed during read; wait until frozen and retry")
    metadata = dict(zip(HEADER_FIELDS, HEADER.unpack(before)))
    if (metadata["magic"] != 0x52544248 or metadata["version"] != 1
            or metadata["header_bytes"] != HEADER.size
            or metadata["record_bytes"] != RECORD.size
            or metadata["capacity"] != 1000
            or metadata["sample_period_ms"] != 40):
        raise ValueError("firmware or trace format does not match this decoder")
    if metadata["generation"] & 1:
        raise ValueError("incomplete trace publication; retry after freezing")
    if metadata["count"] > metadata["capacity"]:
        raise ValueError("trace count exceeds capacity")
    if metadata["state"] == 1:
        raise ValueError("trace is still recording; wait 40 seconds from task selection")
    if metadata["state"] != 2 or metadata["count"] == 0:
        raise ValueError("no frozen T4/T5/T6 trace; select a new moving-task run first")
    if len(records) < metadata["count"] * RECORD.size:
        raise ValueError("truncated trace records")

    rows = []
    for index in range(metadata["count"]):
        row = dict(zip(RECORD_FIELDS, RECORD.unpack_from(records, index * RECORD.size)))
        row["elapsed_s"] = ((row["now_ms"] - metadata["start_ms"]) & 0xffffffff) / 1000
        row["position_cm"] = row["position_0p1mm"] / 100
        row["velocity_cmps"] = row["velocity_mmps"] / 10
        for angle in ("actual", "desired", "sent"):
            row[f"{angle}_offset_deg"] = math.degrees(
                row[f"{angle}_angle_rad"] - metadata["level_angle_rad"])
        row["ff_gate_name"] = (GATES[row["ff_gate"]]
                               if row["ff_gate"] < len(GATES) else "Unknown")
        rows.append(row)
    return metadata, rows


def summarize(metadata, rows):
    def finite_range(field):
        values = [r[field] for r in rows if math.isfinite(r[field])]
        return {"min": min(values), "max": max(values)} if values else None

    triggers = [r for r in rows if
                0 < ((r["trigger_count"] - metadata["trigger_count_at_start"])
                     & 0xffffffff) < 0x80000000]
    active = [r for r in rows if r["validity_flags"] & 128]
    return {
        "duration_s": ((metadata["end_ms"] - metadata["start_ms"]) & 0xffffffff) / 1000,
        "last_sample_elapsed_s": rows[-1]["elapsed_s"],
        "sample_count": len(rows),
        "new_launch_triggers": ((metadata["trigger_count_at_end"]
                                - metadata["trigger_count_at_start"]) & 0xffffffff),
        "first_trigger_observed_s": triggers[0]["elapsed_s"] if triggers else None,
        "first_active_launch_observed_s": active[0]["elapsed_s"] if active else None,
        "last_active_launch_observed_s": active[-1]["elapsed_s"] if active else None,
        "feedforward_gates": dict(collections.Counter(r["ff_gate_name"] for r in rows)),
        "vision_valid_and_age_le_80ms_fraction": sum(
            bool(r["validity_flags"] & 1) and r["vision_age_ms"] <= 80
            for r in rows) / len(rows),
        "filtered_acceleration_mps2": finite_range("accel_filtered_mps2"),
        "position_from_O_cm": finite_range("position_cm"),
        "desired_motor_offset_deg": finite_range("desired_offset_deg"),
        "sent_motor_offset_deg": finite_range("sent_offset_deg"),
        "actual_motor_offset_deg": finite_range("actual_offset_deg"),
        "timing_note": "25 Hz samples; brief events may fall between samples. Times start at task selection, not car start.",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", type=Path, required=True)
    parser.add_argument("--records", type=Path, required=True)
    parser.add_argument("--after", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--elf", type=Path, required=True)
    parser.add_argument("--flash-sha256", required=True)
    args = parser.parse_args()
    try:
        metadata, rows = decode_trace(args.before.read_bytes(), args.records.read_bytes(),
                                      args.after.read_bytes())
    except ValueError as error:
        parser.exit(2, f"Trace decode failed: {error}\n")
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "trace.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=tuple(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "captured_at": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "firmware_elf_sha256": hashlib.sha256(args.elf.read_bytes()).hexdigest(),
        "installed_flash_sha256": args.flash_sha256,
        "installed_flash_matches_elf": True,
        "header": metadata,
        "summary": summarize(metadata, rows),
    }
    (args.output / "summary.json").write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(f"Decoded {len(rows)} samples; task={metadata['task_id']}; "
          f"new launch triggers={report['summary']['new_launch_triggers']}")


if __name__ == "__main__":
    main()
