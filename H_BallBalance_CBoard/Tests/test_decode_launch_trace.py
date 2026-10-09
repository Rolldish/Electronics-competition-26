import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "Tools"))
from decode_launch_trace import decode_trace


def header(*, generation=4, count=2, state=2, start=1000, magic=0x52544248):
    return struct.pack("<IHH13I3f2I", magic, 1, 80, 64, 1000, 40,
                       generation, state, count, 17, 9, 4, start, 1040, 0, 1,
                       1.2461, .03, .09, 2, 1)


def record(now, *, count=0, accel=.25, launch=-2, gate=0):
    return struct.pack("<III9fhhHH8B", now, 10, count,
                       accel, .26, 1.24, 1.20, 1.23, .1, -.5, launch, -2.4,
                       500, -15, 60, 0, 5, 0, gate, 3, 3, 3, 0xff, 0)


class DecodeTests(unittest.TestCase):
    def test_retains_distinct_angles_and_diagnostic_fields(self):
        h = header()
        metadata, rows = decode_trace(h, record(1000) + record(1040, count=1), h)
        self.assertEqual(metadata["task_id"], 4)
        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[0]["desired_angle_rad"], 1.20, places=5)
        self.assertAlmostEqual(rows[0]["sent_angle_rad"], 1.23, places=5)
        self.assertEqual(rows[0]["position_cm"], 5)
        self.assertAlmostEqual(rows[1]["elapsed_s"], .04)
        self.assertEqual(rows[1]["trigger_count"], 1)
        self.assertEqual(rows[0]["vision_age_ms"], 60)

    def test_rejects_mutating_trace(self):
        with self.assertRaisesRegex(ValueError, "changed"):
            decode_trace(header(), record(1000) * 2, header(generation=6))

    def test_rejects_incomplete_publication(self):
        h = header(generation=5)
        with self.assertRaisesRegex(ValueError, "publication"):
            decode_trace(h, record(1000) * 2, h)

    def test_rejects_recording_before_freeze(self):
        h = header(state=1)
        with self.assertRaisesRegex(ValueError, "recording"):
            decode_trace(h, record(1000) * 2, h)

    def test_rejects_old_firmware_and_truncated_dump(self):
        h = header(magic=0)
        with self.assertRaisesRegex(ValueError, "firmware"):
            decode_trace(h, record(1000) * 2, h)
        h = header()
        with self.assertRaisesRegex(ValueError, "truncated"):
            decode_trace(h, record(1000), h)

    def test_rejects_count_overflow(self):
        h = header(count=1001)
        with self.assertRaisesRegex(ValueError, "capacity"):
            decode_trace(h, b"", h)

    def test_ignores_unused_old_array_entries(self):
        h = header(count=1)
        _, rows = decode_trace(h, record(1000) + b"old bytes", h)
        self.assertEqual(len(rows), 1)

    def test_timestamp_wrap(self):
        h = header(start=0xfffffff0)
        _, rows = decode_trace(h, record(0xfffffff0) + record(24), h)
        self.assertAlmostEqual(rows[1]["elapsed_s"], .04)

    def test_invalid_imu_value_is_kept_for_diagnosis(self):
        import math
        h = header(count=1)
        _, rows = decode_trace(h, record(1000, accel=float("nan"), gate=9), h)
        self.assertTrue(math.isnan(rows[0]["accel_filtered_mps2"]))
        self.assertEqual(rows[0]["ff_gate"], 9)


if __name__ == "__main__":
    unittest.main()
