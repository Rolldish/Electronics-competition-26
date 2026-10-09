import importlib.util
import math
import pathlib
import sys
import types
import unittest


SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "Tools"
    / "Simulate-BallControl-RaspberryPi.py"
)


def load_sender_module():
    sys.modules.setdefault("serial", types.ModuleType("serial"))
    spec = importlib.util.spec_from_file_location(
        "simulate_ball_control_raspberrypi", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class SmoothTransitionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sender = load_sender_module()

    def test_half_cosine_transition_has_continuous_position_and_velocity(self):
        start = self.sender.smooth_transition_snapshot(0, 100, 4.0, 0.0)
        middle = self.sender.smooth_transition_snapshot(0, 100, 4.0, 2.0)
        end = self.sender.smooth_transition_snapshot(0, 100, 4.0, 4.0)

        self.assertEqual(start.position_0p1mm, 0)
        self.assertEqual(start.velocity_mmps, 0)
        self.assertEqual(middle.position_0p1mm, 50)
        self.assertEqual(
            middle.velocity_mmps,
            round(10.0 * math.pi / (2.0 * 4.0)),
        )
        self.assertEqual(end.position_0p1mm, 100)
        self.assertEqual(end.velocity_mmps, 0)
        self.assertEqual(
            middle.vision_flags,
            self.sender.VALID_MOVING_BALL_FLAGS,
        )
        self.assertEqual(
            middle.control_flags,
            self.sender.ACTIVE_CONTROL_FLAGS,
        )

    def test_reverse_transition_reports_negative_velocity(self):
        middle = self.sender.smooth_transition_snapshot(100, 0, 4.0, 2.0)

        self.assertEqual(middle.position_0p1mm, 50)
        self.assertLess(middle.velocity_mmps, 0)


if __name__ == "__main__":
    unittest.main()
