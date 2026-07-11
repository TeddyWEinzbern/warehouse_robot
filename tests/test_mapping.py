import unittest

from robot_control.config import CONTROL
from robot_control.mapping import PRESET_FRONT, map_gamepad


class FakeJoystick:
    def __init__(self):
        self.axes = [0.25, -0.5, 0.1, -0.2, -1.0, 1.0]
        self.buttons = [0] * 10
        self.hat = (0, 1)

    def get_axis(self, index): return self.axes[index]
    def get_numbuttons(self): return len(self.buttons)
    def get_button(self, index): return self.buttons[index]
    def get_numhats(self): return 1
    def get_hat(self, index): return self.hat


class MappingTests(unittest.TestCase):
    def test_mapping_combines_drive_arm_and_buttons(self):
        frame = map_gamepad(FakeJoystick(), CONTROL, 7)
        self.assertEqual(frame.sequence, 7)
        self.assertGreater(frame.forward, 0)
        self.assertGreater(frame.turn, 0)
        self.assertEqual(frame.strafe, 1000)
        self.assertTrue(frame.buttons & PRESET_FRONT)


if __name__ == "__main__":
    unittest.main()
