import unittest

from robot_control.input import SDLControllerInput


class FakePygame:
    CONTROLLER_AXIS_TRIGGERLEFT = 4
    CONTROLLER_AXIS_TRIGGERRIGHT = 5
    CONTROLLER_BUTTON_A = 10
    CONTROLLER_BUTTON_B = 11
    CONTROLLER_BUTTON_X = 12
    CONTROLLER_BUTTON_Y = 13
    CONTROLLER_BUTTON_LEFTSHOULDER = 19
    CONTROLLER_BUTTON_RIGHTSHOULDER = 20
    CONTROLLER_BUTTON_BACK = 14
    CONTROLLER_BUTTON_START = 16
    CONTROLLER_BUTTON_DPAD_DOWN = 22
    CONTROLLER_BUTTON_DPAD_LEFT = 23
    CONTROLLER_BUTTON_DPAD_RIGHT = 24
    CONTROLLER_BUTTON_DPAD_UP = 21


class FakeController:
    name = "Xbox One Elite 2 Controller"

    def __init__(self):
        self.axes = {0: 16384, 4: 0, 5: 32768}
        self.buttons = {
            FakePygame.CONTROLLER_BUTTON_START: True,
            FakePygame.CONTROLLER_BUTTON_DPAD_LEFT: True,
            FakePygame.CONTROLLER_BUTTON_DPAD_UP: True,
        }

    @staticmethod
    def get_init():
        return True

    @staticmethod
    def attached():
        return True

    def get_axis(self, index):
        return self.axes.get(index, 0)

    def get_button(self, button):
        return self.buttons.get(button, False)


class SDLControllerInputTests(unittest.TestCase):
    def test_named_start_maps_to_logical_menu_without_raw_button_guess(self):
        controller = SDLControllerInput(FakePygame, FakeController())
        self.assertTrue(controller.get_init())
        self.assertEqual(
            controller.get_name(),
            "Xbox One Elite 2 Controller [SDL mapped]",
        )
        self.assertTrue(controller.get_button(7))
        self.assertFalse(controller.get_button(6))
        self.assertEqual(controller.get_hat(0), (-1, 1))

    def test_axes_are_normalized_to_existing_minus_one_to_one_contract(self):
        controller = SDLControllerInput(FakePygame, FakeController())
        self.assertAlmostEqual(controller.get_axis(0), 0.5, places=3)
        self.assertEqual(controller.get_axis(4), -1.0)
        self.assertEqual(controller.get_axis(5), 1.0)


if __name__ == "__main__":
    unittest.main()
