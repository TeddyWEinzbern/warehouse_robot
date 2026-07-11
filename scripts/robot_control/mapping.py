"""Map an Xbox-style pygame device into a complete robot control frame."""

from __future__ import annotations

from .config import AxisConfig, RobotControlConfig
from .protocol import ControlFrame

PRESET_LEFT = 1 << 0
PRESET_FRONT = 1 << 1
PRESET_RIGHT = 1 << 2
PRESET_STOW = 1 << 3
START_ASSIST = 1 << 4
CANCEL_ASSIST = 1 << 5
EMERGENCY_STOP = 1 << 6
CLEAR_EMERGENCY_STOP = 1 << 7


def _axis(joystick, config: AxisConfig) -> int:
    value = max(-1.0, min(1.0, joystick.get_axis(config.axis)))
    if config.invert:
        value = -value
    if abs(value) < config.deadzone:
        return 0
    magnitude = (abs(value) - config.deadzone) / (1.0 - config.deadzone)
    curved = magnitude ** config.power
    return round((-curved if value < 0 else curved) * 1000)


def _trigger(joystick, axis: int) -> float:
    return max(0.0, min(1.0, (joystick.get_axis(axis) + 1.0) / 2.0))


def _button(joystick, index: int) -> bool:
    return index < joystick.get_numbuttons() and bool(joystick.get_button(index))


def map_gamepad(joystick, config: RobotControlConfig, sequence: int) -> ControlFrame:
    left_trigger = _trigger(joystick, config.left_trigger_axis)
    right_trigger = _trigger(joystick, config.right_trigger_axis)
    height = int((_button(joystick, config.arm_height_up_button) -
                  _button(joystick, config.arm_height_down_button)) * 1000)
    gripper = int(_button(joystick, config.gripper_close_button)) - int(
        _button(joystick, config.gripper_open_button)
    )
    buttons = 0
    hat = joystick.get_hat(0) if joystick.get_numhats() else (0, 0)
    if hat[0] < 0: buttons |= PRESET_LEFT
    if hat[1] > 0: buttons |= PRESET_FRONT
    if hat[0] > 0: buttons |= PRESET_RIGHT
    if hat[1] < 0: buttons |= PRESET_STOW
    if _button(joystick, config.start_assist_button): buttons |= START_ASSIST
    if _button(joystick, config.cancel_assist_button): buttons |= CANCEL_ASSIST
    if _button(joystick, config.estop_button): buttons |= EMERGENCY_STOP
    if _button(joystick, config.clear_estop_button): buttons |= CLEAR_EMERGENCY_STOP
    return ControlFrame(
        sequence=sequence,
        forward=_axis(joystick, config.drive_forward),
        turn=_axis(joystick, config.drive_turn),
        strafe=round((right_trigger - left_trigger) * 1000),
        arm_yaw=_axis(joystick, config.arm_yaw),
        arm_reach=_axis(joystick, config.arm_reach),
        arm_height=height,
        gripper=gripper,
        buttons=buttons,
    )

