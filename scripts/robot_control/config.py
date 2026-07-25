"""Saved controller mapping for the complete robot."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class AxisConfig:
    axis: int
    invert: bool = False
    deadzone: float = 0.08
    power: float = 1.5


@dataclass(frozen=True)
class RobotControlConfig:
    joystick_index: int = 0
    drive_forward: AxisConfig = field(default_factory=lambda: AxisConfig(1, invert=True))
    drive_turn: AxisConfig = field(default_factory=lambda: AxisConfig(0))
    arm_yaw: AxisConfig = field(default_factory=lambda: AxisConfig(2))
    arm_reach: AxisConfig = field(default_factory=lambda: AxisConfig(3, invert=True))
    left_trigger_axis: int = 4
    right_trigger_axis: int = 5
    arm_height_down_button: int = 4  # LB
    arm_height_up_button: int = 5  # RB
    gripper_open_button: int = 0  # A
    gripper_close_button: int = 1  # B
    cancel_assist_button: int = 2  # X
    start_assist_button: int = 3  # Y
    clear_estop_button: int = 6  # View
    estop_button: int = 7  # Menu


CONTROL = RobotControlConfig()
