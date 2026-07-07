"""Small shared data types for the gamepad motor bridge."""

from __future__ import annotations

from dataclasses import dataclass

AXIS_NAMES = ("forward", "turn", "left_trigger", "right_trigger")
CURVE_CHOICES = ("linear", "expo", "cubic", "power", "ease")
RANGE_CHOICES = (
    "auto-trigger",
    "signed",
    "signed-inverted",
    "trigger-signed",
    "trigger-signed-inverted",
    "trigger-unsigned",
    "trigger-unsigned-inverted",
    "centered",
    "centered-positive",
    "centered-negative",
)


@dataclass(frozen=True)
class AxisProfile:
    axis: int
    input_range: str
    invert: bool = False
    deadzone: float = 0.08
    max_output: float = 1.0
    curve: str = "linear"
    curve_power: float = 2.0
    expo: float = 0.35


@dataclass(frozen=True)
class GamepadConfig:
    joystick_index: int
    command_rate_hz: float
    open_input_window: bool
    monitor_only: bool
    print_changed_commands: bool
    axis_profiles: dict[str, AxisProfile]
