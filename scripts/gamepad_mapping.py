"""Gamepad axis normalization, curves, and Arduino command packing."""

from __future__ import annotations

import time
from dataclasses import replace

from gamepad_types import AXIS_NAMES, CURVE_CHOICES, RANGE_CHOICES, AxisProfile, GamepadConfig


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def validate_config(config: GamepadConfig) -> None:
    if config.command_rate_hz <= 0:
        raise SystemExit("GAMEPAD.command_rate_hz must be greater than 0.")
    if config.joystick_index < 0:
        raise SystemExit("GAMEPAD.joystick_index must be 0 or greater.")

    missing = [name for name in AXIS_NAMES if name not in config.axis_profiles]
    if missing:
        raise SystemExit(f"GAMEPAD.axis_profiles is missing: {', '.join(missing)}")

    for name, profile in config.axis_profiles.items():
        validate_axis_profile(name, profile)


def validate_axis_profile(name: str, profile: AxisProfile) -> None:
    if name not in AXIS_NAMES:
        raise SystemExit(f"Unknown axis profile '{name}'. Choose one of: {', '.join(AXIS_NAMES)}")
    if profile.axis < 0:
        raise SystemExit(f"{name}.axis must be 0 or greater.")
    if profile.input_range not in RANGE_CHOICES:
        raise SystemExit(f"{name}.input_range must be one of: {', '.join(RANGE_CHOICES)}")
    if profile.curve not in CURVE_CHOICES:
        raise SystemExit(f"{name}.curve must be one of: {', '.join(CURVE_CHOICES)}")
    if profile.deadzone < 0 or profile.deadzone >= 1:
        raise SystemExit(f"{name}.deadzone must be at least 0 and less than 1.")
    if profile.max_output < 0 or profile.max_output > 1:
        raise SystemExit(f"{name}.max_output must be between 0 and 1.")
    if profile.curve_power <= 0:
        raise SystemExit(f"{name}.curve_power must be greater than 0.")
    if profile.expo < 0 or profile.expo > 1:
        raise SystemExit(f"{name}.expo must be between 0 and 1.")


def validate_joystick_axes(joystick, config: GamepadConfig) -> None:
    axis_count = joystick.get_numaxes()
    for name, profile in config.axis_profiles.items():
        if profile.axis >= axis_count:
            raise SystemExit(
                f"{name} axis {profile.axis} is invalid for this controller. "
                f"Found axes 0..{axis_count - 1}."
            )


def apply_curve(value: float, profile: AxisProfile) -> float:
    value = clamp(value, -1.0, 1.0)
    magnitude = abs(value)

    if profile.curve == "linear":
        curved = magnitude
    elif profile.curve == "cubic":
        curved = magnitude * magnitude * magnitude
    elif profile.curve == "power":
        curved = magnitude ** max(0.1, profile.curve_power)
    elif profile.curve == "ease":
        curved = magnitude * magnitude * (3.0 - 2.0 * magnitude)
    elif profile.curve == "expo":
        expo = clamp(profile.expo, 0.0, 1.0)
        curved = (1.0 - expo) * magnitude + expo * magnitude * magnitude * magnitude
    else:
        curved = magnitude

    return -curved if value < 0 else curved


def axis_to_command(value: float) -> int:
    return round(clamp(value, -1.0, 1.0) * 1000)


def positive_range(input_range: str) -> bool:
    return input_range.startswith("trigger-") or input_range.startswith("centered")


def normalize_axis_value(raw_value: float, input_range: str) -> float:
    if input_range == "signed":
        return clamp(raw_value, -1.0, 1.0)
    if input_range == "signed-inverted":
        return clamp(-raw_value, -1.0, 1.0)
    if input_range == "trigger-signed":
        return clamp((raw_value + 1.0) / 2.0, 0.0, 1.0)
    if input_range == "trigger-signed-inverted":
        return clamp((1.0 - raw_value) / 2.0, 0.0, 1.0)
    if input_range == "trigger-unsigned":
        return clamp(raw_value, 0.0, 1.0)
    if input_range == "trigger-unsigned-inverted":
        return clamp(-raw_value, 0.0, 1.0)
    if input_range == "centered":
        return clamp(abs(raw_value), 0.0, 1.0)
    if input_range == "centered-positive":
        return clamp(raw_value, 0.0, 1.0)
    if input_range == "centered-negative":
        return clamp(-raw_value, 0.0, 1.0)
    raise ValueError(f"Unresolved input range: {input_range}")


def apply_deadzone(value: float, deadzone: float, is_positive: bool) -> float:
    deadzone = clamp(deadzone, 0.0, 0.999)
    if is_positive:
        if value < deadzone:
            return 0.0
        return clamp((value - deadzone) / (1.0 - deadzone), 0.0, 1.0)
    if abs(value) < deadzone:
        return 0.0
    return value


class GamepadMapper:
    def __init__(self, joystick, profiles: dict[str, AxisProfile]) -> None:
        self.joystick = joystick
        self.profiles = self._resolve_profiles(profiles)

    def _sample_resting_axis(self, axis: int) -> float:
        values = []
        try:
            import pygame
        except ImportError:
            return self.joystick.get_axis(axis)

        # pygame can report 0 for joystick axes immediately after init until
        # SDL has processed a few events. Sampling briefly makes auto-trigger
        # detection much less likely to misclassify Xbox -1..+1 triggers.
        deadline = time.monotonic() + 0.12
        while time.monotonic() < deadline:
            pygame.event.pump()
            values.append(self.joystick.get_axis(axis))
            time.sleep(0.01)

        if not values:
            return self.joystick.get_axis(axis)
        return max(values, key=abs)

    def _resolve_profiles(self, profiles: dict[str, AxisProfile]) -> dict[str, AxisProfile]:
        resolved = dict(profiles)
        for name, profile in profiles.items():
            if profile.input_range != "auto-trigger":
                continue

            resting_value = self._sample_resting_axis(profile.axis)
            if resting_value < -0.25:
                input_range = "trigger-signed"
            elif resting_value > 0.25:
                input_range = "trigger-signed-inverted"
            else:
                input_range = "centered"
            resolved[name] = replace(profile, input_range=input_range)

        left = resolved["left_trigger"]
        right = resolved["right_trigger"]
        if (
            left.axis == right.axis
            and left.input_range == "centered"
            and right.input_range == "centered"
        ):
            resolved["left_trigger"] = replace(left, input_range="centered-negative")
            resolved["right_trigger"] = replace(right, input_range="centered-positive")

        return resolved

    def read_profile(self, name: str) -> float:
        profile = self.profiles[name]
        raw_value = self.joystick.get_axis(profile.axis)
        value = normalize_axis_value(raw_value, profile.input_range)
        if profile.invert:
            value = -value
        value = apply_deadzone(value, profile.deadzone, positive_range(profile.input_range))
        value = apply_curve(value, profile)
        return clamp(value * clamp(profile.max_output, 0.0, 1.0), -1.0, 1.0)

    def command(self) -> str:
        forward = self.read_profile("forward")
        turn = self.read_profile("turn")
        left_trigger = self.read_profile("left_trigger")
        right_trigger = self.read_profile("right_trigger")
        strafe = right_trigger - left_trigger
        return (
            f"C:{axis_to_command(forward)},"
            f"{axis_to_command(turn)},"
            f"{axis_to_command(strafe)}\n"
        )

    def describe(self) -> str:
        parts = []
        for name in AXIS_NAMES:
            profile = self.profiles[name]
            parts.append(
                f"{name}=axis{profile.axis}/{profile.input_range}/"
                f"{profile.curve}/deadzone={profile.deadzone:g}/max={profile.max_output:g}"
            )
        return "; ".join(parts)
