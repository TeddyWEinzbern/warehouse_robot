"""Edit this file to tune the gamepad-to-motor bridge.

The command line intentionally only keeps serial/test options. Controller
mapping, curves, deadzones, output limits, joystick selection, and diagnostic
monitor mode live here so a known-good setup can be saved in the project.
"""

from __future__ import annotations

from gamepad_types import AxisProfile, GamepadConfig

HEARTBEAT_INTERVAL_SECONDS = 0.1

GAMEPAD = GamepadConfig(
    joystick_index=0,
    command_rate_hz=30.0,
    open_input_window=False,
    monitor_only=False,
    print_changed_commands=True,
    axis_profiles={
        # Common Xbox SDL mapping on macOS:
        # left stick X=0, left stick Y=1, LT=4, RT=5.
        "forward": AxisProfile(
            axis=1,
            input_range="signed",
            invert=True,
            deadzone=0.08,
            max_output=1.0,
            curve="linear",
        ),
        "turn": AxisProfile(
            axis=0,
            input_range="signed",
            deadzone=0.08,
            max_output=1.0,
            curve="linear",
        ),
        "left_trigger": AxisProfile(
            axis=4,
            input_range="auto-trigger",
            deadzone=0.08,
            max_output=1.0,
            curve="linear",
        ),
        "right_trigger": AxisProfile(
            axis=5,
            input_range="auto-trigger",
            deadzone=0.08,
            max_output=1.0,
            curve="linear",
        ),
    },
)
