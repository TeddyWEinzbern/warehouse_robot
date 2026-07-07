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
            # sign(x) * x^1.5: gentler around center while keeping full output
            # at the end of travel.
            curve="power",
            curve_power=1.5,
        ),
        "turn": AxisProfile(
            axis=0,
            input_range="signed",
            deadzone=0.08,
            max_output=1.0,
            curve="power",
            curve_power=1.5,
        ),
        "left_trigger": AxisProfile(
            axis=4,
            # Xbox controllers commonly expose each trigger as -1 at rest and
            # +1 fully pressed. Keeping this explicit avoids startup auto-detect
            # races where pygame has not reported the resting -1 value yet.
            input_range="trigger-signed",
            deadzone=0.08,
            max_output=1.0,
            curve="linear",
        ),
        "right_trigger": AxisProfile(
            axis=5,
            input_range="trigger-signed",
            deadzone=0.08,
            max_output=1.0,
            curve="linear",
        ),
    },
)
