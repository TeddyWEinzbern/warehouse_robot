#!/usr/bin/env python3
"""Bridge a computer-connected gamepad to the Arduino L293D motor sketch."""

from __future__ import annotations

import argparse
import time

HEARTBEAT_INTERVAL_SECONDS = 0.1
DEFAULT_RAW_COMMAND_DURATION_SECONDS = 1.0
DEFAULT_RAW_COMMAND_INTERVAL_SECONDS = 0.1
TRIGGER_MODE_CHOICES = (
    "auto",
    "signed",
    "signed-inverted",
    "unsigned",
    "unsigned-inverted",
    "centered",
    "centered-positive",
    "centered-negative",
)

try:
    import pygame
except ImportError as exc:
    pygame = None
    PYGAME_IMPORT_ERROR = exc
else:
    PYGAME_IMPORT_ERROR = None

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pyserial. Install with: python3 -m pip install pygame pyserial"
    ) from exc


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send left-stick forward/turn values to an Arduino as "
            "C:<forward>,<turn>,<strafe> commands. Trigger strafe is only used "
            "when the Arduino sketch is set to mecanum mixing."
        )
    )
    parser.add_argument(
        "--port",
        help=(
            "Arduino USB or Bluetooth serial port, for example "
            "/dev/cu.usbmodemXXXX, /dev/cu.HC-05-DevB, or COM3."
        ),
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List available serial ports and exit.",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help="Show live values for every gamepad axis and button without opening serial.",
    )
    parser.add_argument(
        "--window",
        action="store_true",
        help="Open a small pygame window for input focus. Useful on macOS if values stay zero.",
    )
    parser.add_argument("--left-x-axis", type=int, default=0, help="Left stick X axis. Default: 0.")
    parser.add_argument("--left-y-axis", type=int, default=1, help="Left stick Y axis. Default: 1.")
    parser.add_argument(
        "--left-trigger-axis",
        type=int,
        default=4,
        help="Left trigger axis for left strafe. Default: 4, matching common Xbox SDL mappings.",
    )
    parser.add_argument(
        "--right-trigger-axis",
        type=int,
        default=5,
        help="Right trigger axis for right strafe. Default: 5.",
    )
    parser.add_argument(
        "--trigger-mode",
        choices=TRIGGER_MODE_CHOICES,
        default="auto",
        help="Default trigger axis range/direction. Default: auto.",
    )
    parser.add_argument(
        "--left-trigger-mode",
        choices=TRIGGER_MODE_CHOICES,
        help="Override trigger mode for LT only. Defaults to --trigger-mode.",
    )
    parser.add_argument(
        "--right-trigger-mode",
        choices=TRIGGER_MODE_CHOICES,
        help="Override trigger mode for RT only. Defaults to --trigger-mode.",
    )
    parser.add_argument(
        "--deadzone",
        type=float,
        default=0.08,
        help="Ignore tiny joystick movement around center. Default: 0.08.",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=30.0,
        help="Maximum commands per second. Default: 30.",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. USB defaults to 115200; HC-05/HC-06 usually uses 9600.",
    )
    parser.add_argument(
        "--joystick",
        type=int,
        default=0,
        help="Joystick device index when multiple controllers are connected. Default: 0.",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Print every command, even when unchanged.",
    )
    parser.add_argument(
        "--send-command",
        help=(
            "Send one raw command such as C:400,0,0 repeatedly for --duration seconds, "
            "then send C:0,0,0 and exit. This bypasses gamepad input."
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=DEFAULT_RAW_COMMAND_DURATION_SECONDS,
        help="Duration for --send-command in seconds. Default: 1.0.",
    )
    parser.add_argument(
        "--repeat-interval",
        type=float,
        default=DEFAULT_RAW_COMMAND_INTERVAL_SECONDS,
        help="Repeat interval for --send-command in seconds. Default: 0.1.",
    )
    return parser.parse_args()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    return value


def axis_to_command(value: float) -> int:
    return round(clamp(value, -1.0, 1.0) * 1000)


def trigger_to_unit(value: float, mode: str) -> float:
    if mode == "signed":
        return clamp((value + 1.0) / 2.0, 0.0, 1.0)
    if mode == "signed-inverted":
        return clamp((1.0 - value) / 2.0, 0.0, 1.0)
    if mode == "unsigned-inverted":
        return clamp(-value, 0.0, 1.0)
    if mode == "centered":
        return clamp(abs(value), 0.0, 1.0)
    if mode == "centered-positive":
        return clamp(value, 0.0, 1.0)
    if mode == "centered-negative":
        return clamp(-value, 0.0, 1.0)
    return clamp(value, 0.0, 1.0)


def apply_trigger_deadzone(value: float, deadzone: float) -> float:
    if value < deadzone:
        return 0.0
    return clamp((value - deadzone) / (1.0 - deadzone), 0.0, 1.0)


def resolve_trigger_mode(joystick: pygame.joystick.Joystick, axis: int, mode: str) -> str:
    if mode != "auto":
        return mode
    resting_value = joystick.get_axis(axis)
    if resting_value < -0.25:
        return "signed"
    if resting_value > 0.25:
        return "signed-inverted"
    return "centered"


def resolve_requested_mode(specific_mode: str | None, default_mode: str) -> str:
    return specific_mode or default_mode


def format_available_ports() -> str:
    ports = list(list_ports.comports())
    if not ports:
        return "  No serial ports found."
    return "\n".join(f"  {port.device} - {port.description}" for port in ports)


def open_serial_port(port: str, baud: int) -> serial.Serial:
    try:
        return serial.Serial(port, baud, timeout=1)
    except serial.SerialException as exc:
        raise SystemExit(
            f"Could not open serial port {port}: {exc}\n\n"
            "Available serial ports:\n"
            f"{format_available_ports()}\n\n"
            "Use the Arduino USB port or Bluetooth serial port, usually "
            "/dev/cu.usbmodem*, /dev/cu.usbserial*, or /dev/cu.HC-* on macOS."
        ) from exc


def normalize_raw_command(command: str) -> str:
    command = command.strip()
    if not command:
        raise SystemExit("--send-command cannot be empty.")
    return f"{command}\n"


def send_raw_command(args: argparse.Namespace) -> int:
    if not args.port:
        raise SystemExit("Missing --port. Use --list-ports to see available serial ports.")
    if args.duration <= 0:
        raise SystemExit("--duration must be greater than 0.")
    if args.repeat_interval <= 0:
        raise SystemExit("--repeat-interval must be greater than 0.")

    command = normalize_raw_command(args.send_command)
    stop_command = "C:0,0,0\n"
    deadline = time.monotonic() + args.duration

    with open_serial_port(args.port, args.baud) as arduino:
        print(
            f"Sending {command.strip()} to {args.port} at {args.baud} baud "
            f"for {args.duration:0.1f}s."
        )
        while time.monotonic() < deadline:
            arduino.write(command.encode("ascii"))
            arduino.flush()
            time.sleep(args.repeat_interval)
        arduino.write(stop_command.encode("ascii"))
        arduino.flush()
    print("Sent C:0,0,0 stop command.")
    return 0


def require_pygame() -> None:
    if pygame is None:
        raise SystemExit(
            "Missing dependency: pygame. Install with: python3 -m pip install pygame pyserial"
        ) from PYGAME_IMPORT_ERROR


def open_joystick(index: int, use_window: bool) -> pygame.joystick.Joystick:
    require_pygame()
    pygame.init()
    if use_window:
        pygame.display.set_mode((420, 140))
        pygame.display.set_caption("Gamepad Motor Input")

    pygame.joystick.init()

    count = pygame.joystick.get_count()
    if count == 0:
        raise SystemExit("No gamepad found. Connect a USB/Bluetooth controller and try again.")
    if index < 0 or index >= count:
        raise SystemExit(f"Joystick index {index} is invalid. Found {count} controller(s).")

    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    print(f"Using gamepad {index}: {joystick.get_name()}")
    print(f"Axes available: {joystick.get_numaxes()}")
    print(f"Buttons available: {joystick.get_numbuttons()}")
    return joystick


def validate_axis(joystick: pygame.joystick.Joystick, axis: int, name: str) -> None:
    if axis < 0 or axis >= joystick.get_numaxes():
        raise SystemExit(
            f"{name} axis {axis} is invalid for this controller. "
            f"Found axes 0..{joystick.get_numaxes() - 1}."
        )


def monitor_joystick(joystick: pygame.joystick.Joystick, rate: float) -> int:
    interval = 1.0 / rate
    previous_state = None
    last_heartbeat = 0.0
    print("Move sticks/triggers and press buttons. Press Ctrl+C to stop.")
    print("If you used --window, click the pygame window once so it has focus.")

    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                print(f"event axis a{event.axis}={event.value:+0.3f}")
            elif event.type == pygame.JOYBUTTONDOWN:
                print(f"event button b{event.button}=1")
            elif event.type == pygame.JOYBUTTONUP:
                print(f"event button b{event.button}=0")
            elif event.type == pygame.JOYHATMOTION:
                print(f"event hat h{event.hat}={event.value}")
            elif event.type == pygame.QUIT:
                return 0

        axes = tuple(round(joystick.get_axis(axis), 3) for axis in range(joystick.get_numaxes()))
        buttons = tuple(joystick.get_button(button) for button in range(joystick.get_numbuttons()))
        hats = tuple(joystick.get_hat(hat) for hat in range(joystick.get_numhats()))
        state = (axes, buttons, hats)
        now = time.monotonic()

        if state != previous_state or now - last_heartbeat >= 2.0:
            axes_text = " ".join(f"a{axis}:{value:+0.3f}" for axis, value in enumerate(axes))
            buttons_text = " ".join(f"b{button}:{value}" for button, value in enumerate(buttons))
            hats_text = " ".join(f"h{hat}:{value}" for hat, value in enumerate(hats)) or "no hats"
            print(f"state {axes_text} | {buttons_text} | {hats_text}")
            previous_state = state
            last_heartbeat = now

        time.sleep(interval)


def main() -> int:
    args = parse_args()

    if args.list_ports:
        print("Available serial ports:")
        print(format_available_ports())
        return 0

    if args.rate <= 0:
        raise SystemExit("--rate must be greater than 0.")
    if args.deadzone < 0 or args.deadzone >= 1:
        raise SystemExit("--deadzone must be at least 0 and less than 1.")

    if args.send_command:
        return send_raw_command(args)

    joystick = open_joystick(args.joystick, args.window)

    if args.monitor:
        return monitor_joystick(joystick, args.rate)

    if not args.port:
        raise SystemExit("Missing --port. Use --list-ports to see available serial ports.")

    validate_axis(joystick, args.left_x_axis, "Left stick X")
    validate_axis(joystick, args.left_y_axis, "Left stick Y")
    validate_axis(joystick, args.left_trigger_axis, "Left trigger")
    validate_axis(joystick, args.right_trigger_axis, "Right trigger")

    arduino = open_serial_port(args.port, args.baud)

    interval = 1.0 / args.rate
    last_command: str | None = None
    last_sent_at = 0.0

    with arduino:
        time.sleep(2.0)
        requested_left_trigger_mode = resolve_requested_mode(
            args.left_trigger_mode, args.trigger_mode
        )
        requested_right_trigger_mode = resolve_requested_mode(
            args.right_trigger_mode, args.trigger_mode
        )
        left_trigger_mode = resolve_trigger_mode(
            joystick, args.left_trigger_axis, requested_left_trigger_mode
        )
        right_trigger_mode = resolve_trigger_mode(
            joystick, args.right_trigger_axis, requested_right_trigger_mode
        )
        split_centered_trigger_axis = (
            args.left_trigger_axis == args.right_trigger_axis
            and left_trigger_mode == "centered"
            and right_trigger_mode == "centered"
        )

        print(f"Sending gamepad commands to {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
        print(
            "Left stick Y = forward/back, left stick X = rotate. "
            "LT/RT strafe is ignored by the Arduino unless mecanum mixing is enabled."
        )
        print(f"Trigger modes: left={left_trigger_mode}, right={right_trigger_mode}.")

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 0

            forward = -apply_deadzone(joystick.get_axis(args.left_y_axis), args.deadzone)
            turn = apply_deadzone(joystick.get_axis(args.left_x_axis), args.deadzone)

            left_trigger_raw = joystick.get_axis(args.left_trigger_axis)
            right_trigger_raw = joystick.get_axis(args.right_trigger_axis)
            if split_centered_trigger_axis:
                left_trigger = trigger_to_unit(left_trigger_raw, "centered-negative")
                right_trigger = trigger_to_unit(right_trigger_raw, "centered-positive")
            else:
                left_trigger = trigger_to_unit(left_trigger_raw, left_trigger_mode)
                right_trigger = trigger_to_unit(right_trigger_raw, right_trigger_mode)

            left_trigger = apply_trigger_deadzone(left_trigger, args.deadzone)
            right_trigger = apply_trigger_deadzone(right_trigger, args.deadzone)
            strafe = right_trigger - left_trigger

            command = (
                f"C:{axis_to_command(forward)},"
                f"{axis_to_command(turn)},"
                f"{axis_to_command(strafe)}\n"
            )

            command_changed = command != last_command
            now = time.monotonic()
            heartbeat_due = now - last_sent_at >= HEARTBEAT_INTERVAL_SECONDS
            if command_changed or heartbeat_due:
                arduino.write(command.encode("ascii"))
                last_command = command
                last_sent_at = now

            if args.verbose or command_changed:
                print(command.strip(), end="\r", flush=True)

            time.sleep(interval)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        raise SystemExit(0)
    finally:
        if pygame is not None:
            pygame.quit()
