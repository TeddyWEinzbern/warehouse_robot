#!/usr/bin/env python3
"""Bridge a computer-connected gamepad to an Arduino UART motor driver sketch."""

from __future__ import annotations

import argparse
import time

HEARTBEAT_INTERVAL_SECONDS = 0.1

try:
    import pygame
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: pygame. Install with: python3 -m pip install pygame pyserial"
    ) from exc

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
            "Send left-stick forward/turn and trigger strafe values to an Arduino "
            "as C:<forward>,<turn>,<strafe> commands."
        )
    )
    parser.add_argument(
        "--port",
        help="Arduino serial port, for example /dev/cu.usbmodemXXXX or COM3.",
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
        default=2,
        help="Left trigger axis for left strafe. Default: 2.",
    )
    parser.add_argument(
        "--right-trigger-axis",
        type=int,
        default=5,
        help="Right trigger axis for right strafe. Default: 5.",
    )
    parser.add_argument(
        "--trigger-mode",
        choices=("auto", "signed", "unsigned"),
        default="auto",
        help="Trigger axis range: signed is -1..1, unsigned is 0..1. Default: auto.",
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
    parser.add_argument("--baud", type=int, default=115200, help="Arduino USB baud rate.")
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
    return clamp(value, 0.0, 1.0)


def resolve_trigger_mode(joystick: pygame.joystick.Joystick, axis: int, mode: str) -> str:
    if mode != "auto":
        return mode
    return "signed" if joystick.get_axis(axis) < -0.1 else "unsigned"


def format_available_ports() -> str:
    ports = list(list_ports.comports())
    if not ports:
        return "  No serial ports found."
    return "\n".join(f"  {port.device} - {port.description}" for port in ports)


def open_joystick(index: int, use_window: bool) -> pygame.joystick.Joystick:
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

    joystick = open_joystick(args.joystick, args.window)

    if args.monitor:
        return monitor_joystick(joystick, args.rate)

    if not args.port:
        raise SystemExit("Missing --port. Use --list-ports to see available serial ports.")

    validate_axis(joystick, args.left_x_axis, "Left stick X")
    validate_axis(joystick, args.left_y_axis, "Left stick Y")
    validate_axis(joystick, args.left_trigger_axis, "Left trigger")
    validate_axis(joystick, args.right_trigger_axis, "Right trigger")

    try:
        arduino = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        raise SystemExit(
            f"Could not open serial port {args.port}: {exc}\n\n"
            "Available serial ports:\n"
            f"{format_available_ports()}\n\n"
            "Use the Arduino port, usually /dev/cu.usbmodem* or /dev/cu.usbserial* on macOS."
        ) from exc

    interval = 1.0 / args.rate
    last_command: str | None = None
    last_sent_at = 0.0

    with arduino:
        time.sleep(2.0)
        left_trigger_mode = resolve_trigger_mode(joystick, args.left_trigger_axis, args.trigger_mode)
        right_trigger_mode = resolve_trigger_mode(joystick, args.right_trigger_axis, args.trigger_mode)

        print(f"Sending gamepad commands to {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
        print("Left stick Y = forward/back, left stick X = rotate, LT/RT = left/right strafe.")
        print(f"Trigger modes: left={left_trigger_mode}, right={right_trigger_mode}.")

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 0

            forward = -apply_deadzone(joystick.get_axis(args.left_y_axis), args.deadzone)
            turn = apply_deadzone(joystick.get_axis(args.left_x_axis), args.deadzone)

            left_trigger = trigger_to_unit(
                joystick.get_axis(args.left_trigger_axis), left_trigger_mode
            )
            right_trigger = trigger_to_unit(
                joystick.get_axis(args.right_trigger_axis), right_trigger_mode
            )
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
        pygame.quit()
