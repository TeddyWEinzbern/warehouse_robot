#!/usr/bin/env python3
"""Bridge a computer-connected gamepad axis to an Arduino servo over serial."""

from __future__ import annotations

import argparse
import time
from typing import Optional

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
        description="Send joystick axis positions to an Arduino servo as A:<angle> commands."
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
        help="Open a small pygame window for input focus. Useful on macOS if all values stay zero.",
    )
    parser.add_argument(
        "--axis",
        type=int,
        default=0,
        help="Gamepad axis index to read. Default: 0.",
    )
    parser.add_argument(
        "--min",
        dest="min_angle",
        type=int,
        default=0,
        help="Servo angle for axis value -1. Default: 0.",
    )
    parser.add_argument(
        "--max",
        dest="max_angle",
        type=int,
        default=180,
        help="Servo angle for axis value 1. Default: 180.",
    )
    parser.add_argument(
        "--deadzone",
        type=float,
        default=0.05,
        help="Ignore tiny joystick movement around center. Default: 0.05.",
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
        help="Arduino serial baud rate. Default: 115200.",
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
        help="Print the selected axis and angle every cycle, even when unchanged.",
    )
    return parser.parse_args()


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0
    return value


def axis_to_angle(axis_value: float, min_angle: int, max_angle: int) -> int:
    normalized = (axis_value + 1.0) / 2.0
    angle = min_angle + normalized * (max_angle - min_angle)
    return round(clamp(angle, min(min_angle, max_angle), max(min_angle, max_angle)))


def format_available_ports() -> str:
    ports = list(list_ports.comports())
    if not ports:
        return "  No serial ports found."

    return "\n".join(f"  {port.device} - {port.description}" for port in ports)


def open_joystick(index: int, use_window: bool) -> pygame.joystick.Joystick:
    pygame.init()
    if use_window:
        pygame.display.set_mode((360, 120))
        pygame.display.set_caption("Gamepad Servo Input")

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

        axes = tuple(
            round(joystick.get_axis(axis), 3)
            for axis in range(joystick.get_numaxes())
        )
        buttons = tuple(
            joystick.get_button(button)
            for button in range(joystick.get_numbuttons())
        )
        hats = tuple(
            joystick.get_hat(hat)
            for hat in range(joystick.get_numhats())
        )
        state = (axes, buttons, hats)
        now = time.monotonic()

        if state != previous_state or now - last_heartbeat >= 2.0:
            axes_text = " ".join(f"a{axis}:{value:+0.3f}" for axis, value in enumerate(axes))
            buttons_text = " ".join(
                f"b{button}:{value}" for button, value in enumerate(buttons)
            )
            hats_text = " ".join(f"h{hat}:{value}" for hat, value in enumerate(hats))
            if not hats_text:
                hats_text = "no hats"
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

    if args.axis < 0 or args.axis >= joystick.get_numaxes():
        raise SystemExit(f"Axis {args.axis} is invalid for this controller.")

    interval = 1.0 / args.rate
    last_angle: Optional[int] = None

    try:
        arduino = serial.Serial(args.port, args.baud, timeout=1)
    except serial.SerialException as exc:
        raise SystemExit(
            f"Could not open serial port {args.port}: {exc}\n\n"
            "Available serial ports:\n"
            f"{format_available_ports()}\n\n"
            "Use the Arduino port, usually /dev/cu.usbmodem* or /dev/cu.usbserial* on macOS."
        ) from exc

    with arduino:
        time.sleep(2.0)
        print(
            f"Sending axis {args.axis} to {args.port} at {args.baud} baud. "
            "Press Ctrl+C to stop."
        )

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return 0

            axis_value = joystick.get_axis(args.axis)
            axis_value = apply_deadzone(axis_value, args.deadzone)
            angle = axis_to_angle(axis_value, args.min_angle, args.max_angle)

            angle_changed = angle != last_angle
            if angle_changed:
                arduino.write(f"A:{angle}\n".encode("ascii"))
                last_angle = angle

            if args.verbose or angle_changed:
                print(f"axis={axis_value:+.3f} angle={angle:3d}", end="\r", flush=True)

            time.sleep(interval)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        raise SystemExit(0)
    finally:
        pygame.quit()
