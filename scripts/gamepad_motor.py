#!/usr/bin/env python3
"""Bridge a configured computer-connected gamepad to the Arduino motor sketch."""

from __future__ import annotations

import argparse
import time

from gamepad_config import GAMEPAD, HEARTBEAT_INTERVAL_SECONDS
from gamepad_io import (
    close_pygame,
    format_available_ports,
    monitor_joystick,
    open_joystick,
    open_serial_port,
    poll_pygame_quit,
)
from gamepad_mapping import GamepadMapper, validate_config, validate_joystick_axes

DEFAULT_RAW_COMMAND_DURATION_SECONDS = 1.0
DEFAULT_RAW_COMMAND_INTERVAL_SECONDS = 0.1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Send the gamepad configured in scripts/gamepad_config.py to Arduino as "
            "C:<forward>,<turn>,<strafe> commands."
        )
    )
    parser.add_argument("--port", help="Arduino USB or Bluetooth serial port.")
    parser.add_argument("--list-ports", action="store_true", help="List serial ports and exit.")
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Serial baud rate. USB defaults to 115200; HC-05/HC-06 usually uses 9600.",
    )
    parser.add_argument(
        "--send-command",
        help=(
            "Send one raw serial command such as C:400,0,0 repeatedly for --duration seconds, "
            "then send C:0,0,0 and exit."
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


def run_gamepad_bridge(args: argparse.Namespace) -> int:
    if not args.port:
        raise SystemExit("Missing --port. Use --list-ports to see available serial ports.")

    validate_config(GAMEPAD)
    joystick = open_joystick(GAMEPAD.joystick_index, GAMEPAD.open_input_window)
    validate_joystick_axes(joystick, GAMEPAD)

    if GAMEPAD.monitor_only:
        return monitor_joystick(joystick, GAMEPAD.command_rate_hz)

    mapper = GamepadMapper(joystick, GAMEPAD.axis_profiles)
    interval = 1.0 / GAMEPAD.command_rate_hz
    last_command: str | None = None
    last_sent_at = 0.0

    with open_serial_port(args.port, args.baud) as arduino:
        time.sleep(2.0)
        print(f"Sending gamepad commands to {args.port} at {args.baud} baud. Press Ctrl+C to stop.")
        print("Edit scripts/gamepad_config.py to change axes, deadzones, curves, and limits.")
        print(f"Axis maps: {mapper.describe()}")

        while True:
            if poll_pygame_quit():
                return 0

            command = mapper.command()
            command_changed = command != last_command
            now = time.monotonic()
            heartbeat_due = now - last_sent_at >= HEARTBEAT_INTERVAL_SECONDS
            if command_changed or heartbeat_due:
                arduino.write(command.encode("ascii"))
                last_command = command
                last_sent_at = now

            if GAMEPAD.print_changed_commands and command_changed:
                print(command.strip(), end="\r", flush=True)

            time.sleep(interval)


def main() -> int:
    args = parse_args()

    if args.list_ports:
        print("Available serial ports:")
        print(format_available_ports())
        return 0

    if args.send_command:
        return send_raw_command(args)

    return run_gamepad_bridge(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
        raise SystemExit(0)
    finally:
        close_pygame()
