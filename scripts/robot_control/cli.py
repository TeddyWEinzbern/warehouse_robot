"""Explicit command-line entry points for runtime, inspection, and calibration."""

from __future__ import annotations

import argparse
from contextlib import closing
import os
from pathlib import Path
import struct
import sys
import time

from .config import CONTROL
from .input import monitor, open_joystick
from .protocol import MessageType, ProtocolDecoder, encode_message
from .runtime import RobotRuntime
from .transport import SerialConnectionError, list_ports, open_port


def _port_name(entry: str) -> str:
    """Return the device portion of ``list_ports``' human-readable entries."""
    return entry.partition(" - ")[0]


def _windows_port_name(device: str) -> str:
    """Normalize COM ports without applying POSIX filesystem semantics."""
    normalized = device.strip().upper()
    if normalized.startswith("\\\\.\\"):
        normalized = normalized[4:]
    return normalized


def validate_serial_device(device: str, *, platform: str | None = None) -> None:
    """Fail before runtime startup when a requested serial device is absent."""
    if not device or not device.strip():
        raise SerialConnectionError("serial device is required")

    platform = platform or sys.platform
    available = {_port_name(entry) for entry in list_ports()}

    if platform == "win32":
        requested = _windows_port_name(device)
        available_windows = {_windows_port_name(port) for port in available}
        if requested in available_windows:
            return
        raise SerialConnectionError(
            f"serial device {device!r} is not available; run "
            "'warehouse-robot list-ports' to see connected COM ports"
        )

    device_path = Path(device)
    if device in available or (
        device_path.is_absolute() and device_path.is_char_device()
    ):
        return

    if platform == "darwin" and not device.startswith("/dev/"):
        full_path = os.path.join("/dev", device)
        if full_path in available or Path(full_path).is_char_device():
            raise SerialConnectionError(
                f"invalid macOS serial device {device!r}: use the full path "
                f"{full_path!r} (the '/dev/' prefix is required)"
            )
        raise SerialConnectionError(
            f"serial device {device!r} does not exist; macOS device names must use "
            "the full '/dev/...' path. Run 'warehouse-robot list-ports' to find it"
        )

    raise SerialConnectionError(
        f"serial device {device!r} is not an available serial character device; run "
        "'warehouse-robot list-ports' to see connected devices"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Warehouse robot host controller")
    subcommands = parser.add_subparsers(dest="command", required=True)

    run = subcommands.add_parser("run", help="Run the safety runtime and local dashboard")
    run.add_argument("--port", required=True)
    run.add_argument("--baud", type=int, choices=(9600, 38400), default=38400)
    run.add_argument("--web-port", type=int, default=8765)
    run.add_argument("--no-gamepad", action="store_true", help="Telemetry/tuning only")
    run.add_argument("--reconnect-attempts", type=int, default=10)

    subcommands.add_parser("list-ports", help="List serial devices")
    inspect = subcommands.add_parser("monitor", help="Display raw gamepad inputs")
    inspect.add_argument("--joystick", type=int, default=CONTROL.joystick_index)

    calibrate = subcommands.add_parser(
        "calibrate-joint", help="Send one typed command to arm_calibration firmware"
    )
    calibrate.add_argument("--port", required=True)
    calibrate.add_argument("--joint", type=int, choices=range(4), required=True)
    calibrate.add_argument("--angle", type=int, choices=range(181), required=True)
    return parser.parse_args()


def calibrate_joint(device: str, joint: int, angle: int) -> int:
    decoder = ProtocolDecoder()
    with open_port(device, 115200) as link:
        time.sleep(2.0)
        link.write(encode_message(MessageType.DISARM, 0))
        time.sleep(0.1)
        link.write(
            encode_message(
                MessageType.CALIBRATION_COMMAND, 1, struct.pack("<BB", joint, angle)
            )
        )
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            waiting = int(link.in_waiting)
            for message in decoder.feed(link.read(waiting) if waiting else b""):
                if message.message_type == MessageType.ACK:
                    print(f"Joint {joint} commanded to {angle} degrees")
                    return 0
                if message.message_type == MessageType.NACK:
                    raise SerialConnectionError(
                        f"firmware rejected calibration command (reason {message.payload[1]})"
                    )
            time.sleep(0.01)
    raise SerialConnectionError("timed out waiting for calibration acknowledgement")


def run(args: argparse.Namespace) -> int:
    if args.command == "list-ports":
        print("\n".join(list_ports()) or "No serial ports found")
        return 0
    if args.command == "monitor":
        pygame, joystick = open_joystick(args.joystick)
        monitor(pygame, joystick, 10.0)
        return 0
    if args.command == "calibrate-joint":
        validate_serial_device(args.port)
        return calibrate_joint(args.port, args.joint, args.angle)
    if args.reconnect_attempts < 1 or not 1 <= args.web_port <= 65535:
        raise SystemExit("Reconnect attempts and web port must be positive")
    try:
        from aiohttp import web
    except ImportError as exc:
        raise SystemExit("Missing aiohttp. Install with: python3 -m pip install -e .") from exc
    from .web import create_app, reserve_dashboard_socket

    try:
        dashboard_socket = reserve_dashboard_socket("127.0.0.1", args.web_port)
    except OSError as exc:
        raise SystemExit(
            f"Dashboard port 127.0.0.1:{args.web_port} is unavailable: {exc}. "
            "The robot runtime was not started."
        ) from exc

    # Reserve the listener before even enumerating serial devices: an occupied
    # dashboard port must have no effect on the HC-05 or RobotRuntime.
    with closing(dashboard_socket):
        validate_serial_device(args.port)
        runtime = RobotRuntime(
            args.port,
            args.baud,
            use_gamepad=not args.no_gamepad,
            maximum_reconnect_attempts=args.reconnect_attempts,
        )
        print(f"Dashboard: http://127.0.0.1:{args.web_port}")
        web.run_app(create_app(runtime), sock=dashboard_socket)
    return 0


def main() -> int:
    try:
        return run(parse_args())
    except SerialConnectionError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
