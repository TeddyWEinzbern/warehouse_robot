"""Command line entry point for driving and calibrating the robot."""

from __future__ import annotations

import argparse
import time

from .config import CONTROL
from .input import monitor, open_joystick
from .mapping import map_gamepad
from .protocol import encode_control_frame
from .transport import list_ports, open_port


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Unified warehouse robot controller")
    parser.add_argument("--port")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--monitor", action="store_true")
    parser.add_argument("--legacy-ascii", action="store_true")
    parser.add_argument("--send-command", help="Send a legacy C:/W:/M: diagnostic command")
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--calibrate-joint", type=int, choices=range(4))
    parser.add_argument("--angle", type=int, choices=range(181))
    return parser.parse_args()


def require_port(args: argparse.Namespace) -> None:
    if not args.port:
        raise SystemExit("Missing --port. Use --list-ports first.")


def run(args: argparse.Namespace) -> int:
    if args.list_ports:
        print("\n".join(list_ports()) or "No serial ports found")
        return 0
    if args.calibrate_joint is not None:
        require_port(args)
        if args.angle is None:
            raise SystemExit("--calibrate-joint requires --angle")
        with open_port(args.port, 115200) as link:
            time.sleep(2.0)
            link.write(f"J:{args.calibrate_joint},{args.angle}\n".encode("ascii"))
        return 0
    if args.send_command:
        require_port(args)
        stop = b"C:0,0,0\n"
        command = (args.send_command.strip() + "\n").encode("ascii")
        with open_port(args.port, args.baud) as link:
            deadline = time.monotonic() + args.duration
            while time.monotonic() < deadline:
                link.write(command)
                time.sleep(0.1)
            link.write(stop)
        return 0

    pygame, joystick = open_joystick(CONTROL.joystick_index)
    if args.monitor:
        monitor(pygame, joystick, 10.0)
        return 0
    require_port(args)
    sequence = 0
    while True:
        try:
            with open_port(args.port, args.baud) as link:
                print(f"Connected to {args.port} at {args.baud} baud")
                while True:
                    pygame.event.pump()
                    frame = map_gamepad(joystick, CONTROL, sequence)
                    if args.legacy_ascii:
                        payload = f"C:{frame.forward},{frame.turn},{frame.strafe}\n".encode("ascii")
                    else:
                        payload = encode_control_frame(frame)
                    link.write(payload)
                    sequence = (sequence + 1) & 0xFF
                    time.sleep(1.0 / CONTROL.rate_hz)
        except SystemExit as exc:
            print(f"{exc}\nRetrying in 1 second; press Ctrl+C to stop.")
            time.sleep(1.0)
        except OSError as exc:
            print(f"Serial link lost: {exc}. Retrying in 1 second.")
            time.sleep(1.0)


def main() -> int:
    try:
        return run(parse_args())
    except KeyboardInterrupt:
        print("\nStopped")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
