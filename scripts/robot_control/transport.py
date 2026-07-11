"""Serial transport helpers."""

from __future__ import annotations


def require_serial():
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Missing pyserial. Install the project with: python3 -m pip install -e .") from exc
    return serial


def list_ports() -> list[str]:
    require_serial()
    from serial.tools import list_ports as serial_ports

    return [f"{port.device} - {port.description}" for port in serial_ports.comports()]


def open_port(device: str, baud: int):
    serial = require_serial()
    try:
        return serial.Serial(device, baud, timeout=0.1)
    except serial.SerialException as exc:
        raise SystemExit(f"Could not open {device}: {exc}") from exc

