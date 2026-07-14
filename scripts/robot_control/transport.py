"""Bounded serial transport helpers used by the safety runtime."""

from __future__ import annotations


class SerialConnectionError(RuntimeError):
    """A serial device could not be opened or used."""


def require_serial():
    try:
        import serial
    except ImportError as exc:
        raise SerialConnectionError(
            "Missing pyserial. Install the project with: python3 -m pip install -e ."
        ) from exc
    return serial


def list_ports() -> list[str]:
    require_serial()
    from serial.tools import list_ports as serial_ports

    return [f"{port.device} - {port.description}" for port in serial_ports.comports()]


def open_port(device: str, baud: int):
    if not device:
        raise SerialConnectionError("serial device is required")
    if baud not in (9600, 38400, 115200):
        raise SerialConnectionError(f"unsupported baud rate: {baud}")
    serial = require_serial()
    try:
        return serial.Serial(
            device,
            baud,
            timeout=0,
            # A 50 ms deadline is too aggressive for an RFCOMM-backed HC-05.
            # The operating system may briefly buffer a complete protocol frame
            # while the Bluetooth link is being scheduled or re-established.
            write_timeout=0.5,
        )
    except (serial.SerialException, OSError) as exc:
        raise SerialConnectionError(f"could not open {device}: {exc}") from exc
