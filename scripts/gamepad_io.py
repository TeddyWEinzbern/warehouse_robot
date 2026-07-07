"""Serial and pygame I/O helpers for the gamepad motor bridge."""

from __future__ import annotations

import time

try:
    import pygame
except ImportError as exc:
    pygame = None
    PYGAME_IMPORT_ERROR = exc
else:
    PYGAME_IMPORT_ERROR = None

def require_serial():
    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: pyserial. Install with: python3 -m pip install pygame pyserial"
        ) from exc
    return serial


def format_available_ports() -> str:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise SystemExit(
            "Missing dependency: pyserial. Install with: python3 -m pip install pygame pyserial"
        ) from exc

    ports = list(list_ports.comports())
    if not ports:
        return "  No serial ports found."
    return "\n".join(f"  {port.device} - {port.description}" for port in ports)


def open_serial_port(port: str, baud: int):
    serial = require_serial()
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


def require_pygame() -> None:
    if pygame is None:
        raise SystemExit(
            "Missing dependency: pygame. Install with: python3 -m pip install pygame pyserial"
        ) from PYGAME_IMPORT_ERROR


def open_joystick(index: int, use_window: bool):
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


def poll_pygame_quit() -> bool:
    if pygame is None:
        return False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
    return False


def monitor_joystick(joystick, rate_hz: float) -> int:
    interval = 1.0 / rate_hz
    previous_state = None
    last_heartbeat = 0.0
    print("Move sticks/triggers and press buttons. Press Ctrl+C to stop.")

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


def close_pygame() -> None:
    if pygame is not None:
        pygame.quit()
