"""pygame input adapter; pygame is imported only when live input is used."""

from __future__ import annotations

import os


def open_joystick(index: int):
    # SDL's Cocoa video driver may only be initialized on the process main
    # thread (it installs the application menu), but the runtime opens the
    # joystick from its dedicated thread. No window is ever needed, so force
    # the headless drivers before SDL initializes.
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError(
            "Missing pygame. Install the project with: python3 -m pip install -e ."
        ) from exc
    # pygame.init() would initialize every module; only the event pump
    # (display) and joystick subsystems are used here.
    pygame.display.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() <= index:
        raise RuntimeError(f"Joystick index {index} is not available")
    joystick = pygame.joystick.Joystick(index)
    joystick.init()
    return pygame, joystick


def monitor(pygame, joystick, rate_hz: float) -> None:
    import time

    while True:
        pygame.event.pump()
        axes = " ".join(f"a{i}:{joystick.get_axis(i):+.3f}" for i in range(joystick.get_numaxes()))
        buttons = " ".join(f"b{i}:{joystick.get_button(i)}" for i in range(joystick.get_numbuttons()))
        hats = " ".join(f"h{i}:{joystick.get_hat(i)}" for i in range(joystick.get_numhats()))
        print(f"{axes} | {buttons} | {hats}", end="\r", flush=True)
        time.sleep(1.0 / rate_hz)
