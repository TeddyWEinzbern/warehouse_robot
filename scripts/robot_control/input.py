"""pygame input adapter; pygame is imported only when live input is used."""

from __future__ import annotations

import os


class SDLControllerInput:
    """Expose SDL's named gamepad mapping through the existing input shape."""

    def __init__(self, pygame, controller) -> None:
        self._pygame = pygame
        self._controller = controller
        self._buttons = (
            pygame.CONTROLLER_BUTTON_A,
            pygame.CONTROLLER_BUTTON_B,
            pygame.CONTROLLER_BUTTON_X,
            pygame.CONTROLLER_BUTTON_Y,
            pygame.CONTROLLER_BUTTON_LEFTSHOULDER,
            pygame.CONTROLLER_BUTTON_RIGHTSHOULDER,
            pygame.CONTROLLER_BUTTON_BACK,
            pygame.CONTROLLER_BUTTON_START,
        )

    def get_init(self) -> bool:
        return bool(
            self._controller.get_init() and self._controller.attached()
        )

    def get_name(self) -> str:
        return f"{self._controller.name} [SDL mapped]"

    @staticmethod
    def get_numaxes() -> int:
        return 6

    def get_axis(self, index: int) -> float:
        value = int(self._controller.get_axis(index))
        if index in (
            self._pygame.CONTROLLER_AXIS_TRIGGERLEFT,
            self._pygame.CONTROLLER_AXIS_TRIGGERRIGHT,
        ):
            return max(-1.0, min(1.0, value / 16384.0 - 1.0))
        divisor = 32767.0 if value >= 0 else 32768.0
        return max(-1.0, min(1.0, value / divisor))

    def get_numbuttons(self) -> int:
        return len(self._buttons)

    def get_button(self, index: int) -> bool:
        return (
            0 <= index < len(self._buttons)
            and bool(self._controller.get_button(self._buttons[index]))
        )

    @staticmethod
    def get_numhats() -> int:
        return 1

    def get_hat(self, index: int) -> tuple[int, int]:
        if index != 0:
            raise IndexError("controller has only one logical D-pad")
        horizontal = int(
            self._controller.get_button(
                self._pygame.CONTROLLER_BUTTON_DPAD_RIGHT
            )
        ) - int(
            self._controller.get_button(
                self._pygame.CONTROLLER_BUTTON_DPAD_LEFT
            )
        )
        vertical = int(
            self._controller.get_button(
                self._pygame.CONTROLLER_BUTTON_DPAD_UP
            )
        ) - int(
            self._controller.get_button(
                self._pygame.CONTROLLER_BUTTON_DPAD_DOWN
            )
        )
        return horizontal, vertical


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
    try:
        from pygame._sdl2 import controller as sdl_controller

        sdl_controller.init()
        if sdl_controller.is_controller(index):
            controller = sdl_controller.Controller(index)
            controller.init()
            return pygame, SDLControllerInput(pygame, controller)
    except (ImportError, RuntimeError):
        # Unknown devices remain usable through pygame's raw joystick API.
        pass
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
