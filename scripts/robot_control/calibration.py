"""Interactive serial calibration session for the arm_calibration firmware.

Keeps one serial connection open for the whole session (opening the USB port
resets the Uno, so one-shot commands would re-detach the servos every time),
jogs joints with absolute or incremental commands, mirrors firmware telemetry,
and exports a paste-ready calibration block for src/app/BuildConfig.h.
"""

from __future__ import annotations

import struct
import time
from typing import Any, Callable

from .protocol import (
    MessageType,
    ProtocolDecoder,
    encode_message,
    telemetry_to_dict,
)

JOINT_NAMES = ("base", "shoulder", "elbow", "gripper")
JOINT_COUNT = 4
ACK_TIMEOUT_S = 3.0

NACK_REASONS = {
    1: "malformed frame",
    2: "unsupported command",
    3: "invalid state: firmware must be the arm_calibration build and DISARMED",
    4: "parameter revision mismatch",
    5: "blocked by the shoulder/elbow coupling guard (four-bar fold band)",
}

HELP_TEXT = """commands:
  j<n> <angle>     move joint n (0 base, 1 shoulder, 2 elbow, 3 gripper)
                   to an absolute servo angle 0-180, e.g. `j2 95`
  j<n> +<d>|-<d>   nudge joint n by d degrees, e.g. `j1 +2`
  mark j<n> <what> record the current angle of joint n as one of:
                   lower / upper / center (j3 uses open / closed instead)
  dir j<n> +1|-1   record the direction sign for joint n
  s                show commanded angles, telemetry, marks, fold estimate
  export           print the BuildConfig.h block from the recorded marks
  help             show this text
  q                quit (servos keep holding their last position)"""

MARK_NAMES = ("lower", "upper", "center", "open", "closed")


class CalibrationError(RuntimeError):
    """Raised for user-visible command failures inside the session."""


def _parse_joint(token: str) -> int:
    if len(token) == 2 and token[0] == "j" and token[1] in "0123":
        return int(token[1])
    raise CalibrationError(f"unknown joint {token!r}; expected j0..j3")


class CalibrationSession:
    """Line-command driver around one open serial link."""

    def __init__(
        self,
        link: Any,
        *,
        out: Callable[[str], None] = print,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.link = link
        self.out = out
        self.clock = clock
        self.sleep = sleep
        self.decoder = ProtocolDecoder()
        self.sequence = 0
        self.commanded: list[int | None] = [None] * JOINT_COUNT
        self.telemetry: list[int] | None = None
        self.marks: dict[tuple[int, str], int] = {}
        self.directions: dict[int, int] = {}

    # -- transport helpers --------------------------------------------------

    def start(self) -> None:
        """Wait out the auto-reset after port open, then force DISARMED."""
        self.sleep(2.0)
        self._send(MessageType.DISARM)
        self._pump()

    def _next_sequence(self) -> int:
        self.sequence = (self.sequence + 1) % 256
        return self.sequence

    def _send(self, message_type: MessageType, payload: bytes = b"") -> None:
        self.link.write(encode_message(message_type, self._next_sequence(), payload))

    def _pump(self) -> list[Any]:
        waiting = int(getattr(self.link, "in_waiting", 0))
        messages = self.decoder.feed(self.link.read(waiting) if waiting else b"")
        for message in messages:
            if message.message_type == MessageType.SENSOR_ARM_TELEMETRY:
                decoded = telemetry_to_dict(message)
                if decoded and decoded.get("kind") == "sensor_arm":
                    self.telemetry = list(decoded["servo_targets"])
        return messages

    def _command_joint(self, joint: int, angle: int) -> None:
        self._send(
            MessageType.CALIBRATION_COMMAND, struct.pack("<BB", joint, angle)
        )
        deadline = self.clock() + ACK_TIMEOUT_S
        while self.clock() < deadline:
            for message in self._pump():
                if message.message_type == MessageType.ACK:
                    self.commanded[joint] = angle
                    self.out(
                        f"{JOINT_NAMES[joint]} (j{joint}) -> {angle} deg"
                        " (slewing at 60 deg/s)"
                    )
                    return
                if message.message_type == MessageType.NACK:
                    reason = (
                        message.payload[1] if len(message.payload) > 1 else 0
                    )
                    raise CalibrationError(
                        "firmware rejected the command: "
                        + NACK_REASONS.get(reason, f"reason code {reason}")
                    )
            self.sleep(0.01)
        raise CalibrationError("timed out waiting for the firmware acknowledgement")

    # -- state helpers ------------------------------------------------------

    def _current_angle(self, joint: int) -> int:
        if self.commanded[joint] is not None:
            return int(self.commanded[joint])
        raise CalibrationError(
            f"j{joint} has no commanded angle yet; send an absolute command "
            f"first (e.g. `j{joint} 90`)"
        )

    def _fold_estimate(self) -> float | None:
        """Fold angle from marked centers; None until enough data exists."""
        shoulder_center = self.marks.get((1, "center"))
        elbow_center = self.marks.get((2, "center"))
        if shoulder_center is None or elbow_center is None:
            return None
        if self.commanded[1] is None or self.commanded[2] is None:
            return None
        shoulder_offset = self.directions.get(1, 1) * (
            self.commanded[1] - shoulder_center
        )
        elbow_offset = self.directions.get(2, 1) * (
            self.commanded[2] - elbow_center
        )
        return 90.0 + shoulder_offset - elbow_offset

    # -- commands -----------------------------------------------------------

    def handle_line(self, line: str) -> bool:
        """Process one command line. Returns False when the session ends."""
        tokens = line.strip().lower().split()
        if not tokens:
            return True
        try:
            if tokens[0] in ("q", "quit", "exit"):
                return False
            if tokens[0] in ("help", "h", "?"):
                self.out(HELP_TEXT)
            elif tokens[0] in ("s", "status"):
                self._show_status()
            elif tokens[0] == "mark":
                self._handle_mark(tokens)
            elif tokens[0] == "dir":
                self._handle_direction(tokens)
            elif tokens[0] == "export":
                self.out(self.export_block())
            elif tokens[0].startswith("j"):
                self._handle_move(tokens)
            else:
                raise CalibrationError(
                    f"unknown command {tokens[0]!r}; type `help`"
                )
        except CalibrationError as error:
            self.out(f"error: {error}")
        return True

    def _handle_move(self, tokens: list[str]) -> None:
        if len(tokens) != 2:
            raise CalibrationError("expected `j<n> <angle>` or `j<n> +/-<d>`")
        joint = _parse_joint(tokens[0])
        value = tokens[1]
        if value.startswith(("+", "-")):
            try:
                delta = int(value)
            except ValueError:
                raise CalibrationError(f"bad increment {value!r}") from None
            angle = self._current_angle(joint) + delta
        else:
            try:
                angle = int(value)
            except ValueError:
                raise CalibrationError(f"bad angle {value!r}") from None
        if not 0 <= angle <= 180:
            raise CalibrationError(f"angle {angle} is outside 0-180")
        self._command_joint(joint, angle)
        fold = self._fold_estimate()
        if fold is not None:
            self.out(f"fold estimate: {fold:.0f} deg (firmware guard stops at 138)")

    def _handle_mark(self, tokens: list[str]) -> None:
        if len(tokens) != 3:
            raise CalibrationError("expected `mark j<n> <lower|upper|center|open|closed>`")
        joint = _parse_joint(tokens[1])
        name = tokens[2]
        if name not in MARK_NAMES:
            raise CalibrationError(f"unknown mark {name!r}")
        if joint == 3 and name in ("lower", "upper", "center"):
            raise CalibrationError("gripper marks are `open` and `closed`")
        if joint != 3 and name in ("open", "closed"):
            raise CalibrationError(f"j{joint} marks are lower/upper/center")
        angle = self._current_angle(joint)
        self.marks[(joint, name)] = angle
        self.out(f"marked j{joint} {name} = {angle} deg")

    def _handle_direction(self, tokens: list[str]) -> None:
        if len(tokens) != 3 or tokens[2] not in ("+1", "-1", "1"):
            raise CalibrationError("expected `dir j<n> +1` or `dir j<n> -1`")
        joint = _parse_joint(tokens[1])
        self.directions[joint] = 1 if tokens[2] in ("+1", "1") else -1
        self.out(f"j{joint} direction = {self.directions[joint]:+d}")

    def _show_status(self) -> None:
        self._pump()
        lines = ["joint      commanded  telemetry  dir  marks"]
        for joint in range(JOINT_COUNT):
            commanded = self.commanded[joint]
            reported = self.telemetry[joint] if self.telemetry else None
            marks = ", ".join(
                f"{name}={angle}"
                for (mark_joint, name), angle in sorted(self.marks.items())
                if mark_joint == joint
            )
            lines.append(
                f"j{joint} {JOINT_NAMES[joint]:<8}"
                f" {'-' if commanded is None else commanded:>8}"
                f" {'-' if reported is None else reported:>9}"
                f" {self.directions.get(joint, '?'):>4}"
                f"  {marks or '-'}"
            )
        fold = self._fold_estimate()
        if fold is not None:
            lines.append(f"fold estimate: {fold:.0f} deg (guard stops at 138)")
        self.out("\n".join(lines))

    # -- export -------------------------------------------------------------

    def export_block(self) -> str:
        """BuildConfig.h block from the marks, with warnings for gaps."""
        warnings: list[str] = []

        def mark(joint: int, name: str, fallback: int) -> int:
            value = self.marks.get((joint, name))
            if value is None:
                warnings.append(
                    f"j{joint} {name} not marked; using default {fallback}"
                )
                return fallback
            return value

        centers = {
            "BaseZeroDegrees": mark(0, "center", 90),
            "ShoulderZeroDegrees": mark(1, "center", 90),
            "ElbowZeroDegrees": mark(2, "center", 90),
            "GripperOpenDegrees": mark(3, "open", 80),
            "GripperClosedDegrees": mark(3, "closed", 25),
        }
        lowers = [mark(joint, "lower", 0) for joint in range(3)]
        uppers = [mark(joint, "upper", 180) for joint in range(3)]
        gripper_pair = sorted(
            (centers["GripperOpenDegrees"], centers["GripperClosedDegrees"])
        )
        lowers.append(max(0, gripper_pair[0] - 5))
        uppers.append(min(180, gripper_pair[1] + 5))
        signs = []
        for joint in range(JOINT_COUNT):
            sign = self.directions.get(joint)
            if sign is None:
                warnings.append(f"j{joint} direction not set; using default +1")
                sign = 1
            signs.append(sign)

        base_span = abs(uppers[0] - lowers[0])
        if base_span < 180:
            warnings.append(
                f"base travel is {base_span} deg; the mechanism requires >= 180"
            )

        block = "\n".join(
            [
                "// Paste over the matching lines in src/app/BuildConfig.h",
                *[
                    f"constexpr uint8_t {name} = {value};"
                    for name, value in centers.items()
                ],
                "",
                "constexpr uint8_t ServoLowerDegrees[4] = {"
                + ", ".join(str(value) for value in lowers) + "};",
                "constexpr uint8_t ServoUpperDegrees[4] = {"
                + ", ".join(str(value) for value in uppers) + "};",
                "constexpr int8_t ServoDirectionSign[4] = {"
                + ", ".join(str(value) for value in signs) + "};",
            ]
        )
        if warnings:
            block += "\n\n// WARNINGS:\n" + "\n".join(
                f"//   {warning}" for warning in warnings
            )
        return block


def run_session(link: Any, *, out: Callable[[str], None] = print) -> int:
    """Blocking REPL over an already-open link. Returns a process exit code."""
    session = CalibrationSession(link, out=out)
    out("waiting 2 s for the board reset caused by opening the port...")
    session.start()
    out("connected; type `help` for commands, `q` to quit")
    while True:
        try:
            line = input("cal> ")
        except (EOFError, KeyboardInterrupt):
            out("")
            return 0
        if not session.handle_line(line):
            return 0
