"""Interactive serial calibration session for the `calibration` firmware.

Keeps one serial connection open for the whole session, jogs arm joints with
absolute or incremental commands, spins individual drive motors open-loop or
closed-loop while DISARMED, mirrors firmware telemetry, and exports a
paste-ready calibration block for src/app/BuildConfig.h. The host link is
the A4/A5 SoftwareSerial (Bluetooth module or USB-TTL adapter); the hardware
UART belongs to the motor board.
"""

from __future__ import annotations

import struct
import time
from typing import Any, Callable

from .protocol import (
    MessageType,
    ParameterGroup,
    ProtocolDecoder,
    build_parameter_data,
    encode_drive_calibration,
    encode_message,
    encode_parameter_set,
    telemetry_to_dict,
)

JOINT_NAMES = ("base", "shoulder", "elbow", "gripper")
JOINT_COUNT = 4
WHEEL_NAMES = ("fl", "fr", "rl", "rr")
WHEEL_LABELS = ("front-left", "front-right", "rear-left", "rear-right")
ACK_TIMEOUT_S = 3.0
SPIN_MAX_PERCENT = 100
SPIN_MAX_MM_S = 200
SPIN_MAX_SECONDS = 10.0

NACK_REASONS = {
    1: "malformed frame",
    2: "unsupported command",
    3: "invalid state: firmware must be the `calibration` build, DISARMED,"
       " and (for motor spins) the motor board must be connected and powered",
    4: "parameter revision mismatch",
    5: "blocked: shoulder/elbow coupling guard, or spin value/duration"
       " outside the calibration limits",
}

HELP_TEXT = """arm commands:
  j<n> <angle>     move joint n (0 base, 1 shoulder, 2 elbow, 3 gripper)
                   to an absolute servo angle 0-180, e.g. `j2 95`
  j<n> +<d>|-<d>   nudge joint n by d degrees, e.g. `j1 +2`
  mark j<n> <what> record the current angle of joint n as one of:
                   lower / upper / center (j3 uses open / closed instead)
  dir j<n> +1|-1   record the direction sign for joint n
  sync             push recorded center+dir to the firmware so its
                   coupling guard uses the real geometry (runs
                   automatically once a joint has both)
motor commands (wheels raised off the ground!):
  m<n> <pct> [sec]  spin motor board channel n (0-3) open-loop, e.g.
                    `m0 30 2` = channel 0, 30% forward, 2 s; negative
                    percent reverses; seconds default 2, max 10
  v<n> <mm_s> [sec] closed-loop spin, e.g. `v0 150 3` (max 200 mm/s);
                    use after `motor`/`enc` mappings are measured
  stop              stop any running spin immediately
  counts            show encoder increments/totals and measured speeds
  motor <n> <wheel> <fwd|rev>  record which wheel board channel n drives
                    and whether positive spins it forward; wheel is one
                    of fl fr rl rr, e.g. `motor 0 fl fwd`
  enc <n> <wheel> <fwd|rev>    record which wheel encoder channel n
                    counts for, and whether it counts up going forward
  geom <wheel_mm> <counts_per_rev> <track_mm> <wheelbase_mm>
                    record measured wheel geometry, e.g. `geom 60 4680 160 170`
session commands:
  s                show joint state, motor mappings, marks, fold estimate
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


def _parse_channel(token: str) -> int:
    if token in ("0", "1", "2", "3"):
        return int(token)
    raise CalibrationError(f"unknown channel {token!r}; expected 0..3")


def _parse_wheel(token: str) -> int:
    if token in WHEEL_NAMES:
        return WHEEL_NAMES.index(token)
    raise CalibrationError(
        f"unknown wheel {token!r}; expected one of {' '.join(WHEEL_NAMES)}"
    )


def _parse_seconds(tokens: list[str]) -> float:
    if not tokens:
        return 2.0
    try:
        seconds = float(tokens[0])
    except ValueError:
        raise CalibrationError(f"bad duration {tokens[0]!r}") from None
    if not 0 < seconds <= SPIN_MAX_SECONDS:
        raise CalibrationError(
            f"duration must be within 0-{SPIN_MAX_SECONDS:.0f} seconds"
        )
    return seconds


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
        self.revision: int | None = None
        self.synced: dict[int, int] = {}
        # Motor calibration records: board/encoder channel -> (wheel, sign).
        self.motor_map: dict[int, tuple[int, int]] = {}
        self.encoder_map: dict[int, tuple[int, int]] = {}
        self.geometry: tuple[int, int, int, int] | None = None
        self.increments: list[int] | None = None
        self.totals: list[int] | None = None
        self.measured_mm_s: list[int] | None = None

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
            elif message.message_type == MessageType.STATUS_TELEMETRY:
                decoded = telemetry_to_dict(message)
                if decoded and decoded.get("kind") == "status":
                    self.revision = decoded["revision"]
            elif message.message_type == MessageType.ENCODER_TOTALS_TELEMETRY:
                decoded = telemetry_to_dict(message)
                if decoded and decoded.get("kind") == "encoder_totals":
                    self.increments = list(decoded["raw_increments"])
                    self.totals = list(decoded["totals"])
            elif message.message_type == MessageType.DRIVE_FEEDBACK_TELEMETRY:
                decoded = telemetry_to_dict(message)
                if decoded and decoded.get("kind") == "drive_feedback":
                    self.measured_mm_s = list(decoded["measured_speeds"])
            elif (
                message.message_type == MessageType.ACK
                and len(message.payload) >= 3
            ):
                self.revision = int.from_bytes(message.payload[1:3], "little")
        return messages

    def _await_reply(self) -> Any:
        """Pump until an ACK or NACK arrives; raise on NACK or timeout."""
        deadline = self.clock() + ACK_TIMEOUT_S
        while self.clock() < deadline:
            for message in self._pump():
                if message.message_type == MessageType.ACK:
                    return message
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

    def _command_joint(self, joint: int, angle: int) -> None:
        self._send(
            MessageType.CALIBRATION_COMMAND, struct.pack("<BB", joint, angle)
        )
        self._await_reply()
        self.commanded[joint] = angle
        self.out(
            f"{JOINT_NAMES[joint]} (j{joint}) -> {angle} deg"
            " (slewing at 60 deg/s)"
        )

    # -- firmware sync ------------------------------------------------------

    def _center_mark(self, joint: int) -> int | None:
        return self.marks.get((joint, "open" if joint == 3 else "center"))

    def _sync_joint(self, joint: int) -> None:
        """Push center+direction to the firmware so the coupling guard uses
        the real geometry. Travel limits stay wide open (0-180) during
        calibration; the final narrow limits go into BuildConfig.h instead."""
        center = self._center_mark(joint)
        direction = self.directions.get(joint)
        if center is None or direction is None:
            return
        self._pump()
        if self.revision is None:
            self.out(
                f"sync deferred for j{joint}: no firmware revision seen yet;"
                " wait a second and run `sync`"
            )
            return
        data = build_parameter_data(
            ParameterGroup.SERVO, joint,
            {
                "lower": 0, "upper": 180,
                "center_offset": center - 90, "direction": direction,
            },
        )
        for attempt in (1, 2):
            self.link.write(encode_parameter_set(
                self._next_sequence(), ParameterGroup.SERVO, joint,
                self.revision, data,
            ))
            try:
                self._await_reply()
            except CalibrationError as error:
                if "revision mismatch" in str(error) and attempt == 1:
                    continue
                self.out(f"warning: firmware sync for j{joint} failed: {error}")
                return
            self.synced[joint] = self.revision
            self.out(
                f"synced j{joint} center/direction to the firmware"
                f" (revision {self.revision}); coupling guard now uses it"
            )
            return

    def _sync_all(self) -> None:
        pending = False
        for joint in range(JOINT_COUNT):
            if self._center_mark(joint) is not None and joint in self.directions:
                self._sync_joint(joint)
                pending = True
        if not pending:
            self.out(
                "nothing to sync yet: a joint needs both a center mark"
                " (open for j3) and a `dir` before it can be pushed"
            )

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
            elif tokens[0] == "sync":
                self._sync_all()
            elif tokens[0] == "export":
                self.out(self.export_block())
            elif tokens[0] == "stop":
                self._spin(mode=0, channel=0, value=0, seconds=0.0)
                self.out("spin stopped (zero frame resumes immediately)")
            elif tokens[0] == "counts":
                self._show_counts()
            elif tokens[0] == "motor":
                self._handle_motor_record(tokens)
            elif tokens[0] == "enc":
                self._handle_encoder_record(tokens)
            elif tokens[0] == "geom":
                self._handle_geometry(tokens)
            elif tokens[0].startswith("m") and tokens[0][1:].isdigit():
                self._handle_spin(tokens, closed_loop=False)
            elif tokens[0].startswith("v") and tokens[0][1:].isdigit():
                self._handle_spin(tokens, closed_loop=True)
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
        if name in ("center", "open"):
            self._sync_joint(joint)

    def _handle_direction(self, tokens: list[str]) -> None:
        if len(tokens) != 3 or tokens[2] not in ("+1", "-1", "1"):
            raise CalibrationError("expected `dir j<n> +1` or `dir j<n> -1`")
        joint = _parse_joint(tokens[1])
        self.directions[joint] = 1 if tokens[2] in ("+1", "1") else -1
        self.out(f"j{joint} direction = {self.directions[joint]:+d}")
        self._sync_joint(joint)

    # -- motor calibration ----------------------------------------------------

    def _spin(self, *, mode: int, channel: int, value: int, seconds: float) -> None:
        self._send_raw(
            encode_drive_calibration(
                self._next_sequence(), mode, channel, value,
                int(round(seconds * 1000.0)),
            )
        )
        self._await_reply()

    def _send_raw(self, packet: bytes) -> None:
        self.link.write(packet)

    def _handle_spin(self, tokens: list[str], *, closed_loop: bool) -> None:
        unit = "mm/s" if closed_loop else "%"
        limit = SPIN_MAX_MM_S if closed_loop else SPIN_MAX_PERCENT
        usage = (
            f"expected `{tokens[0][0]}<channel> <value> [seconds]`, e.g. "
            + ("`v0 150 3`" if closed_loop else "`m0 30 2`")
        )
        if len(tokens) not in (2, 3):
            raise CalibrationError(usage)
        channel = _parse_channel(tokens[0][1:])
        try:
            value = int(tokens[1])
        except ValueError:
            raise CalibrationError(f"bad value {tokens[1]!r}; {usage}") from None
        if not -limit <= value <= limit:
            raise CalibrationError(f"value must be within +/-{limit} {unit}")
        seconds = _parse_seconds(tokens[2:])
        self._spin(
            mode=1 if closed_loop else 0, channel=channel,
            value=value, seconds=seconds,
        )
        self.out(
            f"channel {channel} spinning at {value} {unit} for {seconds:g} s"
            " (repeats every 50 ms, stops by itself; `stop` to end early)"
        )

    def _handle_motor_record(self, tokens: list[str]) -> None:
        if len(tokens) != 4 or tokens[3] not in ("fwd", "rev"):
            raise CalibrationError("expected `motor <channel> <fl|fr|rl|rr> <fwd|rev>`")
        channel = _parse_channel(tokens[1])
        wheel = _parse_wheel(tokens[2])
        sign = 1 if tokens[3] == "fwd" else -1
        self.motor_map[channel] = (wheel, sign)
        self.out(
            f"recorded: board channel {channel} drives the"
            f" {WHEEL_LABELS[wheel]} wheel, positive = "
            + ("forward" if sign == 1 else "backward")
        )

    def _handle_encoder_record(self, tokens: list[str]) -> None:
        if len(tokens) != 4 or tokens[3] not in ("fwd", "rev"):
            raise CalibrationError("expected `enc <channel> <fl|fr|rl|rr> <fwd|rev>`")
        channel = _parse_channel(tokens[1])
        wheel = _parse_wheel(tokens[2])
        sign = 1 if tokens[3] == "fwd" else -1
        self.encoder_map[channel] = (wheel, sign)
        self.out(
            f"recorded: encoder channel {channel} counts for the"
            f" {WHEEL_LABELS[wheel]} wheel, "
            + ("counts up" if sign == 1 else "counts down")
            + " when the wheel turns forward"
        )

    def _handle_geometry(self, tokens: list[str]) -> None:
        if len(tokens) != 5:
            raise CalibrationError(
                "expected `geom <wheel_mm> <counts_per_rev> <track_mm> <wheelbase_mm>`"
            )
        try:
            values = tuple(int(token) for token in tokens[1:5])
        except ValueError:
            raise CalibrationError("geometry values must be whole numbers") from None
        bounds = ((1, 300), (1, 60000), (1, 1000), (1, 1000))
        names = ("wheel_mm", "counts_per_rev", "track_mm", "wheelbase_mm")
        for value, (low, high), name in zip(values, bounds, names):
            if not low <= value <= high:
                raise CalibrationError(f"{name} must be within {low}-{high}")
        self.geometry = values
        self.out(
            "recorded geometry: wheel {0} mm, {1} counts/rev,"
            " track {2} mm, wheelbase {3} mm".format(*values)
        )

    def _show_counts(self) -> None:
        self._pump()
        if self.increments is None and self.measured_mm_s is None:
            self.out(
                "no encoder telemetry yet; wait a second and retry"
                " (the motor board must be connected and powered)"
            )
            return
        lines = ["channel    increment      total   measured mm/s"]
        for channel in range(4):
            increment = self.increments[channel] if self.increments else None
            total = self.totals[channel] if self.totals else None
            speed = self.measured_mm_s[channel] if self.measured_mm_s else None
            lines.append(
                f"{channel:>7}"
                f" {'-' if increment is None else increment:>12}"
                f" {'-' if total is None else total:>10}"
                f" {'-' if speed is None else speed:>15}"
            )
        self.out("\n".join(lines))

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
        synced = ", ".join(f"j{joint}" for joint in sorted(self.synced))
        lines.append(
            f"firmware sync: {synced or 'none'}"
            " (unsynced joints leave the guard on default direction/center)"
        )
        motor = ", ".join(
            f"ch{channel}->{WHEEL_NAMES[wheel]}{'+' if sign == 1 else '-'}"
            for channel, (wheel, sign) in sorted(self.motor_map.items())
        )
        encoder = ", ".join(
            f"ch{channel}->{WHEEL_NAMES[wheel]}{'+' if sign == 1 else '-'}"
            for channel, (wheel, sign) in sorted(self.encoder_map.items())
        )
        lines.append(f"motor map: {motor or 'none recorded'}")
        lines.append(f"encoder map: {encoder or 'none recorded'}")
        lines.append(
            "geometry: "
            + (
                "wheel {0} mm, {1} counts/rev, track {2} mm, wheelbase {3} mm"
                .format(*self.geometry)
                if self.geometry else "not recorded"
            )
        )
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

        drive_lines = self._drive_export_lines(warnings)

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
                "",
                *drive_lines,
            ]
        )
        if warnings:
            block += "\n\n// WARNINGS:\n" + "\n".join(
                f"//   {warning}" for warning in warnings
            )
        return block

    def _drive_export_lines(self, warnings: list[str]) -> list[str]:
        """BuildConfig.h drive block from the motor/enc/geom records."""
        command_map = [index for index in range(4)]
        command_sign = [1] * 4
        channel_map = [index for index in range(4)]
        encoder_sign = [1] * 4
        for label, records, target_map, target_sign in (
            ("motor", self.motor_map, command_map, command_sign),
            ("enc", self.encoder_map, channel_map, encoder_sign),
        ):
            seen_wheels = [wheel for wheel, _ in records.values()]
            for wheel in range(4):
                count = seen_wheels.count(wheel)
                if count == 0:
                    warnings.append(
                        f"{label} record missing for the {WHEEL_LABELS[wheel]}"
                        f" wheel; using default channel {wheel}"
                    )
                elif count > 1:
                    warnings.append(
                        f"{label} records name the {WHEEL_LABELS[wheel]} wheel"
                        f" {count} times; re-measure"
                    )
            for channel, (wheel, sign) in records.items():
                target_map[wheel] = channel
                target_sign[wheel] = sign
        if self.geometry is None:
            warnings.append("geometry not recorded (`geom ...`); using defaults")
            geometry = (60, 4680, 160, 170)
        else:
            geometry = self.geometry
        return [
            "constexpr int8_t MotorCommandMap[4] = {"
            + ", ".join(str(value) for value in command_map) + "};",
            "constexpr int8_t MotorCommandSign[4] = {"
            + ", ".join(str(value) for value in command_sign) + "};",
            "constexpr int8_t EncoderChannelMap[4] = {"
            + ", ".join(str(value) for value in channel_map) + "};",
            "constexpr int8_t EncoderDirectionSign[4] = {"
            + ", ".join(str(value) for value in encoder_sign) + "};",
            f"constexpr uint16_t WheelDiameterMm = {geometry[0]};",
            f"constexpr uint16_t EncoderCountsPerRevolution = {geometry[1]};",
            f"constexpr uint16_t WheelTrackMm = {geometry[2]};",
            f"constexpr uint16_t WheelbaseMm = {geometry[3]};",
        ]


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
