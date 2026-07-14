"""Safety-oriented serial/gamepad runtime, independent of the local web UI."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import copy
import itertools
import json
import math
import queue
import threading
import time
from typing import Any, Callable

from .config import CONTROL
from .input import open_joystick
from .mapping import map_gamepad
from .protocol import (
    ControlFlag,
    ControlFrame,
    Message,
    MessageType,
    ParameterGroup,
    ProtocolDecoder,
    build_parameter_data,
    encode_control_frame,
    encode_message,
    encode_parameter_set,
    telemetry_to_dict,
)
from .transport import SerialConnectionError, open_port


STATE_NAMES = ["BOOT", "DISARMED", "ARMED", "ESTOP", "FAULT"]
CONTROL_MODE_NAMES = [
    "none", "l293d_open_loop_pwm", "uart_open_loop_pwm", "uart_closed_loop_speed"
]
PROFILE_NAMES = [
    "safe_idle", "l293d_dev", "uart_closed_loop_qualification",
    "uart_closed_loop_robot", "uart_open_loop_calibration", "arm_calibration",
]


@dataclass(order=True)
class RuntimeCommand:
    priority: int
    order: int
    action: str = field(compare=False)
    payload: dict[str, Any] = field(default_factory=dict, compare=False)


class RobotRuntime:
    """Own the serial link and 20 Hz control stream on one dedicated thread."""

    def __init__(
        self,
        device: str,
        baud: int = 38400,
        *,
        use_gamepad: bool = True,
        serial_factory: Callable[[str, int], Any] = open_port,
        clock: Callable[[], float] = time.monotonic,
        maximum_reconnect_attempts: int = 10,
    ) -> None:
        self.device = device
        self.baud = baud
        self.use_gamepad = use_gamepad
        self.serial_factory = serial_factory
        self.clock = clock
        self.maximum_reconnect_attempts = maximum_reconnect_attempts
        self.control_rate_hz = 10.0 if baud == 9600 else CONTROL.rate_hz
        self._control_config = CONTROL
        self._host_parameter_revision = 0
        self._stop = threading.Event()
        self._critical_estop = threading.Event()
        self._thread: threading.Thread | None = None
        self._commands: queue.PriorityQueue[RuntimeCommand] = queue.PriorityQueue(maxsize=64)
        self._command_order = itertools.count()
        self._snapshot_lock = threading.Lock()
        self._snapshot_json = "{}"
        self._serial: Any | None = None
        self._decoder = ProtocolDecoder()
        self._sequence = 0
        self._connected = False
        self._controller_state = "not_started"
        self._last_error = ""
        self._fatal_error = ""
        self._estop_latched = True
        self._pending_neutral_action: MessageType | None = None
        self._neutral_since: float | None = None
        self._latest_control = ControlFrame(sequence=0)
        self._telemetry: dict[str, Any] = {}
        self._parameters: dict[str, dict[str, Any]] = {}
        self._events: list[dict[str, Any]] = []
        self._host_stats = {
            "frames_sent": 0,
            "frames_received": 0,
            "write_failures": 0,
            "reconnects": 0,
            "missed_control_deadlines": 0,
            "max_control_lateness_ms": 0.0,
            "last_control_interval_ms": 0.0,
        }
        self._last_control_sent_at: float | None = None
        self._previous_estop_button = False
        self._previous_clear_estop_button = False
        self._controller_retry_at = 0.0
        self._publish_snapshot()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="warehouse-robot-safety-runtime",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        self._stop.set()
        self._critical_estop.set()
        if self._thread:
            self._thread.join(timeout)

    def submit(self, action: str, payload: dict[str, Any] | None = None) -> bool:
        if action == "estop":
            self._critical_estop.set()
            return True
        priority = 1 if action == "disarm" else 5
        command = RuntimeCommand(priority, next(self._command_order), action, payload or {})
        try:
            self._commands.put_nowait(command)
            return True
        except queue.Full:
            return False

    def snapshot_json(self) -> str:
        with self._snapshot_lock:
            return self._snapshot_json

    def snapshot(self) -> dict[str, Any]:
        return json.loads(self.snapshot_json())

    def _next_sequence(self) -> int:
        value = self._sequence
        self._sequence = (self._sequence + 1) & 0xFF
        return value

    def _record_event(self, level: str, message: str) -> None:
        self._events.append({"time": time.time(), "level": level, "message": message})
        del self._events[:-20]

    def _publish_snapshot(self) -> None:
        status = self._telemetry.get("status", {})
        hello = self._telemetry.get("hello", {})
        state = status.get("state")
        mode = self._telemetry.get("drive_command", {}).get(
            "control_mode", hello.get("control_mode")
        )
        snapshot = {
            "connected": self._connected,
            "device": self.device,
            "baud": self.baud,
            "controller_state": self._controller_state,
            "arm_available": self.use_gamepad,
            "host_estop_latched": self._estop_latched,
            "pending_action": (
                self._pending_neutral_action.name.lower()
                if self._pending_neutral_action is not None else None
            ),
            "state_name": STATE_NAMES[state] if isinstance(state, int) and state < len(STATE_NAMES) else "UNKNOWN",
            "control_mode_name": (
                CONTROL_MODE_NAMES[mode]
                if isinstance(mode, int) and mode < len(CONTROL_MODE_NAMES) else "unknown"
            ),
            "profile_name": (
                PROFILE_NAMES[hello.get("profile")]
                if isinstance(hello.get("profile"), int)
                and hello["profile"] < len(PROFILE_NAMES) else "unknown"
            ),
            "last_error": self._last_error,
            "fatal_error": self._fatal_error,
            "telemetry": copy.deepcopy(self._telemetry),
            "parameters": copy.deepcopy(self._parameters),
            "host_stats": copy.deepcopy(self._host_stats),
            "host_parameter_revision": self._host_parameter_revision,
            "host_input": {
                f"{name}_{field}": round(getattr(axis, field) * scale)
                for name, axis in (
                    ("forward", self._control_config.drive_forward),
                    ("turn", self._control_config.drive_turn),
                    ("arm_yaw", self._control_config.arm_yaw),
                    ("arm_reach", self._control_config.arm_reach),
                )
                for field, scale in (("deadzone", 1000), ("power", 100))
            },
            "protocol_stats": {
                "malformed_frames": self._decoder.malformed_frames,
                "rx_overflows": self._decoder.overflows,
            },
            "events": copy.deepcopy(self._events),
        }
        encoded = json.dumps(snapshot, separators=(",", ":"), allow_nan=False)
        with self._snapshot_lock:
            self._snapshot_json = encoded

    def _write(self, data: bytes) -> bool:
        if self._serial is None:
            return False
        try:
            written = self._serial.write(data)
            if written != len(data):
                raise SerialConnectionError(f"short serial write: {written}/{len(data)}")
            return True
        except Exception as exc:
            self._host_stats["write_failures"] += 1
            self._disconnect(f"serial write failed: {exc}")
            return False

    def _send_simple(self, message_type: MessageType) -> bool:
        return self._write(encode_message(message_type, self._next_sequence()))

    def _connect(self) -> None:
        self._serial = self.serial_factory(self.device, self.baud)
        self._decoder = ProtocolDecoder()
        self._connected = True
        self._last_error = ""
        self._estop_latched = True
        self._pending_neutral_action = None
        self._neutral_since = None
        self._telemetry.clear()
        self._parameters.clear()
        self._control_config = CONTROL
        self._host_parameter_revision = 0
        connected = self._send_simple(MessageType.DISARM)
        connected = self._send_simple(MessageType.HELLO) and connected
        connected = self._send_simple(MessageType.PARAMETER_SNAPSHOT_REQUEST) and connected
        if not connected or not self._connected:
            raise SerialConnectionError("serial link failed during startup handshake")
        self._record_event("info", f"Connected to {self.device} at {self.baud} baud")

    def _disconnect(self, reason: str) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
        self._serial = None
        self._connected = False
        self._estop_latched = True
        self._pending_neutral_action = None
        self._neutral_since = None
        self._telemetry.clear()
        self._parameters.clear()
        self._last_error = reason
        self._record_event("error", reason)

    def _read_messages(self) -> None:
        if self._serial is None:
            return
        try:
            waiting = int(getattr(self._serial, "in_waiting", 0))
            if waiting <= 0:
                return
            data = self._serial.read(min(waiting, 256))
        except Exception as exc:
            self._disconnect(f"serial read failed: {exc}")
            return
        for message in self._decoder.feed(data):
            self._handle_message(message)

    def _handle_message(self, message: Message) -> None:
        decoded = telemetry_to_dict(message)
        kind = decoded.pop("kind")
        self._host_stats["frames_received"] += 1
        if kind == "parameter":
            key = f"{decoded['group']}:{decoded['index']}"
            self._parameters[key] = decoded
        elif kind == "ack":
            self._record_event("info", f"Parameter commit accepted at revision {decoded['revision']}")
            self._send_simple(MessageType.PARAMETER_SNAPSHOT_REQUEST)
        elif kind == "nack":
            self._record_event(
                "error", f"Firmware rejected message {decoded['rejected_type']} (reason {decoded['reason']})"
            )
        elif kind != "unknown":
            self._telemetry[kind] = decoded

    def _read_controller(self) -> ControlFrame:
        if not self.use_gamepad:
            self._controller_state = "disabled"
            return ControlFrame(sequence=self._sequence)
        if not hasattr(self, "_pygame"):
            if self.clock() < self._controller_retry_at:
                raise RuntimeError("controller reconnect backoff")
            self._pygame, self._joystick = open_joystick(self._control_config.joystick_index)
        self._pygame.event.pump()
        if not self._joystick.get_init():
            raise RuntimeError("controller disconnected")
        frame = map_gamepad(self._joystick, self._control_config, self._sequence)
        self._controller_state = self._joystick.get_name()
        estop_pressed = bool(self._joystick.get_button(self._control_config.estop_button))
        clear_pressed = bool(self._joystick.get_button(self._control_config.clear_estop_button))
        if estop_pressed:
            self._estop_latched = True
        if clear_pressed and not self._previous_clear_estop_button:
            self._estop_latched = False
            self._neutral_since = None
            self._pending_neutral_action = MessageType.CLEAR_ESTOP
        self._previous_estop_button = estop_pressed
        self._previous_clear_estop_button = clear_pressed
        return frame

    def _current_control(self) -> ControlFrame:
        try:
            frame = self._read_controller()
        except Exception as exc:
            self._controller_state = f"unavailable: {exc}"
            self._estop_latched = True
            self._controller_retry_at = self.clock() + 1.0
            for attribute in ("_pygame", "_joystick"):
                if hasattr(self, attribute):
                    delattr(self, attribute)
            frame = ControlFrame(sequence=self._sequence)
        return replace(
            frame,
            sequence=self._next_sequence(),
            control_flags=(
                int(ControlFlag.ESTOP_ASSERTED) if self._estop_latched else 0
            ),
        )

    def _send_control(self, now: float) -> None:
        frame = self._current_control()
        self._latest_control = frame
        if frame.neutral():
            if self._neutral_since is None:
                self._neutral_since = now
        else:
            self._neutral_since = None
        if self._write(encode_control_frame(frame)):
            self._host_stats["frames_sent"] += 1
            if self._last_control_sent_at is not None:
                self._host_stats["last_control_interval_ms"] = (
                    now - self._last_control_sent_at
                ) * 1000.0
            self._last_control_sent_at = now
        if (
            self._pending_neutral_action is not None
            and self._neutral_since is not None
            and now - self._neutral_since >= 0.6
        ):
            self._send_simple(self._pending_neutral_action)
            self._pending_neutral_action = None

    def _process_command(self, command: RuntimeCommand) -> None:
        action = command.action
        if action == "disarm":
            self._pending_neutral_action = None
            self._send_simple(MessageType.DISARM)
        elif action == "arm":
            if not self.use_gamepad:
                self._record_event("error", "ARM is disabled when --no-gamepad is active")
                return
            self._pending_neutral_action = MessageType.ARM
        elif action == "clear_estop":
            self._estop_latched = False
            self._neutral_since = None
            self._pending_neutral_action = MessageType.CLEAR_ESTOP
        elif action == "clear_fault":
            self._pending_neutral_action = MessageType.CLEAR_FAULT
        elif action == "refresh_parameters":
            self._send_simple(MessageType.PARAMETER_SNAPSHOT_REQUEST)
        elif action == "set_host_input":
            status = self._telemetry.get("status", {})
            if status.get("state") != 1:
                self._record_event("error", "Host input tuning is allowed only while DISARMED")
                return
            values = command.payload.get("values")
            if not isinstance(values, dict):
                self._record_event("error", "Invalid host input tuning request")
                return
            replacements = {}
            try:
                for name, attribute in (
                    ("forward", "drive_forward"), ("turn", "drive_turn"),
                    ("arm_yaw", "arm_yaw"), ("arm_reach", "arm_reach"),
                ):
                    deadzone_raw = values[f"{name}_deadzone"]
                    power_raw = values[f"{name}_power"]
                    if not all(
                        isinstance(value, (int, float)) and not isinstance(value, bool)
                        and math.isfinite(value)
                        for value in (deadzone_raw, power_raw)
                    ):
                        raise ValueError("all values must be finite numbers")
                    deadzone = float(deadzone_raw) / 1000.0
                    power = float(power_raw) / 100.0
                    if not 0.0 <= deadzone <= 0.30 or not 0.50 <= power <= 3.0:
                        raise ValueError("deadzone must be 0..300 and power must be 50..300")
                    replacements[attribute] = replace(
                        getattr(self._control_config, attribute),
                        deadzone=deadzone, power=power,
                    )
            except (KeyError, TypeError, ValueError) as exc:
                self._record_event("error", f"Invalid host input tuning request: {exc}")
                return
            self._control_config = replace(self._control_config, **replacements)
            self._host_parameter_revision += 1
            self._record_event(
                "info", f"Host input tuning accepted at revision {self._host_parameter_revision}"
            )
        elif action == "set_parameter":
            status = self._telemetry.get("status", {})
            if status.get("state") != 1:
                self._record_event("error", "Runtime parameters can only be committed while DISARMED")
                return
            try:
                group = ParameterGroup[command.payload["group"]]
                index = int(command.payload.get("index", 0))
                values = dict(command.payload["values"])
                data = build_parameter_data(group, index, values)
                revision = int(status.get("revision", 0))
                packet = encode_parameter_set(
                    self._next_sequence(), group, index, revision, data
                )
            except (KeyError, TypeError, ValueError) as exc:
                self._record_event("error", f"Invalid parameter request: {exc}")
                return
            self._write(packet)
        else:
            self._record_event("error", f"Unknown runtime action: {action}")

    def _drain_commands(self) -> None:
        if self._critical_estop.is_set():
            self._critical_estop.clear()
            self._estop_latched = True
            self._pending_neutral_action = None
            if self._connected:
                emergency = replace(
                    self._latest_control,
                    sequence=self._next_sequence(),
                    forward=0, turn=0, strafe=0,
                    control_flags=int(ControlFlag.ESTOP_ASSERTED),
                )
                self._write(encode_control_frame(emergency))
                self._send_simple(MessageType.DISARM)
        for _ in range(16):
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                break
            self._process_command(command)

    def _best_effort_shutdown(self) -> None:
        if self._serial is None:
            return
        try:
            frame = ControlFrame(
                sequence=self._next_sequence(),
                control_flags=int(ControlFlag.ESTOP_ASSERTED),
            )
            self._serial.write(encode_control_frame(frame))
            self._serial.write(encode_message(MessageType.DISARM, self._next_sequence()))
        except Exception:
            pass
        try:
            self._serial.close()
        except Exception:
            pass

    def _run(self) -> None:
        control_period = 1.0 / self.control_rate_hz
        control_deadline = self.clock()
        retry_at = control_deadline
        reconnect_attempts = 0
        while not self._stop.is_set():
            now = self.clock()
            if self._serial is None and now >= retry_at:
                try:
                    self._connect()
                    reconnect_attempts = 0
                    self._host_stats["reconnects"] += 1
                    control_deadline = now
                except Exception as exc:
                    reconnect_attempts += 1
                    self._last_error = str(exc)
                    if reconnect_attempts >= self.maximum_reconnect_attempts:
                        self._fatal_error = (
                            f"Stopped after {reconnect_attempts} failed connection attempts: {exc}"
                        )
                        self._record_event("error", self._fatal_error)
                        self._publish_snapshot()
                        break
                    retry_at = now + min(5.0, 0.5 * reconnect_attempts)

            self._drain_commands()
            self._read_messages()
            if self._serial is not None and now >= control_deadline:
                lateness = now - control_deadline
                self._host_stats["max_control_lateness_ms"] = max(
                    self._host_stats["max_control_lateness_ms"], lateness * 1000.0
                )
                missed = int(lateness / control_period)
                self._host_stats["missed_control_deadlines"] += missed
                control_deadline += (missed + 1) * control_period
                self._send_control(now)
            self._publish_snapshot()
            wait_until = min(
                control_deadline,
                retry_at if self._serial is None else control_deadline,
                now + 0.01,
            )
            self._stop.wait(max(0.001, wait_until - self.clock()))
        self._best_effort_shutdown()
        self._connected = False
        self._publish_snapshot()
