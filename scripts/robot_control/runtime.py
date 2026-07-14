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

ONE_WAY_UNAVAILABLE_ACTIONS = frozenset(
    {"refresh_parameters", "set_host_input", "set_parameter"}
)
ONE_WAY_WARNING = (
    "ONE-WAY MODE: the firmware link is unverified; telemetry and runtime "
    "parameter operations are unavailable"
)


@dataclass(order=True)
class RuntimeCommand:
    priority: int
    order: int
    action: str = field(compare=False)
    payload: dict[str, Any] = field(default_factory=dict, compare=False)


class RobotRuntime:
    """Own the serial link and scheduled control stream on one dedicated thread."""

    def __init__(
        self,
        device: str,
        baud: int = 38400,
        *,
        use_gamepad: bool = True,
        serial_factory: Callable[[str, int], Any] = open_port,
        clock: Callable[[], float] = time.monotonic,
        maximum_reconnect_attempts: int = 10,
        startup_stabilization_seconds: float = 0.5,
        handshake_timeout_seconds: float = 2.0,
        reconnect_initial_delay_seconds: float = 0.5,
        reconnect_reset_seconds: float = 1.0,
        one_way: bool = False,
    ) -> None:
        self.device = device
        self.baud = baud
        self.use_gamepad = use_gamepad
        self.serial_factory = serial_factory
        self.clock = clock
        self.maximum_reconnect_attempts = maximum_reconnect_attempts
        self.startup_stabilization_seconds = max(0.0, startup_stabilization_seconds)
        self.handshake_timeout_seconds = max(0.01, handshake_timeout_seconds)
        self.reconnect_initial_delay_seconds = max(0.001, reconnect_initial_delay_seconds)
        self.reconnect_reset_seconds = max(0.0, reconnect_reset_seconds)
        self.one_way = bool(one_way)
        self.control_rate_hz = 10.0 if baud == 9600 else CONTROL.rate_hz
        self._control_config = CONTROL
        self._host_parameter_revision = 0
        self._stop = threading.Event()
        self._critical_estop = threading.Event()
        self._serial_write_lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._commands: queue.PriorityQueue[RuntimeCommand] = queue.PriorityQueue(maxsize=64)
        self._command_state_lock = threading.Lock()
        self._command_order = itertools.count()
        self._snapshot_lock = threading.Lock()
        self._snapshot_json = "{}"
        self._serial: Any | None = None
        self._decoder = ProtocolDecoder()
        self._sequence = 0
        self._connected = False
        self._link_verified = False
        self._link_state = "disconnected"
        self._retry_at = 0.0
        self._stabilize_until: float | None = None
        self._handshake_deadline: float | None = None
        self._bootstrap_control_at: float | None = None
        self._connected_at: float | None = None
        self._connection_failures = 0
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
            "connection_failures": 0,
            "bootstrap_frames_sent": 0,
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
        deadline = time.monotonic() + max(0.0, timeout)
        self._stop.set()
        self._critical_estop.set()
        remaining = max(0.0, deadline - time.monotonic())
        if self._serial_write_lock.acquire(timeout=remaining):
            self._serial_write_lock.release()
        if self._thread:
            self._thread.join(max(0.0, deadline - time.monotonic()))
            if self._thread.is_alive():
                message = f"Runtime thread did not stop within {timeout:.3f} seconds"
                self._last_error = message
                self._fatal_error = message
                self._link_state = "stop_timeout"
                self._record_event("error", message)
                self._publish_snapshot()

    def submit(self, action: str, payload: dict[str, Any] | None = None) -> bool:
        if action == "estop":
            self._critical_estop.set()
            return True
        if self.one_way and action in ONE_WAY_UNAVAILABLE_ACTIONS:
            self._record_event(
                "warning", f"{action} is unavailable while --one-way is active"
            )
            self._publish_snapshot()
            return False
        priority = 1 if action == "disarm" else 5
        command = RuntimeCommand(priority, next(self._command_order), action, payload or {})
        with self._command_state_lock:
            if not self._connected:
                return False
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
        now = self.clock()
        status = self._telemetry.get("status", {})
        hello = self._telemetry.get("hello", {})
        state = status.get("state")
        mode = self._telemetry.get("drive_command", {}).get(
            "control_mode", hello.get("control_mode")
        )
        snapshot = {
            "connected": self._connected,
            "link_state": self._link_state,
            "one_way": self.one_way,
            "link_verified": self._link_verified,
            "link_verification": (
                "unverified_one_way"
                if self.one_way
                else ("verified" if self._link_verified else "pending")
            ),
            "device": self.device,
            "baud": self.baud,
            "control_rate_hz": self.control_rate_hz,
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
            "connection": {
                "consecutive_failures": self._connection_failures,
                "maximum_attempts": self.maximum_reconnect_attempts,
                "retry_in_ms": round(
                    max(0.0, self._retry_at - now) * 1000.0
                    if self._link_state == "backoff" else 0.0
                ),
                "stabilize_in_ms": round(
                    max(0.0, self._stabilize_until - now) * 1000.0
                    if self._stabilize_until is not None else 0.0
                ),
                "handshake_timeout_in_ms": round(
                    max(0.0, self._handshake_deadline - now) * 1000.0
                    if self._handshake_deadline is not None else 0.0
                ),
                "bootstrap_control_in_ms": round(
                    max(0.0, self._bootstrap_control_at - now) * 1000.0
                    if self._bootstrap_control_at is not None else 0.0
                ),
            },
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
        with self._serial_write_lock:
            if self._stop.is_set() or self._serial is None:
                return False
            try:
                written = self._serial.write(data)
                if written != len(data):
                    raise SerialConnectionError(
                        f"short serial write: {written}/{len(data)}"
                    )
                return True
            except Exception as exc:
                self._host_stats["write_failures"] += 1
                self._fail_connection(f"serial write failed: {exc}")
                return False

    def _send_simple(self, message_type: MessageType) -> bool:
        return self._write(encode_message(message_type, self._next_sequence()))

    def _open_connection(self, now: float) -> None:
        link = self.serial_factory(self.device, self.baud)
        if link is None:
            raise SerialConnectionError("serial factory returned no connection")
        self._serial = link
        self._decoder = ProtocolDecoder()
        with self._command_state_lock:
            self._connected = False
            self._clear_commands_unlocked()
        self._link_verified = False
        self._link_state = "stabilizing"
        self._stabilize_until = now + self.startup_stabilization_seconds
        self._handshake_deadline = None
        self._bootstrap_control_at = None
        self._connected_at = None
        self._estop_latched = True
        self._pending_neutral_action = None
        self._neutral_since = None
        self._telemetry.clear()
        self._parameters.clear()
        self._control_config = CONTROL
        self._host_parameter_revision = 0
        self._last_control_sent_at = None

    def _start_one_way(self, now: float) -> None:
        """Enter explicit degraded operation without requiring firmware replies."""
        if self._serial is None:
            return
        self._link_state = "one_way_starting"
        self._stabilize_until = None
        self._handshake_deadline = None
        self._bootstrap_control_at = None
        if not self._send_simple(MessageType.DISARM):
            return
        safe_control = ControlFrame(
            sequence=self._next_sequence(),
            control_flags=int(ControlFlag.ESTOP_ASSERTED),
        )
        if not self._write(encode_control_frame(safe_control)):
            return
        self._host_stats["bootstrap_frames_sent"] += 1
        with self._command_state_lock:
            self._clear_commands_unlocked()
            self._connected = True
        self._link_verified = False
        self._link_state = "one_way_unverified"
        self._connected_at = now
        self._last_error = ""
        self._host_stats["reconnects"] += 1
        self._record_event("warning", ONE_WAY_WARNING)

    def _start_handshake(self, now: float) -> None:
        if self._serial is None:
            return
        self._link_state = "handshaking"
        self._stabilize_until = None
        self._handshake_deadline = now + self.handshake_timeout_seconds
        if not self._send_simple(MessageType.DISARM):
            return
        if not self._send_simple(MessageType.HELLO):
            return
        if not self._send_simple(MessageType.PARAMETER_SNAPSHOT_REQUEST):
            return
        self._send_bootstrap_control(now)

    def _send_bootstrap_control(self, now: float) -> None:
        """Open the firmware TX window without permitting any robot motion."""
        frame = ControlFrame(
            sequence=self._next_sequence(),
            control_flags=int(ControlFlag.ESTOP_ASSERTED),
        )
        if self._write(encode_control_frame(frame)):
            self._host_stats["bootstrap_frames_sent"] += 1
            period = 0.05 if self.baud == 9600 else 0.02
            self._bootstrap_control_at = now + period

    def _close_connection(self) -> None:
        with self._serial_write_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            self._serial = None
        with self._command_state_lock:
            self._connected = False
            self._clear_commands_unlocked()
        self._link_verified = False
        self._stabilize_until = None
        self._handshake_deadline = None
        self._bootstrap_control_at = None
        self._connected_at = None
        self._estop_latched = True
        self._pending_neutral_action = None
        self._neutral_since = None
        self._telemetry.clear()
        self._parameters.clear()

    def _clear_commands(self) -> None:
        with self._command_state_lock:
            self._clear_commands_unlocked()

    def _clear_commands_unlocked(self) -> None:
        while True:
            try:
                self._commands.get_nowait()
            except queue.Empty:
                return

    def _fail_connection(self, reason: str, now: float | None = None) -> None:
        if self._link_state == "fatal":
            return
        if now is None:
            now = self.clock()
        self._close_connection()
        self._last_error = reason
        self._record_event("error", reason)
        self._connection_failures += 1
        self._host_stats["connection_failures"] += 1
        if self._connection_failures >= self.maximum_reconnect_attempts:
            self._fatal_error = (
                f"Stopped after {self._connection_failures} consecutive serial failures: "
                f"{reason}"
            )
            self._link_state = "fatal"
            self._retry_at = 0.0
            self._record_event("error", self._fatal_error)
            return
        delay = min(
            5.0,
            self.reconnect_initial_delay_seconds
            * (2 ** (self._connection_failures - 1)),
        )
        self._retry_at = now + delay
        self._link_state = "backoff"

    def _read_messages(self, now: float) -> None:
        if self._serial is None:
            return
        try:
            waiting = int(getattr(self._serial, "in_waiting", 0))
            if waiting <= 0:
                return
            data = self._serial.read(min(waiting, 256))
        except Exception as exc:
            self._fail_connection(f"serial read failed: {exc}", now)
            return
        try:
            for message in self._decoder.feed(data):
                self._handle_message(message, now)
                if self._serial is None:
                    break
        except Exception as exc:
            self._fail_connection(f"serial receive processing failed: {exc}", now)

    def _handle_message(self, message: Message, now: float | None = None) -> None:
        if now is None:
            now = self.clock()
        decoded = telemetry_to_dict(message)
        kind = decoded.pop("kind")
        self._host_stats["frames_received"] += 1
        if not self._connected and not (
            kind == "hello" and self._link_state == "handshaking"
        ):
            # Discard stale frames left in the RFCOMM receive buffer.  In
            # particular, an old ACK must not trigger a write during the
            # stabilization interval or before this connection's HELLO reply.
            return
        if kind == "hello":
            self._telemetry[kind] = decoded
            if self._link_state == "handshaking":
                with self._command_state_lock:
                    self._clear_commands_unlocked()
                    self._connected = True
                self._link_verified = True
                self._link_state = "connected"
                self._handshake_deadline = None
                self._bootstrap_control_at = None
                self._connected_at = now
                self._last_error = ""
                self._host_stats["reconnects"] += 1
                self._record_event(
                    "info", f"Connected to {self.device} at {self.baud} baud"
                )
        elif kind == "parameter":
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
                self._connection_failures
                and self._connected_at is not None
                and now - self._connected_at >= self.reconnect_reset_seconds
            ):
                self._connection_failures = 0
        if (
            self._pending_neutral_action is not None
            and self._neutral_since is not None
            and now - self._neutral_since >= 0.6
        ):
            self._send_simple(self._pending_neutral_action)
            self._pending_neutral_action = None

    def _process_command(self, command: RuntimeCommand) -> None:
        action = command.action
        if self.one_way and action in ONE_WAY_UNAVAILABLE_ACTIONS:
            self._record_event(
                "warning", f"{action} is unavailable while --one-way is active"
            )
            return
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
                if not self._connected:
                    return
        for _ in range(16):
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                break
            self._process_command(command)
            if not self._connected:
                break

    def _best_effort_shutdown(self) -> None:
        with self._serial_write_lock:
            if self._serial is None:
                return
            if self._connected:
                try:
                    frame = ControlFrame(
                        sequence=self._next_sequence(),
                        control_flags=int(ControlFlag.ESTOP_ASSERTED),
                    )
                    self._serial.write(encode_control_frame(frame))
                    self._serial.write(
                        encode_message(MessageType.DISARM, self._next_sequence())
                    )
                except Exception:
                    pass
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None

    def _run(self) -> None:
        control_period = 1.0 / self.control_rate_hz
        control_deadline = self.clock()
        self._retry_at = control_deadline
        while not self._stop.is_set():
            now = self.clock()
            was_connected = self._connected
            if self._serial is None and self._link_state != "fatal" and now >= self._retry_at:
                try:
                    self._open_connection(now)
                except Exception as exc:
                    if self._stop.is_set():
                        break
                    self._fail_connection(f"serial open failed: {exc}", now)

            if self._stop.is_set():
                break

            if (
                self._link_state == "stabilizing"
                and self._stabilize_until is not None
                and now >= self._stabilize_until
            ):
                if self.one_way:
                    self._start_one_way(now)
                else:
                    self._start_handshake(now)

            if (
                self._link_state == "handshaking"
                and self._bootstrap_control_at is not None
                and now >= self._bootstrap_control_at
            ):
                self._send_bootstrap_control(now)

            self._read_messages(now)
            if not was_connected and self._connected:
                control_deadline = now
            if (
                self._link_state == "handshaking"
                and self._handshake_deadline is not None
                and now >= self._handshake_deadline
            ):
                self._fail_connection("timed out waiting for HELLO_TELEMETRY", now)

            if self._connected:
                self._drain_commands()
            if self._connected and now >= control_deadline:
                lateness = now - control_deadline
                self._host_stats["max_control_lateness_ms"] = max(
                    self._host_stats["max_control_lateness_ms"], lateness * 1000.0
                )
                missed = int(lateness / control_period)
                self._host_stats["missed_control_deadlines"] += missed
                control_deadline += (missed + 1) * control_period
                self._send_control(now)
            self._publish_snapshot()
            if self._link_state == "fatal":
                break
            wake_times = [now + 0.01]
            if self._connected:
                wake_times.append(control_deadline)
            elif self._link_state == "backoff":
                wake_times.append(self._retry_at)
            elif self._link_state == "stabilizing" and self._stabilize_until is not None:
                wake_times.append(self._stabilize_until)
            elif self._link_state == "handshaking" and self._handshake_deadline is not None:
                wake_times.append(self._handshake_deadline)
                if self._bootstrap_control_at is not None:
                    wake_times.append(self._bootstrap_control_at)
            wait_until = min(wake_times)
            self._stop.wait(max(0.001, wait_until - self.clock()))
        self._best_effort_shutdown()
        with self._command_state_lock:
            self._connected = False
            self._clear_commands_unlocked()
        if self._link_state != "fatal":
            self._link_state = "stopped"
        self._publish_snapshot()
