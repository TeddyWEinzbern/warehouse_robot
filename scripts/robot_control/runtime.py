"""Fail-closed serial/gamepad runtime for compact protocol v4."""

from __future__ import annotations

from dataclasses import dataclass, field, replace
import copy
import itertools
import json
import queue
import threading
import time
from typing import Any, Callable

from .config import CONTROL
from .input import open_joystick
from .mapping import map_gamepad
from .protocol import (
    ControlFrame,
    Message,
    MessageType,
    ProtocolDecoder,
    decode_message_data,
    encode_control_frame,
    encode_message,
    encode_urgent_disarm,
)
from .transport import SerialConnectionError, open_port


STATE_NAMES = ("DISARMED", "ARMED", "FAULT")
STATE_DISARMED = 0
STATE_ARMED = 1
STATE_FAULT = 2
DRIVER_MODE_NAMES = ("disabled", "open_loop", "closed_loop")
PROFILE_NAMES = {
    3: "robot",
    6: "robot_open_loop",
    7: "calibration",
}
FAULT_NAMES = (
    "scheduler_overrun",
    "drive_initialization",
    "encoder_stale",
    "encoder_malformed",
    "encoder_implausible",
    "encoder_sign",
    "drive_stall",
    "drive_mismatch",
    "arm_target",
)
WARNING_NAMES = (
    "drive_unqualified",
    "arm_target_limited",
    "fault_state_unsafe",
    "encoder_timeout_ignored",
    "encoder_sign_ignored",
    "drive_mismatch_ignored",
)
FAULT_STATE_UNSAFE_WARNING = 1 << 2

CONTROL_RATE_HZ = 30.0
CRITICAL_STATUS_RATE_HZ = 2.0
CRITICAL_STATUS_STALE_SECONDS = 1.5
NEUTRAL_CONFIRM_SECONDS = 0.6


def _reschedule_control(
    now: float, previous_deadline: float, period: float
) -> tuple[float, float, int]:
    """Skip missed slots and guarantee one full period after this send."""
    lateness = max(0.0, now - previous_deadline)
    missed = int(lateness / period)
    return now + period, lateness, missed


@dataclass(order=True)
class RuntimeCommand:
    priority: int
    order: int
    action: str = field(compare=False)
    payload: dict[str, Any] = field(default_factory=dict, compare=False)


class RobotRuntime:
    """Own the HC-06 link and the deterministic 30 Hz control stream."""

    def __init__(
        self,
        device: str,
        baud: int = 9600,
        *,
        use_gamepad: bool = True,
        serial_factory: Callable[[str, int], Any] = open_port,
        clock: Callable[[], float] = time.monotonic,
        maximum_reconnect_attempts: int = 10,
        startup_stabilization_seconds: float = 0.5,
        handshake_timeout_seconds: float = 2.0,
        reconnect_initial_delay_seconds: float = 0.5,
        reconnect_reset_seconds: float = 1.0,
    ) -> None:
        if baud not in (9600, 38400):
            raise ValueError("baud must be 9600 or 38400")
        self.device = device
        self.baud = baud
        self.use_gamepad = use_gamepad
        self.serial_factory = serial_factory
        self.clock = clock
        self.maximum_reconnect_attempts = maximum_reconnect_attempts
        self.startup_stabilization_seconds = max(0.0, startup_stabilization_seconds)
        self.handshake_timeout_seconds = max(0.01, handshake_timeout_seconds)
        self.reconnect_initial_delay_seconds = max(
            0.001, reconnect_initial_delay_seconds
        )
        self.reconnect_reset_seconds = max(0.0, reconnect_reset_seconds)
        self.control_rate_hz = CONTROL_RATE_HZ
        self.critical_status_rate_hz = CRITICAL_STATUS_RATE_HZ
        self._control_config = CONTROL

        self._stop = threading.Event()
        self._wake = threading.Event()
        self._urgent_disarm = threading.Event()
        self._serial_write_lock = threading.RLock()
        self._safety_action_lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._commands: queue.PriorityQueue[RuntimeCommand] = queue.PriorityQueue(
            maxsize=32
        )
        self._command_state_lock = threading.Lock()
        self._command_order = itertools.count()
        self._snapshot_lock = threading.Lock()
        self._snapshot_json = "{}"

        self._serial: Any | None = None
        self._decoder = ProtocolDecoder()
        self._generic_sequence = 0
        self._fast_sequence = 0
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
        self._disarm_pending_confirmation = True
        self._pending_neutral_action: MessageType | None = None
        self._neutral_since: float | None = None
        self._latest_control = ControlFrame(sequence=0)
        self._hello: dict[str, Any] = {}
        self._critical_status: dict[str, Any] = {}
        self._last_status_at: float | None = None
        # Conservative projection of when all bytes already accepted by the
        # OS/RFCOMM writer will have left the HC-06 UART toward the Uno.
        self._host_wire_available_at = 0.0
        self._events: list[dict[str, Any]] = []
        self._host_stats = {
            "control_frames_sent": 0,
            "urgent_disarm_frames_sent": 0,
            "frames_received": 0,
            "write_failures": 0,
            "reconnects": 0,
            "connection_failures": 0,
            "bootstrap_frames_sent": 0,
            "missed_control_deadlines": 0,
            "max_control_lateness_ms": 0.0,
            "last_control_interval_ms": 0.0,
            "actual_control_rate_hz": 0.0,
        }
        self._control_send_times: list[float] = []
        self._last_control_sent_at: float | None = None
        self._previous_arm_button = False
        self._previous_disarm_button = False
        self._controller_retry_at = 0.0
        self._publish_snapshot()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._wake.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="warehouse-robot-safety-runtime",
            daemon=True,
        )
        self._thread.start()

    def stop(self, timeout: float = 3.0) -> None:
        deadline = time.monotonic() + max(0.0, timeout)
        self._stop.set()
        self._urgent_disarm.set()
        self._wake.set()
        remaining = max(0.0, deadline - time.monotonic())
        if self._serial_write_lock.acquire(timeout=remaining):
            self._serial_write_lock.release()
        if self._thread:
            self._thread.join(max(0.0, deadline - time.monotonic()))
            if self._thread.is_alive():
                message = (
                    f"Runtime thread did not stop within {timeout:.3f} seconds"
                )
                self._last_error = message
                self._fatal_error = message
                self._link_state = "stop_timeout"
                self._record_event("error", message)
                self._publish_snapshot()

    def submit(self, action: str, payload: dict[str, Any] | None = None) -> bool:
        """Submit a normal runtime/controller action.

        Fault clearing is intentionally excluded; only the loopback WebUI
        entry point may request it.
        """
        return self._submit(action, payload, allow_clear_fault=False)

    def submit_webui(
        self, action: str, payload: dict[str, Any] | None = None
    ) -> bool:
        """Submit an action from the official loopback WebUI boundary."""
        return self._submit(action, payload, allow_clear_fault=True)

    def _submit(
        self,
        action: str,
        payload: dict[str, Any] | None,
        *,
        allow_clear_fault: bool,
    ) -> bool:
        if action == "disarm":
            self._request_urgent_disarm()
            return True
        allowed = {"arm"}
        if allow_clear_fault:
            allowed.add("clear_fault")
        if action not in allowed:
            return False
        command = RuntimeCommand(
            5, next(self._command_order), action, payload or {}
        )
        with self._command_state_lock:
            if not self._connected:
                return False
            try:
                self._commands.put_nowait(command)
            except queue.Full:
                return False
        self._wake.set()
        return True

    def _request_urgent_disarm(self) -> None:
        """Invalidate older intent and wake the queue-bypassing safety path."""
        with self._safety_action_lock:
            self._disarm_pending_confirmation = True
            self._pending_neutral_action = None
            with self._command_state_lock:
                self._clear_commands_unlocked()
            self._urgent_disarm.set()
        self._wake.set()

    def snapshot_json(self) -> str:
        with self._snapshot_lock:
            return self._snapshot_json

    def snapshot(self) -> dict[str, Any]:
        return json.loads(self.snapshot_json())

    def _next_generic_sequence(self) -> int:
        value = self._generic_sequence
        self._generic_sequence = (self._generic_sequence + 1) & 0xFF
        return value

    def _next_fast_sequence(self) -> int:
        value = self._fast_sequence
        self._fast_sequence = (self._fast_sequence + 1) & 0x3F
        return value

    def _record_event(self, level: str, message: str) -> None:
        self._events.append(
            {"time": time.time(), "level": level, "message": message}
        )
        del self._events[:-20]

    @staticmethod
    def _bit_names(value: int, names: tuple[str, ...]) -> list[str]:
        found = [name for bit, name in enumerate(names) if value & (1 << bit)]
        known_mask = (1 << len(names)) - 1
        unknown = value & ~known_mask
        if unknown:
            found.append(f"unknown_0x{unknown:04x}")
        return found

    def _status_fresh(self, now: float) -> bool:
        return (
            self._connected
            and self._last_status_at is not None
            and now - self._last_status_at <= CRITICAL_STATUS_STALE_SECONDS
        )

    def _firmware_ready_to_arm(self, now: float) -> bool:
        return (
            self._status_fresh(now)
            and self._critical_status.get("state") == STATE_DISARMED
            and bool(self._critical_status.get("link_alive", False))
            and bool(self._critical_status.get("ready_to_arm", False))
        )

    def _build_allows_motion(self) -> bool:
        arm_enabled = bool(self._hello.get("arm_enabled", False))
        drive_enabled = bool(self._hello.get("drive_enabled", False))
        return (
            (arm_enabled or drive_enabled)
            and (not arm_enabled or bool(self._hello.get("arm_calibrated", False)))
            and (
                not drive_enabled
                or bool(self._hello.get("drive_calibrated", False))
            )
        )

    def _publish_snapshot(self) -> None:
        now = self.clock()
        status_fresh = self._status_fresh(now)
        status_age_ms = (
            round(max(0.0, now - self._last_status_at) * 1000.0)
            if self._last_status_at is not None
            else None
        )
        state = self._critical_status.get("state") if status_fresh else None
        faults = (
            int(self._critical_status.get("faults", 0)) if status_fresh else 0
        )
        warnings = (
            int(self._critical_status.get("warnings", 0)) if status_fresh else 0
        )
        driver_mode = self._hello.get("driver_mode")
        profile = self._hello.get("profile")
        ready_to_arm = self._firmware_ready_to_arm(now)
        snapshot = {
            "connected": self._connected,
            "link_state": self._link_state,
            "link_verified": self._link_verified,
            "device": self.device,
            "bluetooth_module": "HC-06",
            "baud": self.baud,
            "control_rate_hz": self.control_rate_hz,
            "critical_status_rate_hz": self.critical_status_rate_hz,
            "status_fresh": status_fresh,
            "status_age_ms": status_age_ms,
            "controller_state": self._controller_state,
            "ready_to_arm": ready_to_arm,
            "arm_available": (
                self.use_gamepad
                and ready_to_arm
                and self._build_allows_motion()
                and not self._disarm_pending_confirmation
            ),
            "disarm_pending_confirmation": self._disarm_pending_confirmation,
            "pending_action": (
                self._pending_neutral_action.name.lower()
                if self._pending_neutral_action is not None
                else None
            ),
            "state_name": (
                STATE_NAMES[state]
                if isinstance(state, int) and 0 <= state < len(STATE_NAMES)
                else "UNKNOWN"
            ),
            "faults": faults,
            "fault_names": self._bit_names(faults, FAULT_NAMES),
            "warnings": warnings,
            "warning_names": self._bit_names(warnings, WARNING_NAMES),
            "fault_state_unsafe": bool(
                warnings & FAULT_STATE_UNSAFE_WARNING
            ),
            "last_accepted_control_sequence": (
                self._critical_status.get("last_accepted_control_sequence")
                if status_fresh
                else None
            ),
            "firmware": {
                **copy.deepcopy(self._hello),
                "profile_name": (
                    PROFILE_NAMES[profile]
                    if isinstance(profile, int) and profile in PROFILE_NAMES
                    else (f"profile_{profile}" if isinstance(profile, int) else "unknown")
                ),
                "driver_mode_name": (
                    DRIVER_MODE_NAMES[driver_mode]
                    if isinstance(driver_mode, int)
                    and 0 <= driver_mode < len(DRIVER_MODE_NAMES)
                    else "unknown"
                ),
            },
            "last_error": self._last_error,
            "fatal_error": self._fatal_error,
            "connection": {
                "consecutive_failures": self._connection_failures,
                "maximum_attempts": self.maximum_reconnect_attempts,
                "retry_in_ms": round(
                    max(0.0, self._retry_at - now) * 1000.0
                    if self._link_state == "backoff"
                    else 0.0
                ),
                "stabilize_in_ms": round(
                    max(0.0, self._stabilize_until - now) * 1000.0
                    if self._stabilize_until is not None
                    else 0.0
                ),
                "handshake_timeout_in_ms": round(
                    max(0.0, self._handshake_deadline - now) * 1000.0
                    if self._handshake_deadline is not None
                    else 0.0
                ),
            },
            "host_stats": copy.deepcopy(self._host_stats),
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
                # A successful RFCOMM write commonly means "buffered", not
                # "already clocked out by the HC-06". Project the full 8N1
                # airtime from the later of the previous queue end and this
                # completed write. Starting after write() is conservative if
                # transmission was already progressing while it blocked.
                queued_at = self.clock()
                self._host_wire_available_at = (
                    max(queued_at, self._host_wire_available_at)
                    + len(data) * 10.0 / self.baud
                )
                return True
            except Exception as exc:
                self._host_stats["write_failures"] += 1
                self._fail_connection(f"serial write failed: {exc}")
                return False

    def _send_simple(self, message_type: MessageType) -> bool:
        return self._write(
            encode_message(message_type, self._next_generic_sequence())
        )

    def _send_urgent_disarm(self) -> bool:
        sent = self._write(
            encode_urgent_disarm(self._next_fast_sequence())
        )
        if sent:
            self._host_stats["urgent_disarm_frames_sent"] += 1
        return sent

    def _send_urgent_disarm_burst(self) -> bool:
        """Send three idempotent frames so one dropped packet is harmless."""
        for _ in range(3):
            if not self._send_urgent_disarm():
                return False
        return True

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
        self._disarm_pending_confirmation = True
        self._pending_neutral_action = None
        self._neutral_since = None
        self._urgent_disarm.clear()
        self._hello.clear()
        self._critical_status.clear()
        self._last_status_at = None
        self._last_control_sent_at = None
        self._control_send_times.clear()
        self._host_wire_available_at = now

    def _start_handshake(self, now: float) -> None:
        if self._serial is None:
            return
        self._link_state = "handshaking"
        self._stabilize_until = None
        self._handshake_deadline = now + self.handshake_timeout_seconds
        if not self._send_urgent_disarm_burst():
            return
        if not self._send_simple(MessageType.HELLO):
            return
        # Do not queue the first 11-byte control behind the safety/HELLO
        # prelude. At 9600 baud that would make the following control
        # physically start only 11 bytes later even if Python write calls are
        # 33 ms apart. Wait a conservative full prelude wire time after the
        # final write so the first control starts with an empty HC-06 UART
        # queue.
        prelude_bytes = (
            3 * len(encode_urgent_disarm(0))
            + len(encode_message(MessageType.HELLO, 0))
        )
        self._bootstrap_control_at = (
            self.clock() + prelude_bytes * 10.0 / self.baud
        )

    def _send_bootstrap_control(self) -> None:
        """Transmit neutral controls while awaiting the verified HELLO reply."""
        frame = ControlFrame(sequence=self._next_fast_sequence())
        if self._write(encode_control_frame(frame)):
            # Anchor the next start after write() completed. RFCOMM writes are
            # normally quick, but a blocked writer must lengthen this interval
            # instead of consuming the Uno's reverse-direction response window.
            sent_at = self.clock()
            self._host_stats["bootstrap_frames_sent"] += 1
            self._bootstrap_control_at = (
                sent_at + 1.0 / CONTROL_RATE_HZ
            )

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
        self._disarm_pending_confirmation = True
        self._pending_neutral_action = None
        self._neutral_since = None
        self._urgent_disarm.clear()
        self._hello.clear()
        self._critical_status.clear()
        self._last_status_at = None
        self._host_wire_available_at = 0.0

    def _control_ready_at(self, deadline: float) -> float:
        """Do not queue a control behind event traffic on the HC-06 UART."""
        return max(deadline, self._host_wire_available_at)

    def _bootstrap_control_ready(self, now: float) -> bool:
        return (
            self._bootstrap_control_at is not None
            and now >= self._control_ready_at(self._bootstrap_control_at)
        )

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
                f"Stopped after {self._connection_failures} consecutive serial "
                f"failures: {reason}"
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
            self._fail_connection(
                f"serial receive processing failed: {exc}", now
            )

    def _handle_message(self, message: Message, now: float | None = None) -> None:
        if now is None:
            now = self.clock()
        decoded = decode_message_data(message)
        kind = decoded.get("kind")
        self._host_stats["frames_received"] += 1
        if not self._connected and not (
            kind == "hello" and self._link_state == "handshaking"
        ):
            return
        if kind == "hello":
            if decoded["baud"] != self.baud:
                self._fail_connection(
                    f"firmware reports {decoded['baud']} baud but host uses "
                    f"{self.baud}",
                    now,
                )
                return
            if decoded["profile"] not in (3, 6):
                self._fail_connection(
                    "normal runtime requires a `robot` firmware profile;"
                    f" device reported profile {decoded['profile']}",
                    now,
                )
                return
            self._hello = {key: value for key, value in decoded.items() if key != "kind"}
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
                    "info",
                    f"Verified HC-06 link on {self.device} at {self.baud} baud",
                )
        elif kind == "critical_status":
            state = decoded.get("state")
            sequence = decoded.get("last_accepted_control_sequence")
            if (
                not isinstance(state, int)
                or state < 0
                or state >= len(STATE_NAMES)
                or sequence not in (*range(64), 0xFF)
            ):
                self._record_event("error", "Rejected malformed critical status")
                return
            previous_faults = int(self._critical_status.get("faults", 0))
            faults = int(decoded.get("faults", 0))
            new_faults = faults & ~previous_faults
            if new_faults:
                names = ", ".join(self._bit_names(new_faults, FAULT_NAMES))
                self._record_event(
                    "error",
                    "Firmware fault flags appeared: "
                    f"{names} (new=0x{new_faults:04X}, active=0x{faults:04X})",
                )
            self._critical_status = {
                key: value for key, value in decoded.items() if key != "kind"
            }
            self._last_status_at = now
            if (
                decoded.get("link_alive", False)
                and state != STATE_ARMED
                and self._disarm_pending_confirmation
            ):
                self._disarm_pending_confirmation = False
                self._record_event(
                    "info",
                    "Firmware confirmed a non-ARMED state",
                )
            if not decoded.get("link_alive", False):
                self._request_urgent_disarm()
        elif kind == "ack":
            self._record_event(
                "info",
                f"Firmware accepted message {decoded['acknowledged_type']:#04x}",
            )
        elif kind == "nack":
            self._record_event(
                "error",
                "Firmware rejected message "
                f"{decoded['rejected_type']:#04x} "
                f"(reason {decoded['reason']})",
            )

    def _read_controller(self) -> ControlFrame:
        if not self.use_gamepad:
            self._controller_state = "disabled"
            return ControlFrame(sequence=self._fast_sequence)
        if not hasattr(self, "_pygame"):
            if self.clock() < self._controller_retry_at:
                raise RuntimeError("controller reconnect backoff")
            self._pygame, self._joystick = open_joystick(
                self._control_config.joystick_index
            )
        self._pygame.event.pump()
        if not self._joystick.get_init():
            raise RuntimeError("controller disconnected")
        frame = map_gamepad(
            self._joystick, self._control_config, self._fast_sequence
        )
        self._controller_state = self._joystick.get_name()
        arm_pressed = bool(
            self._joystick.get_button(self._control_config.arm_button)
        )
        disarm_pressed = bool(
            self._joystick.get_button(self._control_config.disarm_button)
        )
        if arm_pressed and not self._previous_arm_button:
            if (
                self._firmware_ready_to_arm(self.clock())
                and not self._disarm_pending_confirmation
                and self.submit("arm")
            ):
                self._record_event("info", "Controller Menu requested ARM")
            else:
                self._record_event(
                    "error",
                    "Controller ARM requires fresh firmware READY status",
                )
        if disarm_pressed and not self._previous_disarm_button:
            self._request_urgent_disarm()
            self._record_event("info", "Controller View requested DISARM")
        self._previous_arm_button = arm_pressed
        self._previous_disarm_button = disarm_pressed
        return frame

    def _current_control(self) -> ControlFrame:
        try:
            frame = self._read_controller()
        except Exception as exc:
            self._controller_state = f"unavailable: {exc}"
            self._request_urgent_disarm()
            self._controller_retry_at = self.clock() + 1.0
            for attribute in ("_pygame", "_joystick"):
                if hasattr(self, attribute):
                    delattr(self, attribute)
            frame = ControlFrame(sequence=self._fast_sequence)
        if self._disarm_pending_confirmation:
            frame = ControlFrame(sequence=self._fast_sequence)
        return replace(frame, sequence=self._next_fast_sequence())

    def _update_actual_rate(self, now: float) -> None:
        self._control_send_times.append(now)
        del self._control_send_times[:-31]
        if len(self._control_send_times) >= 2:
            elapsed = self._control_send_times[-1] - self._control_send_times[0]
            if elapsed > 0:
                self._host_stats["actual_control_rate_hz"] = round(
                    (len(self._control_send_times) - 1) / elapsed, 2
                )

    def _send_control(self, now: float) -> float | None:
        frame = self._current_control()
        # Controller loss/View can discover a DISARM only while reading this
        # frame. Serialize the final urgent check with the control write so an
        # already-known DISARM can never sit behind an 11-byte Control packet.
        with self._safety_action_lock:
            if self._urgent_disarm.is_set():
                self._drain_commands()
                return None
            self._latest_control = frame
            if frame.neutral():
                if self._neutral_since is None:
                    self._neutral_since = now
            else:
                self._neutral_since = None
            if not self._write(encode_control_frame(frame)):
                return None
        sent_at = self.clock()
        self._host_stats["control_frames_sent"] += 1
        if self._last_control_sent_at is not None:
            self._host_stats["last_control_interval_ms"] = round(
                (sent_at - self._last_control_sent_at) * 1000.0, 3
            )
        self._last_control_sent_at = sent_at
        self._update_actual_rate(sent_at)
        if (
            self._connection_failures
            and self._connected_at is not None
            and sent_at - self._connected_at >= self.reconnect_reset_seconds
        ):
            self._connection_failures = 0
        with self._safety_action_lock:
            if (
                self._pending_neutral_action is not None
                and self._neutral_since is not None
                and sent_at - self._neutral_since >= NEUTRAL_CONFIRM_SECONDS
            ):
                action = self._pending_neutral_action
                self._pending_neutral_action = None
                if (
                    self._urgent_disarm.is_set()
                    or self._disarm_pending_confirmation
                    or not self._pending_action_still_safe(action, sent_at)
                ):
                    self._record_event(
                        "error",
                        f"Cancelled {action.name}: safety preconditions changed",
                    )
                    return sent_at
                self._send_simple(action)
        return sent_at

    def _pending_action_still_safe(
        self, action: MessageType, now: float
    ) -> bool:
        if not self._status_fresh(now):
            return False
        state = self._critical_status.get("state")
        faults = int(self._critical_status.get("faults", 0))
        if not self._critical_status.get("link_alive", False):
            return False
        if action == MessageType.ARM:
            return (
                self.use_gamepad
                and self._build_allows_motion()
                and state == STATE_DISARMED
                and faults == 0
                and bool(self._critical_status.get("ready_to_arm", False))
                and not self._disarm_pending_confirmation
            )
        if action == MessageType.CLEAR_FAULT:
            warnings = int(self._critical_status.get("warnings", 0))
            return (
                not self._disarm_pending_confirmation
                and (
                    state == STATE_FAULT
                    or (
                        state == STATE_DISARMED
                        and faults != 0
                        and bool(warnings & FAULT_STATE_UNSAFE_WARNING)
                    )
                )
            )
        return False

    def _process_command(self, command: RuntimeCommand) -> None:
        action = command.action
        now = self.clock()
        status_fresh = self._status_fresh(now)
        state = self._critical_status.get("state") if status_fresh else None
        faults = int(self._critical_status.get("faults", 0)) if status_fresh else 0
        if action == "disarm":
            self._request_urgent_disarm()
        elif action == "arm":
            if not self.use_gamepad:
                self._record_event(
                    "error", "ARM is disabled when --no-gamepad is active"
                )
            elif not self._build_allows_motion():
                self._record_event(
                    "error",
                    "ARM is locked until every enabled motion function is "
                    "calibrated",
                )
            elif (
                not status_fresh
                or state != STATE_DISARMED
                or faults
                or not self._critical_status.get("ready_to_arm", False)
            ):
                self._record_event(
                    "error",
                    "ARM requires fresh firmware READY status",
                )
            elif self._disarm_pending_confirmation:
                self._record_event(
                    "error",
                    "ARM is inhibited until firmware confirms DISARM",
                )
            else:
                self._pending_neutral_action = MessageType.ARM
        elif action == "clear_fault":
            warnings = (
                int(self._critical_status.get("warnings", 0))
                if status_fresh
                else 0
            )
            unsafe_disarmed_fault = (
                state == STATE_DISARMED
                and faults != 0
                and bool(warnings & FAULT_STATE_UNSAFE_WARNING)
            )
            if (
                status_fresh
                and not self._disarm_pending_confirmation
                and (state == STATE_FAULT or unsafe_disarmed_fault)
            ):
                self._pending_neutral_action = MessageType.CLEAR_FAULT
            else:
                self._record_event(
                    "error",
                    "Clear fault requires fresh firmware FAULT state or "
                    "unsafe DISARMED status with active fault flags",
                )
        else:
            self._record_event("error", f"Unknown runtime action: {action}")

    def _drain_commands(self) -> None:
        if self._urgent_disarm.is_set():
            self._urgent_disarm.clear()
            self._disarm_pending_confirmation = True
            self._pending_neutral_action = None
            # Urgent DISARM invalidates every older UI/controller intent.
            with self._command_state_lock:
                self._clear_commands_unlocked()
            if self._serial is not None:
                self._send_urgent_disarm_burst()
                if self._serial is None:
                    return
        for _ in range(16):
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                break
            self._process_command(command)
            if self._serial is None:
                break

    def _best_effort_shutdown(self) -> None:
        with self._serial_write_lock:
            if self._serial is None:
                return
            try:
                for _ in range(3):
                    self._serial.write(
                        encode_urgent_disarm(
                            self._next_fast_sequence()
                        )
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
            if (
                self._serial is None
                and self._link_state != "fatal"
                and now >= self._retry_at
            ):
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
                self._start_handshake(now)
            if (
                self._link_state == "handshaking"
                and self._bootstrap_control_ready(now)
            ):
                self._send_bootstrap_control()

            handshake_control_deadline = self._bootstrap_control_at
            self._read_messages(now)
            if not was_connected and self._connected:
                # A HELLO reply can arrive between bootstrap controls. Keep
                # the first connected control on the same wire-safe 30 Hz
                # schedule instead of sending it immediately in this loop.
                control_deadline = max(
                    now, handshake_control_deadline or now
                )
            if (
                self._link_state == "handshaking"
                and self._handshake_deadline is not None
                and now >= self._handshake_deadline
            ):
                self._fail_connection(
                    "timed out waiting for HELLO_RESPONSE", now
                )

            if self._serial is not None:
                self._drain_commands()
            if (
                self._connected
                and self._connected_at is not None
                and now - self._connected_at > CRITICAL_STATUS_STALE_SECONDS
                and not self._status_fresh(now)
            ):
                # Handle heartbeat loss before any Control write in this
                # iteration. Urgent DISARM must be the next queued wire frame.
                self._request_urgent_disarm()
                self._drain_commands()
                if self._serial is not None:
                    self._fail_connection(
                        "critical status heartbeat became stale", now
                    )
            if (
                self._connected
                and now >= self._control_ready_at(control_deadline)
            ):
                sent_at = self._send_control(now)
                if sent_at is not None:
                    control_deadline, lateness, missed = _reschedule_control(
                        sent_at, control_deadline, control_period
                    )
                    self._host_stats["max_control_lateness_ms"] = max(
                        self._host_stats["max_control_lateness_ms"],
                        lateness * 1000.0,
                    )
                    self._host_stats["missed_control_deadlines"] += missed

            self._publish_snapshot()
            if self._link_state == "fatal":
                break
            wake_times = [now + 0.01]
            if self._connected:
                wake_times.append(
                    self._control_ready_at(control_deadline)
                )
            elif self._link_state == "backoff":
                wake_times.append(self._retry_at)
            elif (
                self._link_state == "stabilizing"
                and self._stabilize_until is not None
            ):
                wake_times.append(self._stabilize_until)
            elif (
                self._link_state == "handshaking"
                and self._handshake_deadline is not None
            ):
                wake_times.append(self._handshake_deadline)
                if self._bootstrap_control_at is not None:
                    wake_times.append(
                        max(
                            self._bootstrap_control_at,
                            self._host_wire_available_at,
                        )
                    )
            wait_until = min(wake_times)
            self._wake.wait(max(0.001, wait_until - self.clock()))
            self._wake.clear()

        self._best_effort_shutdown()
        with self._command_state_lock:
            self._connected = False
            self._clear_commands_unlocked()
        if self._link_state != "fatal":
            self._link_state = "stopped"
        self._publish_snapshot()
