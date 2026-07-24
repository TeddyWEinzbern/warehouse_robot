import struct
import threading
import time
import unittest
from unittest.mock import patch

from robot_control.protocol import (
    ControlFrame,
    MessageType,
    decode_control_frame,
    decode_message,
    encode_control_frame,
    encode_message,
)
from robot_control.runtime import (
    CONTROL_RATE_HZ,
    RobotRuntime,
    RuntimeCommand,
    _reschedule_control,
)


def hello_response(sequence=1, baud=9600, profile=3):
    payload = bytes((profile, 1, 1, 0, 2, 1, 0, baud // 1200))
    return encode_message(MessageType.HELLO_RESPONSE, sequence, payload)


def critical_status(
    sequence=2,
    *,
    state=3,
    faults=0,
    warnings=0,
    last_control=0xFF,
    link_alive=True,
):
    payload = bytes((state,))
    payload += struct.pack("<HH", faults, warnings)
    payload += bytes((last_control, int(link_alive)))
    return encode_message(MessageType.CRITICAL_STATUS, sequence, payload)


class FakeSerial:
    def __init__(
        self,
        *,
        answer_hello=True,
        baud=9600,
        profile=3,
        fail_write_number=None,
    ):
        self.writes = []
        self.write_times = []
        self.closed = False
        self.answer_hello = answer_hello
        self.baud = baud
        self.profile = profile
        self.fail_write_number = fail_write_number
        self._read_buffer = bytearray()
        self._lock = threading.Lock()

    @property
    def in_waiting(self):
        with self._lock:
            return len(self._read_buffer)

    def read(self, length):
        with self._lock:
            result = bytes(self._read_buffer[:length])
            del self._read_buffer[:length]
            return result

    def inject(self, packet):
        with self._lock:
            self._read_buffer.extend(packet)

    def write(self, data):
        with self._lock:
            if self.closed:
                raise OSError("closed")
            if len(self.writes) + 1 == self.fail_write_number:
                raise OSError(f"forced failure on write {self.fail_write_number}")
            self.writes.append(bytes(data))
            self.write_times.append(time.monotonic())
            message = decode_message(data)
            if self.answer_hello and message.message_type == MessageType.HELLO:
                self._read_buffer.extend(
                    hello_response(
                        message.sequence, self.baud, self.profile
                    )
                    + critical_status(message.sequence, state=3)
                )
                self.answer_hello = False
        return len(data)

    def close(self):
        self.closed = True


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


def wait_for(predicate, timeout=0.5):
    deadline = time.monotonic() + timeout
    while not predicate() and time.monotonic() < deadline:
        time.sleep(0.005)
    return predicate()


class RuntimeHandshakeTests(unittest.TestCase):
    def test_handshake_starts_with_redundant_estop_then_neutral_control(self):
        link = FakeSerial()
        runtime = RobotRuntime(
            "fake",
            9600,
            use_gamepad=False,
            serial_factory=lambda *_: link,
            startup_stabilization_seconds=0.0,
        )
        runtime.start()
        self.assertTrue(wait_for(lambda: runtime.snapshot()["connected"]))
        time.sleep(0.12)
        runtime.stop()

        messages = [decode_message(packet) for packet in link.writes]
        self.assertEqual(
            [message.message_type for message in messages[:5]],
            [
                MessageType.ESTOP_ASSERT,
                MessageType.ESTOP_ASSERT,
                MessageType.ESTOP_ASSERT,
                MessageType.DISARM,
                MessageType.HELLO,
            ],
        )
        controls = [
            decode_control_frame(packet)
            for packet, message in zip(link.writes, messages)
            if message.message_type == MessageType.CONTROL
        ]
        self.assertGreaterEqual(len(controls), 3)
        for frame in controls:
            self.assertEqual(
                (
                    frame.forward,
                    frame.turn,
                    frame.strafe,
                    frame.arm_yaw,
                    frame.arm_reach,
                    frame.arm_height,
                    frame.gripper,
                    frame.buttons,
                ),
                (0, 0, 0, 0, 0, 0, 0, 0),
            )

    def test_both_baud_rates_use_same_thirty_hertz_schedule(self):
        for baud in (9600, 38400):
            with self.subTest(baud=baud):
                link = FakeSerial(baud=baud)
                runtime = RobotRuntime(
                    "fake",
                    baud,
                    use_gamepad=False,
                    serial_factory=lambda *_args, link=link: link,
                    startup_stabilization_seconds=0.0,
                )
                self.assertEqual(runtime.control_rate_hz, CONTROL_RATE_HZ)
                runtime.start()
                self.assertTrue(wait_for(lambda: runtime.snapshot()["connected"]))
                time.sleep(0.24)
                runtime.stop()
                control_times = [
                    timestamp
                    for packet, timestamp in zip(link.writes, link.write_times)
                    if decode_message(packet).message_type == MessageType.CONTROL
                ]
                intervals = [
                    later - earlier
                    for earlier, later in zip(
                        control_times, control_times[1:]
                    )
                ]
                self.assertGreaterEqual(len(intervals), 4)
                self.assertTrue(all(0.028 <= interval <= 0.06 for interval in intervals))

    def test_handshake_prelude_drains_before_first_control(self):
        for baud in (9600, 38400):
            with self.subTest(baud=baud):
                link = FakeSerial(baud=baud)
                runtime = RobotRuntime(
                    "fake",
                    baud,
                    use_gamepad=False,
                    serial_factory=lambda *_args, link=link: link,
                    startup_stabilization_seconds=0.0,
                )
                runtime.start()
                self.assertTrue(
                    wait_for(lambda: runtime.snapshot()["connected"])
                )
                self.assertTrue(
                    wait_for(
                        lambda: any(
                            decode_message(packet).message_type
                            == MessageType.CONTROL
                            for packet in link.writes
                        )
                    )
                )
                runtime.stop()
                messages = [
                    decode_message(packet) for packet in link.writes
                ]
                hello_index = next(
                    index
                    for index, message in enumerate(messages)
                    if message.message_type == MessageType.HELLO
                )
                control_index = next(
                    index
                    for index, message in enumerate(messages)
                    if message.message_type == MessageType.CONTROL
                )
                minimum_drain = 26 * 10.0 / baud
                self.assertGreaterEqual(
                    link.write_times[control_index]
                    - link.write_times[hello_index],
                    minimum_drain - 0.002,
                )

    def test_late_control_send_never_catches_up_inside_one_period(self):
        period = 1.0 / CONTROL_RATE_HZ
        next_deadline, lateness, missed = _reschedule_control(
            0.025, 0.0, period
        )
        self.assertAlmostEqual(lateness, 0.025)
        self.assertEqual(missed, 0)
        self.assertAlmostEqual(next_deadline, 0.025 + period)
        self.assertGreaterEqual(next_deadline - 0.025, period)

    def test_mid_period_estop_restarts_wire_safe_control_spacing(self):
        timeline = ManualClock()
        link = FakeSerial(answer_hello=False)
        runtime = RobotRuntime(
            "fake",
            9600,
            use_gamepad=False,
            serial_factory=lambda *_: link,
            clock=timeline,
        )
        runtime._serial = link
        runtime._connected = True
        runtime._link_state = "connected"
        runtime._host_wire_available_at = timeline()

        control = encode_control_frame(
            ControlFrame(sequence=runtime._next_fast_sequence())
        )
        first_start = runtime._control_ready_at(timeline())
        self.assertTrue(runtime._write(control))

        timeline.advance(0.030)
        self.assertTrue(runtime._send_estop_burst())
        nominal_deadline = first_start + 1.0 / CONTROL_RATE_HZ
        delayed_start = runtime._control_ready_at(nominal_deadline)
        self.assertGreater(delayed_start, nominal_deadline)

        timeline.advance(delayed_start - timeline())
        self.assertTrue(runtime._write(control))
        next_deadline, _, _ = _reschedule_control(
            delayed_start, nominal_deadline, 1.0 / CONTROL_RATE_HZ
        )
        next_start = runtime._control_ready_at(next_deadline)
        self.assertAlmostEqual(
            next_start - delayed_start,
            1.0 / CONTROL_RATE_HZ,
        )
        control_wire_time = 11 * 10.0 / 9600
        self.assertGreaterEqual(
            next_start - (delayed_start + control_wire_time),
            0.0218,
        )

    def test_handshake_mid_period_estop_defers_next_bootstrap_control(self):
        timeline = ManualClock()
        link = FakeSerial(answer_hello=False)
        runtime = RobotRuntime(
            "fake",
            9600,
            use_gamepad=False,
            serial_factory=lambda *_: link,
            clock=timeline,
        )
        runtime._serial = link
        runtime._link_state = "handshaking"
        runtime._bootstrap_control_at = timeline()

        self.assertTrue(runtime._bootstrap_control_ready(timeline()))
        runtime._send_bootstrap_control()
        nominal_deadline = runtime._bootstrap_control_at
        self.assertIsNotNone(nominal_deadline)

        timeline.advance(0.030)
        self.assertTrue(runtime._send_estop_burst())
        timeline.advance(nominal_deadline - timeline())
        self.assertFalse(runtime._bootstrap_control_ready(timeline()))

        timeline.advance(runtime._host_wire_available_at - timeline())
        delayed_start = timeline()
        self.assertTrue(runtime._bootstrap_control_ready(delayed_start))
        runtime._send_bootstrap_control()
        self.assertAlmostEqual(
            runtime._bootstrap_control_at - delayed_start,
            1.0 / CONTROL_RATE_HZ,
        )

    def test_blocked_write_anchors_next_control_after_completion(self):
        timeline = ManualClock()

        class DelayedSerial(FakeSerial):
            def write(self, data):
                timeline.advance(0.025)
                return super().write(data)

        link = DelayedSerial(answer_hello=False)
        runtime = RobotRuntime(
            "fake",
            9600,
            use_gamepad=False,
            serial_factory=lambda *_: link,
            clock=timeline,
        )
        runtime._serial = link
        runtime._link_state = "handshaking"
        runtime._bootstrap_control_at = timeline()

        runtime._send_bootstrap_control()
        self.assertAlmostEqual(timeline(), 0.025)
        self.assertAlmostEqual(
            runtime._bootstrap_control_at,
            0.025 + 1.0 / CONTROL_RATE_HZ,
        )

    def test_handshake_timeout_is_fail_closed_and_mentions_v3_reply(self):
        links = []

        def factory(*_args):
            link = FakeSerial(answer_hello=False)
            links.append(link)
            return link

        runtime = RobotRuntime(
            "fake",
            use_gamepad=False,
            serial_factory=factory,
            maximum_reconnect_attempts=2,
            startup_stabilization_seconds=0.0,
            handshake_timeout_seconds=0.02,
            reconnect_initial_delay_seconds=0.01,
        )
        runtime.start()
        self.assertTrue(wait_for(lambda: bool(runtime.snapshot()["fatal_error"])))
        runtime.stop()
        snapshot = runtime.snapshot()
        self.assertEqual(len(links), 2)
        self.assertIn("HELLO_RESPONSE", snapshot["fatal_error"])
        self.assertFalse(snapshot["link_verified"])

    def test_reported_baud_mismatch_rejects_handshake(self):
        link = FakeSerial(baud=38400)
        runtime = RobotRuntime(
            "fake",
            9600,
            use_gamepad=False,
            serial_factory=lambda *_: link,
            maximum_reconnect_attempts=1,
            startup_stabilization_seconds=0.0,
        )
        runtime.start()
        self.assertTrue(wait_for(lambda: bool(runtime.snapshot()["fatal_error"])))
        runtime.stop()
        self.assertIn("38400 baud", runtime.snapshot()["fatal_error"])

    def test_calibration_profile_is_rejected_by_normal_runtime(self):
        link = FakeSerial(profile=7)
        runtime = RobotRuntime(
            "fake",
            use_gamepad=False,
            serial_factory=lambda *_: link,
            maximum_reconnect_attempts=1,
            startup_stabilization_seconds=0.0,
        )
        runtime.start()
        self.assertTrue(
            wait_for(lambda: bool(runtime.snapshot()["fatal_error"]))
        )
        runtime.stop()
        self.assertIn(
            "requires a `robot` firmware profile",
            runtime.snapshot()["fatal_error"],
        )


class RuntimeSafetyTests(unittest.TestCase):
    def make_connected_runtime(self):
        link = FakeSerial(answer_hello=False)
        runtime = RobotRuntime(
            "fake", use_gamepad=True, serial_factory=lambda *_: link
        )
        runtime._serial = link
        runtime._connected = True
        runtime._link_verified = True
        runtime._link_state = "connected"
        runtime._critical_status = {
            "state": 3,
            "faults": 0,
            "warnings": 0,
            "last_accepted_control_sequence": 0xFF,
            "link_alive": True,
        }
        runtime._last_status_at = runtime.clock()
        return runtime, link

    def test_estop_priority_path_sends_three_dedicated_fast_frames(self):
        runtime, link = self.make_connected_runtime()
        for _ in range(32):
            self.assertTrue(runtime.submit("disarm"))
        self.assertTrue(runtime.submit("estop"))
        runtime._drain_commands()
        types = [decode_message(packet).message_type for packet in link.writes]
        self.assertEqual(types[:3], [MessageType.ESTOP_ASSERT] * 3)
        self.assertTrue(runtime.snapshot()["host_estop_latched"])

    def test_estop_discards_older_queued_clear_intent(self):
        runtime, link = self.make_connected_runtime()
        self.assertTrue(runtime.submit("clear_estop"))
        self.assertTrue(runtime.submit("estop"))
        runtime._drain_commands()
        self.assertTrue(runtime._commands.empty())
        self.assertIsNone(runtime._pending_neutral_action)
        runtime._neutral_since = runtime.clock() - 1.0
        runtime._send_control(runtime.clock())
        message_types = [
            decode_message(packet).message_type for packet in link.writes
        ]
        self.assertEqual(
            message_types[:3], [MessageType.ESTOP_ASSERT] * 3
        )
        self.assertNotIn(MessageType.CLEAR_ESTOP, message_types)

    def test_estop_retries_until_firmware_reports_estop(self):
        runtime, link = self.make_connected_runtime()
        runtime._critical_status["state"] = 1
        runtime._estop_retry_at = 0.0
        runtime._retry_estop_if_needed(runtime.clock())
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.ESTOP_ASSERT] * 3,
        )
        runtime._critical_status["state"] = 3
        runtime._estop_retry_at = 0.0
        runtime._retry_estop_if_needed(runtime.clock())
        self.assertEqual(len(link.writes), 3)

    def test_disarm_follows_a_neutral_control_frame(self):
        runtime, link = self.make_connected_runtime()
        runtime._process_command(RuntimeCommand(1, 0, "disarm"))
        self.assertEqual(link.writes, [])
        runtime._send_control(runtime.clock())
        messages = [decode_message(packet) for packet in link.writes]
        self.assertEqual(
            [message.message_type for message in messages],
            [MessageType.CONTROL, MessageType.DISARM],
        )
        self.assertTrue(decode_control_frame(link.writes[0]).neutral())

    def test_clear_estop_stays_latched_until_fresh_disarmed_status(self):
        runtime, link = self.make_connected_runtime()
        runtime._process_command(RuntimeCommand(5, 0, "clear_estop"))
        self.assertTrue(runtime._host_estop_latched)
        self.assertEqual(
            runtime._pending_neutral_action, MessageType.CLEAR_ESTOP
        )

        runtime._neutral_since = runtime.clock() - 1.0
        runtime._send_control(runtime.clock())
        self.assertTrue(runtime._awaiting_clear_estop)
        self.assertTrue(runtime._host_estop_latched)
        self.assertIn(
            MessageType.CLEAR_ESTOP,
            [decode_message(packet).message_type for packet in link.writes],
        )

        runtime._handle_message(
            decode_message(critical_status(state=1, last_control=0)),
            runtime.clock(),
        )
        self.assertFalse(runtime._awaiting_clear_estop)
        self.assertFalse(runtime._host_estop_latched)

    def test_clear_estop_nack_remains_latched_and_retryable(self):
        runtime, _ = self.make_connected_runtime()
        runtime._awaiting_clear_estop = True
        runtime._handle_message(
            decode_message(
                encode_message(
                    MessageType.NACK,
                    4,
                    bytes((MessageType.CLEAR_ESTOP, 4)),
                )
            )
        )
        self.assertTrue(runtime._host_estop_latched)
        self.assertFalse(runtime._awaiting_clear_estop)

    def test_no_gamepad_runtime_refuses_arm(self):
        runtime = RobotRuntime(
            "fake", use_gamepad=False, serial_factory=lambda *_: FakeSerial()
        )
        runtime._connected = True
        runtime._link_verified = True
        runtime._critical_status = {"state": 1, "faults": 0}
        runtime._last_status_at = runtime.clock()
        runtime._host_estop_latched = False
        runtime._process_command(RuntimeCommand(5, 0, "arm"))
        self.assertIsNone(runtime._pending_neutral_action)
        self.assertIn("disabled", runtime._events[-1]["message"])

    def test_status_freshness_controls_visible_state(self):
        runtime, _ = self.make_connected_runtime()
        runtime._publish_snapshot()
        fresh = runtime.snapshot()
        self.assertTrue(fresh["status_fresh"])
        self.assertEqual(fresh["state_name"], "ESTOP")
        with patch(
            "robot_control.runtime.CRITICAL_STATUS_STALE_SECONDS", 0.0
        ):
            time.sleep(0.001)
            runtime._publish_snapshot()
        stale = runtime.snapshot()
        self.assertFalse(stale["status_fresh"])
        self.assertEqual(stale["state_name"], "UNKNOWN")

    def test_link_alive_false_keeps_arm_unavailable(self):
        runtime, _ = self.make_connected_runtime()
        runtime._hello = {
            "arm_enabled": True,
            "arm_calibrated": True,
            "drive_enabled": False,
            "drive_calibrated": False,
        }
        runtime._critical_status.update({"state": 1, "link_alive": False})
        runtime._host_estop_latched = False
        runtime._publish_snapshot()
        self.assertFalse(runtime.snapshot()["arm_available"])
        self.assertFalse(
            runtime._pending_action_still_safe(
                MessageType.ARM, runtime.clock()
            )
        )

    def test_shutdown_sends_estop_burst_and_disarm(self):
        runtime, link = self.make_connected_runtime()
        runtime._best_effort_shutdown()
        self.assertTrue(link.closed)
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.ESTOP_ASSERT] * 3 + [MessageType.DISARM],
        )

    def test_removed_remote_operations_are_rejected(self):
        runtime, _ = self.make_connected_runtime()
        for action in ("refresh_parameters", "set_host_input", "set_parameter"):
            self.assertFalse(runtime.submit(action))
        self.assertTrue(runtime._commands.empty())

    def test_one_way_mode_is_no_longer_an_initialization_option(self):
        with self.assertRaises(TypeError):
            RobotRuntime("fake", one_way=True)  # type: ignore[call-arg]


if __name__ == "__main__":
    unittest.main()
