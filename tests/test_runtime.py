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
    encode_urgent_disarm,
)
from robot_control.runtime import (
    CONTROL_RATE_HZ,
    WARNING_NAMES,
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
    state=0,
    faults=0,
    warnings=0,
    last_control=0xFF,
    link_alive=True,
    ready_to_arm=False,
):
    payload = bytes((state,))
    payload += struct.pack("<HH", faults, warnings)
    flags = int(link_alive) | (int(ready_to_arm) << 1)
    payload += bytes((last_control, flags))
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
                    + critical_status(message.sequence, state=0)
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


class StatusNameTests(unittest.TestCase):
    def test_unsafe_driver_warning_bits_have_stable_names(self):
        self.assertEqual(
            WARNING_NAMES,
            (
                "drive_unqualified",
                "arm_target_limited",
                "fault_state_unsafe",
                "encoder_timeout_ignored",
                "encoder_sign_ignored",
                "drive_mismatch_ignored",
            ),
        )


class RuntimeHandshakeTests(unittest.TestCase):
    def test_handshake_starts_with_urgent_disarm_then_neutral_control(self):
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
            [message.message_type for message in messages[:4]],
            [
                MessageType.URGENT_DISARM,
                MessageType.URGENT_DISARM,
                MessageType.URGENT_DISARM,
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
                minimum_drain = (
                    3 * len(encode_urgent_disarm(0))
                    + len(encode_message(MessageType.HELLO, 0))
                ) * 10.0 / baud
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

    def test_mid_period_disarm_restarts_wire_safe_control_spacing(self):
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
        self.assertTrue(runtime._send_urgent_disarm_burst())
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

    def test_handshake_mid_period_disarm_defers_next_bootstrap_control(self):
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
        self.assertTrue(runtime._send_urgent_disarm_burst())
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

    def test_handshake_timeout_is_fail_closed_and_mentions_reply(self):
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

    def test_stale_status_sends_urgent_disarm_before_disconnect(self):
        link = FakeSerial()
        runtime = RobotRuntime(
            "fake",
            use_gamepad=False,
            serial_factory=lambda *_: link,
            maximum_reconnect_attempts=1,
            startup_stabilization_seconds=0.0,
        )
        with patch(
            "robot_control.runtime.CRITICAL_STATUS_STALE_SECONDS", 0.05
        ):
            runtime.start()
            self.assertTrue(
                wait_for(lambda: runtime.snapshot()["connected"])
            )
            self.assertTrue(
                wait_for(
                    lambda: bool(runtime.snapshot()["fatal_error"]),
                    timeout=0.5,
                )
            )
            runtime.stop()
        types = [
            decode_message(packet).message_type for packet in link.writes
        ]
        self.assertGreaterEqual(
            types.count(MessageType.URGENT_DISARM), 6
        )
        self.assertIn("heartbeat became stale", runtime.snapshot()["fatal_error"])


class RuntimeSafetyTests(unittest.TestCase):
    class FakeEvent:
        @staticmethod
        def pump():
            pass

    class FakePygame:
        event = None

    class FakeJoystick:
        def __init__(self, pressed_button=None, *, connected=True, axes=None):
            self.buttons = [0] * 8
            if pressed_button is not None:
                self.buttons[pressed_button] = 1
            self.connected = connected
            self.axes = axes or {}

        def get_init(self):
            return self.connected

        @staticmethod
        def get_name():
            return "test controller"

        def get_axis(self, index):
            return self.axes.get(index, -1.0 if index in (4, 5) else 0.0)

        def get_numbuttons(self):
            return len(self.buttons)

        def get_button(self, index):
            return self.buttons[index]

        @staticmethod
        def get_numhats():
            return 1

        @staticmethod
        def get_hat(_):
            return (0, 0)

    def make_connected_runtime(self, *, ready_to_arm=True):
        link = FakeSerial(answer_hello=False)
        runtime = RobotRuntime(
            "fake", use_gamepad=True, serial_factory=lambda *_: link
        )
        runtime._serial = link
        runtime._connected = True
        runtime._link_verified = True
        runtime._link_state = "connected"
        runtime._hello = {
            "arm_enabled": True,
            "arm_calibrated": True,
            "drive_enabled": True,
            "drive_calibrated": True,
        }
        runtime._critical_status = {
            "state": 0,
            "faults": 0,
            "warnings": 0,
            "last_accepted_control_sequence": 0xFF,
            "link_alive": True,
            "ready_to_arm": ready_to_arm,
        }
        runtime._last_status_at = runtime.clock()
        runtime._disarm_pending_confirmation = False
        return runtime, link

    def attach_controller(self, runtime, pressed_button=None, **kwargs):
        self.FakePygame.event = self.FakeEvent()
        runtime._pygame = self.FakePygame()
        runtime._joystick = self.FakeJoystick(
            pressed_button, **kwargs
        )

    def test_menu_button_queues_one_arm_request_only_with_fresh_ready(self):
        runtime, _ = self.make_connected_runtime()
        self.attach_controller(runtime, runtime._control_config.arm_button)

        runtime._read_controller()
        self.assertEqual(runtime._commands.qsize(), 1)
        self.assertEqual(runtime._commands.get_nowait().action, "arm")
        self.assertFalse(runtime._urgent_disarm.is_set())
        self.assertEqual(
            runtime._events[-1]["message"], "Controller Menu requested ARM"
        )

        runtime._read_controller()
        self.assertTrue(runtime._commands.empty())

        not_ready, _ = self.make_connected_runtime(ready_to_arm=False)
        self.attach_controller(
            not_ready, not_ready._control_config.arm_button
        )
        not_ready._read_controller()
        self.assertTrue(not_ready._commands.empty())
        self.assertIn("requires fresh firmware READY", not_ready._events[-1]["message"])

    def test_view_button_requests_one_queue_bypassing_urgent_disarm(self):
        runtime, link = self.make_connected_runtime()
        self.assertTrue(runtime.submit("arm"))
        self.assertTrue(runtime.submit_webui("clear_fault"))
        self.attach_controller(runtime, runtime._control_config.disarm_button)

        runtime._read_controller()
        self.assertTrue(runtime._commands.empty())
        self.assertTrue(runtime._urgent_disarm.is_set())
        runtime._drain_commands()
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.URGENT_DISARM] * 3,
        )

        runtime._read_controller()
        runtime._drain_commands()
        self.assertEqual(len(link.writes), 3)
        self.assertEqual(
            sum(
                event["message"] == "Controller View requested DISARM"
                for event in runtime._events
            ),
            1,
        )

    def test_queue_full_cannot_block_urgent_disarm(self):
        runtime, link = self.make_connected_runtime()
        for _ in range(32):
            self.assertTrue(runtime.submit("arm"))
        self.assertFalse(runtime.submit("arm"))
        self.assertTrue(runtime.submit("disarm"))
        self.assertTrue(runtime._commands.empty())
        runtime._drain_commands()
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.URGENT_DISARM] * 3,
        )

    def test_urgent_disarm_discards_older_arm_and_clear_fault_intent(self):
        runtime, link = self.make_connected_runtime()
        runtime._pending_neutral_action = MessageType.ARM
        self.assertTrue(runtime.submit("arm"))
        self.assertTrue(runtime.submit_webui("clear_fault"))
        self.assertTrue(runtime.submit("disarm"))
        self.assertTrue(runtime._commands.empty())
        self.assertIsNone(runtime._pending_neutral_action)
        runtime._drain_commands()
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.URGENT_DISARM] * 3,
        )

    def test_motion_stays_suppressed_until_non_armed_status_confirmation(self):
        runtime, link = self.make_connected_runtime()
        self.attach_controller(
            runtime,
            axes={runtime._control_config.drive_forward.axis: -1.0},
        )
        runtime.submit("disarm")
        runtime._drain_commands()
        self.assertTrue(runtime._current_control().neutral())
        runtime._handle_message(
            decode_message(critical_status(state=1, last_control=0)),
            runtime.clock(),
        )
        self.assertTrue(runtime._disarm_pending_confirmation)
        self.assertTrue(runtime._current_control().neutral())
        runtime._handle_message(
            decode_message(critical_status(state=2, last_control=0)),
            runtime.clock(),
        )
        self.assertFalse(runtime._disarm_pending_confirmation)
        self.assertFalse(runtime._current_control().neutral())
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.URGENT_DISARM] * 3,
        )

    def test_controller_disconnect_sends_disarm_before_any_control(self):
        runtime, link = self.make_connected_runtime()
        self.attach_controller(runtime, connected=False)
        self.assertIsNone(runtime._send_control(runtime.clock()))
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.URGENT_DISARM] * 3,
        )
        self.assertFalse(runtime._urgent_disarm.is_set())
        self.assertTrue(runtime._disarm_pending_confirmation)

    def test_view_disarm_sends_before_any_control(self):
        runtime, link = self.make_connected_runtime()
        self.attach_controller(runtime, runtime._control_config.disarm_button)
        self.assertIsNone(runtime._send_control(runtime.clock()))
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.URGENT_DISARM] * 3,
        )

    def test_no_gamepad_runtime_refuses_arm(self):
        runtime = RobotRuntime(
            "fake", use_gamepad=False, serial_factory=lambda *_: FakeSerial()
        )
        runtime._connected = True
        runtime._link_verified = True
        runtime._critical_status = {
            "state": 0,
            "faults": 0,
            "link_alive": True,
            "ready_to_arm": True,
        }
        runtime._last_status_at = runtime.clock()
        runtime._disarm_pending_confirmation = False
        runtime._process_command(RuntimeCommand(5, 0, "arm"))
        self.assertIsNone(runtime._pending_neutral_action)
        self.assertIn("disabled", runtime._events[-1]["message"])

    def test_status_freshness_controls_visible_state(self):
        runtime, _ = self.make_connected_runtime()
        runtime._publish_snapshot()
        fresh = runtime.snapshot()
        self.assertTrue(fresh["status_fresh"])
        self.assertEqual(fresh["state_name"], "DISARMED")
        with patch(
            "robot_control.runtime.CRITICAL_STATUS_STALE_SECONDS", 0.0
        ):
            time.sleep(0.001)
            runtime._publish_snapshot()
        stale = runtime.snapshot()
        self.assertFalse(stale["status_fresh"])
        self.assertEqual(stale["state_name"], "UNKNOWN")

    def test_fault_flag_rising_edge_records_one_named_timed_error(self):
        runtime, _ = self.make_connected_runtime()
        packet = decode_message(
            critical_status(state=2, faults=0x000C, last_control=0)
        )
        with patch("robot_control.runtime.time.time", return_value=1234.5):
            runtime._handle_message(packet, runtime.clock())
            runtime._handle_message(packet, runtime.clock())

        fault_events = [
            event
            for event in runtime._events
            if "Firmware fault flags appeared" in event["message"]
        ]
        self.assertEqual(len(fault_events), 1)
        self.assertEqual(fault_events[0]["time"], 1234.5)
        self.assertIn("encoder_stale, encoder_malformed", fault_events[0]["message"])
        self.assertIn("new=0x000C, active=0x000C", fault_events[0]["message"])

        runtime._handle_message(
            decode_message(critical_status(state=0, faults=0, last_control=0))
        )
        runtime._handle_message(packet)
        self.assertEqual(
            sum(
                "Firmware fault flags appeared" in event["message"]
                for event in runtime._events
            ),
            2,
        )

    def test_unsafe_disarmed_fault_can_queue_clear_fault(self):
        runtime, _ = self.make_connected_runtime()
        runtime._critical_status.update(
            {
                "state": 0,
                "faults": 0x0040,
                "warnings": 0x0004,
                "ready_to_arm": False,
            }
        )
        self.assertFalse(runtime.submit("clear_fault"))
        self.assertTrue(runtime.submit_webui("clear_fault"))
        runtime._drain_commands()
        self.assertEqual(
            runtime._pending_neutral_action, MessageType.CLEAR_FAULT
        )
        self.assertTrue(
            runtime._pending_action_still_safe(
                MessageType.CLEAR_FAULT, runtime.clock()
            )
        )
        runtime._publish_snapshot()
        self.assertTrue(runtime.snapshot()["fault_state_unsafe"])

    def test_link_alive_false_keeps_arm_unavailable(self):
        runtime, _ = self.make_connected_runtime()
        runtime._hello = {
            "arm_enabled": True,
            "arm_calibrated": True,
            "drive_enabled": False,
            "drive_calibrated": False,
        }
        runtime._critical_status.update(
            {"state": 0, "link_alive": False, "ready_to_arm": True}
        )
        runtime._publish_snapshot()
        self.assertFalse(runtime.snapshot()["arm_available"])
        self.assertFalse(
            runtime._pending_action_still_safe(
                MessageType.ARM, runtime.clock()
            )
        )
        runtime._handle_message(
            decode_message(
                critical_status(
                    state=0, link_alive=False, ready_to_arm=True
                )
            ),
            runtime.clock(),
        )
        self.assertTrue(runtime._urgent_disarm.is_set())

    def test_webui_only_clear_fault_sends_after_neutral_qualification(self):
        runtime, link = self.make_connected_runtime()
        runtime._critical_status.update(
            {"state": 2, "faults": 0x0004, "ready_to_arm": False}
        )
        self.assertFalse(runtime.submit("clear_fault"))
        self.assertTrue(runtime.submit_webui("clear_fault"))
        runtime._drain_commands()
        self.assertEqual(
            runtime._pending_neutral_action, MessageType.CLEAR_FAULT
        )
        runtime._neutral_since = runtime.clock() - 1.0
        with patch.object(
            runtime,
            "_read_controller",
            return_value=ControlFrame(sequence=0),
        ):
            runtime._send_control(runtime.clock())
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.CONTROL, MessageType.CLEAR_FAULT],
        )

    def test_shutdown_sends_only_urgent_disarm_burst(self):
        runtime, link = self.make_connected_runtime()
        runtime._best_effort_shutdown()
        self.assertTrue(link.closed)
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes],
            [MessageType.URGENT_DISARM] * 3,
        )

    def test_removed_remote_operations_are_rejected(self):
        runtime, _ = self.make_connected_runtime()
        for action in (
            "clear_fault",
            "refresh_parameters",
            "set_host_input",
            "set_parameter",
        ):
            self.assertFalse(runtime.submit(action))
        self.assertTrue(runtime._commands.empty())

    def test_one_way_mode_is_no_longer_an_initialization_option(self):
        with self.assertRaises(TypeError):
            RobotRuntime("fake", one_way=True)  # type: ignore[call-arg]


if __name__ == "__main__":
    unittest.main()
