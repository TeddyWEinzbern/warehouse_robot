import threading
import time
import unittest

from robot_control.protocol import (
    ControlFlag,
    MessageType,
    decode_control_frame,
    decode_message,
    encode_message,
)
from robot_control.runtime import RobotRuntime, RuntimeCommand


class FakeSerial:
    def __init__(
        self,
        *,
        answer_hello=True,
        fail_write_number=None,
        hello_after_bootstrap_controls=1,
        hello_baud_code=32,
    ):
        self.writes = []
        self.write_times = []
        self.failed_writes = []
        self.closed = False
        self.answer_hello = answer_hello
        self.fail_write_number = fail_write_number
        self.hello_after_bootstrap_controls = hello_after_bootstrap_controls
        self.hello_baud_code = hello_baud_code
        self._hello_requested = False
        self._hello_answered = False
        self._safe_bootstrap_controls = 0
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

    def write(self, data):
        with self._lock:
            if self.closed:
                raise OSError("closed")
            if len(self.writes) + 1 == self.fail_write_number:
                self.failed_writes.append(bytes(data))
                raise OSError(f"forced failure on write {self.fail_write_number}")
            self.writes.append(bytes(data))
            self.write_times.append(time.monotonic())
            message = decode_message(data)
            if message.message_type == MessageType.HELLO:
                self._hello_requested = True
            if (
                self.answer_hello
                and self._hello_requested
                and not self._hello_answered
                and message.message_type == MessageType.CONTROL
            ):
                frame = decode_control_frame(data)
                safe_bootstrap = (
                    frame.forward == 0
                    and frame.turn == 0
                    and frame.strafe == 0
                    and frame.arm_yaw == 0
                    and frame.arm_reach == 0
                    and frame.arm_height == 0
                    and frame.gripper == 0
                    and frame.buttons == 0
                    and frame.control_flags & ControlFlag.ESTOP_ASSERTED
                )
                if not safe_bootstrap:
                    return len(data)
                self._safe_bootstrap_controls += 1
                if (
                    self._safe_bootstrap_controls
                    < self.hello_after_bootstrap_controls
                ):
                    return len(data)
                self._hello_answered = True
                self._read_buffer.extend(
                    encode_message(
                        MessageType.HELLO_TELEMETRY,
                        message.sequence,
                        bytes((1, 1, 255, 0, 0, 0, self.hello_baud_code, 0)),
                    )
                )
        return len(data)

    def close(self):
        self.closed = True


class RuntimeTests(unittest.TestCase):
    def test_runtime_handshake_and_control_stream_start_safe(self):
        link = FakeSerial()
        runtime = RobotRuntime(
            "fake",
            38400,
            use_gamepad=False,
            serial_factory=lambda _device, _baud: link,
            startup_stabilization_seconds=0.0,
        )
        runtime.start()
        time.sleep(0.14)
        self.assertTrue(runtime.snapshot()["connected"])
        runtime.stop()
        messages = []
        controls = []
        for packet in link.writes:
            try:
                message = decode_message(packet)
                messages.append(message.message_type)
                if message.message_type == MessageType.CONTROL:
                    controls.append(decode_control_frame(packet))
            except ValueError:
                continue
        self.assertEqual(messages[:3], [
            MessageType.DISARM,
            MessageType.HELLO,
            MessageType.PARAMETER_SNAPSHOT_REQUEST,
        ])
        self.assertGreaterEqual(len(controls), 2)
        self.assertTrue(all(frame.control_flags & ControlFlag.ESTOP_ASSERTED for frame in controls))

    def test_startup_stabilization_delays_handshake_and_hello_gates_control(self):
        link = FakeSerial(answer_hello=False)
        runtime = RobotRuntime(
            "fake",
            use_gamepad=False,
            serial_factory=lambda *_: link,
            startup_stabilization_seconds=0.08,
            handshake_timeout_seconds=0.2,
        )
        runtime.start()
        time.sleep(0.03)
        self.assertEqual(link.writes, [])
        self.assertEqual(runtime.snapshot()["link_state"], "stabilizing")
        time.sleep(0.08)
        snapshot = runtime.snapshot()
        self.assertFalse(snapshot["connected"])
        self.assertEqual(snapshot["link_state"], "handshaking")
        message_types = [decode_message(packet).message_type for packet in link.writes]
        self.assertEqual(
            message_types[:3],
            [
                MessageType.DISARM,
                MessageType.HELLO,
                MessageType.PARAMETER_SNAPSHOT_REQUEST,
            ],
        )
        bootstrap_controls = [
            decode_control_frame(packet)
            for packet in link.writes
            if decode_message(packet).message_type == MessageType.CONTROL
        ]
        self.assertTrue(bootstrap_controls)
        self.assertTrue(
            all(
                frame.forward == frame.turn == frame.strafe == 0
                and frame.arm_yaw == frame.arm_reach == frame.arm_height == 0
                and frame.gripper == 0
                and frame.buttons == 0
                and frame.control_flags & ControlFlag.ESTOP_ASSERTED
                for frame in bootstrap_controls
            )
        )
        runtime.stop()

    def test_handshake_timeout_uses_backoff_and_failure_limit(self):
        links = []
        opened_at = []

        def factory(*_args):
            opened_at.append(time.monotonic())
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
            reconnect_initial_delay_seconds=0.02,
        )
        runtime.start()
        deadline = time.monotonic() + 0.5
        while not runtime.snapshot()["fatal_error"] and time.monotonic() < deadline:
            time.sleep(0.005)
        runtime.stop()
        snapshot = runtime.snapshot()
        self.assertEqual(len(links), 2)
        self.assertTrue(all(link.closed for link in links))
        self.assertIn("HELLO_TELEMETRY", snapshot["fatal_error"])
        self.assertEqual(snapshot["connection"]["consecutive_failures"], 2)
        self.assertGreaterEqual(opened_at[1] - opened_at[0], 0.035)

    def test_9600_bootstrap_keeps_tx_window_open_until_delayed_hello(self):
        link = FakeSerial(
            hello_after_bootstrap_controls=5,
            hello_baud_code=8,
        )
        runtime = RobotRuntime(
            "fake",
            9600,
            use_gamepad=False,
            serial_factory=lambda *_: link,
            startup_stabilization_seconds=0.0,
            handshake_timeout_seconds=0.5,
        )
        self.assertEqual(runtime.control_rate_hz, 10.0)
        runtime.start()
        deadline = time.monotonic() + 0.45
        while not runtime.snapshot()["connected"] and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(runtime.snapshot()["connected"])
        runtime.stop()

        bootstrap_indices = [
            index
            for index, packet in enumerate(link.writes)
            if decode_message(packet).message_type == MessageType.CONTROL
        ][:5]
        self.assertEqual(len(bootstrap_indices), 5)
        bootstrap_times = [link.write_times[index] for index in bootstrap_indices]
        intervals = [
            later - earlier
            for earlier, later in zip(bootstrap_times, bootstrap_times[1:])
        ]
        self.assertTrue(all(0.04 <= interval <= 0.07 for interval in intervals))
        self.assertEqual(runtime.snapshot()["host_stats"]["bootstrap_frames_sent"], 5)

    def test_fourth_bootstrap_write_failure_cannot_reconnect_forever(self):
        links = []
        opened_at = []

        def factory(*_args):
            opened_at.append(time.monotonic())
            link = FakeSerial(answer_hello=True, fail_write_number=4)
            links.append(link)
            return link

        runtime = RobotRuntime(
            "fake",
            use_gamepad=False,
            serial_factory=factory,
            maximum_reconnect_attempts=3,
            startup_stabilization_seconds=0.0,
            handshake_timeout_seconds=0.1,
            reconnect_initial_delay_seconds=0.02,
            reconnect_reset_seconds=0.05,
        )
        runtime.start()
        deadline = time.monotonic() + 0.7
        while not runtime.snapshot()["fatal_error"] and time.monotonic() < deadline:
            time.sleep(0.005)
        runtime.stop()
        snapshot = runtime.snapshot()
        self.assertEqual(len(links), 3)
        self.assertTrue(all(link.closed for link in links))
        self.assertEqual(snapshot["connection"]["consecutive_failures"], 3)
        self.assertEqual(snapshot["host_stats"]["write_failures"], 3)
        self.assertIn("forced failure on write 4", snapshot["fatal_error"])
        self.assertTrue(
            all(
                decode_message(link.failed_writes[0]).message_type
                == MessageType.CONTROL
                for link in links
            )
        )
        self.assertGreaterEqual(opened_at[1] - opened_at[0], 0.015)
        self.assertGreaterEqual(opened_at[2] - opened_at[1], 0.035)

    def test_browser_style_estop_uses_priority_path_even_if_queue_is_full(self):
        runtime = RobotRuntime("fake", use_gamepad=False, serial_factory=lambda *_: FakeSerial())
        runtime._connected = True
        for _ in range(64):
            self.assertTrue(runtime.submit("refresh_parameters"))
        self.assertTrue(runtime.submit("estop"))
        self.assertTrue(runtime._critical_estop.is_set())

    def test_disconnect_discards_normal_commands_before_reconnect(self):
        class FailingSerial:
            def write(self, _data):
                raise OSError("link dropped")

            def close(self):
                pass

        runtime = RobotRuntime(
            "fake",
            use_gamepad=True,
            serial_factory=lambda *_: FakeSerial(),
        )
        runtime._serial = FailingSerial()
        runtime._connected = True
        runtime._link_state = "connected"
        self.assertTrue(runtime.submit("disarm"))
        self.assertTrue(runtime.submit("clear_estop"))
        self.assertTrue(runtime.submit("arm"))

        runtime._drain_commands()
        self.assertFalse(runtime._connected)
        self.assertTrue(runtime._estop_latched)
        self.assertIsNone(runtime._pending_neutral_action)
        self.assertTrue(runtime._commands.empty())
        self.assertFalse(runtime.submit("arm"))

        replacement = FakeSerial(answer_hello=False)
        runtime._serial = replacement
        runtime._connected = True
        runtime._link_state = "connected"
        runtime._drain_commands()
        self.assertEqual(replacement.writes, [])

    def test_stop_during_blocking_open_closes_without_sending_frames(self):
        factory_entered = threading.Event()
        release_factory = threading.Event()
        link = FakeSerial(answer_hello=False)

        def blocking_factory(*_args):
            factory_entered.set()
            release_factory.wait(0.5)
            return link

        runtime = RobotRuntime(
            "fake",
            use_gamepad=False,
            serial_factory=blocking_factory,
            startup_stabilization_seconds=0.0,
        )
        runtime.start()
        self.assertTrue(factory_entered.wait(0.2))
        runtime.stop(timeout=0.02)
        timed_out = runtime.snapshot()
        self.assertEqual(timed_out["link_state"], "stop_timeout")
        self.assertIn("did not stop", timed_out["fatal_error"])

        release_factory.set()
        runtime.stop(timeout=0.5)
        self.assertFalse(runtime._thread.is_alive())
        self.assertTrue(link.closed)
        self.assertEqual(link.writes, [])

    def test_connected_stop_sends_only_safe_shutdown_frames_before_close(self):
        link = FakeSerial()
        runtime = RobotRuntime(
            "fake",
            use_gamepad=False,
            serial_factory=lambda *_: link,
            startup_stabilization_seconds=0.0,
        )
        runtime.start()
        deadline = time.monotonic() + 0.3
        while not runtime.snapshot()["connected"] and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(runtime.snapshot()["connected"])

        runtime.stop()
        self.assertTrue(link.closed)
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.writes[-2:]],
            [MessageType.CONTROL, MessageType.DISARM],
        )
        shutdown = decode_control_frame(link.writes[-2])
        self.assertEqual(
            (
                shutdown.forward,
                shutdown.turn,
                shutdown.strafe,
                shutdown.arm_yaw,
                shutdown.arm_reach,
                shutdown.arm_height,
                shutdown.gripper,
                shutdown.buttons,
            ),
            (0, 0, 0, 0, 0, 0, 0, 0),
        )
        self.assertTrue(shutdown.control_flags & ControlFlag.ESTOP_ASSERTED)

    def test_no_gamepad_runtime_refuses_arm_request(self):
        runtime = RobotRuntime("fake", use_gamepad=False, serial_factory=lambda *_: FakeSerial())
        runtime._process_command(RuntimeCommand(5, 0, "arm"))
        self.assertIsNone(runtime._pending_neutral_action)
        self.assertIn("disabled", runtime._events[-1]["message"])

    def test_host_input_tuning_is_atomic_and_disarmed_only(self):
        runtime = RobotRuntime("fake", use_gamepad=True, serial_factory=lambda *_: FakeSerial())
        values = {
            "forward_deadzone": 100, "forward_power": 200,
            "turn_deadzone": 90, "turn_power": 180,
            "arm_yaw_deadzone": 80, "arm_yaw_power": 150,
            "arm_reach_deadzone": 70, "arm_reach_power": 160,
        }
        runtime._process_command(RuntimeCommand(5, 0, "set_host_input", {"values": values}))
        self.assertEqual(runtime._host_parameter_revision, 0)
        runtime._telemetry["status"] = {"state": 1}
        runtime._process_command(RuntimeCommand(5, 1, "set_host_input", {"values": values}))
        self.assertEqual(runtime._host_parameter_revision, 1)
        self.assertAlmostEqual(runtime._control_config.drive_forward.deadzone, 0.1)
        invalid = dict(values, turn_power=float("nan"))
        runtime._process_command(RuntimeCommand(5, 2, "set_host_input", {"values": invalid}))
        self.assertEqual(runtime._host_parameter_revision, 1)


if __name__ == "__main__":
    unittest.main()
