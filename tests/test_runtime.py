import threading
import time
import unittest

from robot_control.protocol import ControlFlag, MessageType, decode_control_frame, decode_message
from robot_control.runtime import RobotRuntime, RuntimeCommand


class FakeSerial:
    def __init__(self):
        self.writes = []
        self.closed = False
        self._lock = threading.Lock()

    @property
    def in_waiting(self):
        return 0

    def read(self, _length):
        return b""

    def write(self, data):
        with self._lock:
            if self.closed:
                raise OSError("closed")
            self.writes.append(bytes(data))
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
        )
        runtime.start()
        time.sleep(0.14)
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

    def test_browser_style_estop_uses_priority_path_even_if_queue_is_full(self):
        runtime = RobotRuntime("fake", use_gamepad=False, serial_factory=lambda *_: FakeSerial())
        for _ in range(64):
            self.assertTrue(runtime.submit("refresh_parameters"))
        self.assertTrue(runtime.submit("estop"))
        self.assertTrue(runtime._critical_estop.is_set())

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
