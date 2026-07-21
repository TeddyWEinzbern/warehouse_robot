"""Unit tests for the interactive calibration session."""

from __future__ import annotations

import struct
import unittest

from robot_control.calibration import CalibrationSession
from robot_control.protocol import MessageType, encode_message


class FakeLink:
    """Serial stand-in that queues scripted firmware replies per write."""

    def __init__(self) -> None:
        self.written: list[bytes] = []
        self.replies: list[bytes] = []
        self._pending = b""

    def queue_reply(self, packet: bytes) -> None:
        self.replies.append(packet)

    def write(self, data: bytes) -> None:
        self.written.append(data)
        if self.replies:
            self._pending += self.replies.pop(0)

    @property
    def in_waiting(self) -> int:
        return len(self._pending)

    def read(self, count: int) -> bytes:
        data, self._pending = self._pending[:count], self._pending[count:]
        return data


def make_session(link: FakeLink) -> tuple[CalibrationSession, list[str]]:
    output: list[str] = []
    session = CalibrationSession(
        link, out=output.append, clock=Clock(), sleep=lambda _: None
    )
    return session, output


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        self.now += 0.05
        return self.now


def ack(sequence: int = 1) -> bytes:
    return encode_message(MessageType.ACK, sequence, bytes([0x11, 0, 0]))


def nack(reason: int, sequence: int = 1) -> bytes:
    return encode_message(MessageType.NACK, sequence, bytes([0x11, reason]))


def sensor_arm(servos: tuple[int, int, int, int]) -> bytes:
    payload = struct.pack("<6H", 0, 0, 0, 0, 0, 0) + bytes([0])
    payload += bytes(servos) + bytes([1, 0]) + struct.pack("<H", 7400)
    payload += bytes([1]) + struct.pack("<H", 10)
    return encode_message(MessageType.SENSOR_ARM_TELEMETRY, 9, payload)


class MoveCommandTests(unittest.TestCase):
    def test_absolute_move_sends_calibration_command_and_tracks_angle(self):
        link = FakeLink()
        link.queue_reply(ack())
        session, output = make_session(link)
        self.assertTrue(session.handle_line("j2 95"))
        self.assertEqual(session.commanded[2], 95)
        self.assertIn("elbow", output[0])
        payload = link.written[-1]
        self.assertIn(bytes([2, 95]), payload)

    def test_incremental_move_requires_a_previous_absolute(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("j1 +5")
        self.assertTrue(output[0].startswith("error:"))
        self.assertEqual(link.written, [])

    def test_incremental_move_offsets_last_commanded(self):
        link = FakeLink()
        link.queue_reply(ack())
        link.queue_reply(ack(2))
        session, _ = make_session(link)
        session.handle_line("j1 90")
        session.handle_line("j1 -3")
        self.assertEqual(session.commanded[1], 87)

    def test_nack_is_translated_to_human_readable_reason(self):
        link = FakeLink()
        link.queue_reply(nack(5))
        session, output = make_session(link)
        session.handle_line("j2 170")
        self.assertIn("coupling guard", output[0])
        self.assertIsNone(session.commanded[2])

    def test_out_of_range_angle_is_rejected_locally(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("j0 181")
        self.assertTrue(output[0].startswith("error:"))
        self.assertEqual(link.written, [])


class MarkAndStatusTests(unittest.TestCase):
    def test_mark_records_current_angle_and_gripper_uses_open_closed(self):
        link = FakeLink()
        link.queue_reply(ack())
        session, output = make_session(link)
        session.handle_line("j3 78")
        session.handle_line("mark j3 open")
        self.assertEqual(session.marks[(3, "open")], 78)
        session.handle_line("mark j3 center")
        self.assertIn("open", output[-1])
        self.assertTrue(output[-1].startswith("error:"))

    def test_status_shows_telemetry_from_sensor_arm_frames(self):
        link = FakeLink()
        link.queue_reply(ack())
        session, output = make_session(link)
        session.handle_line("j0 90")
        link._pending += sensor_arm((90, 91, 92, 80))
        session.handle_line("s")
        self.assertIn("91", output[-1])

    def test_fold_estimate_uses_marked_centers_and_directions(self):
        link = FakeLink()
        for sequence in range(1, 5):
            link.queue_reply(ack(sequence))
        session, output = make_session(link)
        session.handle_line("j1 90")
        session.handle_line("j2 90")
        session.handle_line("mark j1 center")
        session.handle_line("mark j2 center")
        session.handle_line("dir j1 +1")
        session.handle_line("dir j2 +1")
        session.handle_line("j2 45")
        self.assertIn("fold estimate: 135", output[-1])


class ExportTests(unittest.TestCase):
    def test_export_produces_paste_block_with_marked_values(self):
        link = FakeLink()
        for sequence in range(1, 12):
            link.queue_reply(ack(sequence))
        session, _ = make_session(link)
        for command in (
            "j0 90", "mark j0 center", "j0 5", "mark j0 lower",
            "j0 178", "mark j0 upper", "dir j0 +1",
            "j1 88", "mark j1 center", "dir j1 -1",
            "j3 78", "mark j3 open", "j3 24", "mark j3 closed",
        ):
            session.handle_line(command)
        block = session.export_block()
        self.assertIn("constexpr uint8_t BaseZeroDegrees = 90;", block)
        self.assertIn("constexpr uint8_t ShoulderZeroDegrees = 88;", block)
        self.assertIn("constexpr uint8_t GripperOpenDegrees = 78;", block)
        self.assertIn("constexpr uint8_t GripperClosedDegrees = 24;", block)
        self.assertIn("ServoLowerDegrees[4] = {5, ", block)
        self.assertIn("ServoDirectionSign[4] = {1, -1, 1, 1};", block)
        self.assertIn("WARNINGS", block)
        self.assertIn("j2 center not marked", block)

    def test_export_warns_when_base_travel_is_under_180(self):
        link = FakeLink()
        for sequence in range(1, 4):
            link.queue_reply(ack(sequence))
        session, _ = make_session(link)
        for command in (
            "j0 30", "mark j0 lower", "j0 150", "mark j0 upper",
        ):
            session.handle_line(command)
        block = session.export_block()
        self.assertIn("requires >= 180", block)


if __name__ == "__main__":
    unittest.main()
