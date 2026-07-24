"""Unit tests for the protocol-v3 interactive calibration session."""

from __future__ import annotations

import struct
import unittest

from robot_control.calibration import CalibrationSession
from robot_control.protocol import (
    CalibrationReportKind,
    MessageType,
    ProtocolDecoder,
    decode_message,
    encode_message,
)


MUTATING_CALIBRATION_TYPES = {
    MessageType.CAL_ARM_MOVE,
    MessageType.CAL_SET_JOINT_REFERENCE,
    MessageType.CAL_DRIVE_SPIN,
}


class FakeLink:
    """Serial stand-in with optional scripted replies and safe auto-ACKs."""

    def __init__(self) -> None:
        self.written: list[bytes] = []
        self.replies: list[bytes] = []
        self._pending = b""
        self.flushed = False
        self.baudrate = 9600

    def queue_reply(self, packet: bytes) -> None:
        self.replies.append(packet)

    def write(self, data: bytes) -> int:
        self.written.append(data)
        message = decode_message(data)
        if self.replies:
            self._pending += self.replies.pop(0)
        elif message.message_type in MUTATING_CALIBRATION_TYPES:
            self._pending += ack(message.message_type, message.sequence)
        elif message.message_type == MessageType.HELLO:
            self._pending += encode_message(
                MessageType.HELLO_RESPONSE,
                0,
                bytes((7, 1, 1, 1, 2, 0, 0, 8)),
            )
        return len(data)

    @property
    def in_waiting(self) -> int:
        return len(self._pending)

    def read(self, count: int) -> bytes:
        data, self._pending = self._pending[:count], self._pending[count:]
        return data

    def flush(self) -> None:
        self.flushed = True


class Clock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        self.now += 0.05
        return self.now


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds

    def advance(self, seconds: float) -> None:
        self.now += seconds


def make_session(link: FakeLink) -> tuple[CalibrationSession, list[str]]:
    output: list[str] = []
    session = CalibrationSession(
        link, out=output.append, clock=Clock(), sleep=lambda _: None
    )
    return session, output


def ack(message_type: MessageType, sequence: int) -> bytes:
    return encode_message(MessageType.ACK, sequence, bytes((message_type,)))


def nack(message_type: MessageType, reason: int, sequence: int) -> bytes:
    return encode_message(
        MessageType.NACK, sequence, bytes((message_type, reason))
    )


def arm_report(sequence: int, servos=(90, 91, 92, 80)) -> bytes:
    return encode_message(
        MessageType.CAL_REPORT,
        sequence,
        bytes((CalibrationReportKind.ARM, *servos)),
    )


def counts_report(sequence: int, increments, totals, valid_mask=0x0F) -> bytes:
    payload = bytes((CalibrationReportKind.DRIVE_COUNTS,))
    payload += struct.pack("<4h", *increments)
    payload += struct.pack("<4i", *totals)
    payload += bytes((valid_mask,))
    return encode_message(MessageType.CAL_REPORT, sequence, payload)


def speed_report(sequence: int, speeds, valid_mask=0x0F) -> bytes:
    payload = bytes((CalibrationReportKind.DRIVE_SPEED,))
    payload += struct.pack("<4h", *speeds)
    payload += bytes((valid_mask,))
    return encode_message(MessageType.CAL_REPORT, sequence, payload)


def sensor_report(sequence: int, distances, valid_mask=0x3F) -> bytes:
    payload = bytes((CalibrationReportKind.SENSOR,))
    payload += struct.pack("<6H", *distances)
    payload += bytes((valid_mask,))
    return encode_message(MessageType.CAL_REPORT, sequence, payload)


def system_report(sequence: int, minimum_stack_bytes: int) -> bytes:
    payload = bytes((CalibrationReportKind.SYSTEM,))
    payload += struct.pack("<H", minimum_stack_bytes)
    return encode_message(MessageType.CAL_REPORT, sequence, payload)


class MoveAndReferenceTests(unittest.TestCase):
    def test_absolute_move_uses_calibration_only_command(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("j2 90")
        self.assertTrue(session.handle_line("j2 95"))
        self.assertEqual(session.commanded[2], 95)
        message = decode_message(link.written[-1])
        self.assertEqual(message.message_type, MessageType.CAL_ARM_MOVE)
        self.assertEqual(message.payload, bytes((2, 95)))
        self.assertIn("elbow", output[0])

    def test_incremental_move_requires_previous_absolute(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("j1 +5")
        self.assertTrue(output[0].startswith("error:"))
        self.assertEqual(link.written, [])

    def test_first_joint_command_must_be_raw_ninety_anchor(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("j3 80")
        self.assertEqual(link.written, [])
        self.assertIn("must be exactly 90", output[-1])

    def test_nack_reason_four_reports_validation_failure(self):
        link = FakeLink()
        link.queue_reply(nack(MessageType.CAL_ARM_MOVE, 4, 1))
        session, output = make_session(link)
        session.commanded[2] = 90
        session.handle_line("j2 170")
        self.assertIn("validation failed", output[0])
        self.assertEqual(session.commanded[2], 90)

    def test_stale_ack_cannot_satisfy_later_actuator_command(self):
        link = FakeLink()
        link.queue_reply(
            ack(MessageType.CAL_ARM_MOVE, 0)
            + ack(MessageType.CAL_ARM_MOVE, 1)
        )
        session, output = make_session(link)
        session.handle_line("j0 90")
        self.assertEqual(session.commanded[0], 90)
        self.assertFalse(any(line.startswith("error:") for line in output))

    def test_center_and_direction_send_explicit_joint_reference(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("j1 90")
        session.handle_line("j1 88")
        session.handle_line("mark j1 center")
        session.handle_line("dir j1 -1")
        messages = ProtocolDecoder().feed(b"".join(link.written))
        references = [
            message
            for message in messages
            if message.message_type == MessageType.CAL_SET_JOINT_REFERENCE
        ]
        self.assertEqual(len(references), 1)
        self.assertEqual(
            references[0].payload,
            struct.pack("<BBBbb", 1, 0, 180, -2, -1),
        )
        self.assertEqual(session.synced[1], 88)
        self.assertTrue(any("synced j1" in line for line in output))


class OnDemandReportTests(unittest.TestCase):
    def test_status_requests_arm_report_instead_of_streaming_data(self):
        link = FakeLink()
        link.queue_reply(arm_report(1))
        session, output = make_session(link)
        session.handle_line("s")
        request = decode_message(link.written[-1])
        self.assertEqual(request.message_type, MessageType.CAL_READ_ARM)
        self.assertIn("91", output[-1])
        self.assertIn("reported", output[-1])

    def test_status_still_shows_local_records_when_arm_is_disabled(self):
        link = FakeLink()
        session, output = make_session(link)
        session.hello = {"arm_enabled": False}
        session.motor_map[0] = (1, 1)
        session.handle_line("s")
        self.assertEqual(link.written, [])
        self.assertIn("motor map: ch0->fr+", output[-1])

    def test_counts_requests_both_drive_pages(self):
        link = FakeLink()
        link.queue_reply(counts_report(1, (5, -6, 7, -8), (100, -200, 300, -400)))
        link.queue_reply(speed_report(2, (10, -20, 30, -40)))
        session, output = make_session(link)
        session.handle_line("counts")
        requests = [decode_message(packet) for packet in link.written]
        self.assertEqual(
            [(request.message_type, request.payload) for request in requests],
            [
                (MessageType.CAL_READ_DRIVE, b"\x00"),
                (MessageType.CAL_READ_DRIVE, b"\x01"),
            ],
        )
        self.assertIn("-200", output[-1])
        self.assertIn("-40", output[-1])

    def test_sensors_requests_single_sensor_report(self):
        link = FakeLink()
        link.queue_reply(sensor_report(1, (101, 102, 103, 104, 105, 106), 0x2F))
        session, output = make_session(link)
        session.handle_line("sensors")
        request = decode_message(link.written[-1])
        self.assertEqual(request.message_type, MessageType.CAL_READ_SENSOR)
        self.assertIn("106", output[-1])
        self.assertIn("no", output[-1])

    def test_system_report_prints_stack_pass_and_fail(self):
        for remaining, verdict in ((256, "PASS"), (255, "FAIL")):
            with self.subTest(remaining=remaining):
                link = FakeLink()
                link.queue_reply(system_report(1, remaining))
                session, output = make_session(link)
                session.handle_line("stack")
                request = decode_message(link.written[-1])
                self.assertEqual(
                    request.message_type, MessageType.CAL_READ_SYSTEM
                )
                self.assertIn(f"{remaining} B", output[-1])
                self.assertIn(verdict, output[-1])

    def test_invalid_drive_channels_are_not_printed_as_calibration_values(self):
        link = FakeLink()
        link.queue_reply(
            counts_report(
                1,
                (5, -600, 7, -8),
                (100, -20000, 300, -400),
                valid_mask=0x0D,
            )
        )
        link.queue_reply(
            speed_report(2, (10, -2000, 30, -40), valid_mask=0x0D)
        )
        session, output = make_session(link)
        session.handle_line("counts")
        table = output[-1]
        self.assertIn("INVALID - do not calibrate", table)
        self.assertNotIn("-600", table)
        self.assertNotIn("-20000", table)
        self.assertNotIn("-2000", table)


class MotorCommandTests(unittest.TestCase):
    def test_open_and_closed_loop_spins_use_v3_calibration_command(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("m0 30 2")
        session.handle_line("v2 -150")
        messages = [decode_message(packet) for packet in link.written]
        self.assertEqual(messages[0].message_type, MessageType.CAL_DRIVE_SPIN)
        self.assertEqual(messages[0].payload, struct.pack("<BBhH", 0, 0, 30, 2000))
        self.assertEqual(messages[1].payload, struct.pack("<BBhH", 1, 2, -150, 2000))
        self.assertIn("spinning", output[-1])

    def test_spin_values_beyond_limits_are_rejected_locally(self):
        link = FakeLink()
        session, output = make_session(link)
        session.handle_line("m1 130 2")
        session.handle_line("v1 300 2")
        session.handle_line("m1 50 15")
        self.assertEqual(link.written, [])
        self.assertTrue(all(line.startswith("error:") for line in output))

    def test_session_shutdown_sends_fail_closed_estop_burst(self):
        link = FakeLink()
        session, _ = make_session(link)
        session.shutdown()
        self.assertEqual(
            [decode_message(packet).message_type for packet in link.written],
            [MessageType.ESTOP_ASSERT] * 3 + [MessageType.DISARM],
        )
        self.assertTrue(link.flushed)

    def test_start_recovers_prior_estop_with_neutral_stream_and_verifies_hello(self):
        link = FakeLink()
        session, _ = make_session(link)
        session.start()
        messages = [decode_message(packet) for packet in link.written]
        self.assertEqual(
            [message.message_type for message in messages[:3]],
            [MessageType.ESTOP_ASSERT] * 3,
        )
        controls = [
            message
            for message in messages
            if message.message_type == MessageType.CONTROL
        ]
        self.assertGreaterEqual(len(controls), 22)
        self.assertEqual(
            [message.message_type for message in messages[-2:]],
            [MessageType.CLEAR_ESTOP, MessageType.HELLO],
        )
        self.assertNotIn(
            MessageType.DISARM,
            [message.message_type for message in messages[3:]],
        )
        self.assertEqual(session.hello["profile"], 7)

    def test_start_drains_estop_burst_before_timed_control_stream(self):
        timeline = ManualClock()

        class TimedLink(FakeLink):
            def __init__(self) -> None:
                super().__init__()
                self.write_times: list[float] = []

            def write(self, data: bytes) -> int:
                self.write_times.append(timeline())
                return super().write(data)

        link = TimedLink()
        session = CalibrationSession(
            link,
            out=lambda _line: None,
            clock=timeline,
            sleep=timeline.sleep,
        )
        session.start()
        messages = [
            decode_message(packet) for packet in link.written
        ]
        first_control = next(
            index
            for index, message in enumerate(messages)
            if message.message_type == MessageType.CONTROL
        )
        self.assertGreaterEqual(
            link.write_times[first_control] - link.write_times[2],
            12 * 10.0 / 9600,
        )
        control_times = [
            timestamp
            for message, timestamp in zip(messages, link.write_times)
            if message.message_type == MessageType.CONTROL
        ]
        self.assertTrue(
            all(
                abs(later - earlier - 1.0 / 30.0) < 1e-9
                for earlier, later in zip(
                    control_times, control_times[1:]
                )
            )
        )

    def test_stress_sends_only_fixed_rate_neutral_control_frames(self):
        link = FakeLink()
        timeline = ManualClock()
        output: list[str] = []
        session = CalibrationSession(
            link,
            out=output.append,
            clock=timeline,
            sleep=timeline.sleep,
        )
        session.handle_line("stress 1")
        messages = [decode_message(packet) for packet in link.written]
        self.assertEqual(len(messages), 31)
        self.assertTrue(
            all(
                message.message_type == MessageType.CONTROL
                for message in messages
            )
        )
        self.assertIn(
            "target 30 Hz, actual 30.00 Hz, max interval 33.33 ms,"
            " missed slots 0",
            output[-1],
        )

    def test_blocked_write_anchors_next_control_after_completion(self):
        timeline = ManualClock()

        class DelayedLink(FakeLink):
            def __init__(self) -> None:
                super().__init__()
                self.write_starts: list[float] = []
                self.write_completions: list[float] = []

            def write(self, data: bytes) -> int:
                self.write_starts.append(timeline())
                timeline.advance(0.025)
                written = super().write(data)
                self.write_completions.append(timeline())
                return written

        link = DelayedLink()
        output: list[str] = []
        session = CalibrationSession(
            link,
            out=output.append,
            clock=timeline,
            sleep=timeline.sleep,
        )
        session.handle_line("stress 1")
        response_windows = [
            later_start - earlier_completion
            for earlier_completion, later_start in zip(
                link.write_completions, link.write_starts[1:]
            )
        ]
        self.assertEqual(len(link.write_starts), 31)
        self.assertTrue(
            all(
                abs(window - 1.0 / 30.0) < 1e-9
                for window in response_windows
            )
        )
        self.assertIn(
            "actual 17.14 Hz, max interval 58.33 ms, missed slots 0",
            output[-1],
        )


class ExportTests(unittest.TestCase):
    def test_motor_encoder_and_joint_records_shape_export(self):
        link = FakeLink()
        session, _ = make_session(link)
        for command in (
            "j0 90",
            "mark j0 center",
            "j0 5",
            "mark j0 lower",
            "j0 178",
            "mark j0 upper",
            "dir j0 +1",
            "j1 90",
            "j1 88",
            "mark j1 center",
            "dir j1 -1",
            "j3 90",
            "j3 78",
            "mark j3 open",
            "j3 24",
            "mark j3 closed",
            "motor 0 fr fwd",
            "motor 1 fl rev",
            "motor 2 rr fwd",
            "motor 3 rl fwd",
            "enc 0 fr rev",
            "enc 1 fl fwd",
            "enc 2 rr fwd",
            "enc 3 rl fwd",
            "geom 65 4680 160 170",
        ):
            session.handle_line(command)
        block = session.export_block()
        self.assertIn("constexpr uint8_t BaseZeroDegrees = 90;", block)
        self.assertIn("constexpr uint8_t ShoulderZeroDegrees = 88;", block)
        self.assertIn("constexpr uint8_t GripperOpenDegrees = 78;", block)
        self.assertIn("ServoDirectionSign[4] = {1, -1, 1, 1};", block)
        self.assertIn("MotorCommandMap[4] = {1, 0, 3, 2};", block)
        self.assertIn("EncoderChannelMap[4] = {1, 0, 3, 2};", block)
        self.assertIn("constexpr uint16_t WheelDiameterMm = 65;", block)

    def test_missing_records_are_called_out_as_warnings(self):
        session, _ = make_session(FakeLink())
        block = session.export_block()
        self.assertIn("WARNINGS", block)
        self.assertIn("motor record missing", block)
        self.assertIn("geometry not recorded", block)


if __name__ == "__main__":
    unittest.main()
