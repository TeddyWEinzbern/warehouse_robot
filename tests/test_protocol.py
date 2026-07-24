import struct
import unittest

import robot_control.protocol as protocol
from robot_control.protocol import (
    CalibrationReportKind,
    ControlFrame,
    MessageType,
    ProtocolDecoder,
    cobs_encode,
    crc8,
    decode_control_frame,
    decode_message,
    decode_message_data,
    encode_cal_arm_move,
    encode_cal_drive_spin,
    encode_cal_joint_reference,
    encode_cal_read_drive,
    encode_control_frame,
    encode_estop,
    encode_message,
)


def raw_frame(raw_without_crc: bytes) -> bytes:
    raw = raw_without_crc + bytes((crc8(raw_without_crc),))
    return cobs_encode(raw) + b"\x00"


class CompactControlTests(unittest.TestCase):
    def test_control_is_always_exactly_eleven_wire_bytes(self):
        frames = (
            ControlFrame(sequence=0),
            ControlFrame(
                sequence=63,
                forward=-1000,
                turn=1000,
                strafe=345,
                arm_yaw=-555,
                arm_reach=1,
                arm_height=-1000,
                gripper=1,
                buttons=0x3F,
            ),
        )
        for frame in frames:
            packet = encode_control_frame(frame)
            self.assertEqual(len(packet), 11)
            decoded = decode_control_frame(packet)
            self.assertEqual(decoded.sequence, frame.sequence & 0x3F)

    def test_axes_quantize_to_one_percent_and_ternary_fields_round_trip(self):
        decoded = decode_control_frame(
            encode_control_frame(
                ControlFrame(
                    sequence=70,
                    forward=124,
                    turn=-126,
                    strafe=2000,
                    arm_yaw=-2000,
                    arm_reach=5,
                    arm_height=-20,
                    gripper=1,
                    buttons=0x21,
                )
            )
        )
        self.assertEqual(decoded.sequence, 6)
        self.assertEqual(
            (
                decoded.forward,
                decoded.turn,
                decoded.strafe,
                decoded.arm_yaw,
                decoded.arm_reach,
            ),
            (120, -130, 1000, -1000, 0),
        )
        self.assertEqual(decoded.arm_height, -1000)
        self.assertEqual(decoded.gripper, 1)
        self.assertEqual(decoded.buttons, 0x21)

    def test_dedicated_estop_is_four_wire_bytes_and_wraps_sequence(self):
        packet = encode_estop(66)
        self.assertEqual(len(packet), 4)
        message = decode_message(packet)
        self.assertEqual(message.message_type, MessageType.ESTOP_ASSERT)
        self.assertEqual(message.sequence, 2)
        self.assertEqual(message.payload, b"")

    def test_reserved_control_bits_and_out_of_range_wire_axis_are_rejected(self):
        valid = bytes((0x40, 0, 0, 0, 0, 0, 0, 0))
        with self.assertRaisesRegex(ValueError, "ternary"):
            decode_control_frame(raw_frame(valid[:6] + bytes((0x10, 0))))
        with self.assertRaisesRegex(ValueError, "action"):
            decode_control_frame(raw_frame(valid[:7] + bytes((0x40,))))
        with self.assertRaisesRegex(ValueError, "outside"):
            decode_control_frame(
                raw_frame(bytes((0x40, 101, 0, 0, 0, 0, 0, 0)))
            )

    def test_buttons_outside_six_defined_bits_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "six defined"):
            encode_control_frame(ControlFrame(sequence=1, buttons=0x40))

    def test_estop_is_not_repeated_in_control_schema(self):
        self.assertFalse(hasattr(protocol, "ControlFlag"))
        self.assertNotIn("control_flags", ControlFrame.__dataclass_fields__)


class GenericEnvelopeTests(unittest.TestCase):
    def test_generic_v3_round_trip(self):
        packet = encode_message(MessageType.HELLO, 0xA5)
        message = decode_message(packet)
        self.assertEqual(message.message_type, MessageType.HELLO)
        self.assertEqual(message.sequence, 0xA5)
        self.assertEqual(message.payload, b"")

    def test_checksum_and_unknown_type_are_rejected(self):
        packet = bytearray(encode_message(MessageType.HELLO, 1))
        packet[-2] ^= 0x55
        with self.assertRaises(ValueError):
            decode_message(bytes(packet))
        with self.assertRaisesRegex(ValueError, "unknown"):
            decode_message(raw_frame(bytes((3, 0x77, 1, 0))))

    def test_decoder_discards_oversized_packet_until_delimiter(self):
        decoder = ProtocolDecoder(maximum_encoded_length=8)
        valid = encode_message(MessageType.HELLO, 9)
        messages = decoder.feed(b"\x01" * 20 + b"\x00" + valid)
        self.assertEqual(decoder.overflows, 1)
        self.assertEqual(
            [(message.message_type, message.sequence) for message in messages],
            [(MessageType.HELLO, 9)],
        )

    def test_malformed_packet_does_not_contaminate_next_frame(self):
        decoder = ProtocolDecoder()
        messages = decoder.feed(
            b"\x03\x99\x00" + encode_message(MessageType.DISARM, 4)
        )
        self.assertEqual(decoder.malformed_frames, 1)
        self.assertEqual(messages[-1].message_type, MessageType.DISARM)


class ReplyDecodeTests(unittest.TestCase):
    def test_hello_response_uses_explicit_fields_not_capability_bitmap(self):
        message = decode_message(
            encode_message(
                MessageType.HELLO_RESPONSE,
                4,
                bytes((7, 1, 1, 0, 2, 1, 0, 8)),
            )
        )
        decoded = decode_message_data(message)
        self.assertEqual(decoded["profile"], 7)
        self.assertTrue(decoded["arm_enabled"])
        self.assertTrue(decoded["drive_enabled"])
        self.assertFalse(decoded["sensor_enabled"])
        self.assertEqual(decoded["driver_mode"], 2)
        self.assertEqual(decoded["baud"], 9600)

    def test_critical_status_has_only_safety_and_link_fields(self):
        payload = bytes((3,)) + struct.pack("<HH", 0x0101, 0x0002) + bytes((62, 1))
        decoded = decode_message_data(
            decode_message(
                encode_message(MessageType.CRITICAL_STATUS, 7, payload)
            )
        )
        self.assertEqual(
            decoded,
            {
                "kind": "critical_status",
                "state": 3,
                "faults": 0x0101,
                "warnings": 0x0002,
                "last_accepted_control_sequence": 62,
                "link_alive": True,
            },
        )

    def test_calibration_reports_decode_on_demand_pages(self):
        counts = (
            bytes((CalibrationReportKind.DRIVE_COUNTS,))
            + struct.pack("<4h", 1, -2, 3, -4)
            + struct.pack("<4i", 100, -200, 300, -400)
            + bytes((0x0F,))
        )
        decoded = decode_message_data(
            decode_message(
                encode_message(MessageType.CAL_REPORT, 8, counts)
            )
        )
        self.assertEqual(decoded["raw_increments"], [1, -2, 3, -4])
        self.assertEqual(decoded["totals"], [100, -200, 300, -400])
        self.assertEqual(decoded["valid_mask"], 0x0F)

    def test_drive_spin_uses_calibration_only_generic_message(self):
        message = decode_message(encode_cal_drive_spin(9, 1, 2, -150, 2000))
        self.assertEqual(message.message_type, MessageType.CAL_DRIVE_SPIN)
        self.assertEqual(message.payload, struct.pack("<BBhH", 1, 2, -150, 2000))

    def test_calibration_encoders_reject_instead_of_clamping_addresses(self):
        invalid_calls = (
            lambda: encode_cal_arm_move(1, 4, 90),
            lambda: encode_cal_arm_move(1, 0, 181),
            lambda: encode_cal_joint_reference(1, -1, 0, 180, 0, 1),
            lambda: encode_cal_drive_spin(1, 0, 4, 10, 100),
            lambda: encode_cal_drive_spin(1, 2, 0, 10, 100),
            lambda: encode_cal_read_drive(1, 2),
        )
        for encode in invalid_calls:
            with self.subTest(encode=encode):
                with self.assertRaises(ValueError):
                    encode()

    def test_calibration_system_report_decodes_stack_canary_result(self):
        payload = bytes((CalibrationReportKind.SYSTEM,)) + struct.pack("<H", 287)
        decoded = decode_message_data(
            decode_message(
                encode_message(MessageType.CAL_REPORT, 10, payload)
            )
        )
        self.assertEqual(decoded["kind"], "cal_system")
        self.assertEqual(decoded["minimum_untouched_stack_bytes"], 287)


if __name__ == "__main__":
    unittest.main()
