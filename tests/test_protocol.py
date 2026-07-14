import struct
import unittest

from robot_control.protocol import (
    ControlFlag,
    ControlFrame,
    MessageType,
    ParameterGroup,
    ProtocolDecoder,
    build_parameter_data,
    cobs_decode,
    cobs_encode,
    decode_control_frame,
    decode_message,
    encode_control_frame,
    encode_message,
    encode_parameter_set,
    telemetry_to_dict,
)


class ProtocolTests(unittest.TestCase):
    def test_cobs_round_trip_with_zero_bytes(self):
        raw = b"\x00\x01\x00\xff\x02"
        self.assertEqual(cobs_decode(cobs_encode(raw)), raw)

    def test_control_frame_v2_golden_vector_and_clamping(self):
        encoded = encode_control_frame(
            ControlFrame(
                sequence=7, forward=2000, turn=-2000, strafe=123,
                arm_yaw=45, arm_reach=-67, arm_height=89,
                gripper=3, buttons=0x55,
                control_flags=int(ControlFlag.ESTOP_ASSERTED),
            )
        )
        self.assertEqual(
            encoded.hex(),
            "0a02020710e80318fc7b022d04bdff5903015503012100",
        )
        decoded = decode_control_frame(encoded)
        self.assertEqual(decoded.sequence, 7)
        self.assertEqual(decoded.forward, 1000)
        self.assertEqual(decoded.turn, -1000)
        self.assertEqual(decoded.gripper, 1)
        self.assertEqual(decoded.buttons, 0x55)
        self.assertEqual(decoded.control_flags, 1)

    def test_incremental_decoder_handles_fragmentation_and_concatenation(self):
        first = encode_message(MessageType.ARM, 1)
        second = encode_message(MessageType.DISARM, 2)
        decoder = ProtocolDecoder()
        self.assertEqual(decoder.feed(first[:3]), [])
        messages = decoder.feed(first[3:] + second)
        self.assertEqual([message.message_type for message in messages], [MessageType.ARM, MessageType.DISARM])

    def test_decoder_recovers_after_overflow_and_malformed_packet(self):
        decoder = ProtocolDecoder(maximum_encoded_length=8)
        valid = encode_message(MessageType.HELLO, 4)
        messages = decoder.feed(b"x" * 12 + b"\x00" + valid)
        self.assertEqual(decoder.overflows, 1)
        self.assertEqual(messages[-1].message_type, MessageType.HELLO)

    def test_checksum_rejects_corruption(self):
        encoded = bytearray(encode_control_frame(ControlFrame(sequence=1)))
        encoded[2] ^= 0x40
        with self.assertRaises(ValueError):
            decode_control_frame(bytes(encoded))

    def test_parameter_frame_contains_atomic_revision(self):
        data = build_parameter_data(
            ParameterGroup.SERVO,
            2,
            {"lower": 10, "upper": 150, "center_offset": -8, "direction": -1},
        )
        message = decode_message(encode_parameter_set(9, ParameterGroup.SERVO, 2, 0x1234, data))
        self.assertEqual(message.message_type, MessageType.PARAMETER_SET)
        self.assertEqual(message.payload[:4], b"\x01\x02\x34\x12")
        self.assertEqual(message.payload[4:], bytes((10, 150, 248, 255)))

    def test_invalid_direction_is_rejected_host_side(self):
        with self.assertRaises(ValueError):
            build_parameter_data(
                ParameterGroup.OPEN_LOOP_MOTOR,
                0,
                {"minimum_pwm": 20, "maximum_pwm": 80, "direction": 0},
            )

    def test_excessive_runtime_parameters_are_rejected_not_clamped(self):
        with self.assertRaises(ValueError):
            build_parameter_data(
                ParameterGroup.CHASSIS_SPEED,
                0,
                {
                    "forward": 1001, "reverse": 500, "lateral": 500,
                    "yaw": 1000, "wheel": 500,
                },
            )
        with self.assertRaises(ValueError):
            build_parameter_data(
                ParameterGroup.CHASSIS_ACCELERATION,
                0,
                {
                    "forward_accel": 3001, "forward_decel": 800,
                    "reverse_accel": 400, "reverse_decel": 800,
                    "lateral_accel": 400, "lateral_decel": 700,
                    "rotation_accel": 1200, "rotation_decel": 2000,
                },
            )
        with self.assertRaises(ValueError):
            build_parameter_data(
                ParameterGroup.RESPONSE_PROFILE,
                1,
                {
                    "speed_permille": 1001,
                    "acceleration_permille": 1000,
                    "deceleration_permille": 1000,
                },
            )

    def test_encoder_geometry_and_permutation_are_bounded_host_side(self):
        geometry = build_parameter_data(
            ParameterGroup.ENCODER,
            0,
            {
                "wheel_diameter_mm": 60,
                "counts_per_revolution": 4680,
                "wheel_track_mm": 160,
                "wheelbase_mm": 170,
                "semantics": 0,
            },
        )
        self.assertEqual(struct.unpack("<4HB", geometry), (60, 4680, 160, 170, 0))
        mapping = {
            "map_0": 2, "map_1": 0, "map_2": 3, "map_3": 1,
            "sign_0": -1, "sign_1": 1, "sign_2": -1, "sign_3": 1,
        }
        self.assertEqual(
            build_parameter_data(ParameterGroup.ENCODER, 1, mapping),
            bytes((2, 0, 3, 1, 255, 1, 255, 1)),
        )
        mapping["map_3"] = 3
        with self.assertRaises(ValueError):
            build_parameter_data(ParameterGroup.ENCODER, 1, mapping)

    def test_closed_loop_telemetry_keeps_pwm_unavailable(self):
        payload = struct.pack(
            "<BBB10hBH",
            3, 1, 0,
            100, 0, 0,
            80, 0, 0,
            80, 80, 80, 80,
            0, 5,
        )
        decoded = telemetry_to_dict(
            decode_message(encode_message(MessageType.DRIVE_COMMAND_TELEMETRY, 1, payload))
        )
        self.assertEqual(decoded["control_mode"], 3)
        self.assertFalse(decoded["pwm_valid"])
        self.assertEqual(decoded["controller_targets"], [80, 80, 80, 80])


if __name__ == "__main__":
    unittest.main()
