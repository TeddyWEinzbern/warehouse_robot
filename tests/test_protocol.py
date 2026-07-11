import unittest

from robot_control.protocol import (
    ControlFrame,
    cobs_decode,
    cobs_encode,
    decode_control_frame,
    encode_control_frame,
)


class ProtocolTests(unittest.TestCase):
    def test_cobs_round_trip_with_zero_bytes(self):
        raw = b"\x00\x01\x00\xff\x02"
        self.assertEqual(cobs_decode(cobs_encode(raw)), raw)

    def test_control_frame_round_trip_and_clamping(self):
        encoded = encode_control_frame(
            ControlFrame(
                sequence=260, forward=2000, turn=-2000, strafe=123,
                arm_yaw=45, arm_reach=-67, arm_height=89,
                gripper=3, buttons=0x55,
            )
        )
        decoded = decode_control_frame(encoded)
        self.assertEqual(decoded.sequence, 4)
        self.assertEqual(decoded.forward, 1000)
        self.assertEqual(decoded.turn, -1000)
        self.assertEqual(decoded.gripper, 1)
        self.assertEqual(decoded.buttons, 0x55)

    def test_checksum_rejects_corruption(self):
        encoded = bytearray(encode_control_frame(ControlFrame(sequence=1)))
        encoded[2] ^= 0x40
        with self.assertRaises(ValueError):
            decode_control_frame(bytes(encoded))


if __name__ == "__main__":
    unittest.main()

