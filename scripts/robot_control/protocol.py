"""Compact protocol v3 framing and typed robot-control messages.

Normal control and emergency-stop frames use fixed short layouts.  Rare
commands, critical status, and calibration request/reply traffic use the
generic versioned envelope.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct
from typing import Any

PROTOCOL_VERSION = 3
MAX_PAYLOAD = 27
MAX_ENCODED_LENGTH = 36


class MessageType(IntEnum):
    HELLO = 0x01
    CONTROL = 0x02
    ARM = 0x03
    DISARM = 0x04
    CLEAR_ESTOP = 0x05
    CLEAR_FAULT = 0x06
    ESTOP_ASSERT = 0x07  # Synthetic type for the dedicated fast frame.

    CAL_ARM_MOVE = 0x10
    CAL_SET_JOINT_REFERENCE = 0x11
    CAL_DRIVE_SPIN = 0x12
    CAL_READ_DRIVE = 0x13
    CAL_READ_SENSOR = 0x14
    CAL_READ_ARM = 0x15
    CAL_READ_SYSTEM = 0x16

    HELLO_RESPONSE = 0x80
    CRITICAL_STATUS = 0x81
    CAL_REPORT = 0x90
    ACK = 0x91
    NACK = 0x92


class CalibrationReportKind(IntEnum):
    ARM = 1
    DRIVE_COUNTS = 2
    DRIVE_SPEED = 3
    SENSOR = 4
    SYSTEM = 5


@dataclass(frozen=True)
class ControlFrame:
    """Semantic control values.

    Continuous axes stay in the host's established -1000..1000 permille
    domain.  Encoding quantizes them to signed percentage points on the wire.
    """

    sequence: int
    forward: int = 0
    turn: int = 0
    strafe: int = 0
    arm_yaw: int = 0
    arm_reach: int = 0
    arm_height: int = 0
    gripper: int = 0
    buttons: int = 0

    def neutral(self) -> bool:
        return (
            max(
                abs(self.forward),
                abs(self.turn),
                abs(self.strafe),
                abs(self.arm_yaw),
                abs(self.arm_reach),
                abs(self.arm_height),
            )
            <= 30
            and self.gripper == 0
            and self.buttons == 0
        )


@dataclass(frozen=True)
class Message:
    message_type: MessageType
    sequence: int
    payload: bytes


def _finite_int(value: Any, low: int, high: int, name: str) -> int:
    if isinstance(value, bool):
        value = int(value)
    if not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return max(low, min(high, int(round(value))))


def _strict_int(value: Any, low: int, high: int, name: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or int(value) != value
    ):
        raise ValueError(f"{name} must be a whole number")
    result = int(value)
    if not low <= result <= high:
        raise ValueError(f"{name} must be within {low}..{high}")
    return result


def crc8(data: bytes) -> int:
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    output = bytearray([0])
    code_index = 0
    code = 1
    for value in data:
        if value == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
        else:
            output.append(value)
            code += 1
            if code == 0xFF:
                output[code_index] = code
                code_index = len(output)
                output.append(0)
                code = 1
    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ValueError("COBS packet contains an unexpected delimiter")
        index += 1
        end = index + code - 1
        if end > len(data):
            raise ValueError("truncated COBS packet")
        output.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def _frame(raw_without_crc: bytes) -> bytes:
    raw = raw_without_crc + bytes((crc8(raw_without_crc),))
    return cobs_encode(raw) + b"\x00"


def encode_message(
    message_type: MessageType, sequence: int, payload: bytes = b""
) -> bytes:
    if message_type in (MessageType.CONTROL, MessageType.ESTOP_ASSERT):
        raise ValueError("use the dedicated fast-frame encoder")
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD} bytes")
    raw = bytes(
        (PROTOCOL_VERSION, int(message_type), sequence & 0xFF, len(payload))
    ) + payload
    return _frame(raw)


def _decode_raw(packet: bytes) -> bytes:
    encoded = packet[:-1] if packet.endswith(b"\x00") else packet
    if not encoded:
        raise ValueError("empty frame")
    raw = cobs_decode(encoded)
    if len(raw) < 2 or crc8(raw[:-1]) != raw[-1]:
        raise ValueError("invalid frame length or checksum")
    return raw


def decode_message(packet: bytes) -> Message:
    raw = _decode_raw(packet)
    discriminator = raw[0] & 0xC0
    if discriminator == 0x40:
        if len(raw) != 9:
            raise ValueError("invalid compact Control frame length")
        return Message(MessageType.CONTROL, raw[0] & 0x3F, raw[1:8])
    if discriminator == 0x80:
        if len(raw) != 2:
            raise ValueError("invalid emergency-stop frame length")
        return Message(MessageType.ESTOP_ASSERT, raw[0] & 0x3F, b"")
    if len(raw) < 5:
        raise ValueError("invalid generic frame length")
    version, raw_type, sequence, payload_length = raw[:4]
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version: {version}")
    if payload_length > MAX_PAYLOAD or len(raw) != payload_length + 5:
        raise ValueError("payload length does not match envelope")
    try:
        message_type = MessageType(raw_type)
    except ValueError as exc:
        raise ValueError(f"unknown message type: {raw_type:#x}") from exc
    return Message(message_type, sequence, raw[4:-1])


class ProtocolDecoder:
    """Bounded incremental decoder; malformed packets cannot poison the next."""

    def __init__(self, maximum_encoded_length: int = MAX_ENCODED_LENGTH) -> None:
        self._buffer = bytearray()
        self._discarding = False
        self.maximum_encoded_length = maximum_encoded_length
        self.malformed_frames = 0
        self.overflows = 0

    def feed(self, data: bytes) -> list[Message]:
        messages: list[Message] = []
        for value in data:
            if value == 0:
                if self._discarding:
                    self._discarding = False
                    self._buffer.clear()
                    continue
                if self._buffer:
                    try:
                        messages.append(decode_message(bytes(self._buffer)))
                    except ValueError:
                        self.malformed_frames += 1
                    self._buffer.clear()
            elif self._discarding:
                continue
            elif len(self._buffer) < self.maximum_encoded_length:
                self._buffer.append(value)
            else:
                self._buffer.clear()
                self._discarding = True
                self.overflows += 1
        return messages


def _axis_percent(value: Any, name: str) -> int:
    permille = _finite_int(value, -1000, 1000, name)
    # Python's round uses ties-to-even, which is deterministic on both signs
    # and avoids a directional bias around half-percentage inputs.
    return int(round(permille / 10.0))


def _ternary_code(value: Any, name: str) -> int:
    signed = _finite_int(value, -1000, 1000, name)
    return 1 if signed < 0 else (2 if signed > 0 else 0)


def _decode_ternary(code: int, name: str, magnitude: int) -> int:
    if code == 0:
        return 0
    if code == 1:
        return -magnitude
    if code == 2:
        return magnitude
    raise ValueError(f"reserved {name} ternary code")


def encode_control_frame(frame: ControlFrame) -> bytes:
    axes = (
        _axis_percent(frame.forward, "forward"),
        _axis_percent(frame.turn, "turn"),
        _axis_percent(frame.strafe, "strafe"),
        _axis_percent(frame.arm_yaw, "arm_yaw"),
        _axis_percent(frame.arm_reach, "arm_reach"),
    )
    if (
        isinstance(frame.buttons, bool)
        or not isinstance(frame.buttons, int)
        or not 0 <= frame.buttons <= 0x3F
    ):
        raise ValueError("buttons must fit the six defined action bits")
    packed_ternary = _ternary_code(frame.arm_height, "arm_height")
    packed_ternary |= _ternary_code(frame.gripper, "gripper") << 2
    raw = bytes((0x40 | (frame.sequence & 0x3F),))
    raw += struct.pack("<5b", *axes)
    raw += bytes((packed_ternary, frame.buttons))
    return _frame(raw)


def decode_control_frame(packet: bytes) -> ControlFrame:
    raw = _decode_raw(packet)
    if len(raw) != 9 or raw[0] & 0xC0 != 0x40:
        raise ValueError("not a valid compact Control frame")
    axes = struct.unpack("<5b", raw[1:6])
    if any(abs(value) > 100 for value in axes):
        raise ValueError("Control axis is outside -100..100")
    packed = raw[6]
    if packed & 0xF0:
        raise ValueError("reserved Control ternary bits are set")
    if raw[7] & 0xC0:
        raise ValueError("reserved Control action bits are set")
    return ControlFrame(
        sequence=raw[0] & 0x3F,
        forward=axes[0] * 10,
        turn=axes[1] * 10,
        strafe=axes[2] * 10,
        arm_yaw=axes[3] * 10,
        arm_reach=axes[4] * 10,
        arm_height=_decode_ternary(packed & 0x03, "arm height", 1000),
        gripper=_decode_ternary((packed >> 2) & 0x03, "gripper", 1),
        buttons=raw[7],
    )


def encode_estop(sequence: int) -> bytes:
    return _frame(bytes((0x80 | (sequence & 0x3F),)))


def encode_cal_arm_move(sequence: int, joint: int, degrees: int) -> bytes:
    payload = bytes(
        (
            _strict_int(joint, 0, 3, "joint"),
            _strict_int(degrees, 0, 180, "degrees"),
        )
    )
    return encode_message(MessageType.CAL_ARM_MOVE, sequence, payload)


def encode_cal_joint_reference(
    sequence: int,
    joint: int,
    lower: int,
    upper: int,
    center_offset: int,
    direction: int,
) -> bytes:
    payload = struct.pack(
        "<BBBbb",
        _strict_int(joint, 0, 3, "joint"),
        _strict_int(lower, 0, 180, "lower"),
        _strict_int(upper, 0, 180, "upper"),
        _strict_int(center_offset, -90, 90, "center_offset"),
        _strict_int(direction, -1, 1, "direction"),
    )
    if payload[-1] not in (1, 0xFF):
        raise ValueError("direction must be -1 or 1")
    return encode_message(MessageType.CAL_SET_JOINT_REFERENCE, sequence, payload)


def encode_cal_drive_spin(
    sequence: int,
    mode: int,
    channel: int,
    value: int,
    duration_ms: int,
) -> bytes:
    payload = struct.pack(
        "<BBhH",
        _strict_int(mode, 0, 1, "mode"),
        _strict_int(channel, 0, 3, "channel"),
        _strict_int(value, -32768, 32767, "value"),
        _strict_int(duration_ms, 0, 10000, "duration_ms"),
    )
    return encode_message(MessageType.CAL_DRIVE_SPIN, sequence, payload)


def encode_cal_read_drive(sequence: int, page: int) -> bytes:
    return encode_message(
        MessageType.CAL_READ_DRIVE,
        sequence,
        bytes((_strict_int(page, 0, 1, "page"),)),
    )


def decode_message_data(message: Message) -> dict[str, Any]:
    """Decode the small set of host-visible v3 replies into named fields."""

    payload = message.payload
    if message.message_type == MessageType.HELLO_RESPONSE and len(payload) == 8:
        return {
            "kind": "hello",
            "profile": payload[0],
            "arm_enabled": bool(payload[1]),
            "drive_enabled": bool(payload[2]),
            "sensor_enabled": bool(payload[3]),
            "driver_mode": payload[4],
            "arm_calibrated": bool(payload[5]),
            "drive_calibrated": bool(payload[6]),
            "baud": payload[7] * 1200,
        }
    if message.message_type == MessageType.CRITICAL_STATUS and len(payload) == 7:
        return {
            "kind": "critical_status",
            "state": payload[0],
            "faults": int.from_bytes(payload[1:3], "little"),
            "warnings": int.from_bytes(payload[3:5], "little"),
            "last_accepted_control_sequence": payload[5],
            "link_alive": bool(payload[6]),
        }
    if message.message_type == MessageType.CAL_REPORT and payload:
        kind = payload[0]
        if kind == CalibrationReportKind.ARM and len(payload) == 5:
            return {"kind": "cal_arm", "servo_targets": list(payload[1:5])}
        if kind == CalibrationReportKind.DRIVE_COUNTS and len(payload) == 26:
            return {
                "kind": "cal_drive_counts",
                "raw_increments": list(struct.unpack("<4h", payload[1:9])),
                "totals": list(struct.unpack("<4i", payload[9:25])),
                "valid_mask": payload[25],
            }
        if kind == CalibrationReportKind.DRIVE_SPEED and len(payload) == 10:
            return {
                "kind": "cal_drive_speed",
                "measured_speeds": list(struct.unpack("<4h", payload[1:9])),
                "valid_mask": payload[9],
            }
        if kind == CalibrationReportKind.SENSOR and len(payload) == 14:
            return {
                "kind": "cal_sensor",
                "distances_mm": list(struct.unpack("<6H", payload[1:13])),
                "valid_mask": payload[13],
            }
        if kind == CalibrationReportKind.SYSTEM and len(payload) == 3:
            return {
                "kind": "cal_system",
                "minimum_untouched_stack_bytes": int.from_bytes(
                    payload[1:3], "little"
                ),
            }
    if message.message_type == MessageType.ACK and len(payload) == 1:
        return {"kind": "ack", "acknowledged_type": payload[0]}
    if message.message_type == MessageType.NACK and len(payload) == 2:
        return {
            "kind": "nack",
            "rejected_type": payload[0],
            "reason": payload[1],
        }
    return {"kind": "unknown", "type": int(message.message_type)}
