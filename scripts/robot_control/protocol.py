"""Protocol v2 framing, typed messages, telemetry decoding, and parameters."""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum
import math
import struct
from typing import Any, Iterable

PROTOCOL_VERSION = 2
MAX_PAYLOAD = 27


class MessageType(IntEnum):
    HELLO = 0x01
    CONTROL = 0x02
    ARM = 0x03
    DISARM = 0x04
    CLEAR_ESTOP = 0x05
    CLEAR_FAULT = 0x06
    PARAMETER_SET = 0x10
    CALIBRATION_COMMAND = 0x11
    PARAMETER_SNAPSHOT_REQUEST = 0x12
    HELLO_TELEMETRY = 0x80
    STATUS_TELEMETRY = 0x81
    DRIVE_COMMAND_TELEMETRY = 0x82
    DRIVE_FEEDBACK_TELEMETRY = 0x83
    ENCODER_TOTALS_TELEMETRY = 0x84
    SCHEDULER_TELEMETRY = 0x85
    SENSOR_ARM_TELEMETRY = 0x86
    OPEN_LOOP_PWM_TELEMETRY = 0x87
    PARAMETER_SNAPSHOT = 0x90
    ACK = 0x91
    NACK = 0x92


class ParameterGroup(IntEnum):
    SERVO = 1
    OPEN_LOOP_MOTOR = 2
    CHASSIS_SPEED = 3
    CHASSIS_ACCELERATION = 4
    ENCODER = 5
    SENSOR = 6
    ASSIST = 7
    RESPONSE_PROFILE = 8
    UART_OPEN_LOOP = 9
    ARM_GEOMETRY = 10


class ControlFlag(IntEnum):
    ESTOP_ASSERTED = 1 << 0


@dataclass(frozen=True)
class ControlFrame:
    sequence: int
    forward: int = 0
    turn: int = 0
    strafe: int = 0
    arm_yaw: int = 0
    arm_reach: int = 0
    arm_height: int = 0
    gripper: int = 0
    buttons: int = 0
    control_flags: int = 0

    def neutral(self) -> bool:
        return (
            max(
                abs(self.forward), abs(self.turn), abs(self.strafe),
                abs(self.arm_yaw), abs(self.arm_reach), abs(self.arm_height),
            ) <= 30
            and self.gripper == 0
            and not self.control_flags
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
    converted = int(round(value))
    return max(low, min(high, converted))


def _validated_int(value: Any, low: int, high: int, name: str) -> int:
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{name} must be a finite number")
    converted = int(round(value))
    if converted < low or converted > high:
        raise ValueError(f"{name} must be between {low} and {high}")
    return converted


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


def encode_message(message_type: MessageType, sequence: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError(f"payload exceeds {MAX_PAYLOAD} bytes")
    raw = bytes((PROTOCOL_VERSION, int(message_type), sequence & 0xFF, len(payload))) + payload
    return cobs_encode(raw + bytes((crc8(raw),))) + b"\x00"


def decode_message(packet: bytes) -> Message:
    encoded = packet[:-1] if packet.endswith(b"\x00") else packet
    raw = cobs_decode(encoded)
    if len(raw) < 5 or crc8(raw[:-1]) != raw[-1]:
        raise ValueError("invalid frame length or checksum")
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
    """Bounded incremental decoder; malformed packets never contaminate the next one."""

    def __init__(self, maximum_encoded_length: int = 36) -> None:
        self._buffer = bytearray()
        self.maximum_encoded_length = maximum_encoded_length
        self.malformed_frames = 0
        self.overflows = 0

    def feed(self, data: bytes) -> list[Message]:
        messages: list[Message] = []
        for value in data:
            if value == 0:
                if self._buffer:
                    try:
                        messages.append(decode_message(bytes(self._buffer)))
                    except ValueError:
                        self.malformed_frames += 1
                    self._buffer.clear()
            elif len(self._buffer) < self.maximum_encoded_length:
                self._buffer.append(value)
            else:
                self._buffer.clear()
                self.overflows += 1
        return messages


def encode_control_frame(frame: ControlFrame) -> bytes:
    payload = struct.pack(
        "<hhhhhhbHB",
        *(
            _finite_int(value, -1000, 1000, name)
            for value, name in (
                (frame.forward, "forward"), (frame.turn, "turn"),
                (frame.strafe, "strafe"), (frame.arm_yaw, "arm_yaw"),
                (frame.arm_reach, "arm_reach"), (frame.arm_height, "arm_height"),
            )
        ),
        _finite_int(frame.gripper, -1, 1, "gripper"),
        int(frame.buttons) & 0xFFFF,
        int(frame.control_flags) & int(ControlFlag.ESTOP_ASSERTED),
    )
    return encode_message(MessageType.CONTROL, frame.sequence, payload)


def decode_control_frame(packet: bytes) -> ControlFrame:
    message = decode_message(packet)
    if message.message_type != MessageType.CONTROL or len(message.payload) != 16:
        raise ValueError("not a valid Control message")
    values = struct.unpack("<hhhhhhbHB", message.payload)
    return ControlFrame(
        sequence=message.sequence,
        forward=values[0], turn=values[1], strafe=values[2],
        arm_yaw=values[3], arm_reach=values[4], arm_height=values[5],
        gripper=values[6], buttons=values[7], control_flags=values[8],
    )


def encode_parameter_set(
    sequence: int,
    group: ParameterGroup,
    index: int,
    expected_revision: int,
    data: bytes,
) -> bytes:
    payload = struct.pack("<BBH", int(group), index, expected_revision & 0xFFFF) + data
    return encode_message(MessageType.PARAMETER_SET, sequence, payload)


def build_parameter_data(
    group: ParameterGroup, index: int, values: dict[str, Any]
) -> bytes:
    """Validate and encode the semantic parameter forms exposed by the GUI."""
    i16 = lambda key, lo, hi: _validated_int(values[key], lo, hi, key)
    if group == ParameterGroup.SERVO and 0 <= index < 4:
        lower = i16("lower", 0, 179)
        upper = i16("upper", 1, 180)
        if lower >= upper:
            raise ValueError("servo lower limit must be below the upper limit")
        direction = i16("direction", -1, 1)
        if direction not in (-1, 1):
            raise ValueError("servo direction must be -1 or 1")
        return struct.pack(
            "<BBbb", lower, upper,
            i16("center_offset", -90, 90), direction,
        )
    if group == ParameterGroup.OPEN_LOOP_MOTOR and 0 <= index < 4:
        minimum = i16("minimum_pwm", 0, 255)
        maximum = i16("maximum_pwm", 0, 255)
        if minimum > maximum:
            raise ValueError("minimum PWM must not exceed maximum PWM")
        direction = i16("direction", -1, 1)
        if direction not in (-1, 1):
            raise ValueError("motor direction must be -1 or 1")
        return struct.pack("<BBb", minimum, maximum, direction)
    if group == ParameterGroup.UART_OPEN_LOOP and 0 <= index < 4:
        minimum = i16("minimum_percent", 0, 100)
        maximum = i16("maximum_percent", 0, 100)
        if minimum > maximum:
            raise ValueError("minimum percentage must not exceed maximum percentage")
        direction = i16("direction", -1, 1)
        if direction not in (-1, 1):
            raise ValueError("UART PWM direction must be -1 or 1")
        return struct.pack("<BBb", minimum, maximum, direction)
    if group == ParameterGroup.CHASSIS_SPEED and index == 0:
        return struct.pack(
            "<hhhhH", i16("forward", 0, 1000), i16("reverse", 0, 1000),
            i16("lateral", 0, 1000), i16("yaw", 0, 3000),
            i16("wheel", 1, 1000),
        )
    if group == ParameterGroup.CHASSIS_ACCELERATION and index == 0:
        translation_keys = (
            "forward_accel", "forward_decel", "reverse_accel", "reverse_decel",
            "lateral_accel", "lateral_decel",
        )
        rotation_keys = ("rotation_accel", "rotation_decel")
        return struct.pack(
            "<8H",
            *(i16(key, 0, 3000) for key in translation_keys),
            *(i16(key, 0, 8000) for key in rotation_keys),
        )
    if group == ParameterGroup.CHASSIS_ACCELERATION and index == 1:
        return struct.pack(
            "<3H", i16("zero_hold_ms", 0, 500),
            i16("translation_threshold", 0, 100),
            i16("rotation_threshold", 0, 200),
        )
    if group == ParameterGroup.ENCODER and index == 0:
        return struct.pack(
            "<4HB", i16("wheel_diameter_mm", 1, 300),
            i16("counts_per_revolution", 1, 60000),
            i16("wheel_track_mm", 1, 1000), i16("wheelbase_mm", 1, 1000),
            i16("semantics", 0, 1),
        )
    if group == ParameterGroup.ENCODER and index in (1, 2):
        mapping = [i16(f"map_{wheel}", 0, 3) for wheel in range(4)]
        signs = [i16(f"sign_{wheel}", -1, 1) for wheel in range(4)]
        if sorted(mapping) != [0, 1, 2, 3]:
            raise ValueError("encoder and command maps must be permutations of 0..3")
        if any(sign not in (-1, 1) for sign in signs):
            raise ValueError("encoder and command signs must be -1 or 1")
        return struct.pack("<8b", *(mapping + signs))
    if group == ParameterGroup.SENSOR and 0 <= index < 6:
        return struct.pack("<h", i16("offset_mm", -500, 500))
    if group == ParameterGroup.ASSIST and index == 0:
        normal = i16("normal_limit", 0, 1000)
        cargo = i16("cargo_limit", 0, 1000)
        assist = i16("assist_limit", 0, 1000)
        if not assist <= cargo <= normal:
            raise ValueError("assist limit must be at most cargo, and cargo at most normal")
        return struct.pack("<3H", normal, cargo, assist)
    if group == ParameterGroup.ARM_GEOMETRY and index == 0:
        return struct.pack(
            "<8H", i16("first_link_mm", 20, 300),
            i16("second_link_mm", 20, 300), i16("shoulder_height_mm", 0, 500),
            i16("gripper_offset_mm", 0, 200), i16("minimum_reach_mm", 0, 500),
            i16("maximum_reach_mm", 0, 500), i16("minimum_height_mm", 0, 500),
            i16("maximum_height_mm", 0, 500),
        )
    if group == ParameterGroup.ARM_GEOMETRY and index == 1:
        keys = (
            "clearance_height_mm", "preset_reach_mm", "preset_height_mm",
            "stow_reach_mm", "stow_height_mm",
        )
        return struct.pack("<5H", *(i16(key, 0, 500) for key in keys))
    if group == ParameterGroup.RESPONSE_PROFILE:
        if index == 0:
            return struct.pack("<B", i16("profile", 0, 2))
        if 1 <= index <= 3:
            return struct.pack(
                "<3H", i16("speed_permille", 0, 1000),
                i16("acceleration_permille", 0, 1500),
                i16("deceleration_permille", 0, 1500),
            )
    raise ValueError(f"unsupported parameter group/index: {group.name}/{index}")


def _i16s(payload: bytes, offset: int, count: int) -> tuple[int, ...]:
    return struct.unpack_from(f"<{count}h", payload, offset)


def telemetry_to_dict(message: Message) -> dict[str, Any]:
    """Decode a firmware telemetry message into JSON-safe named fields."""
    p = message.payload
    t = message.message_type
    if t == MessageType.HELLO_TELEMETRY and len(p) == 8:
        return {
            "kind": "hello", "profile": p[0], "control_mode": p[1],
            "pwm_unit": p[2], "capability_flags": p[3],
            "revision": int.from_bytes(p[4:6], "little"),
            "bluetooth_baud": p[6] * 1200,
        }
    if t == MessageType.STATUS_TELEMETRY and len(p) == 14:
        return {
            "kind": "status", "state": p[0], "assist_stage": p[1],
            "faults": int.from_bytes(p[2:4], "little"),
            "warnings": int.from_bytes(p[4:6], "little"),
            "command_age_ms": int.from_bytes(p[6:8], "little"),
            "status_flags": p[8], "response_profile": p[9],
            "revision": int.from_bytes(p[10:12], "little"),
            "firmware_malformed_frames": int.from_bytes(p[12:14], "little"),
        }
    if t == MessageType.DRIVE_COMMAND_TELEMETRY and len(p) == 26:
        values = _i16s(p, 3, 10)
        return {
            "kind": "drive_command", "control_mode": p[0], "response_profile": p[1],
            "zero_crossing_mask": p[2],
            "requested_chassis": list(values[0:3]), "ramped_chassis": list(values[3:6]),
            "controller_targets": list(values[6:10]), "pwm_valid": bool(p[23]),
            "motor_command_age_ms": int.from_bytes(p[24:26], "little"),
        }
    if t == MessageType.DRIVE_FEEDBACK_TELEMETRY and len(p) == 27:
        measured = list(_i16s(p, 0, 4))
        errors = list(_i16s(p, 8, 4))
        return {
            "kind": "drive_feedback", "measured_speeds": measured,
            "speed_errors": errors,
            "logical_targets": [measured[i] + errors[i] for i in range(4)],
            "encoder_valid_mask": p[16],
            "error_valid_mask": p[17], "total_valid_mask": p[18],
            "feedback_age_ms": int.from_bytes(p[19:21], "little"),
            "sample_interval_ms": int.from_bytes(p[21:23], "little"),
            "encoder_semantics": p[23], "outstanding_query": p[24],
            "query_age_ms": int.from_bytes(p[25:27], "little"),
        }
    if t == MessageType.ENCODER_TOTALS_TELEMETRY and len(p) == 27:
        return {
            "kind": "encoder_totals", "raw_increments": list(_i16s(p, 0, 4)),
            "totals": list(struct.unpack_from("<4i", p, 8)), "valid_mask": p[24],
            "age_ms": int.from_bytes(p[25:27], "little"),
        }
    if t == MessageType.SCHEDULER_TELEMETRY and len(p) == 26:
        values = struct.unpack_from("<11H", p, 4)
        return {
            "kind": "scheduler", "max_loop_gap_us": int.from_bytes(p[0:4], "little"),
            "missed": {
                "chassis": values[0], "motor": values[1], "encoder": values[2],
                "servo": values[3], "sonar": values[4], "telemetry": values[5],
            },
            "query_timeouts": values[6], "motor_uart_overflows": values[7],
            "dropped_telemetry": values[8], "motion_dt_clamps": values[9],
            "host_rx_overflows": values[10],
        }
    if t == MessageType.SENSOR_ARM_TELEMETRY and len(p) == 24:
        return {
            "kind": "sensor_arm", "sensor_mm": list(struct.unpack_from("<6H", p)),
            "sensor_valid_mask": p[12], "servo_targets": list(p[13:17]),
            "arm_calibrated": bool(p[17]), "cargo_may_be_held": bool(p[18]),
            "battery_mv": int.from_bytes(p[19:21], "little"),
            "battery_valid": bool(p[21]),
            "battery_age_ms": int.from_bytes(p[22:24], "little"),
        }
    if t == MessageType.OPEN_LOOP_PWM_TELEMETRY and len(p) == 10:
        return {
            "kind": "open_loop_pwm", "pwm_unit": p[0], "valid_mask": p[1],
            "commands": list(_i16s(p, 2, 4)),
        }
    if t == MessageType.PARAMETER_SNAPSHOT and len(p) >= 4:
        return {
            "kind": "parameter", "group": p[0], "index": p[1],
            "revision": int.from_bytes(p[2:4], "little"), "data": list(p[4:]),
        }
    if t == MessageType.ACK and len(p) == 3:
        return {
            "kind": "ack", "acknowledged_type": p[0],
            "revision": int.from_bytes(p[1:3], "little"),
        }
    if t == MessageType.NACK and len(p) == 2:
        return {"kind": "nack", "rejected_type": p[0], "reason": p[1]}
    return {"kind": "unknown", "message_type": int(t), "payload": list(p)}


def encode_simple(message_type: MessageType, sequence: int) -> bytes:
    return encode_message(message_type, sequence)


def split_packets(data: Iterable[int]) -> list[bytes]:
    """Test/helper utility for splitting a byte iterable at protocol delimiters."""
    packets: list[bytes] = []
    current = bytearray()
    for value in data:
        current.append(value)
        if value == 0:
            packets.append(bytes(current))
            current.clear()
    return packets
