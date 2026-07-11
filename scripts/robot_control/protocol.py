"""Versioned compact Bluetooth control protocol."""

from __future__ import annotations

from dataclasses import dataclass
import struct

PROTOCOL_VERSION = 1
RAW_FRAME_FORMAT = "<BBhhhhhhbH"
RAW_FRAME_LENGTH = struct.calcsize(RAW_FRAME_FORMAT) + 1


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


def _clamp_axis(value: int) -> int:
    return max(-1000, min(1000, int(value)))


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
            raise ValueError("Truncated COBS packet")
        output.extend(data[index:end])
        index = end
        if code != 0xFF and index < len(data):
            output.append(0)
    return bytes(output)


def encode_control_frame(frame: ControlFrame) -> bytes:
    raw = struct.pack(
        RAW_FRAME_FORMAT,
        PROTOCOL_VERSION,
        frame.sequence & 0xFF,
        _clamp_axis(frame.forward),
        _clamp_axis(frame.turn),
        _clamp_axis(frame.strafe),
        _clamp_axis(frame.arm_yaw),
        _clamp_axis(frame.arm_reach),
        _clamp_axis(frame.arm_height),
        max(-1, min(1, frame.gripper)),
        frame.buttons & 0xFFFF,
    )
    return cobs_encode(raw + bytes([crc8(raw)])) + b"\x00"


def decode_control_frame(packet: bytes) -> ControlFrame:
    encoded = packet[:-1] if packet.endswith(b"\x00") else packet
    raw = cobs_decode(encoded)
    if len(raw) != RAW_FRAME_LENGTH or crc8(raw[:-1]) != raw[-1]:
        raise ValueError("Invalid control frame length or checksum")
    values = struct.unpack(RAW_FRAME_FORMAT, raw[:-1])
    if values[0] != PROTOCOL_VERSION:
        raise ValueError(f"Unsupported control protocol version: {values[0]}")
    return ControlFrame(
        sequence=values[1], forward=values[2], turn=values[3], strafe=values[4],
        arm_yaw=values[5], arm_reach=values[6], arm_height=values[7],
        gripper=values[8], buttons=values[9],
    )

