"""ArduPilot JSON-SITL binary servo packet decoding."""

from __future__ import annotations

import struct
from dataclasses import dataclass


JSON_SERVO_MAGIC_16 = 18458
JSON_SERVO_MAGIC_32 = 29569
JSON_SERVO_HEADER_SIZE = 8


@dataclass(frozen=True)
class JsonServoPacket:
    magic: int
    frame_rate_hz: int
    frame_count: int
    pwm_values: list[int]

    @property
    def channel_count(self) -> int:
        return len(self.pwm_values)


def expected_packet_size(magic: int) -> int | None:
    if int(magic) == JSON_SERVO_MAGIC_16:
        return JSON_SERVO_HEADER_SIZE + 16 * 2
    if int(magic) == JSON_SERVO_MAGIC_32:
        return JSON_SERVO_HEADER_SIZE + 32 * 2
    return None


def decode_json_servo_packet(packet: bytes) -> JsonServoPacket | None:
    """Decode the raw ArduPilot JSON-SITL servo UDP packet.

    Returns None for incomplete or unknown packets.  This mirrors the previous
    inline `SitlTransport` behavior and intentionally performs no correction or
    remapping.
    """
    if len(packet) < JSON_SERVO_HEADER_SIZE:
        return None
    magic = int.from_bytes(packet[0:2], byteorder="little", signed=False)
    frame_size = expected_packet_size(magic)
    if frame_size is None or len(packet) < frame_size:
        return None
    frame_rate_hz = int.from_bytes(packet[2:4], byteorder="little", signed=False)
    frame_count = int.from_bytes(packet[4:8], byteorder="little", signed=False)
    fmt = "<16H" if magic == JSON_SERVO_MAGIC_16 else "<32H"
    try:
        pwm_values = list(struct.unpack_from(fmt, packet, JSON_SERVO_HEADER_SIZE))
    except struct.error:
        return None
    return JsonServoPacket(
        magic=magic,
        frame_rate_hz=frame_rate_hz,
        frame_count=frame_count,
        pwm_values=[int(value) for value in pwm_values],
    )


__all__ = [
    "JSON_SERVO_MAGIC_16",
    "JSON_SERVO_MAGIC_32",
    "JSON_SERVO_HEADER_SIZE",
    "JsonServoPacket",
    "expected_packet_size",
    "decode_json_servo_packet",
]
