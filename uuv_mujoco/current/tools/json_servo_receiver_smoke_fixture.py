"""Fixtures for JSON servo receiver smoke tests."""

from __future__ import annotations

import socket
import struct
import sys
import time
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.transport.json_servo import JSON_SERVO_MAGIC_16, JSON_SERVO_HEADER_SIZE
from sim.transport.json_servo_receiver import JsonServoReceiver


def make_servo_packet(pwm_values: list[int], *, frame_rate_hz: int = 400, frame_count: int = 7) -> bytes:
    if len(pwm_values) != 16:
        raise AssertionError("JSON_SERVO_MAGIC_16 packets require 16 channels")
    header = (
        int(JSON_SERVO_MAGIC_16).to_bytes(2, "little")
        + int(frame_rate_hz).to_bytes(2, "little")
        + int(frame_count).to_bytes(4, "little")
    )
    if len(header) != JSON_SERVO_HEADER_SIZE:
        raise AssertionError("unexpected JSON servo header size")
    return header + struct.pack("<16H", *[int(value) for value in pwm_values])


def make_receiver() -> JsonServoReceiver:
    return JsonServoReceiver(
        listen_addr=("127.0.0.1", 0),
        servo_target=("127.0.0.1", 0),
        sensor_target=("127.0.0.1", 0),
        recv_buffer_bytes=8192,
        send_buffer_bytes=8192,
    )


def receive_until(receiver: JsonServoReceiver, expected_count: int) -> list[Any]:
    deadline = time.monotonic() + 1.0
    packets: list[Any] = []
    while time.monotonic() < deadline and len(packets) < expected_count:
        packets.extend(receiver.receive_packets(8))
        if len(packets) >= expected_count:
            break
        time.sleep(0.01)
    return packets


def udp_socket() -> socket.socket:
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


__all__ = ["make_receiver", "make_servo_packet", "receive_until", "udp_socket"]
