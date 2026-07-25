"""Receive loop for ArduPilot JSON-SITL servo packets."""

from __future__ import annotations

import socket

from .json_servo import JsonServoPacket
from .json_servo_endpoint import UdpAddress
from .json_servo_receiver_decode import decode_received_servo_packet, recv_udp_packet


def receive_json_servo_packets(
    *,
    sock: socket.socket | None,
    max_packets: int,
    packet_size: int,
) -> list[tuple[JsonServoPacket, UdpAddress]]:
    if sock is None:
        return []
    packets: list[tuple[JsonServoPacket, UdpAddress]] = []
    for _ in range(max(0, int(max_packets))):
        received = recv_udp_packet(sock, packet_size)
        if received is None:
            break
        decoded = decode_received_servo_packet(*received)
        if decoded is None:
            continue
        packets.append(decoded)
    return packets


__all__ = ["receive_json_servo_packets"]
