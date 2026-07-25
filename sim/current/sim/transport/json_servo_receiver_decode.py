"""Packet receive/decode helpers for JSON-SITL servo UDP sockets."""

from __future__ import annotations

import socket

from .json_servo import JsonServoPacket, decode_json_servo_packet
from .json_servo_endpoint import UdpAddress


def recv_udp_packet(sock: socket.socket, packet_size: int) -> tuple[bytes, UdpAddress] | None:
    try:
        raw_packet, addr = sock.recvfrom(int(packet_size))
    except BlockingIOError:
        return None
    except Exception:
        return None
    return raw_packet, (str(addr[0]), int(addr[1]))


def decode_received_servo_packet(raw_packet: bytes, addr: UdpAddress) -> tuple[JsonServoPacket, UdpAddress] | None:
    servo_packet = decode_json_servo_packet(raw_packet)
    if servo_packet is None:
        return None
    return servo_packet, addr


__all__ = ["decode_received_servo_packet", "recv_udp_packet"]
