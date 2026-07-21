"""Socket binding for ArduPilot JSON-SITL servo transport."""

from __future__ import annotations

import socket

from .json_servo_endpoint import UdpAddress


def bind_json_servo_socket(
    *,
    listen_addr: UdpAddress,
    recv_buffer_bytes: int,
    send_buffer_bytes: int,
) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, int(recv_buffer_bytes))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, int(send_buffer_bytes))
        sock.bind(listen_addr)
        sock.setblocking(False)
    except Exception:
        try:
            sock.close()
        except Exception:
            pass
        raise
    return sock


__all__ = ["bind_json_servo_socket"]
