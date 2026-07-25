"""Endpoint state for ArduPilot JSON-SITL servo transport."""

from __future__ import annotations

import socket
from dataclasses import dataclass


UdpAddress = tuple[str, int]


@dataclass
class JsonServoEndpointState:
    """Mutable UDP endpoint state shared with the runtime bridge."""

    listen_addr: UdpAddress
    servo_target: UdpAddress
    sensor_target: UdpAddress
    socket: socket.socket | None = None
    client_addr: UdpAddress | None = None
    send_target: UdpAddress | None = None
    send_counter: int = 0


__all__ = ["JsonServoEndpointState", "UdpAddress"]
