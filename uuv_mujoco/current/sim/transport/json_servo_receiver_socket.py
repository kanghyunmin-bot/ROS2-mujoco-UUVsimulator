"""Socket operations for JSON-SITL servo receivers."""

from __future__ import annotations

import socket

from .json_servo import JsonServoPacket
from .json_servo_endpoint import UdpAddress
from .json_servo_receiver_io import (
    bind_json_servo_socket,
    close_json_servo_socket,
    default_json_servo_send_target,
    receive_json_servo_packets,
    send_json_servo_bytes,
)


class JsonServoReceiverSocketMixin:
    def connect(self) -> socket.socket:
        """Bind the nonblocking UDP socket used by ArduPilot JSON SITL."""
        if self.state.socket is not None:
            return self.state.socket
        sock = bind_json_servo_socket(
            listen_addr=self.state.listen_addr,
            recv_buffer_bytes=self._recv_buffer_bytes,
            send_buffer_bytes=self._send_buffer_bytes,
        )
        self.state.socket = sock
        return sock

    def close(self) -> None:
        close_json_servo_socket(self.state)

    def receive_packets(self, max_packets: int, *, packet_size: int = 2048) -> list[tuple[JsonServoPacket, UdpAddress]]:
        """Drain and decode available JSON servo packets from the UDP socket."""
        return receive_json_servo_packets(
            sock=self.state.socket,
            max_packets=max_packets,
            packet_size=packet_size,
        )

    def default_send_target(self) -> UdpAddress:
        return default_json_servo_send_target(self.state)

    def send_bytes(self, payload: bytes, *, target: UdpAddress | None = None) -> tuple[int, UdpAddress]:
        """Send a JSON sensor payload and update send attempt telemetry."""
        return send_json_servo_bytes(state=self.state, payload=payload, target=target)


__all__ = ["JsonServoReceiverSocketMixin"]
