"""State proxy properties for JSON-SITL servo receivers."""

from __future__ import annotations

import socket

from .json_servo_endpoint import UdpAddress


class JsonServoReceiverStateMixin:
    @property
    def socket(self) -> socket.socket | None:
        return self.state.socket

    @property
    def client_addr(self) -> UdpAddress | None:
        return self.state.client_addr

    @client_addr.setter
    def client_addr(self, value: UdpAddress | None) -> None:
        self.state.client_addr = value

    @property
    def send_target(self) -> UdpAddress | None:
        return self.state.send_target

    @property
    def send_counter(self) -> int:
        return self.state.send_counter


__all__ = ["JsonServoReceiverStateMixin"]
