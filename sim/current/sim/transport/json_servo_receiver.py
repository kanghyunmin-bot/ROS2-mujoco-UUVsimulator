"""UDP endpoint state for ArduPilot JSON-SITL servo packets."""

from __future__ import annotations

from .json_servo_endpoint import JsonServoEndpointState, UdpAddress
from .json_servo_receiver_config import positive_env_int
from .json_servo_receiver_socket import JsonServoReceiverSocketMixin
from .json_servo_receiver_state import JsonServoReceiverStateMixin


class JsonServoReceiver(JsonServoReceiverStateMixin, JsonServoReceiverSocketMixin):
    """Own the JSON-SITL UDP socket without interpreting control semantics."""

    def __init__(
        self,
        *,
        listen_addr: UdpAddress,
        servo_target: UdpAddress,
        sensor_target: UdpAddress,
        recv_buffer_bytes: int | None = None,
        send_buffer_bytes: int | None = None,
    ) -> None:
        self.state = JsonServoEndpointState(
            listen_addr=listen_addr,
            servo_target=servo_target,
            sensor_target=sensor_target,
        )
        self._recv_buffer_bytes = int(recv_buffer_bytes or positive_env_int("ROS2_UUV_SITL_RCVBUF", 1024 * 1024))
        self._send_buffer_bytes = int(send_buffer_bytes or positive_env_int("ROS2_UUV_SITL_SNDBUF", 1024 * 1024))


__all__ = [
    "JsonServoEndpointState",
    "JsonServoReceiver",
    "UdpAddress",
]
