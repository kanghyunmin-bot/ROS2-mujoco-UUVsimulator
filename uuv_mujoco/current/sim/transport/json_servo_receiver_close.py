"""Socket close helper for ArduPilot JSON-SITL servo transport."""

from __future__ import annotations

from .json_servo_endpoint import JsonServoEndpointState


def close_json_servo_socket(state: JsonServoEndpointState) -> None:
    sock = state.socket
    state.socket = None
    if sock is not None:
        try:
            sock.close()
        except Exception:
            pass


__all__ = ["close_json_servo_socket"]
