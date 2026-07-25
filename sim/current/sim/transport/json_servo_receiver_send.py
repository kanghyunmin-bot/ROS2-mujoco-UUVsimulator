"""Send bookkeeping for ArduPilot JSON-SITL servo transport."""

from __future__ import annotations

from .json_servo_endpoint import JsonServoEndpointState, UdpAddress


def default_json_servo_send_target(state: JsonServoEndpointState) -> UdpAddress:
    return state.client_addr if state.client_addr is not None else state.sensor_target


def send_json_servo_bytes(
    *,
    state: JsonServoEndpointState,
    payload: bytes,
    target: UdpAddress | None = None,
) -> tuple[int, UdpAddress]:
    send_target = target if target is not None else default_json_servo_send_target(state)
    sock = state.socket
    sent = sock.sendto(payload, send_target) if sock is not None else 0
    state.send_counter += 1
    state.send_target = send_target
    return int(sent), send_target


__all__ = ["default_json_servo_send_target", "send_json_servo_bytes"]
