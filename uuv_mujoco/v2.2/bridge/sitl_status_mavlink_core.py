"""Core MAVLink status fields for SITL telemetry payloads."""

from __future__ import annotations

from .sitl_status_age import wall_age_s


def build_mavlink_core_status_fields(
    *,
    now_wall: float,
    command_endpoint: str,
    command_hb_wall: float,
    servo_hb_wall: float,
    command_servo_msg_count: int,
    command_servo_last_msg_wall: float,
    servo_link_servo_msg_count: int,
    servo_link_servo_last_msg_wall: float,
    rc_override_ready: bool,
    vehicle_armed: bool,
    vehicle_mode: str,
    auto_ready_enabled: bool,
    auto_ready_mode: str,
    auto_ready_state: str,
    auto_ready_done_wall: float,
) -> dict[str, object]:
    return {
        "command_mavlink_endpoint": str(command_endpoint),
        "command_heartbeat_age_s": wall_age_s(now_wall, command_hb_wall),
        "servo_heartbeat_age_s": wall_age_s(now_wall, servo_hb_wall),
        "command_link_servo_msg_count": int(command_servo_msg_count),
        "command_link_servo_age_s": wall_age_s(now_wall, command_servo_last_msg_wall),
        "servo_link_servo_msg_count": int(servo_link_servo_msg_count),
        "servo_link_servo_age_s": wall_age_s(now_wall, servo_link_servo_last_msg_wall),
        "rc_override_ready": bool(rc_override_ready),
        "vehicle_armed": bool(vehicle_armed),
        "vehicle_mode": str(vehicle_mode or ""),
        "auto_ready_enabled": bool(auto_ready_enabled),
        "auto_ready_mode": str(auto_ready_mode),
        "auto_ready_state": str(auto_ready_state),
        "auto_ready_done": bool(auto_ready_done_wall > 0.0 and vehicle_armed),
    }


__all__ = ["build_mavlink_core_status_fields"]
