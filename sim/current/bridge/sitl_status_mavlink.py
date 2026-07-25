"""MAVLink telemetry status payload builder."""

from __future__ import annotations

from typing import Any

from .sitl_status_mavlink_core import build_mavlink_core_status_fields
from .sitl_status_mavlink_extnav import build_extnav_status_fields


def build_mavlink_telemetry_status(
    *,
    base_status: dict[str, Any],
    now_wall: float,
    command_endpoint: str,
    command_hb_wall: float,
    servo_hb_wall: float,
    command_servo_msg_count: int,
    command_servo_last_msg_wall: float,
    servo_link_servo_msg_count: int,
    servo_link_servo_last_msg_wall: float,
    json_servo_frame_count: int | None,
    json_servo_frame_rate_hz: int | None,
    json_servo_last_wall: float,
    json_sensor_packets_sent: int,
    rc_override_ready: bool,
    vehicle_armed: bool,
    vehicle_mode: str,
    auto_ready_enabled: bool,
    auto_ready_mode: str,
    auto_ready_state: str,
    auto_ready_done_wall: float,
    extnav_enabled: bool,
    extnav_required: bool,
    extnav_last_send_wall: float,
    extnav_last_rate_hz: float,
    extnav_min_tx_hz: float,
    extnav_max_stale_s: float,
    extnav_scheduler: str,
    extnav_start_wall: float,
    extnav_grace_s: float,
    extnav_fault: str,
) -> dict[str, object]:
    status = dict(base_status)
    status.update(
        build_mavlink_core_status_fields(
            now_wall=now_wall,
            command_endpoint=command_endpoint,
            command_hb_wall=command_hb_wall,
            servo_hb_wall=servo_hb_wall,
            command_servo_msg_count=command_servo_msg_count,
            command_servo_last_msg_wall=command_servo_last_msg_wall,
            servo_link_servo_msg_count=servo_link_servo_msg_count,
            servo_link_servo_last_msg_wall=servo_link_servo_last_msg_wall,
            json_servo_frame_count=json_servo_frame_count,
            json_servo_frame_rate_hz=json_servo_frame_rate_hz,
            json_servo_last_wall=json_servo_last_wall,
            json_sensor_packets_sent=json_sensor_packets_sent,
            rc_override_ready=rc_override_ready,
            vehicle_armed=vehicle_armed,
            vehicle_mode=vehicle_mode,
            auto_ready_enabled=auto_ready_enabled,
            auto_ready_mode=auto_ready_mode,
            auto_ready_state=auto_ready_state,
            auto_ready_done_wall=auto_ready_done_wall,
        )
    )
    status.update(
        build_extnav_status_fields(
            now_wall=now_wall,
            extnav_enabled=extnav_enabled,
            extnav_required=extnav_required,
            extnav_last_send_wall=extnav_last_send_wall,
            extnav_last_rate_hz=extnav_last_rate_hz,
            extnav_min_tx_hz=extnav_min_tx_hz,
            extnav_max_stale_s=extnav_max_stale_s,
            extnav_scheduler=extnav_scheduler,
            extnav_start_wall=extnav_start_wall,
            extnav_grace_s=extnav_grace_s,
            extnav_fault=extnav_fault,
        )
    )
    return status

__all__ = ["build_mavlink_telemetry_status"]
