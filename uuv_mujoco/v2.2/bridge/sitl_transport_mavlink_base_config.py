"""Base MAVLink endpoint and source-system configuration for SitlTransport."""

from __future__ import annotations

import numpy as np

from bridge.sitl_env import env_flag


def initialize_mavlink_endpoint_state(
    transport: object,
    *,
    sitl_mavlink_endpoint: str,
    sitl_mavlink_servo_hz: float,
    sitl_mavlink_target_sysid: int,
    sitl_mavlink_target_compid: int,
    sitl_mavlink_source_sysid: int,
    sitl_mavlink_source_compid: int,
) -> None:
    transport._sitl_mavlink_endpoint = str(sitl_mavlink_endpoint or "").strip()
    requested_servo_hz = float(sitl_mavlink_servo_hz)
    transport._sitl_mavlink_servo_hz = float(np.clip(requested_servo_hz, 1.0, 20.0))
    if requested_servo_hz > transport._sitl_mavlink_servo_hz + 1e-6:
        print(
            "[sitl_transport] sitl_mavlink_servo_hz clamped to "
            f"{transport._sitl_mavlink_servo_hz:.1f}Hz (requested {requested_servo_hz:.1f}Hz) "
            "to avoid ArduSub message-rate overrun.",
            flush=True,
        )
    transport._sitl_request_servo_interval = (
        not transport._sitl_json_servo_fallback
        or env_flag("ROS2_UUV_SITL_FORCE_SERVO_INTERVAL", False)
    )
    transport._sitl_mavlink_target_sysid = max(0, min(255, int(sitl_mavlink_target_sysid)))
    transport._sitl_mavlink_target_compid = max(0, min(255, int(sitl_mavlink_target_compid)))
    transport._sitl_mavlink_source_system = max(1, min(255, int(sitl_mavlink_source_sysid or 254)))
    transport._sitl_mavlink_source_component = max(1, min(255, int(sitl_mavlink_source_compid or 190)))


def initialize_mavlink_connection_state(transport: object) -> None:
    transport._sitl_mav = None
    transport._sitl_mavutil = None
    transport._sitl_mav_hb = None
    transport._sitl_mav_last_hb_wall = -1.0
    transport._sitl_mav_last_msg_wall = -1.0
    transport._sitl_mav_last_req_wall = -1.0
    transport._sitl_mav_last_heartbeat_send_wall = -1.0
    transport._sitl_mav_last_wait_warn_wall = -1.0
    transport._sitl_mav_target_mismatch_warn_wall = -1.0
    transport._sitl_mav_wait_warn_interval_s = 3.0


__all__ = ["initialize_mavlink_endpoint_state", "initialize_mavlink_connection_state"]
