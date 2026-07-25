"""MAVLink telemetry observer configuration for SitlTransport."""

from __future__ import annotations

import numpy as np

from bridge.sitl_env import env_flag, env_to_float
from sim.transport import MavlinkMessageIntervalRequester, MavlinkTelemetryObserver


def initialize_mavlink_telemetry_state(transport: object) -> None:
    transport._sitl_rcout_telemetry_hz = float(np.clip(env_to_float("ROS2_UUV_RCOU_TELEMETRY_HZ", 2.0), 0.5, 20.0))
    transport._sitl_ap_telemetry_hz = float(
        np.clip(env_to_float("ROS2_UUV_SITL_AP_TELEMETRY_HZ", 10.0), 0.5, 20.0)
    )
    transport._sitl_command_link_telemetry_enabled = env_flag("ROS2_UUV_COMMAND_LINK_TELEMETRY", True)
    transport._sitl_command_link_ap_telemetry_enabled = env_flag("ROS2_UUV_COMMAND_LINK_AP_TELEMETRY", False)
    transport._sitl_ap_telemetry_last_req_wall = -1.0
    transport._mavlink_interval_requester = MavlinkMessageIntervalRequester()
    transport._mavlink_telemetry_observer = MavlinkTelemetryObserver(
        target_system=transport._sitl_mavlink_target_sysid,
        target_component=transport._sitl_mavlink_target_compid,
        endpoint=transport._sitl_mavlink_endpoint,
        requested_hz=transport._sitl_ap_telemetry_hz,
    )
    transport._sitl_ap_telemetry_status = transport._mavlink_telemetry_observer.status_data
    transport._sitl_cmd_servo_last_msg_wall = -1.0
    transport._sitl_cmd_servo_msg_count = 0
    transport._sitl_servo_link_servo_last_msg_wall = -1.0
    transport._sitl_servo_link_servo_msg_count = 0
    transport._sitl_cmd_servo_last_req_wall = -1.0
    transport._sitl_cmd_ap_telemetry_last_req_wall = -1.0


__all__ = ["initialize_mavlink_telemetry_state"]
