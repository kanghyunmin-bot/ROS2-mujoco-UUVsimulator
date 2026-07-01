"""Command MAVLink link state for SitlTransport."""

from __future__ import annotations

import os

from sim.transport import MavlinkCommandLink


def initialize_command_mavlink_state(transport: object) -> None:
    transport._sitl_cmd_mavlink_endpoint = os.getenv("ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "").strip()
    transport._sitl_cmd_mav = None
    transport._sitl_cmd_mav_hb = None
    transport._sitl_cmd_mav_last_hb_wall = -1.0
    transport._sitl_cmd_mav_last_connect_attempt_wall = -1.0
    transport._sitl_cmd_mav_last_heartbeat_send_wall = -1.0


def initialize_vehicle_command_state(transport: object) -> None:
    transport._sitl_vehicle_armed = False
    transport._sitl_last_vehicle_armed = None
    transport._sitl_vehicle_mode = ""
    transport._sitl_last_vehicle_mode = None
    transport._sitl_pending_arm_target: bool | None = None
    transport._sitl_pending_arm_start_wall = -1.0
    transport._sitl_pending_arm_last_send_wall = -1.0
    transport._sitl_pending_arm_reached_after_wall = -1.0
    transport._sitl_pending_arm_neutral_sent = False
    transport._sitl_last_arm_command_target: bool | None = None
    transport._sitl_last_arm_command_wall = -1.0
    transport._sitl_pending_mode: str = ""
    transport._sitl_pending_mode_start_wall = -1.0
    transport._sitl_pending_mode_last_send_wall = -1.0


def initialize_mavlink_command_links(transport: object) -> None:
    transport._sitl_servo_command_link = MavlinkCommandLink(
        endpoint=transport._sitl_mavlink_endpoint,
        source_system=transport._sitl_mavlink_source_system,
        source_component=transport._sitl_mavlink_source_component,
    )
    transport._sitl_command_link = MavlinkCommandLink(
        endpoint=transport._sitl_cmd_mavlink_endpoint,
        source_system=transport._sitl_mavlink_source_system,
        source_component=transport._sitl_mavlink_source_component,
    )


def initialize_mavlink_command_path_state(transport: object) -> None:
    initialize_command_mavlink_state(transport)
    initialize_vehicle_command_state(transport)
    initialize_mavlink_command_links(transport)


__all__ = [
    "initialize_command_mavlink_state",
    "initialize_vehicle_command_state",
    "initialize_mavlink_command_links",
    "initialize_mavlink_command_path_state",
]
