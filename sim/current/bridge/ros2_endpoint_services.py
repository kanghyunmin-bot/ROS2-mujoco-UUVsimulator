"""ROS2 service endpoint construction."""

from __future__ import annotations


def create_ros2_services(bridge) -> None:
    node = bridge.node
    if bridge._mavros_surface_enabled:
        bridge.srv_mavros_cmd_arming = node.create_service(
            bridge.MavrosCommandBool,
            "/mavros/cmd/arming",
            bridge._on_mavros_cmd_arming,
        )
        bridge.srv_mavros_set_mode = node.create_service(
            bridge.MavrosSetMode,
            "/mavros/set_mode",
            bridge._on_mavros_set_mode,
        )
        bridge.srv_mavros_command_long = node.create_service(
            bridge.MavrosCommandLong,
            "/mavros/cmd/command",
            bridge._on_mavros_command_long,
        )
        return

    bridge.srv_mavros_cmd_arming = None
    bridge.srv_mavros_set_mode = None
    bridge.srv_mavros_command_long = None


__all__ = ["create_ros2_services"]
