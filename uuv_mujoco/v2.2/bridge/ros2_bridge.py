"""Lightweight MuJoCo <-> ROS2 bridge with real-robot topic compatibility.

The full topic/service contract lives in
``docs/contracts/ROS2_BRIDGE_SURFACE.md`` so this runtime class stays focused on
construction and method binding.
"""

from __future__ import annotations

from typing import Callable, Optional

import mujoco

from .ros2_bridge_constructor import initialize_ros2_bridge
from .ros2_bridge_method_bindings import bind_ros2_bridge_methods


class Ros2Bridge:
    """Runtime adapter between MuJoCo state and ROS2 topics."""

    _POSITION_TARGET_TYPEMASK_X_IGNORE = 0x01
    _POSITION_TARGET_TYPEMASK_Y_IGNORE = 0x02
    _POSITION_TARGET_TYPEMASK_Z_IGNORE = 0x04
    _POSITION_TARGET_TYPEMASK_YAW_IGNORE = 0x400
    _MAV_CMD_CONDITION_YAW = 115

    def __init__(
        self,
        model: mujoco.MjModel,
        command_callback: Callable[[float, float, float, float], None],
        cmd_limit: float = 15.0,
        publish_images: bool = False,
        image_width: int = 640,
        image_height: int = 360,
        sensor_hz: float = 50.0,
        image_hz: float = 10.0,
        enable_sitl: bool = False,
        sitl_ip: str = "127.0.0.1",
        sitl_port: int = 9002,
        sitl_send_port: int = 9003,
        sitl_mavlink_endpoint: str = "",
        sitl_mavlink_servo_hz: float = 20.0,
        sitl_mavlink_target_sysid: int = 0,
        sitl_mavlink_target_compid: int = 0,
        sitl_mavlink_source_sysid: int = 255,
        sitl_mavlink_source_compid: int = 190,
        camera_calib_left: str = "",
        camera_calib_right: str = "",
        enable_ros: bool = True,
        enable_mavros_surface: bool = True,
        enable_ping360: bool = True,
        ping360_config_path: str = "",
        ping360_overrides: Optional[dict] = None,
    ) -> None:
        init = dict(locals())
        init.pop("self")
        initialize_ros2_bridge(self, init)


bind_ros2_bridge_methods(Ros2Bridge)
