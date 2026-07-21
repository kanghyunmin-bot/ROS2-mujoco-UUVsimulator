"""ROS2/SITL bridge launch wiring for the MuJoCo runner."""

from __future__ import annotations

from typing import Any, Callable

from .ros_bridge_launch_config import ros_bridge_kwargs_from_args
from .ros_bridge_launch_status import handle_ros_bridge_launch_error, print_ros_bridge_launch_summary


def create_ros_bridge_or_exit(
    *,
    args,
    model: Any,
    command_callback: Callable[[float, float, float, float], None],
    cmd_limit: float,
):
    """Create the optional ROS2/SITL bridge with runner-compatible error policy."""
    try:
        from bridge.ros2_bridge import Ros2Bridge

        enable_ros2 = bool(args.ros2)
        if not (args.sitl or enable_ros2):
            return None

        ros_bridge = Ros2Bridge(
            **ros_bridge_kwargs_from_args(
                args=args,
                model=model,
                command_callback=command_callback,
                cmd_limit=cmd_limit,
            )
        )
        print_ros_bridge_launch_summary(args, enable_ros2=enable_ros2)
        return ros_bridge
    except Exception as exc:
        return handle_ros_bridge_launch_error(args, exc)


__all__ = ["create_ros_bridge_or_exit"]
