"""Compatibility exports for Ros2Bridge shutdown step helpers."""

from __future__ import annotations

from .ros2_bridge_shutdown_ros import (
    destroy_ros_node,
    remove_executor_node,
    shutdown_executor,
    shutdown_ros_context,
)
from .ros2_bridge_shutdown_sitl import shutdown_sitl_transport
from .ros2_bridge_shutdown_thread import stop_ros_spin_thread, stop_sitl_poll_thread
from .ros2_stereo_image import close_stereo_image_renderers


__all__ = [
    "destroy_ros_node",
    "remove_executor_node",
    "shutdown_executor",
    "shutdown_ros_context",
    "shutdown_sitl_transport",
    "stop_sitl_poll_thread",
    "stop_ros_spin_thread",
    "close_stereo_image_renderers",
]
