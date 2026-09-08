"""Shutdown public API helper for Ros2Bridge."""

from __future__ import annotations

from .ros2_bridge_shutdown_steps import (
    destroy_ros_node,
    remove_executor_node,
    shutdown_executor,
    shutdown_ros_context,
    shutdown_sitl_transport,
    close_dvl_device_emulator,
    close_stereo_image_renderers,
    stop_ros_spin_thread,
    stop_sitl_poll_thread,
)


def shutdown(self) -> None:
    close_stereo_image_renderers(self)
    close_dvl_device_emulator(self)
    stop_ros_spin_thread(self)
    stop_sitl_poll_thread(self)
    shutdown_sitl_transport(self)
    if not self._enable_ros:
        return
    remove_executor_node(self)
    shutdown_executor(self)
    destroy_ros_node(self)
    shutdown_ros_context(self)


__all__ = ["shutdown"]
