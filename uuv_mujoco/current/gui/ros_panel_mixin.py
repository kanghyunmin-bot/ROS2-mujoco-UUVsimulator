"""ROS2 panel status helpers for the GUI."""

from __future__ import annotations

from .ros_panel_buttons import _refresh_ros2_buttons
from .ros_panel_process_state import (
    _gui_external_mavros_controls_enabled,
    _ros_build_running,
    _ros_pkg_running,
    _rviz_running,
)
from .ros_panel_status import _set_ros_pkg_status, _set_rviz_status
from .ros_panel_visibility import _toggle_ros2_panel


class RosPanelMixin:
    _toggle_ros2_panel = _toggle_ros2_panel
    _ros_pkg_running = _ros_pkg_running
    _ros_build_running = _ros_build_running
    _rviz_running = _rviz_running
    _gui_external_mavros_controls_enabled = _gui_external_mavros_controls_enabled
    _set_ros_pkg_status = _set_ros_pkg_status
    _set_rviz_status = _set_rviz_status
    _refresh_ros2_buttons = _refresh_ros2_buttons


__all__ = ["RosPanelMixin"]
