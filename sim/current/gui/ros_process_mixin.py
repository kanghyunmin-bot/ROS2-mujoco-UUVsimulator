"""Compatibility facade for ROS, RViz, and package process controls."""

from __future__ import annotations

from .process_common_mixin import ProcessCommonMixin
from .ros_logged_process_mixin import RosLoggedProcessMixin
from .ros_package_mixin import RosPackageMixin
from .ros_panel_mixin import RosPanelMixin
from .rviz_process_mixin import RvizProcessMixin


class RosProcessMixin(
    RosPackageMixin,
    RvizProcessMixin,
    RosLoggedProcessMixin,
    RosPanelMixin,
    ProcessCommonMixin,
):
    """Preserve the GUI ROS process control API while splitting ownership."""


__all__ = ["RosProcessMixin"]
