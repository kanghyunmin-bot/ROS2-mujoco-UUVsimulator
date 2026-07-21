"""ROS2 MAVROS package build/start controls for the GUI."""

from __future__ import annotations

from .ros_package_build import start_ros_pkg_build
from .ros_package_stack import start_ros_pkg_stack, toggle_ros_pkg_stack


class RosPackageMixin:
    def _build_ros_pkg(self) -> None:
        start_ros_pkg_build(self)

    def _toggle_ros_pkg_stack(self) -> None:
        toggle_ros_pkg_stack(self)

    def _start_ros_pkg_stack(self) -> None:
        # Simulation path: this starts an external MAVROS helper for inspection
        # and RViz workflows.  The simulator keeps its own lightweight MAVROS
        # control surface by default because it is the deterministic closed-loop
        # path on macOS+Docker.  Set UUV_GUI_USE_EXTERNAL_MAVROS=1 only when
        # the external MAVROS node should own /mavros arm/mode/RC services.
        start_ros_pkg_stack(self)


__all__ = ["RosPackageMixin"]
