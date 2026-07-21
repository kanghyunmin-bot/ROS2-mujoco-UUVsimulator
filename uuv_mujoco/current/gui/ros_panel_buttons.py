"""Button label refresh helpers for the GUI ROS panel."""

from __future__ import annotations


def _refresh_ros2_buttons(self) -> None:
    if self.mavros_toggle_button is not None:
        self.mavros_toggle_button.config(text="MAVROS OFF" if self._ros_pkg_running() else "MAVROS ON")
    if self.rviz_toggle_button is not None:
        self.rviz_toggle_button.config(text="RViz OFF" if self._rviz_running() else "RViz ON")


__all__ = ["_refresh_ros2_buttons"]
