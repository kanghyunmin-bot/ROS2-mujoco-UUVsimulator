"""Visibility controls for the optional ROS2 GUI panel."""

from __future__ import annotations


def _toggle_ros2_panel(self) -> None:
    show = not self.ros2_panel_visible.get()
    self.ros2_panel_visible.set(show)
    if self.ros2_panel_frame is None:
        return
    if show:
        self.ros2_panel_frame.grid()
        if self.ros2_panel_button is not None:
            self.ros2_panel_button.config(text="Hide ROS2")
    else:
        self.ros2_panel_frame.grid_remove()
        if self.ros2_panel_button is not None:
            self.ros2_panel_button.config(text="ROS2 Panel")


__all__ = ["_toggle_ros2_panel"]
