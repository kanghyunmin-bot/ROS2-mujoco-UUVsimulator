"""Viewer keyboard and camera state for the MuJoCo runner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Mapping

from sim.runtime.viewer_control_camera import apply_viewer_camera
from sim.runtime.viewer_control_keys import apply_viewer_key
from sim.runtime.viewer_control_state_factory import create_viewer_control_state
from sim.runtime.viewer_control_state_status import viewer_base_overlay_line, viewer_is_paused
from sim.runtime.viewer_control_state_toggles import (
    set_viewer_camera_mode,
    toggle_viewer_follow_camera,
    toggle_viewer_sensor_overlay,
    toggle_viewer_thruster_labels,
)


@dataclass
class ViewerControlState:
    """Mutable viewer controls that used to be scattered through the runner."""

    enable_pause: bool
    paused: bool
    show_debug: bool
    show_thruster_labels: bool
    show_sensor_overlay: bool
    camera_mode: str
    follow_camera_enabled: bool
    follow_camera_initialized: bool
    follow_distance: float
    follow_elevation: float
    follow_azimuth: float

    @classmethod
    def create(cls, *, args, env_float: Callable[[str, float], float]) -> "ViewerControlState":
        """Create viewer controls from CLI and environment settings."""
        return create_viewer_control_state(cls, args=args, env_float=env_float)

    def toggle_thruster_labels(self) -> None:
        toggle_viewer_thruster_labels(self)

    def toggle_follow_camera(self) -> None:
        toggle_viewer_follow_camera(self)

    def toggle_sensor_overlay(self) -> None:
        toggle_viewer_sensor_overlay(self)

    def set_camera_mode(self, mode: str) -> None:
        set_viewer_camera_mode(self, mode)

    def handle_key(self, keycode: int) -> None:
        """Handle MuJoCo viewer keyboard input."""
        apply_viewer_key(self, keycode)

    def is_paused(self, viewer: Any) -> bool:
        """Return the combined local and viewer-native paused state."""
        return viewer_is_paused(self, viewer)

    def apply_camera(self, *, viewer: Any, mujoco_module: Any, base_id: int, camera_ids: Mapping[str, int]) -> None:
        """Apply the selected camera mode to the MuJoCo viewer camera."""
        apply_viewer_camera(
            self,
            viewer=viewer,
            mujoco_module=mujoco_module,
            base_id=base_id,
            camera_ids=camera_ids,
        )

    def base_overlay_line(self) -> str:
        """Return the default viewer help overlay line."""
        return viewer_base_overlay_line(self)


__all__ = ["ViewerControlState"]
