"""ViewerControlState mutation helpers."""

from __future__ import annotations

from sim.runtime.viewer_control_config import VALID_CAMERA_MODES


def toggle_viewer_thruster_labels(state) -> None:
    state.show_thruster_labels = not state.show_thruster_labels
    if state.show_thruster_labels:
        state.show_debug = True


def toggle_viewer_follow_camera(state) -> None:
    state.follow_camera_enabled = not state.follow_camera_enabled
    if state.follow_camera_enabled:
        state.follow_camera_initialized = False
        state.camera_mode = "follow"
    else:
        state.camera_mode = "free"


def toggle_viewer_sensor_overlay(state) -> None:
    state.show_sensor_overlay = not state.show_sensor_overlay
    if state.show_sensor_overlay:
        state.show_debug = True


def set_viewer_camera_mode(state, mode: str) -> None:
    if mode not in VALID_CAMERA_MODES:
        return
    state.camera_mode = mode
    state.follow_camera_enabled = mode == "follow"
    if state.follow_camera_enabled:
        state.follow_camera_initialized = False


__all__ = [
    "set_viewer_camera_mode",
    "toggle_viewer_follow_camera",
    "toggle_viewer_sensor_overlay",
    "toggle_viewer_thruster_labels",
]
