"""Keyboard action helpers for MuJoCo viewer controls."""

from __future__ import annotations

from typing import Any


KEY_ACTIONS = {
    32: "pause",
    76: "thruster_labels",
    108: "thruster_labels",
    73: "sensor_overlay",
    105: "sensor_overlay",
    67: "follow_camera",
    99: "follow_camera",
    49: "camera:stereo_left",
    50: "camera:stereo_right",
    51: "camera:course_overview",
    52: "camera:course_side",
    48: "camera:free",
}


def apply_viewer_key(state: Any, keycode: int) -> None:
    """Apply one keyboard event to a ViewerControlState-like object."""
    action = KEY_ACTIONS.get(int(keycode))
    if action is None:
        return
    if action == "pause":
        if state.enable_pause:
            state.paused = not state.paused
        return
    if action == "thruster_labels":
        state.toggle_thruster_labels()
        return
    if action == "sensor_overlay":
        state.toggle_sensor_overlay()
        return
    if action == "follow_camera":
        state.toggle_follow_camera()
        return
    if action.startswith("camera:"):
        state.set_camera_mode(action.split(":", 1)[1])


__all__ = ["KEY_ACTIONS", "apply_viewer_key"]
