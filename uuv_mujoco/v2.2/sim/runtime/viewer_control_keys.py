"""Keyboard action helpers for MuJoCo viewer controls."""

from __future__ import annotations

import os
from typing import Any


KEY_ACTIONS = {
    49: "camera:stereo_left",
    321: "camera:stereo_left",
    50: "camera:stereo_right",
    322: "camera:stereo_right",
    51: "camera:course_overview",
    323: "camera:course_overview",
    53: "camera:follow",
    325: "camera:follow",
    54: "sensor_overlay",
    326: "sensor_overlay",
}


def apply_viewer_key(state: Any, keycode: int) -> None:
    """Apply one keyboard event to a ViewerControlState-like object."""
    keycode = int(keycode)
    action = KEY_ACTIONS.get(keycode)
    if action is None:
        _log_viewer_key(state, keycode=keycode, action=None)
        return
    if action == "sensor_overlay":
        state.toggle_sensor_overlay()
    elif action.startswith("camera:"):
        state.set_camera_mode(action.split(":", 1)[1])
    _log_viewer_key(state, keycode=keycode, action=action)


def _log_viewer_key(state: Any, *, keycode: int, action: str | None) -> None:
    if os.environ.get("UUV_VIEWER_KEY_LOG", "").strip().lower() not in {"1", "true", "yes", "on"}:
        return
    if action is None:
        print(f"[viewer-key] keycode={keycode} action=ignored", flush=True)
        return
    print(
        "[viewer-key] "
        f"keycode={keycode} "
        f"action={action} "
        f"camera_mode={getattr(state, 'camera_mode', None)} "
        f"follow={getattr(state, 'follow_camera_enabled', None)} "
        f"sensors={getattr(state, 'show_sensor_overlay', None)}",
        flush=True,
    )


__all__ = ["KEY_ACTIONS", "apply_viewer_key"]
