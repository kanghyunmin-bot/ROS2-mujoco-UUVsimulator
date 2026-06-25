"""ViewerControlState status helpers."""

from __future__ import annotations

from typing import Any


def viewer_is_paused(state, viewer: Any) -> bool:
    native_paused = False
    if state.enable_pause and hasattr(viewer, "is_paused"):
        flag = viewer.is_paused
        native_paused = bool(flag() if callable(flag) else flag)
    return bool(state.enable_pause and (state.paused or native_paused))


def viewer_base_overlay_line(state) -> str:
    overlay_line = "C: follow, 1/2: stereo cam, 0: free, I: sensors, L: thruster labels"
    if state.enable_pause:
        overlay_line = "Space: pause, " + overlay_line
    return overlay_line


__all__ = ["viewer_base_overlay_line", "viewer_is_paused"]
