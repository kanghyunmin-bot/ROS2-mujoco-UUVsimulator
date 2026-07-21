"""Viewer-control setup helpers."""

from __future__ import annotations

from collections.abc import Callable


VALID_CAMERA_MODES = frozenset({"free", "follow", "course_overview", "course_side", "stereo_left", "stereo_right"})


def clip_float(value: float, low: float, high: float) -> float:
    return max(float(low), min(float(high), float(value)))


def resolve_initial_camera_mode(*, args) -> str:
    initial_camera_mode = str(args.viewer_camera_mode or "").strip().lower()
    if not initial_camera_mode:
        return "follow" if args.sitl else "free"
    if initial_camera_mode not in VALID_CAMERA_MODES:
        return "follow" if args.sitl else "free"
    return initial_camera_mode


def follow_camera_settings(
    *,
    env_float: Callable[[str, float], float],
) -> tuple[float, float, float]:
    return (
        clip_float(env_float("UUV_VIEWER_FOLLOW_DISTANCE", 2.2), 0.2, 20.0),
        clip_float(env_float("UUV_VIEWER_FOLLOW_ELEVATION_DEG", -20.0), -89.0, 89.0),
        float(env_float("UUV_VIEWER_FOLLOW_AZIMUTH_DEG", 135.0)),
    )


__all__ = ["VALID_CAMERA_MODES", "clip_float", "follow_camera_settings", "resolve_initial_camera_mode"]
