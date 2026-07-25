"""ViewerControlState construction helpers."""

from __future__ import annotations

from typing import Callable

from sim.runtime.viewer_control_config import follow_camera_settings, resolve_initial_camera_mode


def create_viewer_control_state(cls, *, args, env_float: Callable[[str, float], float]):
    initial_camera_mode = resolve_initial_camera_mode(args=args)
    follow_distance, follow_elevation, follow_azimuth = follow_camera_settings(env_float=env_float)
    return cls(
        enable_pause=bool(args.enable_viewer_pause),
        paused=False,
        show_debug=bool(args.viewer_debug),
        show_thruster_labels=bool(args.viewer_debug),
        show_sensor_overlay=bool(args.viewer_debug),
        camera_mode=initial_camera_mode,
        follow_camera_enabled=initial_camera_mode == "follow",
        follow_camera_initialized=False,
        follow_distance=follow_distance,
        follow_elevation=follow_elevation,
        follow_azimuth=follow_azimuth,
    )


__all__ = ["create_viewer_control_state"]
