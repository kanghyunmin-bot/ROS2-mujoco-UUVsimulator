"""Sensor marker drawing helpers for the MuJoCo viewer scene."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from sim.runtime.viewer_scene_builder import ViewerSceneBuilder


SENSOR_MARKER_SPECS: tuple[tuple[str, str, float, tuple[float, ...], tuple[float, ...]], ...] = (
    ("imu", "IMU", 0.025, (1.0, 1.0, 0.1, 1.0), (1.0, 1.0, 0.3, 1.0)),
    ("bar30", "BAR30", 0.022, (0.2, 0.8, 1.0, 1.0), (0.3, 0.9, 1.0, 1.0)),
    ("dvl", "DVL", 0.025, (0.1, 1.0, 1.0, 1.0), (0.3, 1.0, 1.0, 1.0)),
    ("ping360", "PING360", 0.022, (0.2, 0.6, 1.0, 1.0), (0.3, 0.7, 1.0, 1.0)),
    ("cam_left", "CAM_L", 0.022, (1.0, 0.1, 1.0, 1.0), (1.0, 0.3, 1.0, 1.0)),
    ("cam_right", "CAM_R", 0.022, (1.0, 0.5, 0.1, 1.0), (1.0, 0.6, 0.2, 1.0)),
)


def draw_sensor_markers(scene: ViewerSceneBuilder, *, data: Any, sensor_site_ids: Mapping[str, int]) -> None:
    """Draw named sensor/camera site markers."""
    for key, label, radius, sphere_rgba, label_rgba in SENSOR_MARKER_SPECS:
        site_id = int(sensor_site_ids.get(key, -1))
        if site_id < 0:
            continue
        pos = data.site_xpos[site_id].copy()
        scene.add_sphere(pos, radius, sphere_rgba)
        scene.add_label(label, pos + np.array([0.0, 0.03, 0.0]), label_rgba)


__all__ = ["SENSOR_MARKER_SPECS", "draw_sensor_markers"]
