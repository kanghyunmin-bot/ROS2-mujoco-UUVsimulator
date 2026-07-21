"""Compatibility exports for MuJoCo viewer debug scene drawing helpers."""

from __future__ import annotations

from sim.runtime.viewer_scene_builder import ViewerSceneBuilder
from sim.runtime.viewer_scene_sensors import draw_sensor_markers
from sim.runtime.viewer_scene_thrusters import draw_net_and_buoyancy_debug, draw_thruster_debug


__all__ = [
    "ViewerSceneBuilder",
    "draw_net_and_buoyancy_debug",
    "draw_sensor_markers",
    "draw_thruster_debug",
]
