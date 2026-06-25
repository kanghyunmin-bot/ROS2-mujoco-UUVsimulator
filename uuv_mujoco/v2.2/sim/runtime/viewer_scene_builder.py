"""Primitive MuJoCo user-scene drawing helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from sim.runtime.viewer_scene_bubbles import add_bubble_stream_visual
from sim.runtime.viewer_scene_primitives import add_arrow_geom, add_label_geom, add_sphere_geom


@dataclass
class ViewerSceneBuilder:
    """Small wrapper around MuJoCo user scene geometry allocation."""

    user_scn: Any
    mujoco: Any
    normalize: Any
    sim_time: float
    thrust_force_max: float

    def add_arrow(self, start, direction, magnitude, rgba, thickness: float = 0.02) -> None:
        add_arrow_geom(self.user_scn, self.mujoco, start, direction, magnitude, rgba, thickness)

    def add_sphere(self, position, radius: float, rgba) -> None:
        add_sphere_geom(self.user_scn, self.mujoco, position, radius, rgba)

    def add_label(self, text: str, position, rgba) -> None:
        add_label_geom(self.user_scn, self.mujoco, text, position, rgba)

    def add_bubble_stream(self, start, exhaust_dir, thrust_mag: float) -> None:
        add_bubble_stream_visual(
            add_sphere=self.add_sphere,
            normalize=self.normalize,
            start=start,
            exhaust_dir=exhaust_dir,
            thrust_mag=thrust_mag,
            thrust_force_max=self.thrust_force_max,
            sim_time=self.sim_time,
        )


__all__ = ["ViewerSceneBuilder"]
