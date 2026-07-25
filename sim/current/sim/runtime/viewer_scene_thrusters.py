"""Thruster and aggregate-force viewer debug drawing helpers."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np

from sim.runtime.viewer_scene_builder import ViewerSceneBuilder
from sim.runtime.viewer_scene_thruster_vectors import (
    draw_direction_for_force,
    net_thruster_force,
    thruster_force_and_world_dir,
)


def draw_thruster_debug(
    scene: ViewerSceneBuilder,
    *,
    data: Any,
    model: Any,
    act: Mapping[str, int],
    base_rot,
    thruster_site_ids: Mapping[str, int],
    thruster_names: Sequence[str],
    show_debug: bool,
    show_labels: bool,
) -> None:
    """Draw per-thruster force arrows, bubbles, and optional labels."""
    for name in thruster_names:
        sid = int(thruster_site_ids.get(name, -1))
        if sid < 0:
            continue
        start = data.site_xpos[sid].copy()
        force, world_dir = thruster_force_and_world_dir(
            data=data,
            model=model,
            act=act,
            base_rot=base_rot,
            name=name,
        )
        draw_dir = draw_direction_for_force(force, world_dir)
        if show_debug:
            scene.add_arrow(start, draw_dir, abs(force), (0.2, 0.6, 1.0, 1.0))
            scene.add_bubble_stream(start, -draw_dir, abs(force))
        if show_labels:
            scene.add_label(
                f"{name}:{force:+.1f}",
                start + np.array([0.0, 0.03, 0.0]),
                (0.8, 0.9, 1.0, 1.0),
            )


def draw_net_and_buoyancy_debug(
    scene: ViewerSceneBuilder,
    *,
    data: Any,
    model: Any,
    act: Mapping[str, int],
    base_rot,
    com,
    thruster_names: Sequence[str],
    last_buoy_force,
    last_buoy_point,
) -> None:
    """Draw aggregate thrust and buoyancy vectors."""
    net_force = net_thruster_force(
        data=data,
        model=model,
        act=act,
        base_rot=base_rot,
        thruster_names=thruster_names,
    )
    net_mag = float(np.linalg.norm(net_force))
    if net_mag > 1e-6:
        scene.add_arrow(com, net_force / net_mag, net_mag, (0.95, 0.95, 0.95, 1.0))
        scene.add_label("NET", com + np.array([0.0, 0.05, 0.0]), (0.95, 0.95, 0.95, 1.0))

    buoy_mag = float(np.linalg.norm(last_buoy_force))
    if buoy_mag > 1e-6:
        scene.add_arrow(
            last_buoy_point,
            np.array([0.0, 0.0, 1.0]),
            buoy_mag * 0.05,
            (0.2, 1.0, 0.2, 1.0),
        )
        scene.add_label("BUOY", last_buoy_point + np.array([0.0, 0.08, 0.0]), (0.6, 1.0, 0.6, 1.0))


__all__ = ["draw_net_and_buoyancy_debug", "draw_thruster_debug"]
