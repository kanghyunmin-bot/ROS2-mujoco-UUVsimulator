"""Thruster vector helpers for viewer debug drawing."""

from __future__ import annotations

from typing import Any, Mapping, Sequence

import numpy as np


def thruster_force_and_world_dir(
    *,
    data: Any,
    model: Any,
    act: Mapping[str, int],
    base_rot,
    name: str,
) -> tuple[float, np.ndarray]:
    actuator_id = act[name]
    force = float(data.ctrl[actuator_id])
    gear_dir = model.actuator_gear[actuator_id, :3]
    gear_dir = gear_dir / (np.linalg.norm(gear_dir) + 1e-9)
    return force, base_rot @ gear_dir


def draw_direction_for_force(force: float, world_dir: np.ndarray) -> np.ndarray:
    if force >= 0.0:
        return world_dir
    return -world_dir


def net_thruster_force(
    *,
    data: Any,
    model: Any,
    act: Mapping[str, int],
    base_rot,
    thruster_names: Sequence[str],
) -> np.ndarray:
    net_force = np.zeros(3)
    for name in thruster_names:
        force, world_dir = thruster_force_and_world_dir(
            data=data,
            model=model,
            act=act,
            base_rot=base_rot,
            name=name,
        )
        net_force += world_dir * force
    return net_force


__all__ = [
    "draw_direction_for_force",
    "net_thruster_force",
    "thruster_force_and_world_dir",
]
