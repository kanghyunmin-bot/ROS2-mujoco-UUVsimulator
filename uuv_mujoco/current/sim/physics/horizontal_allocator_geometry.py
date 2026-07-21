"""Matrix builders for horizontal thruster allocation."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-9:
        return vector
    return vector / norm


def horizontal_allocation_matrix(
    *,
    model: Any,
    mujoco_module: Any,
    actuator_ids: Mapping[str, int],
    base_id: int,
    order: tuple[str, ...],
) -> np.ndarray:
    com_body = model.body_ipos[base_id].copy()
    allocation = np.zeros((3, len(order)), dtype=np.float64)
    for index, name in enumerate(order):
        actuator_id = int(actuator_ids[name])
        site_id = int(mujoco_module.mj_name2id(model, mujoco_module.mjtObj.mjOBJ_SITE, f"thr_{name}"))
        force_dir = normalize_vector(model.actuator_gear[actuator_id, :3].copy())
        radius = model.site_pos[site_id].copy() - com_body
        torque = np.cross(radius, force_dir)
        allocation[:, index] = np.array([force_dir[0], force_dir[1], torque[2]], dtype=np.float64)
    return allocation


def row_scaled_pseudo_inverse(allocation: np.ndarray) -> np.ndarray:
    row_scale = np.sum(np.abs(allocation), axis=1)
    row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
    return np.linalg.pinv(allocation / row_scale[:, None])


def saturate_allocator_commands(commands: np.ndarray) -> np.ndarray:
    max_abs = float(np.max(np.abs(commands))) if commands.size else 0.0
    if max_abs > 1.0:
        commands = commands / max_abs
    return np.clip(commands, -1.0, 1.0)


__all__ = [
    "horizontal_allocation_matrix",
    "normalize_vector",
    "row_scaled_pseudo_inverse",
    "saturate_allocator_commands",
]
