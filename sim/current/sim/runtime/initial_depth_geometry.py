"""Geometry primitives for automatic initial Bar30 depth selection."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from .base_state import MuJoCoBaseState


def world_z_from_base_local(*, data, base_state: MuJoCoBaseState, local_pos: np.ndarray) -> float:
    base_rot = data.xmat[base_state.base_id].reshape(3, 3)
    return float((base_state.base_origin_world() + base_rot @ local_pos)[2])


def required_bar30_depth_for_top(
    *,
    base_state: MuJoCoBaseState,
    world_top_z: float,
    margin_m: float,
) -> float:
    return float(world_top_z - base_state.bar30_world_z() + margin_m)


def finite_depth_candidates(candidates: Iterable[tuple[str, float]]) -> list[tuple[str, float]]:
    return [(name, float(depth)) for name, depth in candidates if np.isfinite(depth)]


def select_auto_initial_depth(
    *,
    finite_candidates: list[tuple[str, float]],
    surface_clear_m: float,
    min_depth_m: float,
) -> float:
    if not finite_candidates:
        return max(surface_clear_m, min_depth_m)
    chosen_m = float(max(depth for _, depth in finite_candidates))
    return float(max(chosen_m, surface_clear_m, min_depth_m, 0.0))


__all__ = [
    "finite_depth_candidates",
    "required_bar30_depth_for_top",
    "select_auto_initial_depth",
    "world_z_from_base_local",
]
