"""MuJoCo model-derived candidates for automatic initial Bar30 depth."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from .base_state import MuJoCoBaseState
from .initial_depth_geometry import required_bar30_depth_for_top


def thruster_depth_candidates(
    *,
    mujoco,
    model,
    data,
    base_state: MuJoCoBaseState,
    thruster_names: Iterable[str],
    thruster_immersion_half_height_m: float,
    margin_m: float,
) -> list[tuple[str, float]]:
    candidates: list[tuple[str, float]] = []
    for thr_name in thruster_names:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{thr_name}")
        if sid < 0:
            continue
        world_top_z = float(data.site_xpos[sid, 2]) + thruster_immersion_half_height_m
        candidates.append(
            (
                f"thruster:{thr_name}",
                required_bar30_depth_for_top(
                    base_state=base_state,
                    world_top_z=world_top_z,
                    margin_m=margin_m,
                ),
            )
        )
    return candidates


def fluid_geom_depth_candidates(
    *,
    model,
    data,
    base_state: MuJoCoBaseState,
    fluid_geom_ids: np.ndarray,
    fluid_geom_names: dict[int, str],
    margin_m: float,
) -> list[tuple[str, float]]:
    candidates: list[tuple[str, float]] = []
    for geom_id in fluid_geom_ids:
        geom_id = int(geom_id)
        geom_name = fluid_geom_names.get(geom_id, f"geom_{geom_id}")
        half_z = float(max(model.geom_size[geom_id, 2], 0.0))
        world_top_z = float(data.geom_xpos[geom_id, 2]) + half_z
        candidates.append(
            (
                f"fluid_geom:{geom_name}",
                required_bar30_depth_for_top(
                    base_state=base_state,
                    world_top_z=world_top_z,
                    margin_m=margin_m,
                ),
            )
        )
    return candidates


__all__ = [
    "fluid_geom_depth_candidates",
    "thruster_depth_candidates",
]
