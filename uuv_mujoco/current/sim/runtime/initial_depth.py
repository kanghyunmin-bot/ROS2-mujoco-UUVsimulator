"""Initial depth selection helpers for MuJoCo/SITL starts."""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from .base_state import MuJoCoBaseState
from .initial_depth_candidates import (
    body_component_depth_candidates,
    buoyancy_point_depth_candidates,
    finite_depth_candidates,
    fluid_geom_depth_candidates,
    select_auto_initial_depth,
    thruster_depth_candidates,
)
from .initial_depth_limits import EnvFloat, load_auto_initial_depth_limits
from .initial_depth_profile import vec3_from_profile


def compute_auto_initial_bar30_depth(
    *,
    mujoco,
    model,
    data,
    sim_profile: dict,
    base_state: MuJoCoBaseState,
    fluid_geom_ids: np.ndarray,
    fluid_geom_names: dict[int, str],
    thruster_names: Iterable[str],
    thruster_immersion_half_height_m: float,
    sitl: bool,
    env_float: EnvFloat,
) -> float:
    """Compute the shallowest Bar30 depth that starts the modeled vehicle fully wet."""

    mujoco.mj_forward(model, data)
    limits = load_auto_initial_depth_limits(sitl=sitl, env_float=env_float)
    candidates: list[tuple[str, float]] = limits.seed_candidates()
    candidates.extend(
        thruster_depth_candidates(
            mujoco=mujoco,
            model=model,
            data=data,
            base_state=base_state,
            thruster_names=thruster_names,
            thruster_immersion_half_height_m=thruster_immersion_half_height_m,
            margin_m=limits.margin_m,
        )
    )
    candidates.extend(
        fluid_geom_depth_candidates(
            model=model,
            data=data,
            base_state=base_state,
            fluid_geom_ids=fluid_geom_ids,
            fluid_geom_names=fluid_geom_names,
            margin_m=limits.margin_m,
        )
    )
    candidates.extend(
        body_component_depth_candidates(
            sim_profile=sim_profile,
            data=data,
            base_state=base_state,
            margin_m=limits.margin_m,
        )
    )
    candidates.extend(
        buoyancy_point_depth_candidates(
            sim_profile=sim_profile,
            data=data,
            base_state=base_state,
            margin_m=limits.margin_m,
        )
    )

    finite_candidates = finite_depth_candidates(candidates)
    chosen_m = select_auto_initial_depth(
        finite_candidates=finite_candidates,
        surface_clear_m=limits.surface_clear_m,
        min_depth_m=limits.min_depth_m,
    )
    print(
        "[runtime] auto initial Bar30 depth: "
        f"chosen={chosen_m:.3f} m margin={limits.margin_m:.3f} m "
        f"bar30_source={'bar30_site' if base_state.bar30_site_id >= 0 else 'base_link'}",
        flush=True,
    )
    for name, depth in sorted(finite_candidates, key=lambda item: item[1], reverse=True)[:8]:
        print(f"[runtime]   auto-depth candidate {name}: {depth:.3f} m", flush=True)
    return chosen_m


__all__ = ["compute_auto_initial_bar30_depth", "vec3_from_profile"]
