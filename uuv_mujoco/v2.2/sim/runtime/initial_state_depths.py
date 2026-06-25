"""Initial-depth request parsing for the MuJoCo runner."""

from __future__ import annotations

from collections.abc import Callable, Sequence
from typing import Any

from sim.runtime.initial_depth import compute_auto_initial_bar30_depth
from sim.runtime.initial_state_depth_types import InitialBar30Depth
from sim.runtime.initial_state_drop_start import apply_drop_start_default
from sim.runtime.initial_state_real_start_tolerance import real_start_tolerances


def resolve_initial_bar30_depth(
    *,
    args: Any,
    mujoco: Any,
    model: Any,
    data: Any,
    sim_profile: dict,
    base_state: Any,
    fluid_geom_ids: Any,
    fluid_geom_names: Any,
    thruster_names: Sequence[str],
    thruster_immersion_half_height_m: float,
    env_float: Callable[[str, float], float],
) -> InitialBar30Depth:
    """Resolve --initial-bar30-depth-m into a numeric depth and display label."""

    if args.initial_bar30_depth_m is None:
        return InitialBar30Depth(value_m=None, label="")

    raw_initial_bar30 = str(args.initial_bar30_depth_m).strip()
    initial_bar30_depth_label = raw_initial_bar30
    if raw_initial_bar30.lower() in {"auto", "computed", "submerged", "fully_submerged"}:
        value_m = compute_auto_initial_bar30_depth(
            mujoco=mujoco,
            model=model,
            data=data,
            sim_profile=sim_profile,
            base_state=base_state,
            fluid_geom_ids=fluid_geom_ids,
            fluid_geom_names=fluid_geom_names,
            thruster_names=list(thruster_names),
            thruster_immersion_half_height_m=thruster_immersion_half_height_m,
            sitl=bool(args.sitl),
            env_float=env_float,
        )
        return InitialBar30Depth(
            value_m=float(value_m),
            label=f"{raw_initial_bar30}->{float(value_m):.3f}",
        )

    try:
        value_m = float(raw_initial_bar30)
    except ValueError as exc:
        raise SystemExit(
            "[runtime] invalid --initial-bar30-depth-m value "
            f"{raw_initial_bar30!r}; use a number or 'auto'"
        ) from exc
    return InitialBar30Depth(value_m=float(value_m), label=initial_bar30_depth_label)


__all__ = [
    "InitialBar30Depth",
    "apply_drop_start_default",
    "real_start_tolerances",
    "resolve_initial_bar30_depth",
]
