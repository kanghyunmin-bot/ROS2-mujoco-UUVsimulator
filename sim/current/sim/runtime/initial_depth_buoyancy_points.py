"""Buoyancy-point depth candidates for automatic initial Bar30 depth."""

from __future__ import annotations

from .base_state import MuJoCoBaseState
from .initial_depth_profile_candidate_build import depth_candidate_for_local_top
from .initial_depth_profile_fields import nonnegative_half_height, profile_item_name, profile_item_pos


def buoyancy_point_depth_candidates(
    *,
    sim_profile: dict,
    data,
    base_state: MuJoCoBaseState,
    margin_m: float,
) -> list[tuple[str, float]]:
    candidates: list[tuple[str, float]] = []
    raw_points = sim_profile.get("buoyancy_points")
    if not isinstance(raw_points, list):
        return candidates

    for idx, point in enumerate(raw_points):
        if not isinstance(point, dict):
            continue
        pos = profile_item_pos(point, ("pos",))
        if pos is None:
            continue
        name = profile_item_name(point, f"buoyancy_point_{idx}")
        candidates.append(
            depth_candidate_for_local_top(
                label=f"buoyancy_point:{name}",
                local_pos=pos,
                half_z=nonnegative_half_height(point.get("half_height", 0.0)),
                data=data,
                base_state=base_state,
                margin_m=margin_m,
            )
        )
    return candidates


__all__ = ["buoyancy_point_depth_candidates"]
