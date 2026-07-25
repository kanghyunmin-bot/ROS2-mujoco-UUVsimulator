"""Body-component depth candidates for automatic initial Bar30 depth."""

from __future__ import annotations

from .base_state import MuJoCoBaseState
from .initial_depth_profile_candidate_build import depth_candidate_for_local_top
from .initial_depth_profile_fields import profile_item_name, profile_item_pos, profile_item_size


def body_component_depth_candidates(
    *,
    sim_profile: dict,
    data,
    base_state: MuJoCoBaseState,
    margin_m: float,
) -> list[tuple[str, float]]:
    candidates: list[tuple[str, float]] = []
    raw_components = sim_profile.get("body_components")
    if not isinstance(raw_components, list):
        return candidates

    for idx, component in enumerate(raw_components):
        if not isinstance(component, dict):
            continue
        size = profile_item_size(component)
        pos = profile_item_pos(component, ("buoyancy_pos", "mass_pos", "pos"))
        if size is None or pos is None:
            continue
        name = profile_item_name(component, f"component_{idx}")
        candidates.append(
            depth_candidate_for_local_top(
                label=f"body_component:{name}",
                local_pos=pos,
                half_z=float(max(size[2], 0.0)),
                data=data,
                base_state=base_state,
                margin_m=margin_m,
            )
        )
    return candidates


__all__ = ["body_component_depth_candidates"]
