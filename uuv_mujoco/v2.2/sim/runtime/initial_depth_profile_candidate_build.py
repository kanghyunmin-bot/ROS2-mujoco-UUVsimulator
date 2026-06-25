"""Candidate construction for sim-profile-derived initial depths."""

from __future__ import annotations

import numpy as np

from .base_state import MuJoCoBaseState
from .initial_depth_geometry import required_bar30_depth_for_top, world_z_from_base_local


def depth_candidate_for_local_top(
    *,
    label: str,
    local_pos: np.ndarray,
    half_z: float,
    data,
    base_state: MuJoCoBaseState,
    margin_m: float,
) -> tuple[str, float]:
    world_top_z = world_z_from_base_local(data=data, base_state=base_state, local_pos=local_pos) + half_z
    return (
        label,
        required_bar30_depth_for_top(
            base_state=base_state,
            world_top_z=world_top_z,
            margin_m=margin_m,
        ),
    )


__all__ = ["depth_candidate_for_local_top"]
