"""Initial depth hold state machine helpers."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sim.runtime.initial_hold_actions import InitialHoldActionMixin
from sim.runtime.initial_hold_mapping import InitialHoldMappingMixin
from sim.runtime.initial_hold_pose import coerce_body_vector


@dataclass
class InitialDepthHoldState(InitialHoldMappingMixin, InitialHoldActionMixin):
    """Mutable initial-depth hold state with dict-style compatibility."""

    active: bool
    depth_m: float | None
    bar30_depth_m: float | None
    release_linear_velocity_body: np.ndarray | None
    release_angular_velocity_body: np.ndarray | None
    pose_qpos: np.ndarray | None = None
    release_sim_time: float | None = None
    pending_release_reason: str | None = None

    @classmethod
    def create(
        cls,
        *,
        active: bool,
        initial_depth_m: float | None,
        bar30_depth_m: float | None,
        release_linear_velocity_body,
        release_angular_velocity_body,
    ) -> "InitialDepthHoldState":
        """Create state from parsed CLI values."""
        return cls(
            active=bool(active),
            depth_m=float(initial_depth_m) if initial_depth_m is not None else None,
            bar30_depth_m=bar30_depth_m,
            release_linear_velocity_body=coerce_body_vector(release_linear_velocity_body),
            release_angular_velocity_body=coerce_body_vector(release_angular_velocity_body),
        )


__all__ = ["InitialDepthHoldState"]
