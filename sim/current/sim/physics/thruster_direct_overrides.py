"""Profile and environment overrides for direct thruster gains."""

from __future__ import annotations

import os
from typing import Any, Callable, MutableMapping, Optional, Sequence

from .thruster_direct_env import (
    apply_group_direct_gain_override,
    apply_per_thruster_direct_gain_overrides,
)
from .thruster_direct_profile import apply_profile_direct_gain_scales


def apply_thruster_direct_gain_overrides(
    sim_profile: MutableMapping[str, Any],
    thruster_direct_scale: MutableMapping[str, float],
    *,
    vertical_thrusters: Sequence[str],
    horizontal_thrusters: Sequence[str],
    per_thruster_env: Sequence[tuple[str, str]] = (
        ("UUV_YAW_LF_DIRECT_GAIN_SCALE", "yaw_lf"),
        ("UUV_YAW_LR_DIRECT_GAIN_SCALE", "yaw_lr"),
        ("UUV_YAW_RF_DIRECT_GAIN_SCALE", "yaw_rf"),
        ("UUV_YAW_RR_DIRECT_GAIN_SCALE", "yaw_rr"),
    ),
    env_get: Callable[[str, str], str] = os.environ.get,
    log: Optional[Callable[[str], None]] = None,
) -> None:
    """Apply profile and environment direct-gain overrides to per-thruster scales."""
    apply_profile_direct_gain_scales(
        sim_profile.get("thruster_direct_gain_scales"),
        thruster_direct_scale,
        log=log,
    )
    apply_group_direct_gain_override(
        env_name="UUV_VERTICAL_DIRECT_GAIN_SCALE",
        thruster_names=vertical_thrusters,
        thruster_direct_scale=thruster_direct_scale,
        env_get=env_get,
        log=log,
        label="vertical",
    )
    apply_group_direct_gain_override(
        env_name="UUV_HORIZONTAL_DIRECT_GAIN_SCALE",
        thruster_names=horizontal_thrusters,
        thruster_direct_scale=thruster_direct_scale,
        env_get=env_get,
        log=log,
        label="horizontal",
    )
    apply_per_thruster_direct_gain_overrides(
        per_thruster_env=per_thruster_env,
        thruster_direct_scale=thruster_direct_scale,
        env_get=env_get,
        log=log,
    )


__all__ = ["apply_thruster_direct_gain_overrides"]
