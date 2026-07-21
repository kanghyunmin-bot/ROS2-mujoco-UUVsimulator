"""CoB runtime value extraction for hydrostatic setup."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class HydrostaticCobValues:
    profile_cob_x_offset: float
    profile_cob_z_offset: float
    cob_torque_scale: float
    cob_longitudinal_offset: float
    cob_vertical_offset: float


def read_hydrostatic_cob_values(
    *,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
    env_float: Callable[[str, float], float],
) -> HydrostaticCobValues:
    profile_cob_x_offset = float(sim_profile.get("cob_x_offset", 0.0))
    profile_cob_z_offset = float(sim_profile.get("cob_z_offset", 0.0))
    return HydrostaticCobValues(
        profile_cob_x_offset=profile_cob_x_offset,
        profile_cob_z_offset=profile_cob_z_offset,
        cob_torque_scale=env_float("UUV_COB_TORQUE_SCALE", hydro_cfg.cob_torque_scale),
        cob_longitudinal_offset=env_float("UUV_COB_X_OFFSET_M", profile_cob_x_offset),
        cob_vertical_offset=env_float("UUV_COB_Z_OFFSET_M", profile_cob_z_offset),
    )


__all__ = ["HydrostaticCobValues", "read_hydrostatic_cob_values"]
