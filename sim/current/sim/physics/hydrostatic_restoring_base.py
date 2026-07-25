"""Base env/profile restoring values before real-start trim overrides."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class HydrostaticRestoringBaseValues:
    active: bool
    roll_stiffness: float
    pitch_stiffness: float
    roll_trim: float
    pitch_trim: float
    profile_roll_trim: float
    profile_pitch_trim: float
    release_blend_s: float


def read_hydrostatic_restoring_base_values(
    *,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
    env_float: Callable[[str, float], float],
    env_flag: Callable[[str, bool], bool],
) -> HydrostaticRestoringBaseValues:
    roll_trim = env_float(
        "UUV_HYDROSTATIC_RESTORING_ROLL_TRIM_RAD",
        hydro_cfg.hydrostatic_restoring_roll_trim_rad,
    )
    pitch_trim = env_float(
        "UUV_HYDROSTATIC_RESTORING_PITCH_TRIM_RAD",
        hydro_cfg.hydrostatic_restoring_pitch_trim_rad,
    )
    return HydrostaticRestoringBaseValues(
        active=env_flag(
            "UUV_HYDROSTATIC_RESTORING_ACTIVE",
            hydro_cfg.hydrostatic_restoring_active,
        ),
        roll_stiffness=max(
            env_float(
                "UUV_HYDROSTATIC_RESTORING_ROLL_NM_PER_RAD",
                hydro_cfg.hydrostatic_restoring_roll_stiffness,
            ),
            0.0,
        ),
        pitch_stiffness=max(
            env_float(
                "UUV_HYDROSTATIC_RESTORING_PITCH_NM_PER_RAD",
                hydro_cfg.hydrostatic_restoring_pitch_stiffness,
            ),
            0.0,
        ),
        roll_trim=roll_trim,
        pitch_trim=pitch_trim,
        profile_roll_trim=float(roll_trim),
        profile_pitch_trim=float(pitch_trim),
        release_blend_s=max(
            0.0,
            env_float(
                "UUV_HYDROSTATIC_RESTORING_RELEASE_TRIM_BLEND_S",
                _release_trim_blend_default(sim_profile),
            ),
        ),
    )


def _release_trim_blend_default(sim_profile: dict[str, Any]) -> float:
    restoring_payload = sim_profile.get("hydrostatic_restoring", {})
    if isinstance(restoring_payload, dict):
        return float(restoring_payload.get("release_trim_blend_s", 0.0) or 0.0)
    return 0.0


__all__ = ["HydrostaticRestoringBaseValues", "read_hydrostatic_restoring_base_values"]
