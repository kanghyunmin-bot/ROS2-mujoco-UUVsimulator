"""Hydrostatic restoring parser for simulation profiles."""

from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from .sim_profile_parse_common import bool_from_value


def parse_hydrostatic_restoring(sim_profile: Mapping[str, Any]) -> tuple[bool, float, float, float, float, bool]:
    payload = sim_profile.get("hydrostatic_restoring")
    if not isinstance(payload, Mapping):
        return False, 0.0, 0.0, 0.0, 0.0, False

    active = bool_from_value(payload.get("active"), default=False)
    values = _restoring_values(payload)
    if values is None:
        return False, 0.0, 0.0, 0.0, 0.0, False

    roll_stiffness, pitch_stiffness, roll_trim_rad, pitch_trim_rad = values
    roll_stiffness = max(_finite_or_zero(roll_stiffness), 0.0)
    pitch_stiffness = max(_finite_or_zero(pitch_stiffness), 0.0)
    roll_trim_rad = _finite_or_zero(roll_trim_rad)
    pitch_trim_rad = _finite_or_zero(pitch_trim_rad)
    trim_from_real_start = bool_from_value(payload.get("trim_from_real_start"), default=False)
    return (
        active and (roll_stiffness > 0.0 or pitch_stiffness > 0.0),
        roll_stiffness,
        pitch_stiffness,
        roll_trim_rad,
        pitch_trim_rad,
        trim_from_real_start,
    )


def _restoring_values(payload: Mapping[str, Any]) -> tuple[float, float, float, float] | None:
    try:
        return (
            float(payload.get("roll_stiffness_nm_per_rad", payload.get("roll_stiffness", 0.0))),
            float(payload.get("pitch_stiffness_nm_per_rad", payload.get("pitch_stiffness", 0.0))),
            float(payload.get("roll_trim_rad", payload.get("trim_roll_rad", 0.0))),
            float(payload.get("pitch_trim_rad", payload.get("trim_pitch_rad", 0.0))),
        )
    except (TypeError, ValueError):
        return None


def _finite_or_zero(value: float) -> float:
    return float(value) if np.isfinite(value) else 0.0


__all__ = ["parse_hydrostatic_restoring"]
