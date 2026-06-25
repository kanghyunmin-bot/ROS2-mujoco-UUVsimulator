"""Real-start trim overrides for hydrostatic restoring setup."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Callable


@dataclass(frozen=True)
class RealStartRestoringTrims:
    roll_trim: float
    pitch_trim: float


@dataclass(frozen=True)
class RuntimeRestoringTrims:
    roll_trim: float
    pitch_trim: float
    profile_roll_trim: float
    profile_pitch_trim: float


def read_real_start_restoring_trims(
    *,
    real_start_required: bool,
    env_float: Callable[[str, float], float],
) -> RealStartRestoringTrims:
    if not real_start_required:
        return RealStartRestoringTrims(math.nan, math.nan)
    return RealStartRestoringTrims(
        env_float("UUV_REAL_START_ROLL_RAD", math.nan),
        env_float("UUV_REAL_START_PITCH_RAD", math.nan),
    )


def resolve_runtime_restoring_trims(
    *,
    hydro_cfg: Any,
    real_start_required: bool,
    env_flag: Callable[[str, bool], bool],
    real_start_trims: RealStartRestoringTrims,
    roll_trim: float,
    pitch_trim: float,
    profile_roll_trim: float,
    profile_pitch_trim: float,
) -> RuntimeRestoringTrims:
    trim_from_real_start = env_flag(
        "UUV_HYDROSTATIC_RESTORING_TRIM_FROM_REAL_START",
        hydro_cfg.hydrostatic_restoring_trim_from_real_start,
    )
    if not (trim_from_real_start and real_start_required):
        return RuntimeRestoringTrims(roll_trim, pitch_trim, profile_roll_trim, profile_pitch_trim)
    runtime_roll, profile_roll = _finite_pair(real_start_trims.roll_trim, roll_trim, profile_roll_trim)
    runtime_pitch, profile_pitch = _finite_pair(real_start_trims.pitch_trim, pitch_trim, profile_pitch_trim)
    return RuntimeRestoringTrims(runtime_roll, runtime_pitch, profile_roll, profile_pitch)


def _finite_pair(real_value: float, runtime_value: float, profile_value: float) -> tuple[float, float]:
    if math.isfinite(real_value):
        return float(real_value), float(real_value)
    return runtime_value, profile_value


__all__ = [
    "RealStartRestoringTrims",
    "RuntimeRestoringTrims",
    "read_real_start_restoring_trims",
    "resolve_runtime_restoring_trims",
]
