"""Deterministic, bounded ambient-current fields for pool experiments.

The model is deliberately low order.  It provides spatial gradients and a
small set of harmonic modes without pretending to be CFD.  Every active field
must carry calibration provenance, and every sample is norm-bounded before it
is handed to MuJoCo.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np


_MAX_LOW_FREQUENCY_HZ = 0.5
_MAX_MODE_COUNT = 64
_MAX_CURRENT_SPEED_MPS = 100.0
_MAX_GRADIENT_PER_S = 100.0
_MAX_WAVENUMBER_RAD_PER_M = 1.0e4
_MAX_QUERY_POSITION_M = 1.0e6
_MAX_QUERY_TIME_S = 1.0e12
_TWO_PI = 2.0 * math.pi


def _vector(value: Any, *, length: int, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain {length} finite numbers")
    return result


def _matrix3(value: Any, *, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3, 3) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain a finite 3x3 matrix")
    return result


def _bounded_vector(
    value: Any,
    *,
    length: int,
    label: str,
    max_abs: float,
) -> np.ndarray:
    result = _vector(value, length=length, label=label)
    if float(np.max(np.abs(result))) > max_abs:
        raise ValueError(f"{label} exceeds safe component bound {max_abs:g}")
    return result


@dataclass(frozen=True)
class SpatialMode:
    """One bounded spatial harmonic in world coordinates."""

    amplitude_mps: np.ndarray
    wave_vector_rad_per_m: np.ndarray
    phase_rad: float


@dataclass(frozen=True)
class TemporalMode:
    """One deterministic low-frequency temporal harmonic."""

    amplitude_mps: np.ndarray
    frequency_hz: float
    phase_rad: float


@dataclass(frozen=True)
class CurrentFieldConfig:
    """Parsed ambient-current field parameters."""

    active: bool
    calibration_status: str
    base_velocity_world_mps: np.ndarray
    origin_world_m: np.ndarray
    gradient_per_s: np.ndarray
    spatial_modes: tuple[SpatialMode, ...]
    temporal_modes: tuple[TemporalMode, ...]
    max_speed_mps: float


class DeterministicCurrentField:
    """Evaluate a reproducible, bounded water velocity at position and time."""

    def __init__(self, config: CurrentFieldConfig) -> None:
        self.config = config

    @classmethod
    def from_profile(
        cls,
        sim_profile: Mapping[str, Any],
        *,
        fallback_velocity_world_mps: np.ndarray,
    ) -> "DeterministicCurrentField":
        fallback = _bounded_vector(
            fallback_velocity_world_mps,
            length=3,
            label="fallback_velocity_world_mps",
            max_abs=_MAX_CURRENT_SPEED_MPS,
        )
        payload = sim_profile.get("current_field")
        if payload is None:
            return cls(_inactive_config(fallback))
        if not isinstance(payload, Mapping):
            raise ValueError("current_field must be an object")
        active = payload.get("active", False)
        if not isinstance(active, bool):
            raise ValueError("current_field.active must be boolean")
        if not active:
            return cls(_inactive_config(fallback))

        calibration_status = str(payload.get("calibration_status", "")).strip()
        if not calibration_status:
            raise ValueError("active current_field requires calibration_status")
        base = _bounded_vector(
            payload.get("base_velocity_world_mps", fallback),
            length=3,
            label="current_field.base_velocity_world_mps",
            max_abs=_MAX_CURRENT_SPEED_MPS,
        )
        origin = _bounded_vector(
            payload.get("origin_world_m", (0.0, 0.0, 0.0)),
            length=3,
            label="current_field.origin_world_m",
            max_abs=_MAX_QUERY_POSITION_M,
        )
        gradient = _matrix3(
            payload.get("gradient_per_s", np.zeros((3, 3), dtype=np.float64)),
            label="current_field.gradient_per_s",
        )
        if float(np.max(np.abs(gradient))) > _MAX_GRADIENT_PER_S:
            raise ValueError(
                "current_field.gradient_per_s exceeds safe component bound "
                f"{_MAX_GRADIENT_PER_S:g}"
            )
        max_speed_mps = float(payload.get("max_speed_mps", 1.0))
        if (
            not math.isfinite(max_speed_mps)
            or max_speed_mps <= 0.0
            or max_speed_mps > _MAX_CURRENT_SPEED_MPS
        ):
            raise ValueError(
                "current_field.max_speed_mps must be in "
                f"(0, {_MAX_CURRENT_SPEED_MPS:g}]"
            )
        spatial_modes = _parse_spatial_modes(payload.get("spatial_modes", ()))
        temporal_modes = _parse_temporal_modes(payload.get("turbulence", {}))
        return cls(
            CurrentFieldConfig(
                active=True,
                calibration_status=calibration_status,
                base_velocity_world_mps=base.copy(),
                origin_world_m=origin.copy(),
                gradient_per_s=gradient.copy(),
                spatial_modes=spatial_modes,
                temporal_modes=temporal_modes,
                max_speed_mps=max_speed_mps,
            )
        )

    @property
    def active(self) -> bool:
        return bool(self.config.active)

    def velocity_world(self, position_world_m: np.ndarray, time_s: float) -> np.ndarray:
        """Return water velocity [m/s] at one world position and simulation time."""

        position = _bounded_vector(
            position_world_m,
            length=3,
            label="position_world_m",
            max_abs=_MAX_QUERY_POSITION_M,
        )
        time_value = float(time_s)
        if not math.isfinite(time_value) or abs(time_value) > _MAX_QUERY_TIME_S:
            raise ValueError(
                f"time_s must be finite and within +/-{_MAX_QUERY_TIME_S:g}"
            )
        cfg = self.config
        if not cfg.active:
            return cfg.base_velocity_world_mps.copy()

        offset = position - cfg.origin_world_m
        velocity = cfg.base_velocity_world_mps + cfg.gradient_per_s @ offset
        for mode in cfg.spatial_modes:
            phase = math.remainder(
                float(mode.wave_vector_rad_per_m @ offset) + mode.phase_rad,
                _TWO_PI,
            )
            velocity = velocity + mode.amplitude_mps * math.sin(phase)
        for mode in cfg.temporal_modes:
            phase = math.remainder(
                _TWO_PI * mode.frequency_hz * time_value + mode.phase_rad,
                _TWO_PI,
            )
            velocity = velocity + mode.amplitude_mps * math.sin(phase)

        if not np.all(np.isfinite(velocity)):
            raise FloatingPointError("current field produced a non-finite velocity")
        speed = _overflow_safe_norm(velocity)
        if speed > cfg.max_speed_mps:
            velocity = velocity * (cfg.max_speed_mps / speed)
        return np.asarray(velocity, dtype=np.float64)


def _inactive_config(fallback: np.ndarray) -> CurrentFieldConfig:
    return CurrentFieldConfig(
        active=False,
        calibration_status="disabled",
        base_velocity_world_mps=fallback.copy(),
        origin_world_m=np.zeros(3, dtype=np.float64),
        gradient_per_s=np.zeros((3, 3), dtype=np.float64),
        spatial_modes=(),
        temporal_modes=(),
        max_speed_mps=max(float(np.linalg.norm(fallback)), 1.0),
    )


def _parse_spatial_modes(value: Any) -> tuple[SpatialMode, ...]:
    if not isinstance(value, (list, tuple)):
        raise ValueError("current_field.spatial_modes must be an array")
    if len(value) > _MAX_MODE_COUNT:
        raise ValueError(
            f"current_field.spatial_modes supports at most {_MAX_MODE_COUNT} entries"
        )
    modes: list[SpatialMode] = []
    for index, raw in enumerate(value):
        if not isinstance(raw, Mapping):
            raise ValueError(f"current_field.spatial_modes[{index}] must be an object")
        amplitude = _bounded_vector(
            raw.get("amplitude_mps"),
            length=3,
            label=f"current_field.spatial_modes[{index}].amplitude_mps",
            max_abs=_MAX_CURRENT_SPEED_MPS,
        )
        wave_vector = _bounded_vector(
            raw.get("wave_vector_rad_per_m"),
            length=3,
            label=f"current_field.spatial_modes[{index}].wave_vector_rad_per_m",
            max_abs=_MAX_WAVENUMBER_RAD_PER_M,
        )
        phase = float(raw.get("phase_rad", 0.0))
        if not math.isfinite(phase):
            raise ValueError(f"current_field.spatial_modes[{index}].phase_rad must be finite")
        modes.append(SpatialMode(amplitude, wave_vector, phase))
    return tuple(modes)


def _parse_temporal_modes(value: Any) -> tuple[TemporalMode, ...]:
    if value is None:
        return ()
    if not isinstance(value, Mapping):
        raise ValueError("current_field.turbulence must be an object")
    active = value.get("active", False)
    if not isinstance(active, bool):
        raise ValueError("current_field.turbulence.active must be boolean")
    if not active:
        return ()
    seed = value.get("seed", 0)
    if isinstance(seed, bool) or not isinstance(seed, (int, np.integer)):
        raise ValueError("current_field.turbulence.seed must be an integer")
    raw_modes = value.get("modes", ())
    if not isinstance(raw_modes, (list, tuple)):
        raise ValueError("current_field.turbulence.modes must be an array")
    if len(raw_modes) > _MAX_MODE_COUNT:
        raise ValueError(
            f"current_field.turbulence.modes supports at most {_MAX_MODE_COUNT} entries"
        )
    rng = np.random.default_rng(int(seed))
    modes: list[TemporalMode] = []
    for index, raw in enumerate(raw_modes):
        if not isinstance(raw, Mapping):
            raise ValueError(f"current_field.turbulence.modes[{index}] must be an object")
        amplitude = _bounded_vector(
            raw.get("amplitude_mps"),
            length=3,
            label=f"current_field.turbulence.modes[{index}].amplitude_mps",
            max_abs=_MAX_CURRENT_SPEED_MPS,
        )
        frequency = float(raw.get("frequency_hz", 0.0))
        if (
            not math.isfinite(frequency)
            or frequency <= 0.0
            or frequency > _MAX_LOW_FREQUENCY_HZ
        ):
            raise ValueError(
                "current_field.turbulence frequency_hz must be in "
                f"(0, {_MAX_LOW_FREQUENCY_HZ}]"
            )
        phase_value = raw.get("phase_rad")
        phase = float(rng.uniform(-math.pi, math.pi)) if phase_value is None else float(phase_value)
        if not math.isfinite(phase):
            raise ValueError(f"current_field.turbulence.modes[{index}].phase_rad must be finite")
        modes.append(TemporalMode(amplitude, frequency, phase))
    return tuple(modes)


def _overflow_safe_norm(value: np.ndarray) -> float:
    maximum = float(np.max(np.abs(value)))
    if maximum == 0.0:
        return 0.0
    return maximum * float(np.linalg.norm(value / maximum))


__all__ = [
    "CurrentFieldConfig",
    "DeterministicCurrentField",
    "SpatialMode",
    "TemporalMode",
]
