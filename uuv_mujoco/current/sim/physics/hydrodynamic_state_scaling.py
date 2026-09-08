"""Bounded state-dependent scaling for ellipsoid-style hydrodynamics.

This is a configurable identification seam, not a calibrated vehicle model.
When disabled it returns exact ones, preserving the accepted static plant.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np


_CUSTOM_FIELDS = ("added_mass_diag", "linear_damping_diag", "quadratic_damping_diag")
_ALL_FIELDS = (*_CUSTOM_FIELDS, "mujoco_fluidcoef")
_FIELD_LENGTHS = {
    "added_mass_diag": 6,
    "linear_damping_diag": 6,
    "quadratic_damping_diag": 6,
    "mujoco_fluidcoef": 5,
}


@dataclass(frozen=True)
class HydrodynamicStateScales:
    """Multipliers for the custom 6DOF and MuJoCo five-coefficient paths."""

    added_mass_diag: np.ndarray
    linear_damping_diag: np.ndarray
    quadratic_damping_diag: np.ndarray
    mujoco_fluidcoef: np.ndarray


class HydrodynamicStateScaler:
    """Evaluate coefficient multipliers from speed, depth, and hull tilt."""

    def __init__(
        self,
        *,
        active: bool,
        calibration_status: str,
        reference_speed_mps: float,
        speed_power: float,
        reference_depth_m: float,
        depth_scale_m: float,
        basis_limit: float,
        scale_min: float,
        scale_max: float,
        speed_gains: Mapping[str, np.ndarray],
        depth_gains: Mapping[str, np.ndarray],
        attitude_gains: Mapping[str, np.ndarray],
    ) -> None:
        self.active = bool(active)
        self.calibration_status = str(calibration_status)
        self.reference_speed_mps = float(reference_speed_mps)
        self.speed_power = float(speed_power)
        self.reference_depth_m = float(reference_depth_m)
        self.depth_scale_m = float(depth_scale_m)
        self.basis_limit = float(basis_limit)
        self.scale_min = float(scale_min)
        self.scale_max = float(scale_max)
        self.speed_gains = {key: value.copy() for key, value in speed_gains.items()}
        self.depth_gains = {key: value.copy() for key, value in depth_gains.items()}
        self.attitude_gains = {key: value.copy() for key, value in attitude_gains.items()}

    @classmethod
    def from_profile(cls, sim_profile: Mapping[str, Any]) -> "HydrodynamicStateScaler":
        payload = sim_profile.get("hydrodynamic_state_scaling")
        if payload is None:
            return cls.disabled()
        if not isinstance(payload, Mapping):
            raise ValueError("hydrodynamic_state_scaling must be an object")
        active = payload.get("active", False)
        if not isinstance(active, bool):
            raise ValueError("hydrodynamic_state_scaling.active must be boolean")
        if not active:
            return cls.disabled()

        status = str(payload.get("calibration_status", "")).strip()
        if not status:
            raise ValueError("active hydrodynamic_state_scaling requires calibration_status")
        reference_speed = _positive(payload.get("reference_speed_mps", 0.5), "reference_speed_mps")
        speed_power = _positive(payload.get("speed_power", 1.0), "speed_power")
        reference_depth = _finite(payload.get("reference_depth_m", 1.5), "reference_depth_m")
        depth_scale = _positive(payload.get("depth_scale_m", 2.0), "depth_scale_m")
        basis_limit = _positive(payload.get("basis_limit", 3.0), "basis_limit")
        scale_min = _positive(payload.get("scale_min", 0.5), "scale_min")
        scale_max = _positive(payload.get("scale_max", 2.0), "scale_max")
        if scale_max < scale_min:
            raise ValueError("hydrodynamic_state_scaling.scale_max must be >= scale_min")
        return cls(
            active=True,
            calibration_status=status,
            reference_speed_mps=reference_speed,
            speed_power=speed_power,
            reference_depth_m=reference_depth,
            depth_scale_m=depth_scale,
            basis_limit=basis_limit,
            scale_min=scale_min,
            scale_max=scale_max,
            speed_gains=_gain_group(payload, "speed_gains"),
            depth_gains=_gain_group(payload, "depth_gains"),
            attitude_gains=_gain_group(payload, "attitude_gains"),
        )

    @classmethod
    def disabled(cls) -> "HydrodynamicStateScaler":
        zeros = {key: np.zeros(length, dtype=np.float64) for key, length in _FIELD_LENGTHS.items()}
        return cls(
            active=False,
            calibration_status="disabled",
            reference_speed_mps=1.0,
            speed_power=1.0,
            reference_depth_m=0.0,
            depth_scale_m=1.0,
            basis_limit=1.0,
            scale_min=1.0,
            scale_max=1.0,
            speed_gains=zeros,
            depth_gains=zeros,
            attitude_gains=zeros,
        )

    def evaluate(
        self,
        *,
        relative_speed_mps: float,
        depth_m: float,
        base_rotation_world: np.ndarray,
    ) -> HydrodynamicStateScales:
        """Return bounded scale vectors for the given UUV state."""

        if not self.active:
            return _ones_scales()
        speed = max(_finite(relative_speed_mps, "relative_speed_mps"), 0.0)
        depth = _finite(depth_m, "depth_m")
        rotation = np.asarray(base_rotation_world, dtype=np.float64)
        if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
            raise ValueError("base_rotation_world must be a finite 3x3 matrix")

        speed_basis = min((speed / self.reference_speed_mps) ** self.speed_power, self.basis_limit)
        depth_basis = float(
            np.clip(
                (depth - self.reference_depth_m) / self.depth_scale_m,
                -self.basis_limit,
                self.basis_limit,
            )
        )
        # An ellipsoid is invariant to flipping an axis.  The projected-shape
        # tilt basis therefore uses |body-z dot world-z| and remains in [0, 1].
        attitude_basis = 1.0 - abs(float(np.clip(rotation[2, 2], -1.0, 1.0)))
        values = {}
        for key in _ALL_FIELDS:
            scale = (
                1.0
                + self.speed_gains[key] * speed_basis
                + self.depth_gains[key] * depth_basis
                + self.attitude_gains[key] * attitude_basis
            )
            values[key] = np.clip(scale, self.scale_min, self.scale_max)
        return HydrodynamicStateScales(**values)


class FluidcoefStateScalingRuntime:
    """Apply a non-compounding five-coefficient scale after other wrappers."""

    def __init__(self, *, model, fluid_geom_ids, enabled: bool, update_unscaled) -> None:
        self.model = model
        self.geom_ids = np.asarray(tuple(int(item) for item in fluid_geom_ids), dtype=np.int32)
        self.enabled = bool(enabled) and self.geom_ids.size > 0
        self.update_unscaled = update_unscaled
        self.scale = np.ones(5, dtype=np.float64)

    def set_scale(self, value: np.ndarray) -> None:
        scale = np.asarray(value, dtype=np.float64)
        if scale.shape != (5,) or not np.all(np.isfinite(scale)) or np.any(scale <= 0.0):
            raise ValueError("MuJoCo fluidcoef state scale must contain five positive finite values")
        self.scale[:] = scale

    def update(self, rel_lin_vel_body: np.ndarray, ang_vel_body: np.ndarray) -> None:
        self.update_unscaled(rel_lin_vel_body, ang_vel_body)
        if self.enabled:
            self.model.geom_fluid[self.geom_ids, 1:6] *= self.scale[None, :]


def _ones_scales() -> HydrodynamicStateScales:
    return HydrodynamicStateScales(
        added_mass_diag=np.ones(6, dtype=np.float64),
        linear_damping_diag=np.ones(6, dtype=np.float64),
        quadratic_damping_diag=np.ones(6, dtype=np.float64),
        mujoco_fluidcoef=np.ones(5, dtype=np.float64),
    )


def _gain_group(payload: Mapping[str, Any], name: str) -> dict[str, np.ndarray]:
    raw_group = payload.get(name, {})
    if not isinstance(raw_group, Mapping):
        raise ValueError(f"hydrodynamic_state_scaling.{name} must be an object")
    unknown = set(raw_group) - set(_ALL_FIELDS)
    if unknown:
        raise ValueError(
            f"hydrodynamic_state_scaling.{name} has unknown fields: {sorted(unknown)}"
        )
    result: dict[str, np.ndarray] = {}
    for key, length in _FIELD_LENGTHS.items():
        value = np.asarray(raw_group.get(key, np.zeros(length)), dtype=np.float64)
        if value.shape != (length,) or not np.all(np.isfinite(value)):
            raise ValueError(
                f"hydrodynamic_state_scaling.{name}.{key} must contain {length} finite numbers"
            )
        result[key] = value
    return result


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"hydrodynamic_state_scaling.{label} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"hydrodynamic_state_scaling.{label} must be finite")
    return result


def _positive(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise ValueError(f"hydrodynamic_state_scaling.{label} must be positive")
    return result


__all__ = [
    "FluidcoefStateScalingRuntime",
    "HydrodynamicStateScaler",
    "HydrodynamicStateScales",
]
