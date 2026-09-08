"""Bounded axial-inflow correction for fixed-pitch underwater thrusters.

The existing command-to-force curve represents static (bollard-pull) thrust.
This module applies an optional low-order normalized axial-speed correction
without claiming to resolve propeller advance ratio, wake, or blade physics.
Legacy configuration fields retain ``advance_ratio`` in their names for
compatibility; the value is not ``J = V_A / (n D)``.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping

import numpy as np


@dataclass(frozen=True)
class ThrusterInflowConfig:
    """Parameters for the bounded axial-inflow thrust correction."""

    enabled: bool = False
    reference_speed_mps: float = 1.0
    minimum_reference_speed_mps: float = 0.15
    command_exponent: float = 0.5
    gain_per_advance_ratio: float = 0.35
    minimum_multiplier: float = 0.45
    maximum_multiplier: float = 1.20

    @classmethod
    def from_mapping(cls, params: Mapping[str, Any]) -> "ThrusterInflowConfig":
        """Parse an inflow configuration from the global thruster parameters."""

        minimum_multiplier = _finite_float(
            params.get("inflow_minimum_multiplier", cls.minimum_multiplier),
            default=cls.minimum_multiplier,
        )
        maximum_multiplier = _finite_float(
            params.get("inflow_maximum_multiplier", cls.maximum_multiplier),
            default=cls.maximum_multiplier,
        )
        minimum_multiplier = float(np.clip(minimum_multiplier, 0.0, 1.0))
        maximum_multiplier = float(np.clip(maximum_multiplier, 1.0, 3.0))
        return cls(
            enabled=bool(params.get("inflow_enabled", cls.enabled)),
            reference_speed_mps=max(
                _finite_float(
                    params.get("inflow_reference_speed_mps", cls.reference_speed_mps),
                    default=cls.reference_speed_mps,
                ),
                1.0e-6,
            ),
            minimum_reference_speed_mps=max(
                _finite_float(
                    params.get(
                        "inflow_minimum_reference_speed_mps",
                        cls.minimum_reference_speed_mps,
                    ),
                    default=cls.minimum_reference_speed_mps,
                ),
                1.0e-6,
            ),
            command_exponent=float(
                np.clip(
                    _finite_float(
                        params.get("inflow_command_exponent", cls.command_exponent),
                        default=cls.command_exponent,
                    ),
                    0.0,
                    2.0,
                )
            ),
            gain_per_advance_ratio=max(
                _finite_float(
                    params.get(
                        "inflow_gain_per_advance_ratio",
                        cls.gain_per_advance_ratio,
                    ),
                    default=cls.gain_per_advance_ratio,
                ),
                0.0,
            ),
            minimum_multiplier=minimum_multiplier,
            maximum_multiplier=max(maximum_multiplier, minimum_multiplier),
        )


@dataclass(frozen=True)
class ThrusterInflowResult:
    """One evaluated inflow correction."""

    force_n: float
    multiplier: float
    axial_advance_speed_mps: float
    advance_ratio: float


_PROFILE_FIELDS = {
    "reference_speed_mps": "inflow_reference_speed_mps",
    "minimum_reference_speed_mps": "inflow_minimum_reference_speed_mps",
    "command_exponent": "inflow_command_exponent",
    "gain_per_advance_ratio": "inflow_gain_per_advance_ratio",
    "minimum_multiplier": "inflow_minimum_multiplier",
    "maximum_multiplier": "inflow_maximum_multiplier",
}


def thruster_inflow_profile_overrides(
    sim_profile: Mapping[str, Any],
) -> tuple[dict[str, float | bool], str, str] | None:
    """Return strict global-parameter overrides from ``thruster_inflow``.

    An explicit profile section is separate from the editable global JSON so
    research profiles can enable the model without changing the accepted
    default plant. Active configurations must disclose calibration status and
    provenance.
    """

    payload = sim_profile.get("thruster_inflow")
    if payload is None:
        return None
    if not isinstance(payload, Mapping):
        raise ValueError("thruster_inflow must be an object")
    active = payload.get("active", False)
    if not isinstance(active, bool):
        raise ValueError("thruster_inflow.active must be boolean")
    if not active:
        return ({"inflow_enabled": False}, "disabled", "disabled")

    status = _required_profile_string(payload, "calibration_status")
    provenance = _required_profile_string(payload, "provenance")
    values: dict[str, float | bool] = {"inflow_enabled": True}
    for profile_key, global_key in _PROFILE_FIELDS.items():
        if profile_key not in payload:
            continue
        raw_value = payload[profile_key]
        if isinstance(raw_value, (bool, np.bool_)):
            raise ValueError(f"thruster_inflow.{profile_key} must be numeric")
        try:
            parsed = float(raw_value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"thruster_inflow.{profile_key} must be numeric") from exc
        if not math.isfinite(parsed):
            raise ValueError(f"thruster_inflow.{profile_key} must be finite")
        values[global_key] = parsed

    reference_speed = float(
        values.get("inflow_reference_speed_mps", ThrusterInflowConfig.reference_speed_mps)
    )
    minimum_reference_speed = float(
        values.get(
            "inflow_minimum_reference_speed_mps",
            ThrusterInflowConfig.minimum_reference_speed_mps,
        )
    )
    command_exponent = float(
        values.get("inflow_command_exponent", ThrusterInflowConfig.command_exponent)
    )
    gain = float(
        values.get(
            "inflow_gain_per_advance_ratio",
            ThrusterInflowConfig.gain_per_advance_ratio,
        )
    )
    minimum_multiplier = float(
        values.get("inflow_minimum_multiplier", ThrusterInflowConfig.minimum_multiplier)
    )
    maximum_multiplier = float(
        values.get("inflow_maximum_multiplier", ThrusterInflowConfig.maximum_multiplier)
    )
    if not 0.0 < reference_speed <= 20.0:
        raise ValueError("thruster_inflow.reference_speed_mps must be in (0, 20]")
    if not 0.0 < minimum_reference_speed <= reference_speed:
        raise ValueError(
            "thruster_inflow.minimum_reference_speed_mps must be positive and no "
            "greater than reference_speed_mps"
        )
    if not 0.0 <= command_exponent <= 2.0:
        raise ValueError("thruster_inflow.command_exponent must be in [0, 2]")
    if not 0.0 <= gain <= 10.0:
        raise ValueError("thruster_inflow.gain_per_advance_ratio must be in [0, 10]")
    if not 0.0 <= minimum_multiplier <= 1.0:
        raise ValueError("thruster_inflow.minimum_multiplier must be in [0, 1]")
    if not 1.0 <= maximum_multiplier <= 3.0:
        raise ValueError("thruster_inflow.maximum_multiplier must be in [1, 3]")
    if minimum_multiplier > maximum_multiplier:
        raise ValueError(
            "thruster_inflow.minimum_multiplier must not exceed maximum_multiplier"
        )
    config = ThrusterInflowConfig(
        enabled=True,
        reference_speed_mps=reference_speed,
        minimum_reference_speed_mps=minimum_reference_speed,
        command_exponent=command_exponent,
        gain_per_advance_ratio=gain,
        minimum_multiplier=minimum_multiplier,
        maximum_multiplier=maximum_multiplier,
    )
    values.update(
        {
            "inflow_reference_speed_mps": config.reference_speed_mps,
            "inflow_minimum_reference_speed_mps": config.minimum_reference_speed_mps,
            "inflow_command_exponent": config.command_exponent,
            "inflow_gain_per_advance_ratio": config.gain_per_advance_ratio,
            "inflow_minimum_multiplier": config.minimum_multiplier,
            "inflow_maximum_multiplier": config.maximum_multiplier,
        }
    )
    return values, status, provenance


def apply_thruster_inflow(
    static_force_n: float,
    *,
    command_fraction: float,
    thrust_axis_world: np.ndarray,
    site_velocity_world_mps: np.ndarray,
    water_velocity_world_mps: np.ndarray,
    config: ThrusterInflowConfig,
) -> ThrusterInflowResult:
    """Correct static thrust for local axial water inflow.

    Positive advance speed means the thruster site is moving through water in
    the commanded thrust direction.  At fixed command this reduces thrust;
    an opposing flow can increase it, within configured bounds.
    """

    static_force = float(static_force_n)
    if not config.enabled or abs(static_force) <= 1.0e-12:
        return ThrusterInflowResult(
            force_n=static_force,
            multiplier=1.0,
            axial_advance_speed_mps=0.0,
            advance_ratio=0.0,
        )

    axis = _unit_vector(thrust_axis_world, label="thrust_axis_world")
    site_velocity = _vector3(site_velocity_world_mps, label="site_velocity_world_mps")
    water_velocity = _vector3(water_velocity_world_mps, label="water_velocity_world_mps")
    commanded_axis = math.copysign(1.0, static_force) * axis
    axial_advance_speed = float((site_velocity - water_velocity) @ commanded_axis)

    command_magnitude = float(np.clip(abs(command_fraction), 0.0, 1.0))
    effective_reference_speed = max(
        config.minimum_reference_speed_mps,
        config.reference_speed_mps * command_magnitude**config.command_exponent,
    )
    advance_ratio = axial_advance_speed / effective_reference_speed
    multiplier = float(
        np.clip(
            1.0 - config.gain_per_advance_ratio * advance_ratio,
            config.minimum_multiplier,
            config.maximum_multiplier,
        )
    )
    return ThrusterInflowResult(
        force_n=static_force * multiplier,
        multiplier=multiplier,
        axial_advance_speed_mps=axial_advance_speed,
        advance_ratio=advance_ratio,
    )


def _finite_float(value: Any, *, default: float) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return float(default)
    return parsed if math.isfinite(parsed) else float(default)


def _required_profile_string(payload: Mapping[str, Any], key: str) -> str:
    value = payload.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"active thruster_inflow requires {key}")
    return value.strip()


def _vector3(value: np.ndarray, *, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain three finite values")
    return result


def _unit_vector(value: np.ndarray, *, label: str) -> np.ndarray:
    vector = _vector3(value, label=label)
    norm = float(np.linalg.norm(vector))
    if norm <= 1.0e-12:
        raise ValueError(f"{label} must have non-zero length")
    return vector / norm


__all__ = [
    "ThrusterInflowConfig",
    "ThrusterInflowResult",
    "apply_thruster_inflow",
    "thruster_inflow_profile_overrides",
]
