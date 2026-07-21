"""Safety validation for user-editable simulation-profile physics values.

The limits here are deliberately broad enough for calibration sweeps, while
rejecting values that cannot represent this vehicle (for example a one metre
centre-of-buoyancy offset on a sub-metre hull).  This module is shared by the
runtime and both GUI frontends so an unsafe file cannot bypass GUI checks.
"""

from __future__ import annotations

import math
from typing import Any, Mapping


# Inclusive limits for values exposed by the physics tuning GUI.  Vector
# limits apply independently to every component.
SIM_PROFILE_PARAM_LIMITS: dict[str, tuple[float, float]] = {
    "buoyancy_scale": (0.95, 1.05),
    "buoyancy_slope_scale": (0.10, 5.0),
    "cob_torque_scale": (0.0, 5.0),
    "cob_z_offset": (-0.25, 0.25),
    "cob_x_offset": (-0.20, 0.20),
    "buoyancy_point_blend": (0.0, 1.0),
    "thruster_force_max": (1.0, 100.0),
    "mujoco_fluidcoef_scale": (0.0, 20.0),
    "current_world": (-3.0, 3.0),
    "water_current_world": (-3.0, 3.0),
    "body_inertia_scale_xyz": (0.10, 5.0),
    "yaw_torque_scale": (0.0, 5.0),
}

SIM_PROFILE_VECTOR_LENGTHS: dict[str, int] = {
    "mujoco_fluidcoef_scale": 5,
    "current_world": 3,
    "water_current_world": 3,
    "body_inertia_scale_xyz": 3,
}

# Runtime-consumed numeric fields without a useful universal calibration range.
# They still need type/finite checks: JSON booleans are Python integers and
# would otherwise silently become 0.0/1.0 at the float(...) call sites.
SIM_PROFILE_FINITE_SCALAR_FIELDS = frozenset(
    {
        "air_angular_drag",
        "air_linear_drag",
        "angular_drag",
        "forward_speed_z_r",
        "half_height",
        "heave_damping_scale",
        "heave_extra_damping_n_per_mps",
        "horizontal_thruster_z_offset_m",
        "hydro_pitch_moment_coeff",
        "hydro_vertical_lift_coeff",
        "hydro_vertical_lift_deadband_mps",
        "hydro_vertical_lift_power",
        "hydro_yawrate_heave_neg_coeff",
        "hydro_yawrate_heave_pos_coeff",
        "hydro_yawrate_heave_speed_deadband_mps",
        "hydro_yawrate_heave_yaw_deadband_radps",
        "linear_drag",
        "linear_z_w",
        "quadratic_z_abs_w_w",
        "spin_gain",
        "surface_heave_damping",
        "thruster_voltage",
    }
)

# The legacy/custom 6DOF path accepts either one six-vector or separate
# three-vectors.  A malformed explicit value currently falls back silently, so
# validate shape and finiteness whenever the user supplied one.
SIM_PROFILE_OPTIONAL_VECTOR_LENGTHS: dict[str, int] = {
    "added_mass_diag": 6,
    "added_mass_linear": 3,
    "added_mass_angular": 3,
    "linear_damping_diag": 6,
    "linear_damping_linear": 3,
    "linear_damping_angular": 3,
    "quadratic_damping_diag": 6,
    "quadratic_damping_linear": 3,
    "quadratic_damping_angular": 3,
}

_FOSSEN_COEFFICIENT_KEYS: dict[str, frozenset[str]] = {
    "linear": frozenset({"y_v", "y_r", "n_v", "n_r", "z_w", "z_q", "m_w", "m_q"}),
    "forward_speed_linear": frozenset(
        {"y_v", "y_r", "n_v", "n_r", "z_w", "z_q", "z_r", "m_w", "m_q"}
    ),
    "quadratic": frozenset(
        {
            "x_abs_u_u",
            "y_abs_v_v",
            "n_abs_r_r",
            "z_abs_w_w",
            "m_abs_q_q",
            "y_abs_r_r",
            "n_abs_v_v",
            "z_abs_q_q",
            "m_abs_w_w",
        }
    ),
    "added_mass": frozenset(
        {"x_u", "y_v", "z_w", "k_p", "m_q", "n_r", "y_r_n_v", "z_q_m_w"}
    ),
}


def validate_profile_param_value(key: str, value: Any, *, label: str | None = None) -> Any:
    """Return *value* after finite/range validation for a known physics key."""

    name = str(key)
    display = str(label or name)
    limits = SIM_PROFILE_PARAM_LIMITS.get(name)
    expected_length = SIM_PROFILE_VECTOR_LENGTHS.get(name)
    if isinstance(value, (list, tuple)):
        if expected_length is None:
            raise TypeError(f"{display} must be a scalar")
        if len(value) != expected_length:
            raise ValueError(f"{display} must have exactly {expected_length} numbers")
        return [
            _validated_number(item, f"{display}[{index}]", limits)
            for index, item in enumerate(value)
        ]
    if expected_length is not None:
        raise TypeError(f"{display} must contain exactly {expected_length} numbers")
    return _validated_number(value, display, limits)


def validate_sim_profile(profile: Mapping[str, Any], *, profile_name: str = "") -> None:
    """Reject unsafe values before they can reach MuJoCo runtime setup."""

    prefix = f"profile '{profile_name}' " if profile_name else "profile "
    if "mujoco_fluidcoef_immersion_scale" in profile and not isinstance(
        profile["mujoco_fluidcoef_immersion_scale"], bool
    ):
        raise ValueError(f"{prefix}mujoco_fluidcoef_immersion_scale must be boolean")
    for key, limits in SIM_PROFILE_PARAM_LIMITS.items():
        if key not in profile:
            continue
        try:
            validate_profile_param_value(key, profile[key])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{prefix}{exc}") from exc

    try:
        _validate_water_current_aliases(profile)
        _validate_finite_scalar_fields(profile)
        _validate_optional_vectors(profile)
        _validate_hydrostatic_restoring(profile)
        _validate_fossen_residual_hydro(profile)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{prefix}{exc}") from exc


def _validate_water_current_aliases(profile: Mapping[str, Any]) -> None:
    """Prevent two valid aliases from silently selecting different currents."""

    if "current_world" not in profile or "water_current_world" not in profile:
        return
    current = validate_profile_param_value("current_world", profile["current_world"])
    water_current = validate_profile_param_value(
        "water_current_world",
        profile["water_current_world"],
    )
    if any(
        not math.isclose(left, right, rel_tol=0.0, abs_tol=1.0e-12)
        for left, right in zip(current, water_current)
    ):
        raise ValueError(
            "current_world and water_current_world must match when both are supplied; "
            "use current_world as the canonical field"
        )


def _validate_finite_scalar_fields(profile: Mapping[str, Any]) -> None:
    for key in SIM_PROFILE_FINITE_SCALAR_FIELDS:
        if key in profile and profile[key] is not None:
            _validated_number(profile[key], key, None)


def _validate_optional_vectors(profile: Mapping[str, Any]) -> None:
    for key, expected_length in SIM_PROFILE_OPTIONAL_VECTOR_LENGTHS.items():
        if key not in profile or profile[key] is None:
            continue
        _validated_vector(profile[key], key, expected_length)


def _validate_hydrostatic_restoring(profile: Mapping[str, Any]) -> None:
    if "hydrostatic_restoring" not in profile:
        return
    restoring = profile["hydrostatic_restoring"]
    if not isinstance(restoring, Mapping):
        raise TypeError("hydrostatic_restoring must be an object")
    _validate_optional_bool(restoring, "active", "hydrostatic_restoring.active")
    _validate_optional_bool(
        restoring,
        "trim_from_real_start",
        "hydrostatic_restoring.trim_from_real_start",
    )
    for key in (
        "roll_stiffness_nm_per_rad",
        "pitch_stiffness_nm_per_rad",
        "roll_stiffness",
        "pitch_stiffness",
    ):
        if key in restoring:
            _validated_number(restoring[key], f"hydrostatic_restoring.{key}", (0.0, 1000.0))
    for key in ("roll_trim_rad", "pitch_trim_rad", "trim_roll_rad", "trim_pitch_rad"):
        if key in restoring:
            _validated_number(restoring[key], f"hydrostatic_restoring.{key}", None)
    if "release_trim_blend_s" in restoring:
        _validated_number(
            restoring["release_trim_blend_s"],
            "hydrostatic_restoring.release_trim_blend_s",
            (0.0, float("inf")),
        )


def _validate_fossen_residual_hydro(profile: Mapping[str, Any]) -> None:
    if "fossen_residual_hydro" not in profile:
        return
    fossen = profile["fossen_residual_hydro"]
    if not isinstance(fossen, Mapping):
        raise TypeError("fossen_residual_hydro must be an object")
    _validate_optional_bool(fossen, "active", "fossen_residual_hydro.active")

    # These optional direct forms are reserved for matrix-based extensions.
    # The current runtime uses the named coefficient groups below; validating a
    # supplied direct form prevents malformed arrays from entering a future
    # active consumer without rejecting unrelated extension metadata.
    if "added_mass_diag" in fossen:
        _validated_vector(
            fossen["added_mass_diag"],
            "fossen_residual_hydro.added_mass_diag",
            6,
        )
    if "added_mass_matrix" in fossen:
        _validated_matrix(
            fossen["added_mass_matrix"],
            "fossen_residual_hydro.added_mass_matrix",
            (6, 6),
        )

    for group_name, coefficient_keys in _FOSSEN_COEFFICIENT_KEYS.items():
        if group_name not in fossen:
            continue
        group = fossen[group_name]
        if not isinstance(group, Mapping):
            raise TypeError(f"fossen_residual_hydro.{group_name} must be an object")
        if group_name == "added_mass":
            _validate_optional_bool(
                group,
                "active",
                "fossen_residual_hydro.added_mass.active",
            )
        for coefficient in coefficient_keys:
            if coefficient not in group:
                continue
            _validated_number(
                group[coefficient],
                f"fossen_residual_hydro.{group_name}.{coefficient}",
                None,
            )


def _validate_optional_bool(payload: Mapping[str, Any], key: str, label: str) -> None:
    if key in payload and not isinstance(payload[key], bool):
        raise TypeError(f"{label} must be boolean")


def _validated_vector(value: Any, label: str, expected_length: int) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != expected_length:
        raise ValueError(f"{label} must have exactly {expected_length} numbers")
    return [_validated_number(item, f"{label}[{index}]", None) for index, item in enumerate(value)]


def _validated_matrix(
    value: Any,
    label: str,
    expected_shape: tuple[int, int],
) -> list[list[float]]:
    rows, columns = expected_shape
    if not isinstance(value, (list, tuple)) or len(value) != rows:
        raise ValueError(f"{label} must have shape {rows}x{columns}")
    matrix: list[list[float]] = []
    for row_index, row in enumerate(value):
        try:
            matrix.append(_validated_vector(row, f"{label}[{row_index}]", columns))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{label} must have shape {rows}x{columns}: {exc}") from exc
    return matrix


def _validated_number(
    value: Any,
    label: str,
    limits: tuple[float, float] | None,
) -> float:
    if isinstance(value, bool):
        raise TypeError(f"{label} must be numeric, not boolean")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise TypeError(f"{label} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    if limits is not None:
        lower, upper = limits
        if number < lower or number > upper:
            raise ValueError(f"{label} must be in [{lower:g}, {upper:g}], got {number:g}")
    return number


__all__ = [
    "SIM_PROFILE_PARAM_LIMITS",
    "SIM_PROFILE_FINITE_SCALAR_FIELDS",
    "SIM_PROFILE_OPTIONAL_VECTOR_LENGTHS",
    "SIM_PROFILE_VECTOR_LENGTHS",
    "validate_profile_param_value",
    "validate_sim_profile",
]
