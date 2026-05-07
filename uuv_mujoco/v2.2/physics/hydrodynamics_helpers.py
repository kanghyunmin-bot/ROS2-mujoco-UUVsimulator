"""Small math helpers for underwater vehicle dynamics."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class EllipsoidHydroEstimate:
    """Baseline hydrodynamic coefficients derived from an equivalent ellipsoid."""

    semi_axes: np.ndarray
    displaced_volume: float
    added_mass_diag: np.ndarray
    linear_damping_diag: np.ndarray
    quadratic_damping_diag: np.ndarray


def skew(vec: Iterable[float]) -> np.ndarray:
    x, y, z = np.asarray(list(vec), dtype=np.float64)
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


def added_mass_coriolis(added_mass_diag: np.ndarray, nu_body: np.ndarray) -> np.ndarray:
    """Return a simple 6-DOF added-mass Coriolis matrix.

    `nu_body` follows Fossen ordering `[u, v, w, p, q, r]`.
    """

    mass_matrix = np.diag(np.asarray(added_mass_diag, dtype=np.float64))
    nu = np.asarray(nu_body, dtype=np.float64).reshape(6)
    linear = nu[:3]
    angular = nu[3:]
    a_term = mass_matrix[:3, :3] @ linear + mass_matrix[:3, 3:] @ angular
    b_term = mass_matrix[3:, :3] @ linear + mass_matrix[3:, 3:] @ angular
    coriolis = np.zeros((6, 6), dtype=np.float64)
    coriolis[:3, 3:] = -skew(a_term)
    coriolis[3:, :3] = -skew(a_term)
    coriolis[3:, 3:] = -skew(b_term)
    return coriolis


def first_order_response(current: float, target: float, dt: float, tau_up: float, tau_down: float) -> float:
    if dt <= 0.0:
        return current
    tau = tau_up if abs(target) >= abs(current) else tau_down
    tau = max(float(tau), 1e-6)
    alpha = 1.0 - math.exp(-dt / tau)
    return float(current + alpha * (target - current))


def shape_thruster_command(command: float, deadzone: float, command_limit: float) -> float:
    """Apply deadzone and saturation, returning a normalized command in [-1, 1]."""

    deadzone = float(np.clip(deadzone, 0.0, 0.95))
    command_limit = float(np.clip(command_limit, deadzone + 1e-3, 1.0))
    signed = float(np.clip(command, -1.0, 1.0))
    sign = -1.0 if signed < 0.0 else 1.0
    magnitude = min(abs(signed), command_limit)
    if magnitude <= deadzone:
        return 0.0
    span = max(command_limit - deadzone, 1e-6)
    return sign * float(np.clip((magnitude - deadzone) / span, 0.0, 1.0))


def polyval_ascending(coeffs: Iterable[float], x: float) -> float:
    total = 0.0
    power = 1.0
    for coeff in coeffs:
        total += float(coeff) * power
        power *= x
    return total


def scaled_polynomial_force(magnitude: float, coeffs: Iterable[float], force_max: float) -> float:
    magnitude = float(np.clip(magnitude, 0.0, 1.0))
    coeff_list = list(coeffs)
    if not coeff_list:
        return force_max * magnitude
    raw = polyval_ascending(coeff_list, magnitude)
    raw_full = polyval_ascending(coeff_list, 1.0)
    if abs(raw_full) < 1e-9:
        return force_max * magnitude
    return float(force_max * raw / raw_full)


def submerged_fraction_linear(depth: float, half_height: float) -> float:
    """Return linear immersed fraction for a box-like hydrostatic proxy."""

    half_height = max(float(half_height), 1e-6)
    return float(np.clip((float(depth) + half_height) / (2.0 * half_height), 0.0, 1.0))


def submerged_fraction_ellipsoid(depth: float, half_height: float) -> float:
    """Return immersed fraction for an ellipsoid cut by a horizontal water plane.

    This assumes a symmetric ellipsoid-like displaced shape in the vertical axis.
    The resulting volume fraction varies smoothly with depth and has zero slope
    near the fully dry / fully submerged limits, which reduces z-axis chatter
    around the waterline compared with a linear ramp.
    """

    half_height = max(float(half_height), 1e-6)
    normalized_depth = float(np.clip(float(depth) / half_height, -1.0, 1.0))
    frac = 0.5 + 0.75 * normalized_depth - 0.25 * (normalized_depth ** 3)
    return float(np.clip(frac, 0.0, 1.0))


def submerged_fraction(depth: float, half_height: float, model: str = "ellipsoid") -> float:
    """Return immersed fraction using the selected hydrostatic proxy model."""

    name = str(model).strip().lower()
    if name == "linear":
        return submerged_fraction_linear(depth, half_height)
    return submerged_fraction_ellipsoid(depth, half_height)


def ellipsoid_volume(semi_axes: Iterable[float]) -> float:
    a, b, c = np.asarray(list(semi_axes), dtype=np.float64)
    if min(a, b, c) <= 0.0:
        raise ValueError("semi_axes must be positive")
    return float((4.0 / 3.0) * math.pi * a * b * c)


def ellipsoid_projected_areas(semi_axes: Iterable[float]) -> np.ndarray:
    a, b, c = np.asarray(list(semi_axes), dtype=np.float64)
    if min(a, b, c) <= 0.0:
        raise ValueError("semi_axes must be positive")
    return np.array(
        [
            math.pi * b * c,
            math.pi * a * c,
            math.pi * a * b,
        ],
        dtype=np.float64,
    )


def ellipsoid_depolarization_factors(semi_axes: Iterable[float], samples: int = 4096) -> np.ndarray:
    """Approximate triaxial-ellipsoid depolarization factors via numeric quadrature."""

    a, b, c = np.asarray(list(semi_axes), dtype=np.float64)
    if min(a, b, c) <= 0.0:
        raise ValueError("semi_axes must be positive")
    samples = int(max(samples, 512))
    t = np.linspace(0.0, 1.0 - 1e-7, samples, dtype=np.float64)
    s = t / np.maximum(1.0 - t, 1e-12)
    jac = 1.0 / np.maximum((1.0 - t) ** 2, 1e-12)
    a2, b2, c2 = a * a, b * b, c * c
    root = np.sqrt((s + a2) * (s + b2) * (s + c2))
    prefactor = (a * b * c) / 2.0
    factors = []
    integrate = getattr(np, "trapezoid", None)
    if integrate is None:
        integrate = np.trapz
    for axis_sq in (a2, b2, c2):
        integrand = prefactor / ((s + axis_sq) * root)
        factors.append(float(integrate(integrand * jac, t)))
    out = np.clip(np.array(factors, dtype=np.float64), 1e-9, 1.0)
    total = float(np.sum(out))
    if total <= 1e-9:
        return np.full(3, 1.0 / 3.0, dtype=np.float64)
    return out / total


def estimate_ellipsoid_hydrodynamics(
    semi_axes: Iterable[float],
    fluid_density: float,
    *,
    effective_cd_linear: Iterable[float] = (0.10, 0.11, 0.13),
    effective_cd_angular: Iterable[float] = (2.2, 2.4, 1.8),
    added_mass_scale_linear: Iterable[float] = (1.0, 1.0, 1.0),
    added_mass_scale_angular: Iterable[float] = (1.2, 1.2, 1.0),
    linear_damping_ratio_linear: float = 2.0,
    linear_damping_ratio_angular: float = 1.0,
    reference_speed_linear: float = 0.30,
    reference_speed_angular: float = 0.75,
) -> EllipsoidHydroEstimate:
    """Build a tuned ellipsoid baseline for the existing 6-DOF coefficient model.

    The goal is not a full CFD-grade model. This provides a shape-based starting
    point that can still be tuned with the existing per-axis coefficient overrides.
    """

    semi = np.asarray(list(semi_axes), dtype=np.float64).reshape(3)
    if np.any(semi <= 0.0):
        raise ValueError("semi_axes must be positive")
    rho = float(max(fluid_density, 1e-6))
    volume = ellipsoid_volume(semi)
    areas = ellipsoid_projected_areas(semi)
    depol = ellipsoid_depolarization_factors(semi)

    cd_linear = np.clip(np.asarray(list(effective_cd_linear), dtype=np.float64).reshape(3), 0.0, 10.0)
    cd_angular = np.clip(np.asarray(list(effective_cd_angular), dtype=np.float64).reshape(3), 0.0, 20.0)
    scale_linear = np.clip(np.asarray(list(added_mass_scale_linear), dtype=np.float64).reshape(3), 0.0, 5.0)
    scale_angular = np.clip(np.asarray(list(added_mass_scale_angular), dtype=np.float64).reshape(3), 0.0, 5.0)

    base_added_linear = rho * volume * depol / np.maximum(1.0 - depol, 1e-6)
    added_mass_linear = base_added_linear * scale_linear

    # Approximate rotational added inertia from translational added mass and the
    # squared distance of the perpendicular semi-axes.
    radius_sq = np.array(
        [
            0.5 * (semi[1] ** 2 + semi[2] ** 2),
            0.5 * (semi[0] ** 2 + semi[2] ** 2),
            0.5 * (semi[0] ** 2 + semi[1] ** 2),
        ],
        dtype=np.float64,
    )
    paired_added_linear = np.array(
        [
            0.5 * (added_mass_linear[1] + added_mass_linear[2]),
            0.5 * (added_mass_linear[0] + added_mass_linear[2]),
            0.5 * (added_mass_linear[0] + added_mass_linear[1]),
        ],
        dtype=np.float64,
    )
    added_mass_angular = paired_added_linear * radius_sq * scale_angular

    quadratic_linear = 0.5 * rho * areas * cd_linear
    char_length = np.array(
        [
            0.5 * (semi[1] + semi[2]),
            0.5 * (semi[0] + semi[2]),
            0.5 * (semi[0] + semi[1]),
        ],
        dtype=np.float64,
    )
    quadratic_angular = 0.5 * rho * areas * cd_angular * (char_length ** 3)

    linear_linear = quadratic_linear * max(float(reference_speed_linear), 1e-6) * max(
        float(linear_damping_ratio_linear), 0.0
    )
    linear_angular = quadratic_angular * max(float(reference_speed_angular), 1e-6) * max(
        float(linear_damping_ratio_angular), 0.0
    )

    return EllipsoidHydroEstimate(
        semi_axes=semi,
        displaced_volume=float(volume),
        added_mass_diag=np.concatenate((added_mass_linear, added_mass_angular)).astype(np.float64, copy=False),
        linear_damping_diag=np.concatenate((linear_linear, linear_angular)).astype(np.float64, copy=False),
        quadratic_damping_diag=np.concatenate((quadratic_linear, quadratic_angular)).astype(np.float64, copy=False),
    )
