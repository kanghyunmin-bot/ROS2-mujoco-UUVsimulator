"""Validated full-matrix Fossen hydrodynamics for a rigid underwater vehicle.

The model is opt-in and deliberately independent from the active MuJoCo
runtime.  It provides a calibration seam for a constant 6-DOF added-mass
matrix and symmetric linear and quadratic damping matrices without silently
changing the accepted diagonal model.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping

import numpy as np


_CONFIG_KEY = "hydrodynamic_matrices"
_MATRIX_SHAPE = (6, 6)
_SYMMETRY_TOLERANCE = 1.0e-9
_PASSIVITY_TOLERANCE = 1.0e-9
_MAX_MATRIX_ABS_ENTRY = 1.0e9
_MAX_STATE_ABS_COMPONENT = 1.0e6
_MAX_WRENCH_BOUND = 1.0e20


@dataclass(frozen=True)
class HydrodynamicMatricesConfig:
    """Validated body-frame generalized hydrodynamic matrices.

    Generalized-vector ordering is ``[u, v, w, p, q, r]``.  Added-mass
    entries carry the corresponding generalized inertia units, linear
    damping entries carry generalized wrench per velocity units, and
    quadratic damping entries carry generalized wrench per squared velocity
    units.
    """

    active: bool
    calibration_status: str
    provenance: str
    reference_point: str
    added_mass_6x6: np.ndarray
    linear_damping_6x6: np.ndarray
    quadratic_damping_6x6: np.ndarray
    max_force_n: float
    max_torque_nm: float


class FullMatrixHydrodynamics:
    """Evaluate a constant full 6-DOF hydrodynamic model in the body frame."""

    def __init__(self, config: HydrodynamicMatricesConfig) -> None:
        self.config = config

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, Any] | None,
    ) -> "FullMatrixHydrodynamics":
        """Construct from a direct ``hydrodynamic_matrices`` mapping."""

        return cls(parse_hydrodynamic_matrices(value))

    @classmethod
    def from_profile(
        cls,
        sim_profile: Mapping[str, Any] | None,
    ) -> "FullMatrixHydrodynamics":
        """Construct from the optional section of a simulation profile."""

        if sim_profile is None:
            return cls(_disabled_config())
        if not isinstance(sim_profile, Mapping):
            raise ValueError("sim_profile must be an object")
        return cls.from_mapping(sim_profile.get(_CONFIG_KEY))

    @property
    def active(self) -> bool:
        """Return whether this opt-in model is enabled."""

        return bool(self.config.active)

    def added_mass_coriolis_body(self, velocity_body: np.ndarray) -> np.ndarray:
        """Return the Fossen added-mass Coriolis matrix for body velocity.

        Args:
            velocity_body: Body-relative velocity ``[u, v, w, p, q, r]``
                [m/s, rad/s].

        Returns:
            A 6x6 generalized Coriolis matrix.  A disabled model returns an
            exact zero matrix without inspecting the input.
        """

        if not self.active:
            return np.zeros(_MATRIX_SHAPE, dtype=np.float64)
        velocity = _vector6(velocity_body, label="velocity_body")
        return added_mass_coriolis_matrix(self.config.added_mass_6x6, velocity)

    def wrench_body(
        self,
        velocity_body: np.ndarray,
        acceleration_body: np.ndarray,
    ) -> np.ndarray:
        """Return the hydrodynamic body wrench ``[Fx,Fy,Fz,Mx,My,Mz]``.

        The evaluated model is
        ``-M_A nu_dot - C_A(nu) nu - D_l nu - Q Lambda |z| z``,
        where ``D_q = Q Lambda Q.T`` and ``z = Q.T nu``. This modal form
        preserves quadratic scaling, allows cross-axis output, and guarantees
        nonnegative damping power for positive-semidefinite ``D_q``.

        Args:
            velocity_body: Body-relative velocity ``[u, v, w, p, q, r]``
                [m/s, rad/s].
            acceleration_body: Body-relative acceleration in matching Fossen
                ordering [m/s^2, rad/s^2].

        Returns:
            Generalized body wrench [N, N*m].  A disabled model returns an
            exact zero vector without inspecting either input.
        """

        if not self.active:
            return np.zeros(6, dtype=np.float64)

        velocity = _vector6(velocity_body, label="velocity_body")
        acceleration = _vector6(acceleration_body, label="acceleration_body")
        cfg = self.config
        try:
            with np.errstate(over="raise", invalid="raise"):
                coriolis = added_mass_coriolis_matrix(
                    cfg.added_mass_6x6,
                    velocity,
                )
                eigenvalues, eigenvectors = np.linalg.eigh(
                    cfg.quadratic_damping_6x6
                )
                eigenvalues = np.maximum(eigenvalues, 0.0)
                modal_velocity = eigenvectors.T @ velocity
                quadratic_wrench = eigenvectors @ (
                    eigenvalues
                    * np.abs(modal_velocity)
                    * modal_velocity
                )
                wrench = -(
                    cfg.added_mass_6x6 @ acceleration
                    + coriolis @ velocity
                    + cfg.linear_damping_6x6 @ velocity
                    + quadratic_wrench
                )
        except (FloatingPointError, np.linalg.LinAlgError) as exc:
            raise FloatingPointError(
                "full-matrix hydrodynamics produced an invalid wrench"
            ) from exc
        if not np.all(np.isfinite(wrench)):
            raise FloatingPointError(
                "full-matrix hydrodynamics produced a non-finite wrench"
            )
        return _limit_wrench(
            wrench,
            max_force_n=cfg.max_force_n,
            max_torque_nm=cfg.max_torque_nm,
        )


def parse_hydrodynamic_matrices(
    value: Mapping[str, Any] | None,
    *,
    symmetry_tolerance: float = _SYMMETRY_TOLERANCE,
    passivity_tolerance: float = _PASSIVITY_TOLERANCE,
) -> HydrodynamicMatricesConfig:
    """Parse and validate an optional full-matrix hydrodynamics mapping.

    Active configurations require explicit calibration status, provenance,
    and all three 6x6 matrices.  Added mass must be symmetric positive
    semidefinite.  Damping matrices must also be symmetric positive
    semidefinite; signed off-diagonal coupling is allowed.  Eigenvalues down
    to ``-passivity_tolerance`` are accepted to accommodate floating-point
    roundoff.
    """

    symmetry_tol = _nonnegative_finite(
        symmetry_tolerance,
        label="symmetry_tolerance",
    )
    passivity_tol = _nonnegative_finite(
        passivity_tolerance,
        label="passivity_tolerance",
    )
    if value is None:
        return _disabled_config()
    if not isinstance(value, Mapping):
        raise ValueError(f"{_CONFIG_KEY} must be an object")

    active = value.get("active", False)
    if not isinstance(active, (bool, np.bool_)):
        raise ValueError(f"{_CONFIG_KEY}.active must be boolean")
    if not active:
        return _disabled_config()

    calibration_status = _nonempty_string(
        value.get("calibration_status"),
        label=f"{_CONFIG_KEY}.calibration_status",
    )
    provenance = _nonempty_string(
        value.get("provenance"),
        label=f"{_CONFIG_KEY}.provenance",
    )
    reference_point = _nonempty_string(
        value.get("reference_point", "center_of_mass"),
        label=f"{_CONFIG_KEY}.reference_point",
    ).lower()
    if reference_point != "center_of_mass":
        raise ValueError(
            f"{_CONFIG_KEY}.reference_point currently supports only center_of_mass"
        )
    added_mass = _symmetric_passive_matrix(
        value.get("added_mass_6x6"),
        label=f"{_CONFIG_KEY}.added_mass_6x6",
        symmetry_tolerance=symmetry_tol,
        passivity_tolerance=passivity_tol,
    )
    linear_damping = _symmetric_passive_matrix(
        value.get("linear_damping_6x6"),
        label=f"{_CONFIG_KEY}.linear_damping_6x6",
        symmetry_tolerance=symmetry_tol,
        passivity_tolerance=passivity_tol,
    )
    quadratic_damping = _symmetric_passive_matrix(
        value.get("quadratic_damping_6x6"),
        label=f"{_CONFIG_KEY}.quadratic_damping_6x6",
        symmetry_tolerance=symmetry_tol,
        passivity_tolerance=passivity_tol,
    )
    return HydrodynamicMatricesConfig(
        active=True,
        calibration_status=calibration_status,
        provenance=provenance,
        reference_point=reference_point,
        added_mass_6x6=added_mass,
        linear_damping_6x6=linear_damping,
        quadratic_damping_6x6=quadratic_damping,
        max_force_n=_positive_bounded(
            value.get("max_force_n", 2.0e4),
            label=f"{_CONFIG_KEY}.max_force_n",
            upper=_MAX_WRENCH_BOUND,
        ),
        max_torque_nm=_positive_bounded(
            value.get("max_torque_nm", 1.0e4),
            label=f"{_CONFIG_KEY}.max_torque_nm",
            upper=_MAX_WRENCH_BOUND,
        ),
    )


def added_mass_coriolis_matrix(
    added_mass_6x6: np.ndarray,
    velocity_body: np.ndarray,
) -> np.ndarray:
    """Construct the general Fossen added-mass Coriolis matrix.

    The construction supports arbitrary translation-rotation coupling.  If
    ``[a, b] = M_A nu``, the block form is
    ``[[0, -S(a)], [-S(a), -S(b)]]``.  It is skew-symmetric by construction,
    so ``nu.T @ C_A @ nu`` is zero up to floating-point roundoff.

    Args:
        added_mass_6x6: Constant generalized added-mass matrix.
        velocity_body: Body-relative velocity ``[u, v, w, p, q, r]``
            [m/s, rad/s].

    Returns:
        The 6x6 added-mass Coriolis matrix in Fossen ordering.
    """

    matrix = _matrix6(added_mass_6x6, label="added_mass_6x6")
    velocity = _vector6(velocity_body, label="velocity_body")
    generalized_momentum = matrix @ velocity
    linear_momentum = generalized_momentum[:3]
    angular_momentum = generalized_momentum[3:]

    coriolis = np.zeros(_MATRIX_SHAPE, dtype=np.float64)
    coriolis[:3, 3:] = -_skew(linear_momentum)
    coriolis[3:, :3] = -_skew(linear_momentum)
    coriolis[3:, 3:] = -_skew(angular_momentum)
    return coriolis


def _disabled_config() -> HydrodynamicMatricesConfig:
    zero = _readonly(np.zeros(_MATRIX_SHAPE, dtype=np.float64))
    return HydrodynamicMatricesConfig(
        active=False,
        calibration_status="disabled",
        provenance="disabled",
        reference_point="center_of_mass",
        added_mass_6x6=zero,
        linear_damping_6x6=zero,
        quadratic_damping_6x6=zero,
        max_force_n=0.0,
        max_torque_nm=0.0,
    )


def _symmetric_passive_matrix(
    value: Any,
    *,
    label: str,
    symmetry_tolerance: float,
    passivity_tolerance: float,
) -> np.ndarray:
    matrix = _matrix6(value, label=label)
    if not np.allclose(
        matrix,
        matrix.T,
        rtol=0.0,
        atol=symmetry_tolerance,
    ):
        max_error = float(np.max(np.abs(matrix - matrix.T)))
        raise ValueError(
            f"{label} must be symmetric within {symmetry_tolerance:g}; "
            f"maximum mismatch is {max_error:g}"
        )

    # Canonicalize roundoff-level asymmetry before storing the matrix.  This
    # keeps energy and Coriolis identities exact to numerical precision.
    matrix = 0.5 * (matrix + matrix.T)
    try:
        eigenvalues = np.linalg.eigvalsh(matrix)
    except np.linalg.LinAlgError as exc:
        raise ValueError(f"{label} eigenvalue decomposition failed") from exc
    minimum_eigenvalue = float(eigenvalues[0])
    if minimum_eigenvalue < -passivity_tolerance:
        raise ValueError(
            f"{label} must be positive semidefinite within "
            f"{passivity_tolerance:g}; minimum eigenvalue is "
            f"{minimum_eigenvalue:g}"
        )
    return _readonly(matrix)


def _matrix6(value: Any, *, label: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain a finite 6x6 matrix") from exc
    if result.shape != _MATRIX_SHAPE or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain a finite 6x6 matrix")
    maximum_entry = float(np.max(np.abs(result)))
    if maximum_entry > _MAX_MATRIX_ABS_ENTRY:
        raise ValueError(
            f"{label} exceeds safe absolute-entry bound "
            f"{_MAX_MATRIX_ABS_ENTRY:g}"
        )
    return result.copy()


def _vector6(value: Any, *, label: str) -> np.ndarray:
    try:
        result = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must contain six finite numbers") from exc
    if result.shape != (6,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{label} must contain six finite numbers")
    if float(np.max(np.abs(result))) > _MAX_STATE_ABS_COMPONENT:
        raise ValueError(
            f"{label} exceeds safe component bound {_MAX_STATE_ABS_COMPONENT:g}"
        )
    return result


def _limit_wrench(
    wrench: np.ndarray,
    *,
    max_force_n: float,
    max_torque_nm: float,
) -> np.ndarray:
    force_norm = float(np.linalg.norm(wrench[:3]))
    torque_norm = float(np.linalg.norm(wrench[3:]))
    scale = min(
        1.0,
        1.0 if force_norm <= max_force_n else max_force_n / force_norm,
        1.0 if torque_norm <= max_torque_nm else max_torque_nm / torque_norm,
    )
    return wrench * scale


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )


def _nonempty_string(value: Any, *, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{label} must be a non-empty string")
    return value.strip()


def _nonnegative_finite(value: Any, *, label: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{label} must be a finite nonnegative number")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a finite nonnegative number") from exc
    if not np.isfinite(result) or result < 0.0:
        raise ValueError(f"{label} must be a finite nonnegative number")
    return result


def _positive_bounded(value: Any, *, label: str, upper: float) -> float:
    result = _nonnegative_finite(value, label=label)
    if result <= 0.0 or result > upper:
        raise ValueError(f"{label} must be in (0, {upper:g}]")
    return result


def _readonly(value: np.ndarray) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64).copy()
    result.setflags(write=False)
    return result


__all__ = [
    "FullMatrixHydrodynamics",
    "HydrodynamicMatricesConfig",
    "added_mass_coriolis_matrix",
    "parse_hydrodynamic_matrices",
]
