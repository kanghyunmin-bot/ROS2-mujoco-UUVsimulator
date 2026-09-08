"""Vectorized, distributed hull hydrodynamics for a MuJoCo UUV.

The shipped patch coefficients are explicitly uncalibrated engineering
priors. This module also accepts identified coefficient sets when their
calibration status and provenance are recorded.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable, Mapping

import numpy as np


CurrentSample = np.ndarray | Callable[[np.ndarray, float], np.ndarray]
SurfaceSample = float | np.ndarray | Callable[[np.ndarray, float], float]

_CONFIG_KEY = "distributed_hydrodynamics"
_CALIBRATION_KEY = "distributed_hydrodynamics_calibration"
_CALIBRATION_KEYS = {
    "buoyancy_position_offset_body_m",
    "calibration_status",
    "coefficient_provenance",
    "normal_drag_axis_scale",
    "residual_restoring",
    "residual_linear_damping",
    "residual_quadratic_damping",
}
_MAX_PATCH_COUNT = 100_000
_MAX_PATCH_RADIUS_M = 1_000.0
_MAX_PATCH_AREA_M2 = 1.0e4
_MAX_PATCH_VOLUME_M3 = 1.0e4
_MAX_COEFFICIENT = 100.0
_MAX_FLUID_DENSITY_KG_M3 = 1.0e5
_MAX_GRAVITY_MPS2 = 1.0e3
_MAX_RELATIVE_SPEED_MPS = 1.0e4
_MAX_WRENCH_BOUND = 1.0e20
_ROTATION_ATOL = 1.0e-7


@dataclass(frozen=True)
class DistributedHydrodynamicsConfig:
    """Validated distributed-hydrodynamics parameters.

    Array rows correspond to hull patches.  Positions are relative to the
    body origin [m], areas are [m^2], volume shares are [m^3], and patch
    half-heights are vertical immersion smoothing distances [m].
    """

    active: bool
    calibration_status: str
    coefficient_provenance: str
    fluid_density_kg_m3: float
    gravity_mps2: float
    buoyancy_scale: float
    buoyancy_position_offset_body_m: np.ndarray
    residual_restoring_stiffness_nm_per_rad: np.ndarray
    residual_linear_damping: np.ndarray
    residual_quadratic_damping: np.ndarray
    patch_names: tuple[str, ...]
    positions_body_m: np.ndarray
    normals_body: np.ndarray
    areas_m2: np.ndarray
    volume_shares_m3: np.ndarray
    half_heights_m: np.ndarray
    normal_drag_coefficients: np.ndarray
    skin_drag_coefficients: np.ndarray
    max_relative_speed_mps: float
    max_patch_force_n: float
    max_total_force_n: float
    max_total_torque_nm: float

    @property
    def patch_count(self) -> int:
        """Return the number of configured hull patches."""

        return int(self.positions_body_m.shape[0])


@dataclass(frozen=True)
class DistributedHydrodynamicsResult:
    """Distributed force evaluation in world and body coordinates."""

    wrench_reference_position_world_m: np.ndarray
    positions_world_m: np.ndarray
    currents_world_mps: np.ndarray
    point_velocities_world_mps: np.ndarray
    relative_velocities_world_mps: np.ndarray
    submerged_fractions: np.ndarray
    buoyancy_forces_world_n: np.ndarray
    form_drag_forces_world_n: np.ndarray
    skin_drag_forces_world_n: np.ndarray
    patch_forces_world_n: np.ndarray
    patch_torques_world_nm: np.ndarray
    residual_damping_wrench_body: np.ndarray
    residual_restoring_torque_world_nm: np.ndarray
    force_world_n: np.ndarray
    torque_world_nm: np.ndarray
    force_body_n: np.ndarray
    torque_body_nm: np.ndarray
    safety_scale: float


class DistributedHullHydrodynamics:
    """Evaluate hydrostatic and drag forces at body-fixed hull patches."""

    def __init__(self, config: DistributedHydrodynamicsConfig) -> None:
        self.config = config

    @classmethod
    def from_profile(cls, sim_profile: Mapping[str, Any]) -> "DistributedHullHydrodynamics":
        """Build an opt-in model from a simulation-profile mapping."""

        payload = sim_profile.get(_CONFIG_KEY)
        if payload is None:
            return cls(_disabled_config())
        if not isinstance(payload, Mapping):
            raise ValueError(f"{_CONFIG_KEY} must be an object")
        active = payload.get("active", False)
        if not isinstance(active, bool):
            raise ValueError(f"{_CONFIG_KEY}.active must be boolean")
        if not active:
            return cls(_disabled_config())

        calibration = sim_profile.get(_CALIBRATION_KEY, {})
        if not isinstance(calibration, Mapping):
            raise ValueError(f"{_CALIBRATION_KEY} must be an object")
        unknown_calibration_keys = set(calibration) - _CALIBRATION_KEYS
        if unknown_calibration_keys:
            names = ", ".join(sorted(str(item) for item in unknown_calibration_keys))
            raise ValueError(f"{_CALIBRATION_KEY} contains unsupported keys: {names}")
        effective_payload = dict(payload)
        effective_payload.update(calibration)

        status = _required_string(effective_payload, "calibration_status")
        provenance = _required_string(effective_payload, "coefficient_provenance")
        fluid_density = _positive_bounded(
            effective_payload.get("fluid_density_kg_m3", 997.0),
            "fluid_density_kg_m3",
            _MAX_FLUID_DENSITY_KG_M3,
        )
        gravity = _positive_bounded(
            effective_payload.get("gravity_mps2", 9.80665),
            "gravity_mps2",
            _MAX_GRAVITY_MPS2,
        )
        default_normal_drag = _bounded_nonnegative(
            effective_payload.get("normal_drag_coefficient", 1.0),
            "normal_drag_coefficient",
            _MAX_COEFFICIENT,
        )
        default_skin_drag = _bounded_nonnegative(
            effective_payload.get("skin_drag_coefficient", 0.01),
            "skin_drag_coefficient",
            _MAX_COEFFICIENT,
        )

        raw_patches = effective_payload.get("patches")
        if not isinstance(raw_patches, (list, tuple)) or not raw_patches:
            raise ValueError(f"active {_CONFIG_KEY}.patches must be a non-empty array")
        if len(raw_patches) > _MAX_PATCH_COUNT:
            raise ValueError(f"{_CONFIG_KEY}.patches exceeds {_MAX_PATCH_COUNT} entries")
        patch_arrays = _parse_patches(
            raw_patches,
            default_normal_drag=default_normal_drag,
            default_skin_drag=default_skin_drag,
        )
        buoyancy_position_offset = _vector3(
            effective_payload.get("buoyancy_position_offset_body_m", [0.0, 0.0, 0.0]),
            "buoyancy_position_offset_body_m",
        )
        if float(np.linalg.norm(buoyancy_position_offset)) > _MAX_PATCH_RADIUS_M:
            raise ValueError("buoyancy_position_offset_body_m exceeds safe radius")
        normal_drag_axis_scale = _vector3(
            effective_payload.get("normal_drag_axis_scale", [1.0, 1.0, 1.0]),
            "normal_drag_axis_scale",
        )
        if np.any(normal_drag_axis_scale < 0.0) or np.any(
            normal_drag_axis_scale > _MAX_COEFFICIENT
        ):
            raise ValueError("normal_drag_axis_scale is outside safe bounds")
        residual_restoring = _parse_residual_restoring(
            effective_payload.get("residual_restoring")
        )

        positions_body_m = patch_arrays[1].copy()
        buoyant_rows = patch_arrays[4] > 0.0
        positions_body_m[buoyant_rows] += buoyancy_position_offset
        if np.any(np.linalg.norm(positions_body_m, axis=1) > _MAX_PATCH_RADIUS_M):
            raise ValueError("calibrated patch position exceeds safe radius")
        normal_drag_coefficients = patch_arrays[6] * (
            np.square(patch_arrays[2]) @ normal_drag_axis_scale
        )
        if np.any(normal_drag_coefficients > _MAX_COEFFICIENT):
            raise ValueError("calibrated normal drag coefficient exceeds safe bound")

        return cls(
            DistributedHydrodynamicsConfig(
                active=True,
                calibration_status=status,
                coefficient_provenance=provenance,
                fluid_density_kg_m3=fluid_density,
                gravity_mps2=gravity,
                buoyancy_scale=_bounded_nonnegative(
                    sim_profile.get("buoyancy_scale", 1.0),
                    "profile buoyancy_scale",
                    2.0,
                ),
                buoyancy_position_offset_body_m=buoyancy_position_offset,
                residual_restoring_stiffness_nm_per_rad=residual_restoring,
                residual_linear_damping=_damping_vector(effective_payload, "residual_linear_damping"),
                residual_quadratic_damping=_damping_vector(effective_payload, "residual_quadratic_damping"),
                patch_names=patch_arrays[0],
                positions_body_m=positions_body_m,
                normals_body=patch_arrays[2],
                areas_m2=patch_arrays[3],
                volume_shares_m3=patch_arrays[4],
                half_heights_m=patch_arrays[5],
                normal_drag_coefficients=normal_drag_coefficients,
                skin_drag_coefficients=patch_arrays[7],
                max_relative_speed_mps=_positive_bounded(
                    effective_payload.get("max_relative_speed_mps", 5.0),
                    "max_relative_speed_mps",
                    _MAX_RELATIVE_SPEED_MPS,
                ),
                max_patch_force_n=_positive_bounded(
                    effective_payload.get("max_patch_force_n", 1.0e4),
                    "max_patch_force_n",
                    _MAX_WRENCH_BOUND,
                ),
                max_total_force_n=_positive_bounded(
                    effective_payload.get("max_total_force_n", 2.0e4),
                    "max_total_force_n",
                    _MAX_WRENCH_BOUND,
                ),
                max_total_torque_nm=_positive_bounded(
                    effective_payload.get("max_total_torque_nm", 1.0e4),
                    "max_total_torque_nm",
                    _MAX_WRENCH_BOUND,
                ),
            )
        )

    @property
    def active(self) -> bool:
        """Return whether distributed force evaluation is enabled."""

        return bool(self.config.active)

    def evaluate(
        self,
        *,
        body_position_world_m: np.ndarray,
        rotation_world_from_body: np.ndarray,
        linear_velocity_world_mps: np.ndarray,
        angular_velocity_world_radps: np.ndarray,
        current_world_mps: CurrentSample,
        surface_height_world_m: SurfaceSample,
        wrench_reference_position_body_m: np.ndarray | None = None,
        time_s: float = 0.0,
    ) -> DistributedHydrodynamicsResult:
        """Return bounded distributed buoyancy and drag loads.

        The world frame is assumed to use +Z upward.  A current callable is
        sampled independently at each patch as ``callback(position, time_s)``.
        A current array may contain one shared vector or one vector per patch.
        Surface height accepts the same shared/per-patch convention; a callable
        returns a scalar world-Z height for each patch position.
        Wrench outputs and aggregate safety limits are evaluated about
        ``wrench_reference_position_body_m``. The default is the body origin.
        """

        if not self.active:
            return _zero_result()

        position = _vector3(body_position_world_m, "body_position_world_m")
        rotation = _rotation_matrix(rotation_world_from_body)
        linear_velocity = _vector3(linear_velocity_world_mps, "linear_velocity_world_mps")
        angular_velocity = _vector3(
            angular_velocity_world_radps,
            "angular_velocity_world_radps",
        )
        wrench_reference_body = (
            np.zeros(3, dtype=np.float64)
            if wrench_reference_position_body_m is None
            else _vector3(
                wrench_reference_position_body_m,
                "wrench_reference_position_body_m",
            )
        )
        sample_time = _finite(time_s, "time_s")
        cfg = self.config

        offsets_world = cfg.positions_body_m @ rotation.T
        positions_world = position[None, :] + offsets_world
        wrench_reference_world = position + rotation @ wrench_reference_body
        wrench_offsets_world = positions_world - wrench_reference_world[None, :]
        normals_world = cfg.normals_body @ rotation.T
        _require_finite("transformed patch geometry", positions_world, normals_world)
        currents = _sample_currents(current_world_mps, positions_world, sample_time)
        surface_heights = _sample_surface_heights(
            surface_height_world_m,
            positions_world,
            sample_time,
        )

        point_velocities = linear_velocity[None, :] + np.cross(
            angular_velocity[None, :],
            offsets_world,
        )
        _require_finite("patch point velocities", point_velocities)
        with np.errstate(over="ignore", invalid="ignore"):
            relative_velocities = point_velocities - currents
        _require_finite("patch relative velocities", relative_velocities)
        relative_velocities = _clip_row_norms(
            relative_velocities,
            cfg.max_relative_speed_mps,
        )
        with np.errstate(over="ignore", invalid="ignore"):
            immersion_numerators = (
                surface_heights
                - positions_world[:, 2]
                + cfg.half_heights_m
            )
        _require_finite("patch immersion depths", immersion_numerators)
        submerged = np.clip(
            immersion_numerators / (2.0 * cfg.half_heights_m),
            0.0,
            1.0,
        )

        buoyancy = np.zeros_like(positions_world)
        buoyancy[:, 2] = (
            cfg.fluid_density_kg_m3
            * cfg.gravity_mps2
            * cfg.volume_shares_m3
            * submerged
            * cfg.buoyancy_scale
        )

        normal_speeds = np.sum(relative_velocities * normals_world, axis=1)
        form_drag = (
            -0.5
            * cfg.fluid_density_kg_m3
            * cfg.normal_drag_coefficients
            * cfg.areas_m2
            * submerged
            * np.abs(normal_speeds)
            * normal_speeds
        )[:, None] * normals_world

        tangential_velocities = relative_velocities - normal_speeds[:, None] * normals_world
        tangential_speeds = np.linalg.norm(tangential_velocities, axis=1)
        skin_drag = (
            -0.5
            * cfg.fluid_density_kg_m3
            * cfg.skin_drag_coefficients
            * cfg.areas_m2
            * submerged
            * tangential_speeds
        )[:, None] * tangential_velocities

        form_drag, skin_drag = _limit_patch_dynamic_forces(
            form_drag,
            skin_drag,
            cfg.max_patch_force_n,
        )
        dynamic_forces = form_drag + skin_drag
        dynamic_torques = np.cross(wrench_offsets_world, dynamic_forces)
        dynamic_force_world = np.sum(dynamic_forces, axis=0)
        dynamic_torque_world = np.sum(dynamic_torques, axis=0)
        # Residual coefficients use [u,v,w,p,q,r] at the wrench reference
        # (the inertial centre in production). Sample current at that point.
        residual = np.zeros(6, dtype=np.float64)
        if np.any(cfg.residual_linear_damping) or np.any(cfg.residual_quadratic_damping):
            if callable(current_world_mps) or np.asarray(current_world_mps).shape == (3,):
                reference_current = _sample_currents(
                    current_world_mps, wrench_reference_world[None, :], sample_time
                )[0]
            else:
                raise ValueError("residual damping requires shared current or a spatial current callable")
            reference_velocity = linear_velocity + np.cross(
                angular_velocity, rotation @ wrench_reference_body
            )
            nu = np.concatenate((rotation.T @ (reference_velocity - reference_current),
                                 rotation.T @ angular_velocity))
            volume = float(np.sum(cfg.volume_shares_m3))
            immersion = float(np.dot(submerged, cfg.volume_shares_m3) / volume) if volume > 0.0 else 0.0
            residual = -immersion * (cfg.residual_linear_damping * nu
                                      + cfg.residual_quadratic_damping * np.abs(nu) * nu)
            _require_finite("residual damping", residual)
        dynamic_force_world += rotation @ residual[:3]
        dynamic_torque_world += rotation @ residual[3:]
        safety_scale = _aggregate_safety_scale(
            dynamic_force_world,
            dynamic_torque_world,
            max_force_n=cfg.max_total_force_n,
            max_torque_nm=cfg.max_total_torque_nm,
        )
        if safety_scale < 1.0:
            form_drag = form_drag * safety_scale
            skin_drag = skin_drag * safety_scale
        residual *= safety_scale
        # Safety limiting is hydrodynamic-only. Hydrostatic support must never
        # change with vehicle speed or a drag spike.
        patch_forces = buoyancy + form_drag + skin_drag
        patch_torques = np.cross(wrench_offsets_world, patch_forces)
        force_world = np.sum(patch_forces, axis=0) + rotation @ residual[:3]
        roll, pitch = _roll_pitch_from_rotation(rotation)
        residual_restoring_torque_body = (
            -cfg.residual_restoring_stiffness_nm_per_rad
            * np.array([roll, pitch, 0.0], dtype=np.float64)
        )
        residual_restoring_torque_world = rotation @ residual_restoring_torque_body
        torque_world = (np.sum(patch_torques, axis=0) + residual_restoring_torque_world
                        + rotation @ residual[3:])

        force_body = rotation.T @ force_world
        torque_body = rotation.T @ torque_world
        _validate_finite_result(
            buoyancy,
            form_drag,
            skin_drag,
            patch_forces,
            patch_torques,
            residual_restoring_torque_world,
            force_world,
            torque_world,
            force_body,
            torque_body,
        )
        return DistributedHydrodynamicsResult(
            wrench_reference_position_world_m=wrench_reference_world,
            positions_world_m=positions_world,
            currents_world_mps=currents,
            point_velocities_world_mps=point_velocities,
            relative_velocities_world_mps=relative_velocities,
            submerged_fractions=submerged,
            buoyancy_forces_world_n=buoyancy,
            form_drag_forces_world_n=form_drag,
            skin_drag_forces_world_n=skin_drag,
            patch_forces_world_n=patch_forces,
            patch_torques_world_nm=patch_torques,
            residual_damping_wrench_body=residual,
            residual_restoring_torque_world_nm=residual_restoring_torque_world,
            force_world_n=force_world,
            torque_world_nm=torque_world,
            force_body_n=force_body,
            torque_body_nm=torque_body,
            safety_scale=safety_scale,
        )


def _disabled_config() -> DistributedHydrodynamicsConfig:
    empty_vectors = np.zeros((0, 3), dtype=np.float64)
    empty_scalars = np.zeros(0, dtype=np.float64)
    return DistributedHydrodynamicsConfig(
        active=False,
        calibration_status="disabled",
        coefficient_provenance="disabled",
        fluid_density_kg_m3=0.0,
        gravity_mps2=0.0,
        buoyancy_scale=1.0,
        buoyancy_position_offset_body_m=np.zeros(3, dtype=np.float64),
        residual_restoring_stiffness_nm_per_rad=np.zeros(3, dtype=np.float64),
        residual_linear_damping=np.zeros(6),
        residual_quadratic_damping=np.zeros(6),
        patch_names=(),
        positions_body_m=empty_vectors.copy(),
        normals_body=empty_vectors.copy(),
        areas_m2=empty_scalars.copy(),
        volume_shares_m3=empty_scalars.copy(),
        half_heights_m=empty_scalars.copy(),
        normal_drag_coefficients=empty_scalars.copy(),
        skin_drag_coefficients=empty_scalars.copy(),
        max_relative_speed_mps=0.0,
        max_patch_force_n=0.0,
        max_total_force_n=0.0,
        max_total_torque_nm=0.0,
    )


def _zero_result() -> DistributedHydrodynamicsResult:
    empty_vectors = np.zeros((0, 3), dtype=np.float64)
    empty_scalars = np.zeros(0, dtype=np.float64)
    zero = np.zeros(3, dtype=np.float64)
    return DistributedHydrodynamicsResult(
        wrench_reference_position_world_m=zero.copy(),
        positions_world_m=empty_vectors.copy(),
        currents_world_mps=empty_vectors.copy(),
        point_velocities_world_mps=empty_vectors.copy(),
        relative_velocities_world_mps=empty_vectors.copy(),
        submerged_fractions=empty_scalars,
        buoyancy_forces_world_n=empty_vectors.copy(),
        form_drag_forces_world_n=empty_vectors.copy(),
        skin_drag_forces_world_n=empty_vectors.copy(),
        patch_forces_world_n=empty_vectors.copy(),
        patch_torques_world_nm=empty_vectors.copy(),
        residual_damping_wrench_body=np.zeros(6),
        residual_restoring_torque_world_nm=zero.copy(),
        force_world_n=zero.copy(),
        torque_world_nm=zero.copy(),
        force_body_n=zero.copy(),
        torque_body_nm=zero.copy(),
        safety_scale=1.0,
    )


def _parse_patches(
    raw_patches: list[Any] | tuple[Any, ...],
    *,
    default_normal_drag: float,
    default_skin_drag: float,
) -> tuple[
    tuple[str, ...],
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    names: list[str] = []
    positions: list[np.ndarray] = []
    normals: list[np.ndarray] = []
    areas: list[float] = []
    volume_shares: list[float] = []
    half_heights: list[float] = []
    normal_drag: list[float] = []
    skin_drag: list[float] = []
    for index, raw_patch in enumerate(raw_patches):
        label = f"patches[{index}]"
        if not isinstance(raw_patch, Mapping):
            raise ValueError(f"{_CONFIG_KEY}.{label} must be an object")
        name = str(raw_patch.get("name", f"patch_{index}")).strip()
        if not name:
            raise ValueError(f"{_CONFIG_KEY}.{label}.name must not be empty")
        position = _vector3(raw_patch.get("position_body_m"), f"{label}.position_body_m")
        if float(np.linalg.norm(position)) > _MAX_PATCH_RADIUS_M:
            raise ValueError(f"{_CONFIG_KEY}.{label}.position_body_m exceeds safe radius")
        normal = _vector3(raw_patch.get("normal_body"), f"{label}.normal_body")
        normal_norm = float(np.linalg.norm(normal))
        if not math.isclose(normal_norm, 1.0, rel_tol=0.0, abs_tol=1.0e-6):
            raise ValueError(f"{_CONFIG_KEY}.{label}.normal_body must be a unit vector")
        area = _bounded_nonnegative(
            raw_patch.get("area_m2"),
            f"{label}.area_m2",
            _MAX_PATCH_AREA_M2,
        )
        volume_share = _bounded_nonnegative(
            raw_patch.get("volume_share_m3"),
            f"{label}.volume_share_m3",
            _MAX_PATCH_VOLUME_M3,
        )
        half_height = _positive_bounded(
            raw_patch.get("half_height_m"),
            f"{label}.half_height_m",
            _MAX_PATCH_RADIUS_M,
        )
        normal_drag_coefficient = _bounded_nonnegative(
            raw_patch.get("normal_drag_coefficient", default_normal_drag),
            f"{label}.normal_drag_coefficient",
            _MAX_COEFFICIENT,
        )
        skin_drag_coefficient = _bounded_nonnegative(
            raw_patch.get("skin_drag_coefficient", default_skin_drag),
            f"{label}.skin_drag_coefficient",
            _MAX_COEFFICIENT,
        )
        quadrature_offsets = _quadrature_offsets_body_m(
            raw_patch.get("quadrature_spans_body_m"),
            normal=normal,
            label=f"{label}.quadrature_spans_body_m",
        )
        quadrature_count = len(quadrature_offsets)
        for quadrature_index, offset in enumerate(quadrature_offsets):
            quadrature_position = position + offset
            if float(np.linalg.norm(quadrature_position)) > _MAX_PATCH_RADIUS_M:
                raise ValueError(
                    f"{_CONFIG_KEY}.{label} quadrature position exceeds safe radius"
                )
            names.append(
                name
                if quadrature_count == 1
                else f"{name}__q{quadrature_index}"
            )
            positions.append(quadrature_position)
            normals.append(normal)
            areas.append(area / quadrature_count)
            volume_shares.append(volume_share / quadrature_count)
            half_heights.append(half_height)
            normal_drag.append(normal_drag_coefficient)
            skin_drag.append(skin_drag_coefficient)
            if len(names) > _MAX_PATCH_COUNT:
                raise ValueError(
                    f"expanded {_CONFIG_KEY}.patches exceeds "
                    f"{_MAX_PATCH_COUNT} entries"
                )
    if len(set(names)) != len(names):
        raise ValueError(f"{_CONFIG_KEY}.patches names must be unique")
    return (
        tuple(names),
        np.stack(positions),
        np.stack(normals),
        np.asarray(areas, dtype=np.float64),
        np.asarray(volume_shares, dtype=np.float64),
        np.asarray(half_heights, dtype=np.float64),
        np.asarray(normal_drag, dtype=np.float64),
        np.asarray(skin_drag, dtype=np.float64),
    )


def _quadrature_offsets_body_m(
    raw_spans: Any,
    *,
    normal: np.ndarray,
    label: str,
) -> tuple[np.ndarray, ...]:
    """Return one-, two-, or four-point tangent-plane Gauss offsets."""

    if raw_spans is None:
        return (np.zeros(3, dtype=np.float64),)
    try:
        spans = np.asarray(raw_spans, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{_CONFIG_KEY}.{label} must contain one or two finite 3-vectors"
        ) from exc
    if spans.ndim != 2 or spans.shape[1] != 3 or spans.shape[0] not in {1, 2}:
        raise ValueError(
            f"{_CONFIG_KEY}.{label} must contain one or two finite 3-vectors"
        )
    if not np.all(np.isfinite(spans)):
        raise ValueError(f"{_CONFIG_KEY}.{label} must contain finite values")
    norms = np.linalg.norm(spans, axis=1)
    if np.any(norms <= 0.0) or np.any(norms > _MAX_PATCH_RADIUS_M):
        raise ValueError(f"{_CONFIG_KEY}.{label} span lengths are outside safe bounds")
    tangent_errors = np.abs(spans @ normal)
    if np.any(tangent_errors > 1.0e-6 * norms):
        raise ValueError(f"{_CONFIG_KEY}.{label} spans must be tangent to the patch")
    if spans.shape[0] == 2:
        orthogonality = abs(float(spans[0] @ spans[1]))
        if orthogonality > 1.0e-6 * float(norms[0] * norms[1]):
            raise ValueError(f"{_CONFIG_KEY}.{label} spans must be mutually orthogonal")

    gauss_spans = spans / math.sqrt(3.0)
    offsets = [np.zeros(3, dtype=np.float64)]
    for span in gauss_spans:
        offsets = [base + sign * span for base in offsets for sign in (-1.0, 1.0)]
    return tuple(offsets)


def _sample_currents(
    sample: CurrentSample,
    positions_world: np.ndarray,
    time_s: float,
) -> np.ndarray:
    count = positions_world.shape[0]
    if callable(sample):
        currents = np.stack(
            [
                _vector3(sample(position.copy(), time_s), "current callback result")
                for position in positions_world
            ]
        )
    else:
        currents = np.asarray(sample, dtype=np.float64)
        if currents.shape == (3,):
            currents = np.broadcast_to(currents, (count, 3)).copy()
        elif currents.shape != (count, 3):
            raise ValueError(f"current_world_mps must have shape (3,) or ({count}, 3)")
        if not np.all(np.isfinite(currents)):
            raise ValueError("current_world_mps must contain finite values")
    return currents


def _sample_surface_heights(
    sample: SurfaceSample,
    positions_world: np.ndarray,
    time_s: float,
) -> np.ndarray:
    count = positions_world.shape[0]
    if callable(sample):
        values = np.asarray(
            [
                _finite(sample(position.copy(), time_s), "surface callback result")
                for position in positions_world
            ],
            dtype=np.float64,
        )
    else:
        values = np.asarray(sample, dtype=np.float64)
        if values.shape == ():
            values = np.full(count, float(values), dtype=np.float64)
        elif values.shape != (count,):
            raise ValueError(f"surface_height_world_m must be scalar or shape ({count},)")
        if not np.all(np.isfinite(values)):
            raise ValueError("surface_height_world_m must contain finite values")
    return values


def _clip_row_norms(values: np.ndarray, limit: float) -> np.ndarray:
    # Compute scale factors without squaring the original values, so very
    # large but finite inputs cannot overflow inside ``np.linalg.norm``.
    max_components = np.max(np.abs(values), axis=1)
    normalized = np.divide(
        values,
        max_components[:, None],
        out=np.zeros_like(values),
        where=max_components[:, None] > 0.0,
    )
    normalized_norms = np.linalg.norm(normalized, axis=1)
    with np.errstate(over="ignore", divide="ignore", invalid="ignore"):
        limit_over_max = np.divide(
            limit,
            max_components,
            out=np.full_like(max_components, np.inf),
            where=max_components > 0.0,
        )
        scales = np.minimum(
            1.0,
            np.divide(
                limit_over_max,
                normalized_norms,
                out=np.ones_like(normalized_norms),
                where=normalized_norms > 0.0,
            ),
        )
    return values * scales[:, None]


def _limit_patch_dynamic_forces(
    form_drag: np.ndarray,
    skin_drag: np.ndarray,
    limit_n: float,
) -> tuple[np.ndarray, np.ndarray]:
    combined = form_drag + skin_drag
    norms = np.maximum.reduce(
        (
            np.linalg.norm(form_drag, axis=1),
            np.linalg.norm(skin_drag, axis=1),
            np.linalg.norm(combined, axis=1),
        )
    )
    scales = np.minimum(
        1.0,
        np.divide(
            limit_n,
            norms,
            out=np.ones_like(norms),
            where=norms > 0.0,
        ),
    )
    return (
        form_drag * scales[:, None],
        skin_drag * scales[:, None],
    )


def _aggregate_safety_scale(
    force_world: np.ndarray,
    torque_world: np.ndarray,
    *,
    max_force_n: float,
    max_torque_nm: float,
) -> float:
    force_norm = float(np.linalg.norm(force_world))
    torque_norm = float(np.linalg.norm(torque_world))
    force_scale = 1.0 if force_norm <= max_force_n else max_force_n / force_norm
    torque_scale = 1.0 if torque_norm <= max_torque_nm else max_torque_nm / torque_norm
    return float(min(1.0, force_scale, torque_scale))


def _damping_vector(payload: Mapping[str, Any], name: str) -> np.ndarray:
    """Nonnegative diagonal coefficients in SI units, ordered u,v,w,p,q,r."""
    values = np.asarray(payload.get(name, np.zeros(6)), dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)) or np.any(values < 0.0) or np.any(values > 1.0e6):
        raise ValueError(f"{name} must contain six finite nonnegative coefficients <= 1e6")
    return values.copy()


def _parse_residual_restoring(value: Any) -> np.ndarray:
    if value is None:
        return np.zeros(3, dtype=np.float64)
    if not isinstance(value, Mapping):
        raise ValueError(f"{_CONFIG_KEY}.residual_restoring must be an object")
    active = value.get("active", False)
    if not isinstance(active, bool):
        raise ValueError(f"{_CONFIG_KEY}.residual_restoring.active must be boolean")
    if not active:
        return np.zeros(3, dtype=np.float64)
    return np.array(
        [
            _bounded_nonnegative(
                value.get("roll_stiffness_nm_per_rad"),
                "residual_restoring.roll_stiffness_nm_per_rad",
                _MAX_WRENCH_BOUND,
            ),
            _bounded_nonnegative(
                value.get("pitch_stiffness_nm_per_rad"),
                "residual_restoring.pitch_stiffness_nm_per_rad",
                _MAX_WRENCH_BOUND,
            ),
            0.0,
        ],
        dtype=np.float64,
    )


def _roll_pitch_from_rotation(rotation: np.ndarray) -> tuple[float, float]:
    roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
    pitch = math.atan2(
        -float(rotation[2, 0]),
        math.hypot(float(rotation[0, 0]), float(rotation[1, 0])),
    )
    return roll, pitch


def _rotation_matrix(value: Any) -> np.ndarray:
    rotation = np.asarray(value, dtype=np.float64)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation_world_from_body must be a finite 3x3 matrix")
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=_ROTATION_ATOL, rtol=0.0):
        raise ValueError("rotation_world_from_body must be orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=_ROTATION_ATOL):
        raise ValueError("rotation_world_from_body must be a proper rotation")
    return rotation


def _validate_finite_result(*values: np.ndarray) -> None:
    try:
        _require_finite("distributed hydrodynamics result", *values)
    except ValueError as exc:
        raise FloatingPointError(str(exc)) from exc


def _require_finite(label: str, *values: np.ndarray) -> None:
    if any(not np.all(np.isfinite(value)) for value in values):
        raise ValueError(f"{label} must contain only finite values")


def _required_string(payload: Mapping[str, Any], key: str) -> str:
    raw_value = payload.get(key)
    if not isinstance(raw_value, str):
        raise ValueError(f"active {_CONFIG_KEY} requires string {key}")
    value = raw_value.strip()
    if not value:
        raise ValueError(f"active {_CONFIG_KEY} requires {key}")
    return value


def _vector3(value: Any, label: str) -> np.ndarray:
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{_CONFIG_KEY}.{label} must contain three finite numbers")
    return result


def _finite(value: Any, label: str) -> float:
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{_CONFIG_KEY}.{label} must be numeric")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{_CONFIG_KEY}.{label} must be numeric") from exc
    if not math.isfinite(result):
        raise ValueError(f"{_CONFIG_KEY}.{label} must be finite")
    return result


def _positive(value: Any, label: str) -> float:
    result = _finite(value, label)
    if result <= 0.0:
        raise ValueError(f"{_CONFIG_KEY}.{label} must be positive")
    return result


def _positive_bounded(value: Any, label: str, upper: float) -> float:
    result = _positive(value, label)
    if result > upper:
        raise ValueError(f"{_CONFIG_KEY}.{label} must be <= {upper:g}")
    return result


def _bounded_nonnegative(value: Any, label: str, upper: float) -> float:
    result = _finite(value, label)
    if result < 0.0 or result > upper:
        raise ValueError(f"{_CONFIG_KEY}.{label} must be in [0, {upper:g}]")
    return result


__all__ = [
    "CurrentSample",
    "DistributedHullHydrodynamics",
    "DistributedHydrodynamicsConfig",
    "DistributedHydrodynamicsResult",
    "SurfaceSample",
]
