"""Single-owner validation for opt-in advanced hydrodynamic models."""

from __future__ import annotations

import math

import numpy as np


def validate_advanced_hydrodynamics_ownership(
    *,
    use_custom_hydrodynamics: bool,
    hydro_cfg,
    values,
    wrenches,
    state_coefficient_scaler,
    distributed_hydrodynamics,
    full_matrix_hydrodynamics,
    neutral_volume_m3: float,
    rho_kg_m3: float,
    gravity_world_mps2,
) -> None:
    """Reject configurations that would silently double-count fluid loads."""

    distributed_active = bool(distributed_hydrodynamics.active)
    matrix_active = bool(full_matrix_hydrodynamics.active)
    cfd_active = bool(wrenches.cfd_dynamic_wrench_enabled)
    if (distributed_active or matrix_active) and not use_custom_hydrodynamics:
        raise ValueError(
            "distributed/full-matrix hydrodynamics require --fluid-model distributed "
            "(or legacy/custom) so MuJoCo ellipsoid fluid is disabled"
        )

    if cfd_active and not distributed_active and not matrix_active:
        cfd_output_dofs = _cfd_output_dofs(wrenches.cfd_dynamic_wrench_axes)
        if not use_custom_hydrodynamics:
            raise ValueError(
                "cfd_dynamic_wrench contains total force and cannot be layered on "
                "MuJoCo ellipsoid drag; use a single-owner custom profile"
            )
        if any(
            abs(float(values.linear_damping_diag[dof])) > 1.0e-12
            or abs(float(values.quadratic_damping_diag[dof])) > 1.0e-12
            for dof in cfd_output_dofs
        ):
            raise ValueError(
                "cfd_dynamic_wrench contains total force and overlaps legacy "
                "diagonal damping; zero each CFD-owned translational axis"
            )
        if 2 in cfd_output_dofs and (
            float(values.surface_heave_damping) > 1.0e-12
            or 2 in _empirical_hydrodynamic_output_dofs(values)
        ):
            raise ValueError(
                "cfd_dynamic_wrench owns heave total force; disable surface and "
                "empirical heave terms on the same axis"
            )

    if distributed_active:
        gravity_world = np.asarray(gravity_world_mps2, dtype=np.float64)
        if (
            gravity_world.shape != (3,)
            or not np.all(np.isfinite(gravity_world))
            or abs(float(gravity_world[0])) > 1.0e-9
            or abs(float(gravity_world[1])) > 1.0e-9
            or float(gravity_world[2]) >= 0.0
        ):
            raise ValueError(
                "distributed_hydrodynamics currently requires finite vertical "
                "gravity [0, 0, negative] and a +Z-up world"
            )
        if _nonzero(values.linear_damping_diag) or _nonzero(values.quadratic_damping_diag):
            raise ValueError(
                "distributed_hydrodynamics owns hull drag; set legacy linear_damping_diag "
                "and quadratic_damping_diag to zero"
            )
        if float(values.surface_heave_damping) > 1.0e-12:
            raise ValueError(
                "distributed_hydrodynamics owns waterline drag; set surface_heave_damping to zero"
            )
        if bool(hydro_cfg.hydrostatic_restoring_active):
            raise ValueError(
                "distributed_hydrodynamics owns buoyancy restoring moments; disable hydrostatic_restoring"
            )
        if bool(state_coefficient_scaler.active):
            raise ValueError(
                "hydrodynamic_state_scaling does not scale distributed patches; disable it in this profile"
            )
        if cfd_active:
            raise ValueError(
                "distributed_hydrodynamics owns hull drag; disable cfd_dynamic_wrench "
                "to avoid adding a second translational drag wrench"
            )
        if _empirical_hydrodynamic_output_dofs(values):
            raise ValueError(
                "distributed_hydrodynamics owns hull force and moment response; "
                "disable empirical pitch/lift/yaw-heave/heave-extra terms"
            )

        cfg = distributed_hydrodynamics.config
        configured_volume = float(np.sum(cfg.volume_shares_m3))
        expected_volume = float(neutral_volume_m3)
        if expected_volume > 1.0e-12:
            relative_error = abs(configured_volume - expected_volume) / expected_volume
            if relative_error > 0.05:
                raise ValueError(
                    "distributed patch volume must match neutral_volume "
                    f"within 5% (configured={configured_volume:.6f}m^3, "
                    f"expected={expected_volume:.6f}m^3)"
                )
        if not math.isclose(cfg.fluid_density_kg_m3, rho_kg_m3, rel_tol=0.0, abs_tol=1.0e-6):
            raise ValueError(
                "distributed_hydrodynamics.fluid_density_kg_m3 must match the scene fluid density"
            )
        gravity_magnitude = -float(gravity_world[2])
        if not math.isclose(
            cfg.gravity_mps2,
            gravity_magnitude,
            rel_tol=0.0,
            abs_tol=1.0e-6,
        ):
            raise ValueError(
                "distributed_hydrodynamics.gravity_mps2 must match the MuJoCo gravity magnitude"
            )

    if matrix_active:
        if bool(state_coefficient_scaler.active):
            raise ValueError(
                "hydrodynamic_state_scaling is not applied to full 6x6 matrices; "
                "disable it or identify state-dependent matrices explicitly"
            )
        if _nonzero(values.added_mass_diag):
            raise ValueError(
                "hydrodynamic_matrices owns added mass; set legacy added_mass_diag to zero"
            )
        if _nonzero(values.linear_damping_diag) or _nonzero(values.quadratic_damping_diag):
            raise ValueError(
                "hydrodynamic_matrices owns matrix damping; set legacy linear_damping_diag "
                "and quadratic_damping_diag to zero"
            )
        if bool(
            getattr(
                wrenches,
                "fossen_residual_requested_added_mass_active",
                wrenches.fossen_residual_added_mass_active,
            )
        ):
            raise ValueError(
                "hydrodynamic_matrices owns added mass; disable fossen_residual_hydro.added_mass"
            )
        if cfd_active and _matrix_cfd_damping_overlap(
            full_matrix_hydrodynamics.config,
            wrenches.cfd_dynamic_wrench_axes,
        ):
            raise ValueError(
                "hydrodynamic_matrices damping and cfd_dynamic_wrench own the same "
                "translational output axis; disable one owner or zero that matrix row"
            )
        empirical_dofs = _empirical_hydrodynamic_output_dofs(values)
        if empirical_dofs and _matrix_damping_owns_dofs(
            full_matrix_hydrodynamics.config,
            empirical_dofs,
        ):
            raise ValueError(
                "hydrodynamic_matrices damping and empirical hydrodynamics own the "
                "same heave/pitch output axis; disable one owner or zero that matrix row"
            )

    if distributed_active and bool(getattr(wrenches, "fossen_residual_requested_active", False)):
        raise ValueError("distributed_hydrodynamics owns damping; disable fossen_residual_hydro")

    if distributed_active and matrix_active:
        cfg = full_matrix_hydrodynamics.config
        if _nonzero(cfg.linear_damping_6x6) or _nonzero(cfg.quadratic_damping_6x6):
            raise ValueError(
                "distributed patches own damping in the combined profile; keep full-matrix "
                "linear/quadratic damping zero to avoid double counting"
            )


def _nonzero(value) -> bool:
    return bool(np.any(np.abs(np.asarray(value, dtype=np.float64)) > 1.0e-12))


def _matrix_cfd_damping_overlap(config, cfd_axes) -> bool:
    """Return whether matrix damping and the CFD table share a force axis."""

    axis_to_dof = {"x": 0, "y": 1, "z": 2}
    try:
        active_axes = tuple(cfd_axes)
    except TypeError:
        return True
    for axis in active_axes:
        dof = axis_to_dof.get(str(axis).lower())
        if dof is None:
            return True
        if _nonzero(config.linear_damping_6x6[dof, :]) or _nonzero(
            config.quadratic_damping_6x6[dof, :]
        ):
            return True
    return False


def _cfd_output_dofs(cfd_axes) -> set[int]:
    axis_to_dof = {"x": 0, "y": 1, "z": 2}
    try:
        axes = tuple(cfd_axes)
    except TypeError as exc:
        raise ValueError("cfd_dynamic_wrench axes must be x/y/z") from exc
    output_dofs: set[int] = set()
    for axis in axes:
        dof = axis_to_dof.get(str(axis).lower())
        if dof is None:
            raise ValueError("cfd_dynamic_wrench axes must be x/y/z")
        output_dofs.add(dof)
    if not output_dofs:
        raise ValueError("active cfd_dynamic_wrench must own at least one axis")
    return output_dofs


def _empirical_hydrodynamic_output_dofs(values) -> set[int]:
    """Return generalized-wrench rows owned by enabled empirical terms."""

    output_dofs: set[int] = set()
    if abs(float(values.hydro_pitch_moment_coeff)) > 1.0e-12:
        output_dofs.add(4)
    if any(
        abs(float(value)) > 1.0e-12
        for value in (
            values.hydro_vertical_lift_coeff,
            values.hydro_yawrate_heave_pos_coeff,
            values.hydro_yawrate_heave_neg_coeff,
            values.heave_extra_damping_n_per_mps,
        )
    ):
        output_dofs.add(2)
    return output_dofs


def _matrix_damping_owns_dofs(config, output_dofs: set[int]) -> bool:
    for dof in output_dofs:
        if _nonzero(config.linear_damping_6x6[dof, :]) or _nonzero(
            config.quadratic_damping_6x6[dof, :]
        ):
            return True
    return False


__all__ = ["validate_advanced_hydrodynamics_ownership"]
