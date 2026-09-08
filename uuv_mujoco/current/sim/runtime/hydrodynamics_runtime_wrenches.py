"""CFD, residual, and Fossen hydrodynamic runtime construction."""

from __future__ import annotations

from typing import Callable

from sim.physics.cfd_dynamic_wrench import build_cfd_dynamic_wrench_runtime
from sim.physics.fossen_residual import (
    build_fossen_residual_runtime,
    build_residual_hydro_runtime,
)
from sim.runtime.hydrodynamics_runtime_types import HydrodynamicWrenchRuntime


def build_hydrodynamic_wrench_runtime(
    *,
    sim_profile: dict,
    use_custom_hydrodynamics: bool,
    heave_extra_damping_n_per_mps: float,
    env_float,
    env_flag,
    to_float_array,
    log: Callable[[str], None],
) -> HydrodynamicWrenchRuntime:
    cfd_dynamic_wrench = build_cfd_dynamic_wrench_runtime(
        sim_profile,
        env_flag=env_flag,
        env_float=env_float,
        to_float_array=to_float_array,
        log=log,
    )
    if cfd_dynamic_wrench.owns_z and heave_extra_damping_n_per_mps > 1.0e-9:
        log(
            "[physics] CFD dynamic wrench owns z/heave; "
            "skipping heave_extra_damping_n_per_mps to avoid double-counting z drag"
        )

    residual_hydro = build_residual_hydro_runtime(
        sim_profile,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        env_float=env_float,
        log=log,
    )
    fossen_residual = build_fossen_residual_runtime(
        sim_profile,
        use_custom_hydrodynamics=use_custom_hydrodynamics,
        env_float=env_float,
        env_flag=env_flag,
        log=log,
    )

    return HydrodynamicWrenchRuntime(
        cfd_dynamic_wrench=cfd_dynamic_wrench,
        cfd_dynamic_wrench_enabled=cfd_dynamic_wrench.enabled,
        cfd_dynamic_wrench_scale=cfd_dynamic_wrench.scale,
        cfd_dynamic_wrench_debug=cfd_dynamic_wrench.debug,
        cfd_dynamic_wrench_last_log_sim_t={"value": -10.0},
        cfd_dynamic_wrench_axes=cfd_dynamic_wrench.axes,
        cfd_dynamic_wrench_owns_z=cfd_dynamic_wrench.owns_z,
        residual_hydro=residual_hydro,
        residual_hydro_active=residual_hydro.active,
        residual_hydro_coeffs=residual_hydro.coeffs,
        fossen_residual=fossen_residual,
        fossen_residual_active=fossen_residual.active,
        fossen_residual_added_mass_active=fossen_residual.added_mass_active,
        fossen_residual_requested_active=fossen_residual.requested_active,
        fossen_residual_requested_added_mass_active=(
            fossen_residual.requested_added_mass_active
        ),
        fossen_residual_linear=fossen_residual.linear,
        fossen_residual_forward_speed=fossen_residual.forward_speed,
        fossen_residual_quadratic=fossen_residual.quadratic,
        fossen_residual_added_mass_matrix=fossen_residual.added_mass_matrix,
    )


__all__ = ["build_hydrodynamic_wrench_runtime"]
