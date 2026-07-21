"""Individual hydrodynamics runtime log sections."""

from __future__ import annotations

from typing import Callable

import numpy as np

from sim.runtime.hydrodynamics_runtime_types import HydrodynamicsRuntimeValues


def log_thruster_loop(*, model, thruster_loop_hz: float, log: Callable[[str], None]) -> None:
    log(
        f"[runtime] thruster loop rate: {thruster_loop_hz:.1f} Hz "
        f"(physics dt={float(model.opt.timestep):.4f}s)"
    )


def log_buoyancy_setup(
    *,
    values: HydrodynamicsRuntimeValues,
    vehicle_mass: float,
    rho: float,
    water_surface_z: float,
    log: Callable[[str], None],
) -> None:
    log(
        "[physics] buoyancy setup: "
        f"scale={values.buoyancy_scale:.3f}, mass={vehicle_mass:.3f}kg, "
        f"rho={rho:.1f}, neutral_volume={values.neutral_volume:.5f}m^3, "
        f"half_height={values.half_height:.3f}, model={values.buoyancy_model}, "
        f"slope_scale={values.buoyancy_slope_scale:.2f}, "
        f"water_surface_z={water_surface_z:.3f}"
    )


def log_thruster_immersion(
    *,
    thruster_air_force_scale: float,
    thruster_immersion_half_height_m: float,
    log: Callable[[str], None],
) -> None:
    log(
        "[physics] thruster immersion force scale: "
        f"air_scale={thruster_air_force_scale:.3f}, "
        f"half_height={thruster_immersion_half_height_m:.3f}m"
    )


def log_yaw_torque_scale(*, values: HydrodynamicsRuntimeValues, log: Callable[[str], None]) -> None:
    if abs(values.yaw_torque_scale - values.yaw_torque_scale_config) > 1e-9:
        log(
            f"[physics] yaw torque scale: {values.yaw_torque_scale:.3f} "
            f"(direct T200 mode bypassed configured {values.yaw_torque_scale_config:.3f})"
        )
    else:
        log(f"[physics] yaw torque scale: {values.yaw_torque_scale:.3f}")


def log_custom_hydrodynamics(
    *,
    values: HydrodynamicsRuntimeValues,
    hydro_cfg,
    log: Callable[[str], None],
) -> None:
    log(
        "[physics] hydro 6DOF: "
        f"source={hydro_cfg.model_source}, "
        f"surface_heave_damping={values.surface_heave_damping:.3f} "
        f"(z_scale={values.heave_damping_scale:.3f}, effective={values.full_heave_damping:.3f}), "
        f"added_mass={np.array2string(values.added_mass_diag, precision=3)}, "
        f"lin_damp={np.array2string(values.linear_damping_diag, precision=3)}, "
        f"quad_damp={np.array2string(values.quadratic_damping_diag, precision=3)}, "
        f"current_world={np.array2string(values.water_current_world, precision=3)}"
    )
    if hydro_cfg.ellipsoid_semi_axes is not None:
        log(
            "[physics] ellipsoid 6DOF baseline: "
            f"semi_axes={np.array2string(hydro_cfg.ellipsoid_semi_axes, precision=3)}m, "
            f"neutral_volume={values.neutral_volume:.5f}m^3"
        )


def log_mujoco_hydrodynamics_source(*, log: Callable[[str], None]) -> None:
    log(
        "[physics] hydrodynamics baseline: MuJoCo built-in ellipsoid fluidcoef; "
        "legacy Python full-6DOF damping is inactive. Explicit profile-enabled residual "
        "Fossen added-mass/damping or CFD terms are layered separately and logged above. "
        "Other active knobs: hydrostatic buoyancy/restoring, thruster tuning, "
        "mujoco_fluidcoef_scale, mujoco_fluidcoef_geom_scales, and dynamic_fluidcoef."
    )


__all__ = [
    "log_buoyancy_setup",
    "log_custom_hydrodynamics",
    "log_mujoco_hydrodynamics_source",
    "log_thruster_immersion",
    "log_thruster_loop",
    "log_yaw_torque_scale",
]
