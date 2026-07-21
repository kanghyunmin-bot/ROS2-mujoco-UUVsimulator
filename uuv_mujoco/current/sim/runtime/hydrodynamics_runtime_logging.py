"""Hydrodynamics runtime diagnostic logging."""

from __future__ import annotations

from typing import Callable

from sim.runtime.hydrodynamics_runtime_types import HydrodynamicsRuntimeValues
from sim.runtime.hydrodynamics_runtime_log_sections import (
    log_buoyancy_setup,
    log_custom_hydrodynamics,
    log_mujoco_hydrodynamics_source,
    log_thruster_immersion,
    log_thruster_loop,
    log_yaw_torque_scale,
)


def log_hydrodynamics_runtime_setup(
    *,
    values: HydrodynamicsRuntimeValues,
    hydro_cfg,
    model,
    vehicle_mass: float,
    rho: float,
    water_surface_z: float,
    thruster_air_force_scale: float,
    thruster_immersion_half_height_m: float,
    thruster_loop_hz: float,
    use_custom_hydrodynamics: bool,
    log: Callable[[str], None],
) -> None:
    log_thruster_loop(model=model, thruster_loop_hz=thruster_loop_hz, log=log)
    log_buoyancy_setup(
        values=values,
        vehicle_mass=vehicle_mass,
        rho=rho,
        water_surface_z=water_surface_z,
        log=log,
    )
    log_thruster_immersion(
        thruster_air_force_scale=thruster_air_force_scale,
        thruster_immersion_half_height_m=thruster_immersion_half_height_m,
        log=log,
    )
    log_yaw_torque_scale(values=values, log=log)
    if use_custom_hydrodynamics:
        log_custom_hydrodynamics(values=values, hydro_cfg=hydro_cfg, log=log)
    else:
        log_mujoco_hydrodynamics_source(log=log)


__all__ = [
    "log_buoyancy_setup",
    "log_custom_hydrodynamics",
    "log_hydrodynamics_runtime_setup",
    "log_mujoco_hydrodynamics_source",
    "log_thruster_immersion",
    "log_thruster_loop",
    "log_yaw_torque_scale",
]
