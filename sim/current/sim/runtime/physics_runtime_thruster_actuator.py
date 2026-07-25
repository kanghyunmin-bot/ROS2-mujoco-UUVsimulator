"""Create the MuJoCo thruster actuator runtime."""

from __future__ import annotations

from typing import Any

from sim.runtime.thruster_actuator_runtime import ThrusterActuatorRuntime


def create_thruster_actuator_runtime(
    *,
    model: Any,
    data: Any,
    mujoco_module: Any,
    actuator_ids: dict[str, int],
    ctrlrange: dict[str, tuple[float, float]],
    all_thruster_names: list[str],
    thruster_global: dict,
    thruster_scale: dict[str, float],
    thruster_direct_scale: dict[str, float],
    thruster_reverse_asymmetry: dict[str, float | None],
    thruster_tau_up: dict[str, float | None],
    thruster_tau_down: dict[str, float | None],
    perf_cfg: dict[str, Any],
    thruster_force_max: float,
    water_surface_z: float,
    thruster_air_force_scale: float,
    thruster_immersion_half_height_m: float,
    buoyancy_model: str,
    yaw_torque_scale: float,
    yaw_torque_thruster_scales: dict[str, float],
    yaw_thrusters: list[str],
    spin_gain: float,
) -> ThrusterActuatorRuntime:
    return ThrusterActuatorRuntime.create(
        model=model,
        data=data,
        mujoco_module=mujoco_module,
        actuator_ids=actuator_ids,
        ctrlrange=ctrlrange,
        all_thruster_names=all_thruster_names,
        thruster_global=thruster_global,
        thruster_scale=thruster_scale,
        thruster_direct_scale=thruster_direct_scale,
        thruster_reverse_asymmetry=thruster_reverse_asymmetry,
        thruster_tau_up=thruster_tau_up,
        thruster_tau_down=thruster_tau_down,
        perf_cfg=perf_cfg,
        thruster_force_max=thruster_force_max,
        water_surface_z=water_surface_z,
        thruster_air_force_scale=thruster_air_force_scale,
        thruster_immersion_half_height_m=thruster_immersion_half_height_m,
        buoyancy_model=buoyancy_model,
        yaw_torque_scale=yaw_torque_scale,
        yaw_torque_thruster_scales=yaw_torque_thruster_scales,
        yaw_thrusters=yaw_thrusters,
        spin_gain=spin_gain,
    )


__all__ = ["create_thruster_actuator_runtime"]
