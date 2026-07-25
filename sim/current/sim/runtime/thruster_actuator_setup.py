"""Initial state assembly for MuJoCo thruster actuator runtime."""

from __future__ import annotations

import numpy as np

from sim.physics.actuator_geometry import actuator_site_ids, propeller_joint_maps


def _zero_thruster_map(names: list[str]) -> dict[str, float]:
    return {name: 0.0 for name in names}


def build_thruster_actuator_kwargs(
    *,
    model,
    data,
    mujoco_module,
    actuator_ids: dict[str, int],
    ctrlrange: np.ndarray,
    all_thruster_names: list[str],
    thruster_global: dict,
    thruster_scale: dict[str, float],
    thruster_direct_scale: dict[str, float],
    thruster_reverse_asymmetry: dict[str, float | None],
    thruster_tau_up: dict[str, float | None],
    thruster_tau_down: dict[str, float | None],
    perf_cfg: dict,
    thruster_force_max: float,
    water_surface_z: float,
    thruster_air_force_scale: float,
    thruster_immersion_half_height_m: float,
    buoyancy_model: str,
    yaw_torque_scale: float,
    yaw_torque_thruster_scales: dict[str, float],
    yaw_thrusters: list[str],
    spin_gain: float,
) -> dict:
    thruster_names = list(all_thruster_names)
    propeller_maps = propeller_joint_maps(
        model=model,
        mujoco_module=mujoco_module,
        names=thruster_names,
    )
    return {
        "model": model,
        "data": data,
        "actuator_ids": actuator_ids,
        "ctrlrange": ctrlrange,
        "all_thruster_names": thruster_names,
        "site_ids": actuator_site_ids(
            model=model,
            mujoco_module=mujoco_module,
            actuator_ids=actuator_ids,
            names=thruster_names,
        ),
        "state": _zero_thruster_map(thruster_names),
        "target": _zero_thruster_map(thruster_names),
        "force_cmd": _zero_thruster_map(thruster_names),
        "prop_phase": _zero_thruster_map(thruster_names),
        "prop_qpos_adr": propeller_maps.qpos_adr,
        "prop_dof_adr": propeller_maps.dof_adr,
        "prop_spin_sign": propeller_maps.spin_sign,
        "thruster_global": thruster_global,
        "thruster_scale": thruster_scale,
        "thruster_direct_scale": thruster_direct_scale,
        "thruster_reverse_asymmetry": thruster_reverse_asymmetry,
        "thruster_tau_up": thruster_tau_up,
        "thruster_tau_down": thruster_tau_down,
        "perf_cfg": perf_cfg,
        "thruster_force_max": float(thruster_force_max),
        "water_surface_z": float(water_surface_z),
        "thruster_air_force_scale": float(thruster_air_force_scale),
        "thruster_immersion_half_height_m": float(thruster_immersion_half_height_m),
        "buoyancy_model": str(buoyancy_model),
        "yaw_torque_scale": float(yaw_torque_scale),
        "yaw_torque_thruster_scales": dict(yaw_torque_thruster_scales),
        "yaw_thrusters": list(yaw_thrusters),
        "spin_gain": float(spin_gain),
        "last_reaction_torque_world": np.zeros(3, dtype=np.float64),
        "last_force_body": np.zeros(3, dtype=np.float64),
        "last_torque_body": np.zeros(3, dtype=np.float64),
    }


__all__ = ["build_thruster_actuator_kwargs"]
