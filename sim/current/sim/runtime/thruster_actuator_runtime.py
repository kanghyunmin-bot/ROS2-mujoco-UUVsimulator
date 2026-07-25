"""MuJoCo thruster actuator update runtime."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

from sim.runtime.thruster_actuator_forces import update_thruster_forces
from sim.runtime.thruster_actuator_immersion import force_immersion_scale
from sim.runtime.thruster_actuator_setup import build_thruster_actuator_kwargs
from sim.runtime.thruster_actuator_visuals import update_propeller_visuals
from sim.runtime.thruster_command_targets import apply_direct_command_targets


@dataclass
class ThrusterActuatorRuntime:
    model: object
    data: object
    actuator_ids: dict[str, int]
    ctrlrange: np.ndarray
    all_thruster_names: list[str]
    site_ids: dict[str, int]
    state: dict[str, float]
    target: dict[str, float]
    force_cmd: dict[str, float]
    prop_phase: dict[str, float]
    prop_qpos_adr: dict[str, int]
    prop_dof_adr: dict[str, int]
    prop_spin_sign: dict[str, float]
    thruster_global: dict
    thruster_scale: dict[str, float]
    thruster_direct_scale: dict[str, float]
    thruster_reverse_asymmetry: dict[str, float | None]
    thruster_tau_up: dict[str, float | None]
    thruster_tau_down: dict[str, float | None]
    perf_cfg: dict
    thruster_force_max: float
    water_surface_z: float
    thruster_air_force_scale: float
    thruster_immersion_half_height_m: float
    buoyancy_model: str
    yaw_torque_scale: float
    yaw_torque_thruster_scales: dict[str, float]
    yaw_thrusters: list[str]
    spin_gain: float
    last_reaction_torque_world: np.ndarray
    last_force_body: np.ndarray
    last_torque_body: np.ndarray

    @classmethod
    def create(cls, **kwargs) -> "ThrusterActuatorRuntime":
        return cls(**build_thruster_actuator_kwargs(**kwargs))

    def force_immersion_scale(self, thr_name: str) -> float:
        """Scale plant force by the actual thruster site water immersion."""
        return force_immersion_scale(self, thr_name)

    def update_forces(self, dt: float, *, base_id: int) -> None:
        update_thruster_forces(self, dt, base_id=base_id)

    def update_propeller_visuals(self, dt: float) -> None:
        update_propeller_visuals(self, dt)

    def apply_direct_command_targets(
        self,
        *,
        command_state,
        mix_horizontal_thrusters: Callable[[float, float, float], np.ndarray],
        vertical_names: list[str],
        horizontal_order: list[str],
    ) -> tuple[float, float, float, float]:
        """Convert recent ROS bridge commands into thruster targets."""
        return apply_direct_command_targets(
            self,
            command_state=command_state,
            mix_horizontal_thrusters=mix_horizontal_thrusters,
            vertical_names=vertical_names,
            horizontal_order=horizontal_order,
        )


__all__ = ["ThrusterActuatorRuntime"]
