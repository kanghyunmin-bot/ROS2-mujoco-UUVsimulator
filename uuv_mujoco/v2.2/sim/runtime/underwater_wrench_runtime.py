"""Underwater hydrostatic and hydrodynamic wrench runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from sim.runtime.underwater_hydrodynamics_runtime import (
    apply_hydrodynamic_wrenches,
    update_relative_acceleration,
)
from sim.runtime.underwater_hydrostatic_runtime import apply_hydrostatic_wrench


@dataclass
class UnderwaterWrenchRuntime:
    """Apply the single-owner underwater wrench to MuJoCo xfrc_applied."""

    model: object
    data: object
    base_id: int
    world_qpos_adr: int
    world_qvel_adr: int
    water_surface_z: float
    rho: float
    gravity: float
    use_custom_hydrodynamics: bool
    hydrostatic: object
    hydrodynamics: object
    initial_depth_hold: dict
    thruster_actuator_runtime: object
    base_origin_world: Callable[[], np.ndarray]
    body_velocity_local: Callable[[], tuple[np.ndarray, np.ndarray]]
    update_dynamic_fluidcoef: Callable[[np.ndarray, np.ndarray], None]
    last_buoy_force: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    last_buoy_point: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    prev_rel_nu_body: np.ndarray = field(default_factory=lambda: np.zeros(6, dtype=np.float64))
    last_flow_world: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))

    def apply(self, dt: float) -> None:
        """Apply hydrostatics plus the selected single-owner hydrodynamic model."""

        hyd = self.hydrodynamics
        data = self.data
        base_id = int(self.base_id)

        data.xfrc_applied[base_id, :] = 0.0
        com = data.xipos[base_id].copy()
        base_rot = data.xmat[base_id].reshape(3, 3)
        base_origin = self.base_origin_world()
        lin_vel_body, ang_vel_body = self.body_velocity_local()
        current_body = base_rot.T @ hyd.water_current_world
        rel_lin_vel_body = lin_vel_body - current_body
        rel_lin_vel_world = base_rot @ rel_lin_vel_body
        self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)

        hydrostatic_result = apply_hydrostatic_wrench(self, base_rot=base_rot, base_origin=base_origin, com=com)

        # MuJoCo xfrc_applied layout: [force_xyz, torque_xyz].
        data.xfrc_applied[base_id, 0:3] += hydrostatic_result.buoy_force_world
        data.xfrc_applied[base_id, 3:6] += hydrostatic_result.buoy_tau_world

        nu_rel_body = np.concatenate((rel_lin_vel_body, ang_vel_body))
        rel_acc_body = update_relative_acceleration(self, nu_rel_body, dt)
        apply_hydrodynamic_wrenches(
            self,
            base_rot=base_rot,
            lin_vel_body=lin_vel_body,
            rel_lin_vel_body=rel_lin_vel_body,
            rel_lin_vel_world=rel_lin_vel_world,
            ang_vel_body=ang_vel_body,
            nu_rel_body=nu_rel_body,
            rel_acc_body=rel_acc_body,
            submerged=hydrostatic_result.submerged,
            buoyancy_submerged=hydrostatic_result.buoyancy_submerged,
        )

        data.xfrc_applied[base_id, 3:6] += self.thruster_actuator_runtime.last_reaction_torque_world

        self.last_buoy_force = hydrostatic_result.buoy_force_world
        self.last_buoy_point = hydrostatic_result.buoy_point_world
