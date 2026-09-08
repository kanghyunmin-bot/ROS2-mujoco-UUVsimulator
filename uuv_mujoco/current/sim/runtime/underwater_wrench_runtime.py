"""Underwater hydrostatic and hydrodynamic wrench runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import numpy as np

from sim.runtime.thruster_actuator_wrench import reaction_torque_world_for_rotation
from sim.runtime.underwater_hydrodynamics_runtime import (
    apply_hydrodynamic_wrenches,
    update_relative_acceleration,
)
from sim.runtime.underwater_relative_acceleration import (
    reset_relative_acceleration_on_hold_transition,
)
from sim.runtime.underwater_distributed_hydrodynamics import (
    apply_distributed_hydrodynamics,
)
from sim.runtime.underwater_hydrostatic_runtime import apply_hydrostatic_wrench
from sim.runtime.underwater_hydrostatic_samples import body_component_samples, buoyancy_point_samples


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
    prev_rel_nu_valid: bool = False
    prev_rel_sample_time_s: float = float("nan")
    last_flow_world: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    last_hydrodynamic_state_scales: object | None = None
    last_distributed_hydrodynamics_result: object | None = None
    last_full_matrix_wrench_body: np.ndarray = field(
        default_factory=lambda: np.zeros(6, dtype=np.float64)
    )
    cached_body_component_samples: tuple = field(init=False, repr=False)
    cached_buoyancy_point_samples: tuple = field(init=False, repr=False)
    previous_initial_depth_hold_active: bool = field(init=False, repr=False)

    def __post_init__(self) -> None:
        # Component geometry and CoB offsets are immutable for one runtime.
        # Building copied sample arrays every 5 ms added allocations to the
        # hottest vehicle-wrench path without changing any physics.
        hs = self.hydrostatic
        self.cached_body_component_samples = body_component_samples(hs.active_body_components, hs)
        self.cached_buoyancy_point_samples = buoyancy_point_samples(hs.active_buoyancy_points, hs)
        self.previous_initial_depth_hold_active = bool(
            self.initial_depth_hold.get("active", False)
        )

    def apply(self, dt: float) -> None:
        """Apply hydrostatics plus the selected single-owner hydrodynamic model."""

        hyd = self.hydrodynamics
        data = self.data
        base_id = int(self.base_id)
        reset_relative_acceleration_on_hold_transition(
            self,
            hold_active=bool(self.initial_depth_hold.get("active", False)),
        )

        data.xfrc_applied[base_id, :] = 0.0
        com = data.xipos[base_id].copy()
        base_rot = data.xmat[base_id].reshape(3, 3)
        base_origin = self.base_origin_world()
        lin_vel_body, ang_vel_body = self.body_velocity_local()
        # Body linear velocity and full-matrix coefficients are referenced to
        # the inertial centre, so their shared current sample must use the same
        # point when the field has a spatial gradient.
        hyd.current_field_runtime.update(com, float(data.time))
        free_surface = getattr(hyd, "free_surface", None)
        if free_surface is not None and free_surface.active:
            hyd.water_current_world[:] = hyd.water_environment_runtime.velocity_world(
                com,
                float(data.time),
            )
            if not self.use_custom_hydrodynamics:
                self.model.opt.wind[:] = hyd.water_current_world
        current_body = base_rot.T @ hyd.water_current_world
        rel_lin_vel_body = lin_vel_body - current_body
        rel_lin_vel_world = base_rot @ rel_lin_vel_body
        environment = getattr(hyd, "water_environment_runtime", None)
        surface_height = (
            float(self.water_surface_z)
            if environment is None
            else environment.surface_height_world_m(base_origin, float(data.time))
        )
        coefficient_scales = hyd.state_coefficient_scaler.evaluate(
            relative_speed_mps=float(np.linalg.norm(rel_lin_vel_body)),
            depth_m=float(surface_height - base_origin[2]),
            base_rotation_world=base_rot,
        )
        hyd.state_fluidcoef_runtime.set_scale(coefficient_scales.mujoco_fluidcoef)
        self.update_dynamic_fluidcoef(rel_lin_vel_body, ang_vel_body)

        hydrostatic_result = apply_distributed_hydrodynamics(
            self,
            base_rot=base_rot,
            base_origin=base_origin,
            com=com,
            lin_vel_com_body=lin_vel_body,
            ang_vel_body=ang_vel_body,
        )
        if hydrostatic_result is None:
            hydrostatic_result = apply_hydrostatic_wrench(
                self,
                base_rot=base_rot,
                base_origin=base_origin,
                com=com,
            )

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
            coefficient_scales=coefficient_scales,
            submerged=hydrostatic_result.submerged,
            buoyancy_submerged=hydrostatic_result.buoyancy_submerged,
        )

        data.xfrc_applied[base_id, 3:6] += reaction_torque_world_for_rotation(
            self.thruster_actuator_runtime,
            base_rot,
        )

        self.last_buoy_force = hydrostatic_result.buoy_force_world
        self.last_buoy_point = hydrostatic_result.buoy_point_world
        self.last_hydrodynamic_state_scales = coefficient_scales
