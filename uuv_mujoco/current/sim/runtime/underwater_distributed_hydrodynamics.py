"""Apply distributed hull-patch buoyancy and drag to MuJoCo."""

from __future__ import annotations

from typing import Any

import numpy as np

from sim.runtime.underwater_wrench_types import HydrostaticWrenchResult


def apply_distributed_hydrodynamics(
    runtime: Any,
    *,
    base_rot: np.ndarray,
    base_origin: np.ndarray,
    com: np.ndarray,
    lin_vel_com_body: np.ndarray,
    ang_vel_body: np.ndarray,
) -> HydrostaticWrenchResult | None:
    """Apply patch loads and return their hydrostatic summary when enabled."""

    hyd = runtime.hydrodynamics
    model = getattr(hyd, "distributed_hydrodynamics", None)
    if model is None or not model.active:
        runtime.last_distributed_hydrodynamics_result = None
        return None

    environment = hyd.water_environment_runtime
    angular_velocity_world = base_rot @ ang_vel_body
    # ``mj_objectVelocity(..., mjOBJ_BODY, ...)`` reports the linear velocity
    # at the inertial centre, while configured patch offsets are measured from
    # the MuJoCo body origin. Convert the reference-point velocity before the
    # distributed core adds ``omega x patch_offset``.
    linear_velocity_origin_world = (
        base_rot @ lin_vel_com_body
        + np.cross(angular_velocity_world, base_origin - com)
    )
    result = model.evaluate(
        body_position_world_m=base_origin,
        rotation_world_from_body=base_rot,
        linear_velocity_world_mps=linear_velocity_origin_world,
        angular_velocity_world_radps=angular_velocity_world,
        current_world_mps=environment.velocity_world,
        surface_height_world_m=environment.surface_height_world_m,
        wrench_reference_position_body_m=base_rot.T @ (com - base_origin),
        time_s=float(runtime.data.time),
        current_batch_sampler=getattr(environment, "velocity_world_batch", None),
        surface_batch_sampler=getattr(environment, "surface_height_world_m_batch", None),
    )

    # The distributed core and its safety limiter use the same inertial-centre
    # reference as MuJoCo's xfrc_applied torque.
    offsets_from_com = result.positions_world_m - com[None, :]
    base_id = int(runtime.base_id)
    runtime.data.xfrc_applied[base_id, 0:3] += result.force_world_n
    runtime.data.xfrc_applied[base_id, 3:6] += result.torque_world_nm

    buoyancy_forces = result.buoyancy_forces_world_n
    buoyancy_force = np.sum(buoyancy_forces, axis=0)
    buoyancy_torque = (
        np.sum(np.cross(offsets_from_com, buoyancy_forces), axis=0)
        + result.residual_restoring_torque_world_nm
    )
    buoyancy_weights = np.linalg.norm(buoyancy_forces, axis=1)
    total_buoyancy_weight = float(np.sum(buoyancy_weights))
    if total_buoyancy_weight > 1.0e-12:
        buoyancy_point = np.sum(
            result.positions_world_m * buoyancy_weights[:, None],
            axis=0,
        ) / total_buoyancy_weight
    else:
        buoyancy_point = com.copy()

    volume_shares = model.config.volume_shares_m3
    total_volume = float(np.sum(volume_shares))
    if total_volume > 1.0e-12:
        submerged = float(
            np.dot(result.submerged_fractions, volume_shares) / total_volume
        )
    else:
        submerged = float(np.mean(result.submerged_fractions))

    runtime.last_distributed_hydrodynamics_result = result
    return HydrostaticWrenchResult(
        submerged=submerged,
        buoyancy_submerged=submerged,
        buoy_force_world=buoyancy_force,
        buoy_tau_world=buoyancy_torque,
        buoy_point_world=buoyancy_point,
    )


__all__ = ["apply_distributed_hydrodynamics"]
