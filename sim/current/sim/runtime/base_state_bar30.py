"""Bar30 pressure-site depth helpers for the MuJoCo base state."""

from __future__ import annotations

from sim.runtime.base_state_free_joint import base_origin_world, reset_free_joint_velocity


def bar30_world_z(*, data, base_id: int, bar30_site_id: int) -> float:
    if bar30_site_id >= 0:
        return float(data.site_xpos[bar30_site_id, 2])
    return float(base_origin_world(data=data, base_id=base_id)[2])


def bar30_depth_now_m(
    *,
    data,
    water_surface_z: float,
    base_id: int,
    bar30_site_id: int,
) -> float:
    return float(max(0.0, float(water_surface_z) - bar30_world_z(
        data=data,
        base_id=base_id,
        bar30_site_id=bar30_site_id,
    )))


def set_bar30_depth(
    *,
    mujoco,
    model,
    data,
    world_qpos_adr: int,
    world_qvel_adr: int,
    water_surface_z: float,
    base_id: int,
    bar30_site_id: int,
    depth_m: float,
    reset_velocity: bool,
) -> None:
    mujoco.mj_forward(model, data)
    target_site_z = float(water_surface_z) - float(depth_m)
    z_delta = target_site_z - bar30_world_z(
        data=data,
        base_id=base_id,
        bar30_site_id=bar30_site_id,
    )
    data.qpos[world_qpos_adr + 2] += z_delta
    if reset_velocity:
        reset_free_joint_velocity(data=data, world_qvel_adr=world_qvel_adr)
    mujoco.mj_forward(model, data)


__all__ = ["bar30_depth_now_m", "bar30_world_z", "set_bar30_depth"]
