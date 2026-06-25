"""Runtime pool XY-scale override application."""

from __future__ import annotations

from sim.physics.pool_runtime_geom import pool_geom_id


def apply_pool_xy_scale(model, mujoco_module, pool_xy_scale: float) -> None:
    floor_id = pool_geom_id(model, mujoco_module, "pool_floor")
    if floor_id >= 0:
        model.geom_size[floor_id, 0] *= pool_xy_scale
        model.geom_size[floor_id, 1] *= pool_xy_scale

    water_surface_id = pool_geom_id(model, mujoco_module, "water_surface")
    if water_surface_id >= 0:
        model.geom_size[water_surface_id, 0] *= pool_xy_scale
        model.geom_size[water_surface_id, 1] *= pool_xy_scale

    for wall_name in ("pool_wall_px", "pool_wall_nx"):
        wall_id = pool_geom_id(model, mujoco_module, wall_name)
        if wall_id >= 0:
            model.geom_pos[wall_id, 0] *= pool_xy_scale
            model.geom_size[wall_id, 1] *= pool_xy_scale
    for wall_name in ("pool_wall_py", "pool_wall_ny"):
        wall_id = pool_geom_id(model, mujoco_module, wall_name)
        if wall_id >= 0:
            model.geom_pos[wall_id, 1] *= pool_xy_scale
            model.geom_size[wall_id, 0] *= pool_xy_scale

    print(
        "[physics] pool XY scale: "
        f"UUV_POOL_XY_SCALE={pool_xy_scale:.3f} "
        "(plant replay default avoids artificial wall contact)",
        flush=True,
    )


__all__ = ["apply_pool_xy_scale"]
