"""Runtime pool-depth override application."""

from __future__ import annotations

from sim.physics.pool_runtime_geom import pool_geom_id


def apply_pool_depth_override(model, mujoco_module, pool_depth_override_m: float) -> None:
    floor_half_height_m = 0.05
    wall_half_height_m = max(0.5 * pool_depth_override_m, 0.05)
    wall_center_z_m = -wall_half_height_m

    floor_id = pool_geom_id(model, mujoco_module, "pool_floor")
    if floor_id >= 0:
        model.geom_pos[floor_id, 2] = -(pool_depth_override_m + floor_half_height_m)
        model.geom_size[floor_id, 2] = floor_half_height_m

    for wall_name in ("pool_wall_px", "pool_wall_nx", "pool_wall_py", "pool_wall_ny"):
        wall_id = pool_geom_id(model, mujoco_module, wall_name)
        if wall_id >= 0:
            model.geom_pos[wall_id, 2] = wall_center_z_m
            model.geom_size[wall_id, 2] = wall_half_height_m

    water_vis_id = pool_geom_id(model, mujoco_module, "water_vis")
    if water_vis_id >= 0:
        model.geom_pos[water_vis_id, 2] = wall_center_z_m
        model.geom_size[water_vis_id, 2] = wall_half_height_m

    print(
        "[physics] pool depth override: "
        f"UUV_POOL_DEPTH_M={pool_depth_override_m:.3f}m "
        "(collision floor/walls and water visual adjusted at runtime)",
        flush=True,
    )


__all__ = ["apply_pool_depth_override"]
