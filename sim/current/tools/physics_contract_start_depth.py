"""Start-depth candidate calculations for static physics contract audits."""

from __future__ import annotations

from typing import Any

from physics_contract_model import component_half_height, fluid_geom_ids, geom_local_top_z, mujoco, site_local_z
from physics_contract_types import PHYSICAL_VERTICAL_THRUSTERS, PHYSICAL_YAW_THRUSTERS


def build_start_depth_candidates(
    *,
    model: Any,
    base_id: int,
    bar30_local_z: float,
    hydro_cfg: Any,
    surface_depth_m: float,
    minimum_bar30_depth_m: float,
    thruster_half_height_m: float,
    margin_m: float,
) -> list[tuple[str, float]]:
    start_candidates = [
        ("surface_depth_clearance", float(surface_depth_m + 0.05)),
        ("minimum_requested_depth", float(minimum_bar30_depth_m)),
    ]
    for thr_name in PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS:
        site_z = site_local_z(model, f"thr_{thr_name}")
        if site_z is None:
            continue
        start_candidates.append(
            (
                f"thruster:{thr_name}",
                float(site_z + thruster_half_height_m - bar30_local_z + margin_m),
            )
        )
    for gid in fluid_geom_ids(model):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom_{gid}"
        top_z = geom_local_top_z(model, base_id, gid)
        start_candidates.append((f"fluid_geom:{name}", float(top_z - bar30_local_z + margin_m)))
    for component in hydro_cfg.body_components:
        top_z = float(component.buoyancy_pos[2] + component_half_height(component))
        start_candidates.append((f"body_component:{component.name}", float(top_z - bar30_local_z + margin_m)))
    return start_candidates


def auto_depths_from_candidates(start_candidates: list[tuple[str, float]], bar30_local_z: float) -> tuple[float, float]:
    auto_bar30_depth = float(max(depth for _, depth in start_candidates))
    auto_base_depth = float(auto_bar30_depth + bar30_local_z)
    return auto_bar30_depth, auto_base_depth


__all__ = ["auto_depths_from_candidates", "build_start_depth_candidates"]
