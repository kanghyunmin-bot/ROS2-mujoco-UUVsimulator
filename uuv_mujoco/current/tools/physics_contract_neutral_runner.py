"""Neutral open-plant simulation runner for physics contract audits."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from physics_contract_model import mujoco, simulate_neutral_open_plant
from physics_contract_types import NeutralSimSummary


def run_neutral_open_plant_sims(
    *,
    args: Any,
    model: Any,
    base_id: int,
    audited_depths: list[tuple[str, float]],
    output_dir: Path,
    vehicle_mass: float,
    rho: float,
    gravity: float,
    neutral_volume: float,
    buoyancy_scale: float,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
) -> list[NeutralSimSummary]:
    neutral_sims: list[NeutralSimSummary] = []
    if args.simulate_s <= 0.0:
        return neutral_sims
    world_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint")
    if world_joint_id < 0:
        raise SystemExit("world_joint free joint not found")
    world_qpos_adr = int(model.jnt_qposadr[world_joint_id])
    world_qvel_adr = int(model.jnt_dofadr[world_joint_id])
    for label, depth in audited_depths:
        sim_data = mujoco.MjData(model)
        neutral_sims.append(
            simulate_neutral_open_plant(
                model=model,
                data=sim_data,
                base_id=base_id,
                world_qpos_adr=world_qpos_adr,
                world_qvel_adr=world_qvel_adr,
                base_depth_m=depth,
                water_surface_z=float(args.water_surface_z),
                duration_s=float(args.simulate_s),
                output_csv=output_dir / f"neutral_open_plant_{label}.csv",
                vehicle_mass=vehicle_mass,
                rho=rho,
                gravity=gravity,
                neutral_volume=neutral_volume,
                buoyancy_scale=buoyancy_scale,
                hydro_cfg=hydro_cfg,
                sim_profile=sim_profile,
            )
        )
    return neutral_sims


__all__ = ["run_neutral_open_plant_sims"]
