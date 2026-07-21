"""Neutral-PWM open-plant simulation for physics contract audits."""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from physics_contract_geometry import set_base_depth  # noqa: E402
from physics_contract_mujoco import mujoco  # noqa: E402
from physics_contract_neutral_buoyancy import (  # noqa: E402
    build_neutral_buoyancy_context,
)
from physics_contract_neutral_loop import run_neutral_open_loop  # noqa: E402
from physics_contract_neutral_output import (  # noqa: E402
    build_neutral_sim_summary,
)
from physics_contract_types import NeutralMotionSamples, NeutralSimSummary  # noqa: E402


def simulate_neutral_open_plant(
    *,
    model: mujoco.MjModel,
    data: mujoco.MjData,
    base_id: int,
    world_qpos_adr: int,
    world_qvel_adr: int,
    base_depth_m: float,
    water_surface_z: float,
    duration_s: float,
    output_csv: Path,
    vehicle_mass: float,
    rho: float,
    gravity: float,
    neutral_volume: float,
    buoyancy_scale: float,
    hydro_cfg: Any,
    sim_profile: dict[str, Any],
) -> NeutralSimSummary:
    """Run an open-plant neutral-PWM drift test with custom hydrostatic buoyancy."""

    set_base_depth(model, data, world_qpos_adr, world_qvel_adr, water_surface_z, base_depth_m)
    buoyancy_context = build_neutral_buoyancy_context(
        base_id=base_id,
        world_qpos_adr=world_qpos_adr,
        water_surface_z=water_surface_z,
        rho=rho,
        gravity=gravity,
        neutral_volume=neutral_volume,
        buoyancy_scale=buoyancy_scale,
        hydro_cfg=hydro_cfg,
        sim_profile=sim_profile,
    )
    motion_samples = NeutralMotionSamples()
    depths, vz_down_values = run_neutral_open_loop(
        model=model,
        data=data,
        base_id=base_id,
        world_qvel_adr=world_qvel_adr,
        water_surface_z=water_surface_z,
        duration_s=duration_s,
        output_csv=output_csv,
        vehicle_mass=vehicle_mass,
        gravity=gravity,
        buoyancy_context=buoyancy_context,
        motion_samples=motion_samples,
    )

    return build_neutral_sim_summary(
        label=f"neutral_open_plant_base_depth_{base_depth_m:.3f}",
        duration_s=duration_s,
        base_depth_m=base_depth_m,
        depths=depths,
        vz_down_values=vz_down_values,
        output_csv=output_csv,
        motion_samples=motion_samples,
    )
