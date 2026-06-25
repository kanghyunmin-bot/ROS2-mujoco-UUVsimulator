"""Open-plant neutral-PWM simulation loop."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from physics_contract_mujoco import mujoco
from physics_contract_neutral_buoyancy import apply_neutral_buoyancy
from physics_contract_neutral_output import open_neutral_csv_writer, write_neutral_sample
from physics_contract_neutral_timing import neutral_sample_every, neutral_step_count


def run_neutral_open_loop(
    *,
    model: Any,
    data: Any,
    base_id: int,
    world_qvel_adr: int,
    water_surface_z: float,
    duration_s: float,
    output_csv: Path,
    vehicle_mass: float,
    gravity: float,
    buoyancy_context: Any,
) -> tuple[list[float], list[float]]:
    dt = float(model.opt.timestep)
    steps = neutral_step_count(duration_s, dt)
    sample_every = neutral_sample_every(dt)
    depths: list[float] = []
    vz_down_values: list[float] = []

    file_obj, writer = open_neutral_csv_writer(output_csv)
    with file_obj:
        for step in range(steps + 1):
            data.xfrc_applied[base_id, :] = 0.0
            mujoco.mj_forward(model, data)
            buoyancy_z, weighted_submerged = apply_neutral_buoyancy(data, buoyancy_context)
            if step % sample_every == 0:
                depth, vz_down = sample_neutral_state(
                    writer=writer,
                    data=data,
                    base_id=base_id,
                    world_qvel_adr=world_qvel_adr,
                    water_surface_z=water_surface_z,
                    vehicle_mass=vehicle_mass,
                    gravity=gravity,
                    buoyancy_z=buoyancy_z,
                    weighted_submerged=weighted_submerged,
                )
                depths.append(depth)
                vz_down_values.append(vz_down)
            if step < steps:
                mujoco.mj_step(model, data)
    return depths, vz_down_values


def sample_neutral_state(
    *,
    writer,
    data: Any,
    base_id: int,
    world_qvel_adr: int,
    water_surface_z: float,
    vehicle_mass: float,
    gravity: float,
    buoyancy_z: float,
    weighted_submerged: float,
) -> tuple[float, float]:
    depth = float(water_surface_z - data.xpos[base_id, 2])
    vz_down = -float(data.qvel[world_qvel_adr + 2])
    write_neutral_sample(
        writer,
        sim_time=float(data.time),
        depth=depth,
        vz_down=vz_down,
        buoyancy_z=buoyancy_z,
        weight=float(vehicle_mass * gravity),
        submerged_fraction=weighted_submerged,
    )
    return depth, vz_down


__all__ = ["run_neutral_open_loop", "sample_neutral_state"]
