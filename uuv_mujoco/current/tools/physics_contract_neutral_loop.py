"""Open-plant neutral-PWM simulation loop."""

from __future__ import annotations

from pathlib import Path
from typing import Any
import math

from physics_contract_mujoco import mujoco
from physics_contract_neutral_buoyancy import apply_neutral_buoyancy
from physics_contract_neutral_output import open_neutral_csv_writer, write_neutral_sample
from physics_contract_neutral_timing import neutral_sample_every, neutral_step_count
from physics_contract_quat import rpy_rad_from_quat_wxyz
from physics_contract_types import NeutralMotionSamples


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
    motion_samples: NeutralMotionSamples | None = None,
) -> tuple[list[float], list[float]]:
    dt = float(model.opt.timestep)
    steps = neutral_step_count(duration_s, dt)
    sample_every = neutral_sample_every(dt)
    depths: list[float] = []
    vz_down_values: list[float] = []
    motion = motion_samples if motion_samples is not None else NeutralMotionSamples()

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
                    world_qpos_adr=buoyancy_context.world_qpos_adr,
                    world_qvel_adr=world_qvel_adr,
                    water_surface_z=water_surface_z,
                    vehicle_mass=vehicle_mass,
                    gravity=gravity,
                    buoyancy_z=buoyancy_z,
                    weighted_submerged=weighted_submerged,
                    motion_samples=motion,
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
    world_qpos_adr: int,
    world_qvel_adr: int,
    water_surface_z: float,
    vehicle_mass: float,
    gravity: float,
    buoyancy_z: float,
    weighted_submerged: float,
    motion_samples: NeutralMotionSamples | None = None,
) -> tuple[float, float]:
    depth = float(water_surface_z - data.xpos[base_id, 2])
    vz_down = -float(data.qvel[world_qvel_adr + 2])
    roll_rad, pitch_rad, yaw_rad = rpy_rad_from_quat_wxyz(
        data.qpos[world_qpos_adr + 3 : world_qpos_adr + 7]
    )
    roll_deg = math.degrees(roll_rad)
    pitch_deg = math.degrees(pitch_rad)
    yaw_deg = math.degrees(yaw_rad)
    angular_rate_x = float(data.qvel[world_qvel_adr + 3])
    angular_rate_y = float(data.qvel[world_qvel_adr + 4])
    angular_rate_z = float(data.qvel[world_qvel_adr + 5])
    angular_speed = math.sqrt(
        angular_rate_x * angular_rate_x
        + angular_rate_y * angular_rate_y
        + angular_rate_z * angular_rate_z
    )
    write_neutral_sample(
        writer,
        sim_time=float(data.time),
        depth=depth,
        vz_down=vz_down,
        buoyancy_z=buoyancy_z,
        weight=float(vehicle_mass * gravity),
        submerged_fraction=weighted_submerged,
        roll_deg=roll_deg,
        pitch_deg=pitch_deg,
        yaw_deg=yaw_deg,
        angular_rate_x_rad_s=angular_rate_x,
        angular_rate_y_rad_s=angular_rate_y,
        angular_rate_z_rad_s=angular_rate_z,
        angular_speed_rad_s=angular_speed,
    )
    if motion_samples is not None:
        motion_samples.roll_deg.append(roll_deg)
        motion_samples.pitch_deg.append(pitch_deg)
        motion_samples.yaw_deg.append(yaw_deg)
        motion_samples.angular_rate_x_rad_s.append(angular_rate_x)
        motion_samples.angular_rate_y_rad_s.append(angular_rate_y)
        motion_samples.angular_rate_z_rad_s.append(angular_rate_z)
        motion_samples.angular_speed_rad_s.append(angular_speed)
    return depth, vz_down


__all__ = ["run_neutral_open_loop", "sample_neutral_state"]
