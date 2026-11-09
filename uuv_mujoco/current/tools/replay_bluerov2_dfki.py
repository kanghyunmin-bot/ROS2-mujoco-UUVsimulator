"""Replay DFKI BlueROV2 servo PWM through the production MuJoCo runtime."""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

from replay_recorded_pwm_plant import RUNTIME, create_runtime, sha256, validate_pwm_rows

AXES = np.array([1.0, -1.0, -1.0])
VELOCITY = ("surge_mps", "sway_mps", "heave_mps")
RATE = ("roll_rate_radps", "pitch_rate_radps", "yaw_rate_radps")
ANGLE = ("roll_deg", "pitch_deg", "yaw_deg")
COLUMNS = ("t_s", "surge_mps", "sway_mps", "depth_m", *ANGLE, "heave_mps", *RATE,
           "base_x_m", "base_y_m", "base_z_m", "floor_contacts", "contacts")


def read_rows(path: Path) -> np.ndarray:
    rows = np.genfromtxt(path, delimiter=",", names=True, ndmin=1)
    if not len(rows) or not np.isfinite(rows["t_s"]).all() or np.any(np.diff(rows["t_s"]) <= 0):
        raise ValueError(f"Invalid timestamp sequence in {path}")
    return rows


def initial_state(real: np.ndarray) -> np.void:
    required = ("depth_m", *VELOCITY, *RATE, *ANGLE)
    valid = np.isfinite(np.column_stack([real[name] for name in required])).all(axis=1)
    if not valid.any():
        raise ValueError("No complete initial state in the measured track")
    return real[np.flatnonzero(valid)[0]]


def record_state(initial, stamp: float, floor: int) -> list[float]:
    data = initial.data
    rotation = data.xmat[initial.base_id].reshape(3, 3)
    roll, pitch, yaw = Rotation.from_matrix(rotation).as_euler("xyz", degrees=True)
    twist = data.qvel[initial.world_qvel_adr:initial.world_qvel_adr + 6]
    body_velocity = (rotation.T @ twist[:3]) * AXES
    body_rate = twist[3:] * AXES
    position = initial.base_origin_world()
    floor_contacts = sum(floor in (contact.geom1, contact.geom2) for contact in data.contact)
    return [stamp, *body_velocity[:2], initial.water_surface_z - position[2],
            roll, -pitch, 90.0 - yaw, body_velocity[2], *body_rate, *position,
            floor_contacts, data.ncon]


def run(args: argparse.Namespace) -> None:
    real = read_rows(args.real)
    seed = initial_state(real)
    start = float(seed["t_s"])
    end = float(real["t_s"][-1])
    if args.duration_s is not None:
        end = min(end, start + args.duration_s)
    if end <= start or not np.isfinite(args.depth_m) or args.depth_m <= 0:
        raise ValueError("Positive start depth and a nonempty replay window are required")
    pwm = read_rows(args.pwm)
    rows = np.column_stack([pwm["t_s"], *(pwm[f"pwm{i}"] for i in range(1, 7))])
    validate_pwm_rows(rows, motor_count=6)

    os.environ.update(
        UUV_RUN_MODE="plant_replay", ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE="1",
        ROS2_UUV_SITL_ALLOW_DIRECT_CMD="0", UUV_STARTUP_ALIGNMENT_HOLD="0",
        UUV_MJ_THRUSTER_DEBUG_CSV="", UUV_REAL_START_SEED_THRUSTER_STATE="0",
    )
    sys.path.insert(0, str(RUNTIME))
    import run_uuv_mujoco as runner_module
    from sim.runtime.model_binary_cache import scene_dependency_fingerprint

    scene = RUNTIME / "scenes" / "bluerov2_dfki_scene.xml"
    roll, pitch, yaw = (float(seed[name]) for name in ANGLE)
    command = [
        "--vehicle", "bluerov2_standard", "--headless", "--pwm-layout", args.pwm_layout,
        "--scene", str(scene), "--profile", args.profile,
        "--initial-bar30-depth-m", str(args.depth_m),
        "--initial-rpy-rad", *map(str, np.deg2rad([roll, -pitch, 90.0 - yaw])),
        "--hold-initial-depth-until-release",
        "--release-linear-velocity-body", *map(str, AXES * [seed[name] for name in VELOCITY]),
        "--release-angular-velocity-body", *map(str, AXES * [seed[name] for name in RATE]),
        "--thruster-voltage-trace", str(args.voltage.resolve()),
    ]
    if args.buoyancy_scale is not None:
        command.extend(["--buoyancy-scale", str(args.buoyancy_scale)])
    if args.profile_file is not None:
        command.extend(["--profile-file", str(args.profile_file.resolve())])
    profile_path = args.profile_file or RUNTIME / "config" / "sim_profiles_bluerov2.json"
    model_fingerprint = scene_dependency_fingerprint(scene)
    hashes = {str(path): sha256(path) for path in (args.pwm, args.real, args.voltage, profile_path)}
    runner_args = runner_module.build_parser().parse_args(command)
    args.output.mkdir(parents=True, exist_ok=False)
    runner, initial, controls, setup, steps, profile = create_runtime(runner_args)
    assert controls.ros_bridge_runtime.get() is None and profile.perf_cfg["direct"]
    physics = setup.physics
    floor = runner.mujoco.mj_name2id(initial.model, runner.mujoco.mjtObj.mjOBJ_GEOM, "pool_floor")
    dt = float(initial.model.opt.timestep)
    origin = float(initial.data.time)
    cursor = int(np.searchsorted(rows[:, 0], start + 1e-9, side="right")) - 1
    first = cursor
    packets = []
    first_contact = None
    wall_start = time.monotonic()
    try:
        if cursor >= 0:
            physics.sitl_servo_runtime.on_packet(rows[cursor, 1:].astype(int).tolist())
            packets.append((start, rows[cursor, 0], cursor))
        if not initial.initial_depth_runtime.release("DFKI measured start"):
            raise RuntimeError("Could not release the initial pose")
        with (args.output / "simulation.csv").open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(COLUMNS)
            writer.writerow(record_state(initial, start, floor))
            next_sample = start + .01
            while True:
                stamp = start + float(initial.data.time) - origin
                if stamp >= end - 1e-9:
                    break
                while cursor + 1 < len(rows) and rows[cursor + 1, 0] <= stamp + 1e-9:
                    cursor += 1
                    physics.sitl_servo_runtime.on_packet(rows[cursor, 1:].astype(int).tolist())
                    packets.append((stamp, rows[cursor, 0], cursor))
                physics.sitl_servo_runtime.last_wall["value"] = time.monotonic()
                previous = float(initial.data.time)
                steps.run_step(False, publish_ros=False)
                data = initial.data
                if data.time <= previous or not (np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()):
                    raise RuntimeError(f"Invalid physics state at {stamp:.6f} s")
                stamp = start + float(data.time) - origin
                if data.ncon and first_contact is None:
                    first_contact = stamp
                if stamp >= next_sample - 1e-9 or stamp >= end - 1e-9:
                    writer.writerow(record_state(initial, stamp, floor))
                    next_sample += .01
        if not packets:
            raise RuntimeError("No measured motor packet was applied")
        packets = np.asarray(packets)
        expected = np.flatnonzero((rows[:, 0] >= start) & (rows[:, 0] < stamp - dt - 1e-9))
        applied = packets[packets[:, 1] >= start]
        if not np.isin(expected, applied[:, 2]).all() or np.any(applied[:, 0] - applied[:, 1] > dt + 1e-8):
            raise RuntimeError("PWM application missed a packet or exceeded one physics step")
        np.savetxt(args.output / "packets.csv", packets, delimiter=",", comments="",
                   header="applied_s,original_s,row", fmt=("%.9f", "%.9f", "%d"))
        result = dict(
            complete=True, vehicle="bluerov2_standard", start_s=start, end_s=stamp, timestep_s=dt,
            elapsed_wall_s=time.monotonic() - wall_start, source_packets=len(packets),
            previous_command_index=first, first_contact_s=first_contact,
            pressure_start_depth_m=args.depth_m, voltage_reference="recorded pack voltage proxy",
            model_fingerprint=model_fingerprint, arguments=command, sha256=hashes,
        )
        (args.output / "run.json").write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result, indent=2), flush=True)
    finally:
        physics.thruster_debug_runtime.close()
        setup.model_io.qgc_video.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pwm", type=Path, required=True)
    parser.add_argument("--real", type=Path, required=True)
    parser.add_argument("--voltage", type=Path, required=True)
    parser.add_argument("--depth-m", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--buoyancy-scale", type=float)
    parser.add_argument("--profile", default="bluerov2_standard")
    parser.add_argument("--profile-file", type=Path)
    parser.add_argument("--pwm-layout", choices=("standard", "dfki"), default="dfki")
    run(parser.parse_args())
