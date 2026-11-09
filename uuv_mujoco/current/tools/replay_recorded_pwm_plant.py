"""Replay frozen PWM through the production MuJoCo physics without ROS or an FCU.

The research case specification references an existing RC replay. Model settings,
initial pose, depth-hold release and sensor sites come from that reference. This
tool changes the actuator input only; it never fits physical coefficients.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

RUNTIME = Path(__file__).resolve().parents[1]
WORKSPACE = RUNTIME.parents[1]
SOURCES = ("real_pwm", "rc_pwm", "rc_pwm_sampled")
STATE_COLUMNS = [
    "bag_time_s", "pressure_depth_m", "qx", "qy", "qz", "qw",
    "gyro_x_radps", "gyro_y_radps", "gyro_z_radps",
    "base_x_m", "base_y_m", "base_z_m", "vx_mps", "vy_mps", "vz_mps",
    "floor_contacts", "contacts", *[f"ch{i}_pwm" for i in range(1, 9)],
]


def sha256(path: Path) -> str:
    """Return a file digest without loading a large trajectory into memory."""
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while data := stream.read(1024 * 1024):
            digest.update(data)
    return digest.hexdigest()


def profile_file_paths(arguments: list[str]) -> list[Path]:
    """Return distinct profile files actually selected by the replay command."""
    return list(dict.fromkeys(
        Path(arguments[i + 1]) for i, value in enumerate(arguments) if value == "--profile-file"
    ))


def validate_pwm_rows(rows: np.ndarray, *, motor_count: int = 8) -> None:
    """Check received times [s] and active PWM commands [µs], preserving disabled values."""
    if rows.ndim != 2 or rows.shape[1] != 1 + motor_count or not len(rows):
        raise ValueError(f"Expected time and exactly {motor_count} PWM channels")
    if not np.isfinite(rows).all() or np.any(np.diff(rows[:, 0]) <= 0):
        raise ValueError("PWM observations must be finite and strictly time ordered")
    pwm = rows[:, 1:]
    disabled = (pwm == 0) | (pwm == 65535)
    if not np.all(disabled | ((pwm >= 800) & (pwm <= 2200))):
        raise ValueError("Unsupported PWM command; refusing silent clipping")
    if not np.array_equal(pwm, np.rint(pwm)):
        raise ValueError("PWM observations must contain integer microseconds")


def runtime_arguments(reference: Path, output: Path) -> list[str]:
    """Adapt frozen launch arguments to an isolated plant without changing the model."""
    commands = json.loads((reference / "commands.json").read_text())["mujoco"]
    args = commands[2:]
    removed = {"--sitl", "--ros2", "--ros2-real-pkg-compat"}
    result = []
    i = 0
    while i < len(args):
        value = args[i]
        if value in removed:
            i += 1
            continue
        if value == "--sitl-mavlink-endpoint":
            i += 2
            continue
        result.append(value.replace("/workspace/", str(WORKSPACE) + "/"))
        i += 1
    source_scene = reference / "bag_scene.xml"
    scene = output / "scene.xml"
    scene.write_text(source_scene.read_text().replace("/workspace/", str(WORKSPACE) + "/"))
    result[result.index("--scene") + 1] = str(scene)
    return result


def create_runtime(args):
    """Use the same setup, raw-PWM conversion, actuator and physics step as the GUI."""
    sys.path.insert(0, str(RUNTIME))
    import run_uuv_mujoco as runner

    profile = runner.load_runner_profile_setup(
        args, env_get=os.environ.get, env_flag=runner._env_flag
    )
    assert profile.plant_replay_direct_rcout
    initial = runner.load_runner_initial_setup(
        args=args, mujoco_module=runner.mujoco, sim_profile=profile.sim_profile,
        uuv_run_mode=profile.uuv_run_mode, env_float=runner._env_float,
        env_flag=runner._env_flag, env_get=os.getenv,
        to_float_array=runner._to_float_array, to_float_matrix=runner._to_float_matrix,
    )
    controls = runner.create_runtime_control_bridge_setup(
        args=args, model=initial.model, data=initial.data,
        initial_depth_hold=initial.initial_depth_hold,
        initial_depth_runtime=initial.initial_depth_runtime,
        initial_runtime_state=initial.initial_runtime_state,
        water_surface_z=initial.water_surface_z,
        scene_fluid_density=initial.scene_fluid_density,
        base_origin_world=initial.base_origin_world,
        bar30_depth_now_m=initial.bar30_depth_now_m,
        world_qpos_adr=initial.world_qpos_adr,
        env_flag=runner._env_flag, env_float=runner._env_float,
    )
    assert controls.ros_bridge_runtime.get() is None
    physics = runner.create_runner_physics_setup(
        args=args, mujoco_module=runner.mujoco, np_module=np,
        config_dir=runner.CONFIG_DIR, sim_profile=profile.sim_profile,
        perf_cfg=profile.perf_cfg, initial_setup=initial, control_setup=controls,
        plant_replay_direct_rcout=True, env_float=runner._env_float,
        env_flag=runner._env_flag, to_float_array=runner._to_float_array, log=print,
    )
    steps = runner.create_runner_step_setup(
        args=args, mujoco_module=runner.mujoco, initial_setup=initial,
        control_setup=controls, physics_setup=physics, plant_replay_direct_rcout=True,
    )
    return runner, initial, controls, physics, steps, profile


def run(args) -> dict:
    """Run native received PWM with zero-order hold on simulation time [s]."""
    output = args.output_dir.resolve()
    if output.exists():
        raise FileExistsError(output)
    spec = json.loads(args.case_spec.read_text())
    reference = Path(spec["reference"])
    manifest = json.loads((reference / "manifest.json").read_text())
    rows = np.load(spec["pwm"])[args.source]
    validate_pwm_rows(rows)
    end = min(spec["end"], args.max_duration_s) if args.max_duration_s else spec["end"]
    rows = rows[rows[:, 0] <= end + 1e-9]
    output.mkdir(parents=True)
    for key, value in manifest["environment"].items():
        os.environ[key] = value
    os.environ.update(
        UUV_RUN_MODE="plant_replay", ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE="1",
        ROS2_UUV_SITL_ALLOW_DIRECT_CMD="0", UUV_STARTUP_ALIGNMENT_HOLD="0",
        UUV_MJ_THRUSTER_DEBUG_CSV="", UUV_REAL_START_SEED_THRUSTER_STATE="0",
    )
    sys.path.insert(0, str(RUNTIME))
    import run_uuv_mujoco as runner_module

    command = runtime_arguments(reference, output)
    runner_args = runner_module.build_parser().parse_args(command)
    assert not runner_args.sitl and not runner_args.ros2
    runner, initial, controls, setup, steps, profile = create_runtime(runner_args)
    physics = setup.physics
    assert profile.perf_cfg["direct"] and profile.perf_cfg["active"]
    assert not controls.sitl_allow_direct_cmd
    model, data = initial.model, initial.data
    floor = runner.mujoco.mj_name2id(model, runner.mujoco.mjtObj.mjOBJ_GEOM, "pool_floor")
    dt = float(model.opt.timestep)
    origin = spec["origin"]
    begin = float(spec.get("begin_s", 0.))
    count, cursor = 0, -1
    release = None
    initial_applied = None
    records, packets = [], []
    next_sample = origin
    started = time.monotonic()
    progress = -1
    try:
        while data.time < origin + end - 1e-9:
            t = float(data.time) - origin
            # Apply every due packet once, in its original received order.
            while cursor + 1 < len(rows) and rows[cursor + 1, 0] <= t + 1e-9:
                cursor += 1
                physics.sitl_servo_runtime.on_packet(rows[cursor, 1:].astype(int).tolist())
                packets.append([t, rows[cursor, 0], cursor])
            # Keeping the target fresh here is an offline ZOH, not a generated packet.
            physics.sitl_servo_runtime.last_wall["value"] = time.monotonic()
            if release is None and t >= spec["release_s"] - 1e-9:
                assert initial.initial_depth_runtime.release("frozen RC release schedule")
                release = t
                initial_applied = {
                    "bag_time_s": t,
                    "pressure_depth_m": initial.bar30_depth_now_m(),
                    "quaternion_wxyz": setup.model_io.sensor_value("imu_quat").tolist(),
                    "angular_velocity_body_radps": setup.model_io.sensor_value("imu_gyro").tolist(),
                }
            previous_time = float(data.time)
            steps.run_step(False, publish_ros=False)
            if float(data.time) <= previous_time:
                raise RuntimeError("MuJoCo time did not advance; possible solver reset")
            count += 1
            if data.time + 1e-9 >= next_sample:
                q = np.asarray(setup.model_io.sensor_value("imu_quat"))
                gyro = np.asarray(setup.model_io.sensor_value("imu_gyro"))
                contacts = list(data.contact)
                records.append([
                    float(data.time) - origin, initial.bar30_depth_now_m(),
                    *q[[1, 2, 3, 0]], *gyro,
                    *initial.base_origin_world(),
                    *data.qvel[initial.world_qvel_adr:initial.world_qvel_adr + 3],
                    sum(c.geom1 == floor or c.geom2 == floor for c in contacts), data.ncon,
                    *physics.sitl_servo_pwm_values,
                ])
                next_sample = float(data.time) + .01
            if not np.isfinite(data.qpos).all() or not np.isfinite(data.qvel).all():
                raise RuntimeError("Nonfinite MuJoCo trajectory")
            band = int(max(0., t) // 30)
            if band != progress:
                print(f"{spec['bag']} {args.source}: {t:.1f}/{end:.1f}s", flush=True)
                progress = band
        assert cursor + 1 == len(rows), (cursor, len(rows))
        record = dict(
            complete=True, bag=spec["bag"], source=args.source,
            elapsed_wall_s=time.monotonic() - started, steps=count, timestep_s=dt,
            begin_s=begin, end_s=end, origin=origin, release_s=release,
            initial_applied=initial_applied,
            initial_state=spec.get("initial_state"),
            priming_packets=int(np.sum(rows[:, 0] < begin)) if "begin_s" in spec else 0,
            source_packets=len(rows), packet_order_preserved=True,
            input_gap_max_s=float(np.diff(rows[:, 0]).max()) if len(rows) > 1 else None,
            input_policy=(
                "Native receipt times, zero-order hold. Neutral before first observation; "
                "last command retained through gaps. No interpolated/fitted PWM or latency shift."
            ),
            reference=str(reference), arguments=command, columns=STATE_COLUMNS,
            physical_parameters_fitted=bool(manifest.get("physical_parameters_fitted", False)),
            network_or_fcu=False,
            surface_pressure_pa=spec["surface_pressure_pa"],
            sha256={str(p): sha256(p) for p in [
                args.case_spec, Path(spec["pwm"]), reference/"manifest.json",
                reference/"bag_scene.xml", RUNTIME/"config/sim_profiles_bag0402_effective.json",
                Path(__file__), RUNTIME/"sim/runtime/simulation_step_physics.py",
                RUNTIME/"sim/runtime/thruster_actuator_forces.py",
                *profile_file_paths(command),
            ]},
        )
        np.savez_compressed(
            output / "plant.npz", states=np.asarray(records), packets=np.asarray(packets)
        )
        (output/"run.json").write_text(json.dumps(record, indent=2) + "\n")
        summary = ["bag", "source", "elapsed_wall_s", "source_packets", "steps"]
        print(json.dumps({k: record[k] for k in summary}), flush=True)
        return record
    finally:
        physics.thruster_debug_runtime.close()
        setup.model_io.qgc_video.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case_spec", type=Path, required=True)
    parser.add_argument("--output_dir", type=Path, required=True)
    parser.add_argument("--source", choices=SOURCES, required=True)
    parser.add_argument("--max_duration_s", type=float)
    run(parser.parse_args())
