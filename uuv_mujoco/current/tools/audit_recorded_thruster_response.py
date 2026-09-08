#!/usr/bin/env python3
"""Audit high-rate actuator evidence and replay PWM through the corrected model.

This is an actuator-only comparison at fixed, submerged geometry. It does not
predict closed-loop vehicle motion: recorded controller PWM remains unchanged.
"""

import argparse
import contextlib
import csv
import hashlib
import io
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from check_research_pool_physics import _build_runtime
from physics.thruster_curve_helpers import shape_thruster_command
from physics.thruster_mapping import ARDUSUB_VECTORED_6DOF_SERVO_MAP as MAP, ARDUSUB_VECTORED_6DOF_SERVO_SIGNS as SIGNS
from sim.physics.thruster_force_polynomial import force_from_polynomial_model
from sim.physics.thruster_force_performance import pwm_to_force_from_performance
from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.runtime.sitl_servo_pwm import packet_commands_from_pwm


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def audit_controller(tlog_path):
    from pymavlink import mavutil
    connection = mavutil.mavlink_connection(str(tlog_path))
    mode, armed, rc, rc_stamp, boot = None, False, None, -1.0, 0.0
    changes, neutral_servo, parameters = [], [], {}
    while True:
        message = connection.recv_match(type=["RC_CHANNELS", "HEARTBEAT", "SERVO_OUTPUT_RAW", "PARAM_VALUE"], blocking=False)
        if message is None:
            break
        kind = message.get_type()
        if kind == "HEARTBEAT" and message.type == 12:
            if mode != message.custom_mode:
                changes.append({"approx_boot_s": boot, "mode": int(message.custom_mode)})
            mode = message.custom_mode
            armed = bool(message.base_mode & 128)
        elif kind == "RC_CHANNELS":
            rc = [getattr(message, f"chan{i}_raw") for i in range(1, 7)]
            rc_stamp, boot = message._timestamp, message.time_boot_ms / 1000
        elif kind == "SERVO_OUTPUT_RAW" and mode == 0 and armed and rc is not None:
            if 0 <= message._timestamp - rc_stamp <= .3 and all(abs(v - 1500) <= 25 for v in rc):
                neutral_servo.append([getattr(message, f"servo{i}_raw") for i in range(1, 5)])
        elif kind == "PARAM_VALUE":
            name = message.param_id
            if isinstance(name, bytes):
                name = name.decode().rstrip("\0")
            if name in ("ATC_RAT_YAW_P", "ATC_RAT_YAW_I", "ATC_RAT_YAW_D", "MOT_THST_EXPO"):
                parameters[name] = float(message.param_value)
    connection.close()
    commands = np.array(neutral_servo)
    return {"source_sha256": digest(tlog_path), "mode_changes": changes,
        "neutral_armed_stabilize_servo_samples": len(commands),
        "neutral_stabilize_any_horizontal_saturated_samples": int(np.sum(np.any(np.abs(commands - 1500) >= 396, axis=1))) if len(commands) else 0,
        "neutral_stabilize_horizontal_pwm_ranges": [[int(commands[:, i].min()), int(commands[:, i].max())] for i in range(4)] if len(commands) else [],
        "parameters": parameters,
        "interpretation": "Low-rate telemetry establishes command saturation at neutral RC; it is not a high-rate actuator replay or a physical-vehicle measurement."}


def audit(csv_path, legacy_path, output, tlog_path=None):
    import mujoco
    with csv_path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    times = np.array([float(r["sim_time"]) for r in rows])
    if len(times) < 2 or np.any(np.diff(times) <= 0) or not np.all(np.isfinite(times)):
        raise ValueError("expected ordered finite high-rate samples")
    if np.max(np.diff(times)) > .011:
        raise ValueError("PWM replay requires uninterrupted 100 Hz evidence")
    legacy = json.loads(legacy_path.read_text())
    with contextlib.redirect_stdout(io.StringIO()):
        runtime = _build_runtime(mujoco, profile_name="research_pool_distributed", fluid_model="legacy", use_custom_hydrodynamics=True)
    model, data, actuator = runtime.model, runtime.data, runtime.thruster_actuator
    base_id = int(runtime.state.base_id)
    data.qpos[int(runtime.state.world_qpos_adr) + 2] = -1.5
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    names = actuator.all_thruster_names
    recorded_force = np.array([[float(r[f"{n}_force"]) for n in names] for r in rows])
    recorded_yaw = np.array([float(r["thr_torque_body_z"]) for r in rows])
    # Reconstruct force from recorded effective drive, independently of PWM lag.
    legacy_static = np.array([[
        force_from_polynomial_model(name=n,
            command_shaped=shape_thruster_command(float(r[f"{n}_state"]), legacy["global"]["deadzone"], 1),
            gain=legacy["per_thruster"][n]["gain_scale"], thruster_global=legacy["global"],
            thruster_force_max=21, thruster_reverse_asymmetry={})
        for n in names] for r in rows])
    valid = np.abs(legacy_static) > .1
    ratios = recorded_force[valid] / legacy_static[valid]
    per_channel = {}
    for channel, name in enumerate(MAP, start=1):
        pwm = np.array([float(r[f"sitl_ch{channel}_pwm"]) for r in rows])
        active = np.abs(pwm - 1500) > 25
        per_channel[name] = {
            "pwm_range_us": [float(pwm.min()), float(pwm.max())],
            "saturation_fraction_of_non_deadband_samples": float(np.mean(np.abs(pwm[active] - 1500) >= 396)) if np.any(active) else 0,
            "recorded_force_range_n": [float(recorded_force[:, names.index(name)].min()), float(recorded_force[:, names.index(name)].max())],
        }
    variants, outputs = [], {"sim_time": times, "recorded_yaw_nm": recorded_yaw}
    for voltage in (12, 16, 20):
        with contextlib.redirect_stdout(io.StringIO()):
            actuator.perf_cfg = load_thruster_performance_config(ROOT / "config/thruster_performance.json", requested_voltage=voltage, direct=True)
        actuator.state.update({n: 0 for n in names})
        forces, torque = [], []
        for i, row in enumerate(rows):
            pwm = [int(row[f"sitl_ch{channel}_pwm"]) for channel in range(1, 9)]
            actuator.target.update(packet_commands_from_pwm(all_thruster_names=names, raw_map=MAP, servo_signs=SIGNS, pwm_values=pwm))
            data.time = float(times[i])
            actuator.update_forces(.01 if i == 0 else float(times[i] - times[i-1]), base_id=base_id)
            forces.append([actuator.force_cmd[n] for n in names])
            torque.append(float(actuator.last_torque_body[2]))
        forces, torque = np.array(forces), np.array(torque)
        outputs[f"measured_{voltage}v_yaw_nm"] = torque
        variants.append({"voltage_v": voltage, "peak_yaw_torque_nm": float(np.max(np.abs(torque))),
            "force_range_n": [float(forces.min()), float(forces.max())],
            "yaw_torque_rms_nm": float(np.sqrt(np.mean(torque ** 2)))})
    static_comparison = []
    for pwm in (1510, 1525, 1540, 1600, 1700, 1800, 1900):
        command = (pwm - 1500) / 400
        row = {"pwm_us": pwm, "legacy_horizontal_n": force_from_polynomial_model(
            name="yaw_lf", command_shaped=shape_thruster_command(command, .002, 1), gain=3.3,
            thruster_global=legacy["global"], thruster_force_max=21, thruster_reverse_asymmetry={})}
        for voltage in (12, 16, 20):
            with contextlib.redirect_stdout(io.StringIO()):
                config = load_thruster_performance_config(ROOT / "config/thruster_performance.json", requested_voltage=voltage, direct=True)
            row[f"measured_{voltage}v_n"] = pwm_to_force_from_performance(command, config)
        static_comparison.append(row)
    report = {
        "source": str(csv_path), "source_sha256": digest(csv_path), "legacy_params_sha256": digest(legacy_path),
        "sample_count": len(rows), "duration_s": float(times[-1] - times[0]),
        "sample_period_range_s": [float(np.diff(times).min()), float(np.diff(times).max())],
        "recorded_peak_yaw_rate_deg_s": float(np.rad2deg(max(abs(float(r["ang_vel_body_z"])) for r in rows))),
        "recorded_peak_yaw_torque_nm": float(np.max(np.abs(recorded_yaw))),
        "force_to_legacy_static_ratio_range": [float(ratios.min()), float(ratios.max())],
        "per_channel": per_channel, "static_pwm_comparison": static_comparison, "actuator_replay": variants,
        "limitations": ["Simulation log, not measured physical robot response.",
            "Replay fixes fully submerged geometry and disables uncalibrated inflow; recorded controller PWM is held.",
            "No closed-loop yaw improvement claim follows from this comparison.",
            "Actual ESC bus voltage, mounted effectiveness and transient response remain unidentified."],
        "configuration_sha256": {str(p.relative_to(ROOT)): digest(p) for p in [ROOT/"config/thruster_params.json", ROOT/"config/sim_profiles.json", ROOT/"config/thruster_performance.json"]},
    }
    if tlog_path is not None:
        report["controller_evidence"] = audit_controller(tlog_path)
    output.mkdir(parents=True, exist_ok=True)
    (output / "audit.json").write_text(json.dumps(report, indent=2))
    with (output / "actuator_replay.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(outputs)
        writer.writerows(zip(*outputs.values()))
    print(json.dumps({k: report[k] for k in ("sample_count", "recorded_peak_yaw_rate_deg_s", "recorded_peak_yaw_torque_nm", "force_to_legacy_static_ratio_range", "actuator_replay")}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--legacy-params", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--tlog", type=Path)
    args = parser.parse_args()
    audit(args.csv, args.legacy_params, args.out, args.tlog)
