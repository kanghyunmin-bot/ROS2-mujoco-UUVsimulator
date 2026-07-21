#!/usr/bin/env python3
"""Run one real-RCOU plant replay case and generate the sensor overlay.

This runner keeps the plant parity loop reproducible:

  plant input PWM CSV -> /uuv_mujoco/rc/out_override -> MuJoCo sensors

It does not arm ArduSub and does not use RC override as controller input.
By default the plant input is real /mavros/rc/out from --real-csv.  Use
--input-csv to test a separate actuator stream, such as high-rate SITL JSON
servo output from sensor_replay_sitl_json.py, while keeping --real-csv as the
real sensor/initial-state/overlay contract.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_REAL_CSV = ROOT / "debug/controller_parity_412/real_20260401_feedback/real_controller_feedback_20hz.csv"
DEFAULT_FULL_BAG_CLEAN_START_S = 69.35
DEFAULT_FULL_BAG_CLEAN_END_S = 147.15


def finite(value: object, default: float = math.nan) -> float:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return out if math.isfinite(out) else default


def truthy(value: object) -> bool:
    text = str(value).strip().lower()
    return text in {"1", "true", "yes", "armed"}


def csv_time_extent(path: Path) -> tuple[float, float, list[dict[str, str]]]:
    rows: list[dict[str, str]] = []
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(dict(row))
    times = [finite(row.get("t_s")) for row in rows]
    times = [value for value in times if math.isfinite(value)]
    if not times:
        raise RuntimeError(f"{path} has no finite t_s rows")
    return min(times), max(times), rows


def csv_fieldnames(path: Path) -> set[str]:
    with path.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return set(reader.fieldnames or [])


def looks_like_real_rcout_telemetry(input_csv: Path, real_csv: Path) -> bool:
    if input_csv.resolve() != real_csv.resolve():
        return False
    fields = csv_fieldnames(input_csv)
    rc_out = {f"rc_out_ch{idx}" for idx in range(1, 9)}
    if not rc_out.issubset(fields):
        return False
    high_rate_markers = {
        "source",
        "source_system",
        "source_component",
        "servo_packet_wall_s",
        "json_servo_wall_s",
    }
    return not bool(fields.intersection(high_rate_markers))


def valid_pwm_values(values: list[int]) -> bool:
    return len(values) >= 8 and all(800 <= int(value) <= 2200 for value in values[:8])


def choose_input_time_column(fieldnames: set[str], requested: str) -> str:
    if requested != "auto":
        if requested not in fieldnames:
            raise SystemExit(f"input CSV has no requested time column {requested!r}")
        return requested
    if "t_real_s" in fieldnames:
        return "t_real_s"
    if "t_s" in fieldnames:
        return "t_s"
    raise SystemExit("input CSV has no t_s or t_real_s column")


def choose_input_pwm_prefix(fieldnames: set[str]) -> str:
    for prefix in ("rc_out", "input_rcout", "mavlink_source_servo"):
        if all(f"{prefix}_ch{idx}" in fieldnames for idx in range(1, 9)):
            return prefix
    raise SystemExit(
        "input CSV has no C1-C8 PWM columns with prefix "
        "rc_out/input_rcout/mavlink_source_servo"
    )


def pwm_to_norm(pwm: int) -> float:
    if int(pwm) <= 0 or int(pwm) == 65535:
        return 0.0
    return max(-1.0, min(1.0, (float(pwm) - 1500.0) / 400.0))


def first_order_response(current: float, target: float, dt: float, tau_up: float, tau_down: float) -> float:
    if dt <= 0.0:
        return float(current)
    tau = float(tau_up) if abs(float(target)) >= abs(float(current)) else float(tau_down)
    tau = max(tau, 1.0e-6)
    alpha = 1.0 - math.exp(-float(dt) / tau)
    return float(current + alpha * (target - current))


def load_thruster_tau(path: Path) -> tuple[dict[str, float], dict[str, tuple[float, float]]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    global_cfg = dict(payload.get("global", {}))
    default_tau_up = float(global_cfg.get("tau_up", 0.06))
    default_tau_down = float(global_cfg.get("tau_down", 0.09))
    per_thruster: dict[str, tuple[float, float]] = {}
    for name, cfg in dict(payload.get("per_thruster", {})).items():
        if not isinstance(cfg, dict):
            continue
        tau_up = float(cfg.get("tau_up", default_tau_up))
        tau_down = float(cfg.get("tau_down", default_tau_down))
        per_thruster[str(name)] = (max(tau_up, 1.0e-6), max(tau_down, 1.0e-6))
    return (
        {"tau_up": max(default_tau_up, 1.0e-6), "tau_down": max(default_tau_down, 1.0e-6)},
        per_thruster,
    )


def estimate_release_thruster_history_seed(
    *,
    input_csv: Path,
    input_time_column: str,
    source_t_s: float,
    history_s: float,
    thruster_params_path: Path,
    scope: str,
    scale: float,
) -> dict[str, object]:
    if history_s <= 0.0:
        return {}
    if str(ROOT) not in sys.path:
        sys.path.insert(0, str(ROOT))
    from physics.thruster_mapping import (  # type: ignore
        ARDUSUB_VECTORED_6DOF_SERVO_MAP,
        ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
        PHYSICAL_VERTICAL_THRUSTERS,
    )

    fieldnames = csv_fieldnames(input_csv)
    selected_time_column = choose_input_time_column(fieldnames, input_time_column)
    pwm_prefix = choose_input_pwm_prefix(fieldnames)
    servo_map = list(ARDUSUB_VECTORED_6DOF_SERVO_MAP)
    servo_signs = [float(value) for value in ARDUSUB_VECTORED_6DOF_SERVO_SIGNS]
    vertical_set = set(PHYSICAL_VERTICAL_THRUSTERS)
    if scope == "all":
        selected_seed_names = set(servo_map)
    elif scope == "vertical":
        selected_seed_names = vertical_set
    elif scope == "horizontal":
        selected_seed_names = set(servo_map) - vertical_set
    else:
        raise SystemExit(f"unknown --seed-release-thruster-history-scope {scope!r}")
    seed_scale = float(max(-2.0, min(2.0, float(scale))))
    window_start_s = float(source_t_s) - max(0.0, float(history_s))
    zero_target = {name: 0.0 for name in servo_map}
    events: list[tuple[float, dict[str, float]]] = []
    last_before_window: tuple[float, dict[str, float]] | None = None
    source_event_count = 0

    with input_csv.open("r", newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            t_s = finite(row.get(selected_time_column))
            if not math.isfinite(t_s) or t_s > float(source_t_s) + 1.0e-9:
                continue
            channels = [
                int(round(finite(row.get(f"{pwm_prefix}_ch{idx}"), 0.0)))
                for idx in range(1, 9)
            ]
            if not valid_pwm_values(channels):
                continue
            source_event_count += 1
            target = dict(zero_target)
            for idx, thruster_name in enumerate(servo_map):
                target[thruster_name] = float(
                    max(-1.0, min(1.0, pwm_to_norm(channels[idx]) * servo_signs[idx]))
                )
            if t_s < window_start_s:
                last_before_window = (float(t_s), target)
            else:
                events.append((float(t_s), target))

    events.sort(key=lambda item: item[0])
    if last_before_window is not None:
        events.insert(0, (window_start_s, last_before_window[1]))
    if not events:
        raise SystemExit(
            "cannot compute release thruster history seed: "
            f"no valid PWM rows in {input_csv} before source_t_s={source_t_s:.6f}"
        )

    global_tau, per_tau = load_thruster_tau(thruster_params_path)
    state = dict(zero_target)
    current_target = dict(zero_target)
    current_t = window_start_s
    applied_events = 0
    for event_t, event_target in events:
        event_t = min(max(float(event_t), window_start_s), float(source_t_s))
        dt = max(0.0, event_t - current_t)
        for name in servo_map:
            tau_up, tau_down = per_tau.get(name, (global_tau["tau_up"], global_tau["tau_down"]))
            state[name] = first_order_response(state[name], current_target[name], dt, tau_up, tau_down)
        current_target = dict(event_target)
        current_t = event_t
        applied_events += 1
    final_dt = max(0.0, float(source_t_s) - current_t)
    for name in servo_map:
        tau_up, tau_down = per_tau.get(name, (global_tau["tau_up"], global_tau["tau_down"]))
        state[name] = first_order_response(state[name], current_target[name], final_dt, tau_up, tau_down)

    return {
        "mode": "first_order_pwm_history",
        "input_csv": str(input_csv),
        "input_time_column": selected_time_column,
        "pwm_prefix": pwm_prefix,
        "scope": str(scope),
        "scale": float(seed_scale),
        "thruster_params": str(thruster_params_path),
        "source_t_s": float(source_t_s),
        "history_s": float(history_s),
        "window_start_s": float(window_start_s),
        "source_event_count_before_source": int(source_event_count),
        "history_event_count": int(applied_events),
        "state": {
            name: float(max(-1.0, min(1.0, seed_scale * state[name])))
            for name in servo_map
            if name in selected_seed_names
        },
        "full_state": {name: float(max(-1.0, min(1.0, state[name]))) for name in servo_map},
        "final_target": {name: float(current_target[name]) for name in servo_map},
    }


def first_active_controller_time(rows: list[dict[str, str]]) -> float | None:
    for row in rows:
        t_s = finite(row.get("t_s"))
        if not math.isfinite(t_s):
            continue
        mode = str(row.get("mode", "")).strip()
        armed = truthy(row.get("armed"))
        rc_out = [finite(row.get(f"rc_out_ch{idx}"), 0.0) for idx in range(1, 9)]
        has_valid_output = all(800.0 <= value <= 2200.0 for value in rc_out)
        nonzero_output = any(abs(value) > 1.0 for value in rc_out)
        if armed and mode and has_valid_output and nonzero_output:
            return t_s
    return None


def first_armed_mode_time(
    rows: list[dict[str, str]],
    *,
    mode: str,
    lower_bound_s: float,
) -> float | None:
    expected_mode = str(mode).strip()
    for row in rows:
        t_s = finite(row.get("t_s"))
        if not math.isfinite(t_s) or t_s < float(lower_bound_s) - 1.0e-9:
            continue
        if str(row.get("mode", "")).strip() != expected_mode:
            continue
        if not truthy(row.get("armed")):
            continue
        return float(t_s)
    return None


def quat_xyzw_to_rotmat(x: float, y: float, z: float, w: float) -> list[list[float]] | None:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-9:
        return None
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def mat_transpose_vec_mul(matrix: list[list[float]], vector: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2],
    )


def infer_ros_imu_accel_z_scale(rows: list[dict[str, str]], start_s: float) -> float | None:
    """Infer the real MAVROS IMU z surface scale from the start sample.

    The simulator keeps the internal SITL IMU as physical specific force.  The
    April 1 real /mavros/imu/data surface is lower than the gravity projection
    by about 9%, so plant sensor replay needs this as a ROS observation-surface
    calibration, not as a dynamics or SITL JSON change.
    """
    if not rows:
        return None
    row = min(rows, key=lambda item: abs(finite(item.get("t_s")) - float(start_s)))
    quat = tuple(finite(row.get(key)) for key in ("imu_quat_x", "imu_quat_y", "imu_quat_z", "imu_quat_w"))
    accel_z = finite(row.get("imu_accel_z"))
    if not (all(math.isfinite(value) for value in quat) and math.isfinite(accel_z)):
        return None
    rot_body_to_enu = quat_xyzw_to_rotmat(*quat)
    if rot_body_to_enu is None:
        return None
    # Stationary specific force in ROS/body frame is -R^T * gravity_enu.
    predicted = mat_transpose_vec_mul(rot_body_to_enu, (0.0, 0.0, 9.80665))
    predicted_z = float(predicted[2])
    if abs(predicted_z) < 1.0:
        return None
    scale = float(accel_z / predicted_z)
    if not math.isfinite(scale):
        return None
    return max(0.5, min(1.5, scale))


def resolve_replay_window(real_csv: Path, start: float | None, end: float | None) -> tuple[float, float]:
    t_min, t_max, rows = csv_time_extent(real_csv)
    active_start = first_active_controller_time(rows)
    clean_althold_start = first_armed_mode_time(
        rows,
        mode="ALT_HOLD",
        lower_bound_s=DEFAULT_FULL_BAG_CLEAN_START_S,
    )

    if start is None:
        if t_max <= 100.0 and active_start is not None:
            resolved_start = active_start
        else:
            resolved_start = clean_althold_start or DEFAULT_FULL_BAG_CLEAN_START_S
    else:
        resolved_start = float(start)

    if end is None:
        if t_max <= 100.0:
            resolved_end = t_max
        else:
            resolved_end = DEFAULT_FULL_BAG_CLEAN_END_S
    else:
        resolved_end = float(end)

    if resolved_start < t_min - 1.0e-9:
        raise SystemExit(f"--start {resolved_start:.3f}s is before CSV start {t_min:.3f}s")
    if resolved_end > t_max + 1.0e-9:
        raise SystemExit(f"--end {resolved_end:.3f}s is after CSV end {t_max:.3f}s")
    if t_max <= 100.0 and start is None:
        print(
            "[case] auto window: extracted-short CSV detected; "
            f"using first active controller time {resolved_start:.3f}s -> {resolved_end:.3f}s "
            "instead of the full-bag 69.35s default",
            flush=True,
        )
    elif start is not None and t_max <= 100.0 and resolved_start >= 60.0:
        print(
            "[case] warning: this looks like an extracted-short CSV but --start is late in that "
            f"already-shifted timeline ({resolved_start:.3f}s). Verify this is intentional.",
            flush=True,
        )
    return resolved_start, resolved_end


def run_checked(cmd: list[str], *, cwd: Path, env: dict[str, str] | None = None) -> None:
    print("+ " + " ".join(cmd), flush=True)
    subprocess.run(cmd, cwd=str(cwd), env=env, check=True)


def stop_process_group(proc: subprocess.Popen[bytes], *, timeout_s: float = 8.0) -> None:
    if proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout_s)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(timeout=timeout_s)


def reset_sim_stack() -> None:
    reset_script = ROOT / "reset_uuv_sim.sh"
    if not reset_script.exists():
        return
    subprocess.run(
        [str(reset_script), "--sim-only"],
        cwd=str(ROOT),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def real_start_state_contract(
    real_csv: Path,
    start_s: float,
    velocity_source: str,
    *,
    dvl_z_blend: float = 1.0,
    pose_fd_window_s: float = 1.0,
) -> dict[str, object]:
    tools_dir = ROOT / "tools"
    if str(tools_dir) not in sys.path:
        sys.path.insert(0, str(tools_dir))
    import real_start_state  # type: ignore

    previous = os.environ.get("UUV_REAL_START_VELOCITY_SOURCE")
    previous_blend = os.environ.get("UUV_REAL_START_DVL_Z_BLEND")
    previous_pose_fd = os.environ.get("UUV_REAL_START_POSE_FD_WINDOW_S")
    os.environ["UUV_REAL_START_VELOCITY_SOURCE"] = str(velocity_source)
    os.environ["UUV_REAL_START_DVL_Z_BLEND"] = str(float(dvl_z_blend))
    os.environ["UUV_REAL_START_POSE_FD_WINDOW_S"] = str(float(pose_fd_window_s))
    try:
        return dict(real_start_state.build_state(real_csv, start_s))
    finally:
        if previous is None:
            os.environ.pop("UUV_REAL_START_VELOCITY_SOURCE", None)
        else:
            os.environ["UUV_REAL_START_VELOCITY_SOURCE"] = previous
        if previous_blend is None:
            os.environ.pop("UUV_REAL_START_DVL_Z_BLEND", None)
        else:
            os.environ["UUV_REAL_START_DVL_Z_BLEND"] = previous_blend
        if previous_pose_fd is None:
            os.environ.pop("UUV_REAL_START_POSE_FD_WINDOW_S", None)
        else:
            os.environ["UUV_REAL_START_POSE_FD_WINDOW_S"] = previous_pose_fd


def validate_real_start_state_contract(state: dict[str, object], requested_start_s: float) -> None:
    errors: list[str] = []
    source_t = finite(state.get("source_t_s"))
    if not math.isfinite(source_t):
        errors.append("source_t_s is not finite")
    elif abs(source_t - float(requested_start_s)) > 0.051:
        errors.append(
            f"nearest source_t_s={source_t:.6f}s is too far from requested start "
            f"{float(requested_start_s):.6f}s"
        )

    source_fields = (
        "depth_source",
        "base_depth_source",
        "base_xy_source",
        "attitude_source",
        "velocity_source",
        "angular_velocity_source",
    )
    for field in source_fields:
        value = str(state.get(field, "")).strip()
        if not value or value.startswith("fallback"):
            errors.append(f"{field}={value or '<empty>'}")

    finite_fields = (
        "static_pressure_pa",
        "bar30_surface_pressure_pa",
        "baro_json_depth_m",
        "baro_frontend_depth_m",
        "base_depth_m",
        "depth_m",
        "roll_rad",
        "pitch_rad",
        "yaw_rad",
        "body_vx_mps",
        "body_vy_mps",
        "body_vz_mps",
        "body_wx_radps",
        "body_wy_radps",
        "body_wz_radps",
    )
    for field in finite_fields:
        if not math.isfinite(finite(state.get(field))):
            errors.append(f"{field} is not finite")

    mode = str(state.get("mode", "")).strip().upper()
    if mode != "ALT_HOLD":
        errors.append(f"mode={mode or '<empty>'}, expected ALT_HOLD")
    if not truthy(state.get("armed")):
        errors.append("armed=false")

    if errors:
        raise SystemExit(
            "real start-state contract is incomplete; refusing to run plant replay with fallback state:\n"
            + "\n".join(f"  - {item}" for item in errors)
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--real-csv", type=Path, default=DEFAULT_REAL_CSV)
    parser.add_argument(
        "--input-csv",
        type=Path,
        default=None,
        help=(
            "Optional plant actuator PWM CSV. If omitted, --real-csv rc_out_ch* is replayed. "
            "If set to sitl_controller_io.csv, rc_out_ch* high-rate JSON servo frames are replayed "
            "while --real-csv still supplies initial state and overlay sensors."
        ),
    )
    parser.add_argument(
        "--input-time-column",
        choices=("auto", "t_s", "t_real_s"),
        default="auto",
        help="Time column to use in --input-csv; auto prefers t_real_s when present.",
    )
    parser.add_argument("--start", type=float, default=None)
    parser.add_argument("--end", type=float, default=None)
    parser.add_argument(
        "--plot-start",
        type=float,
        default=None,
        help="Real timestamp where overlay/metrics begin. Use this with an earlier --start to pre-roll the plant.",
    )
    parser.add_argument(
        "--plot-end",
        type=float,
        default=None,
        help="Real timestamp where overlay/metrics end. Defaults to --end.",
    )
    parser.add_argument("--publish-hz", type=float, default=50.0)
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument(
        "--input-lag-s",
        type=float,
        default=-0.45,
        help=(
            "Plant input timestamp correction. The April-1 real /mavros/rc/out "
            "stream is MAVLink telemetry received on ROS after the motor output "
            "was generated, so the default advances the replayed PWM by 0.45s. "
            "The probe records t_real_s independently from input_source_t_s."
        ),
    )
    parser.add_argument("--time-base", choices=("sim", "wall"), default="sim")
    parser.add_argument(
        "--start-velocity-source",
        choices=("local_pose", "dvl", "local_xy_dvl_z", "local_xy_dvl_z_blend", "local_xy_pose_z_fd"),
        default="local_pose",
        help=(
            "Release velocity source for real-start plant replay. local_pose uses "
            "MAVROS local velocity rotated to body; dvl uses raw DVL twist; "
            "local_xy_dvl_z keeps local_pose surge/sway and uses DVL heave; "
            "local_xy_dvl_z_blend blends local heave with DVL heave; "
            "local_xy_pose_z_fd keeps local velocity x/y and derives world-z "
            "velocity from local_pose_z finite difference."
        ),
    )
    parser.add_argument(
        "--start-velocity-dvl-z-blend",
        type=float,
        default=1.0,
        help=(
            "Blend used by --start-velocity-source local_xy_dvl_z_blend. "
            "0 keeps local_pose z velocity, 1 uses DVL z velocity."
        ),
    )
    parser.add_argument(
        "--start-velocity-pose-fd-window-s",
        type=float,
        default=1.0,
        help=(
            "Forward window used by --start-velocity-source local_xy_pose_z_fd "
            "to estimate ENU z velocity from local_pose_z finite difference."
        ),
    )
    parser.add_argument("--prehold-s", type=float, default=0.0)
    parser.add_argument("--input-precondition-s", type=float, default=0.0)
    parser.add_argument(
        "--seed-release-thruster-state",
        choices=("0", "1"),
        default="0",
        help=(
            "Seed first-order thruster internal state from the latched replay PWM "
            "target when the real-start hold is released. Default is off because "
            "low-rate /mavros/rc/out telemetry cannot fully reconstruct the "
            "pre-release actuator state."
        ),
    )
    parser.add_argument(
        "--seed-release-thruster-state-scale",
        type=float,
        default=1.0,
        help="Scale applied to the release thruster-state seed when --seed-release-thruster-state=1.",
    )
    parser.add_argument(
        "--seed-release-thruster-vertical-scale",
        type=float,
        default=None,
        help=(
            "Optional vertical-thruster scale for release state seeding. "
            "Defaults to --seed-release-thruster-state-scale."
        ),
    )
    parser.add_argument(
        "--seed-release-thruster-horizontal-scale",
        type=float,
        default=None,
        help=(
            "Optional horizontal/yaw-thruster scale for release state seeding. "
            "Defaults to --seed-release-thruster-state-scale."
        ),
    )
    parser.add_argument(
        "--seed-release-thruster-forward-scale",
        type=float,
        default=None,
        help="Optional forward-axis scale for horizontal release state seeding.",
    )
    parser.add_argument(
        "--seed-release-thruster-sway-scale",
        type=float,
        default=None,
        help="Optional sway-axis scale for horizontal release state seeding.",
    )
    parser.add_argument(
        "--seed-release-thruster-yaw-scale",
        type=float,
        default=None,
        help="Optional yaw-axis scale for horizontal release state seeding.",
    )
    parser.add_argument(
        "--seed-release-thruster-history-s",
        type=float,
        default=0.0,
        help=(
            "Estimate per-thruster release internal state by replaying this many "
            "seconds of plant-input PWM history through the same first-order "
            "thruster tau model. This implies --seed-release-thruster-state=1 "
            "for the simulator process and writes release_thruster_history_seed.json."
        ),
    )
    parser.add_argument(
        "--seed-release-thruster-history-scope",
        choices=("all", "vertical", "horizontal"),
        default="all",
        help=(
            "Which thruster subset receives the history-estimated release state. "
            "Non-selected thrusters fall back to the normal current-target seed "
            "and axis scale options."
        ),
    )
    parser.add_argument(
        "--seed-release-thruster-history-scale",
        type=float,
        default=1.0,
        help=(
            "Scale applied to the history-estimated release state before it is "
            "passed to the simulator. Negative values are allowed only for "
            "explicit sign diagnostics."
        ),
    )
    parser.add_argument("--startup-wait-s", type=float, default=34.0)
    parser.add_argument("--service-timeout-s", type=float, default=30.0)
    parser.add_argument(
        "--sensor-ready-timeout-s",
        type=float,
        default=15.0,
        help="Probe wait time for the ROS sensor surface before releasing initial hold.",
    )
    parser.add_argument(
        "--release-snapshot-timeout-s",
        type=float,
        default=0.0,
        help=(
            "Probe wait time for the post-release sensor snapshot before writing "
            "the replay t=0 row. Default is 0 because waiting shifts replay "
            "zero while the plant is already moving."
        ),
    )
    parser.add_argument(
        "--geometry-depth-source",
        choices=("bar30", "base"),
        default="base",
        help=(
            "Real-start geometry depth source passed to the simulator. "
            "'base' starts base_link at the real local_position depth and "
            "then calibrates the Bar30 pressure datum to the real sample; "
            "'bar30' starts only the pressure sensor at the real /depth value."
        ),
    )
    parser.add_argument("--horizontal-gain", type=float, default=None)
    parser.add_argument("--vertical-gain", type=float, default=None)
    parser.add_argument("--yaw-lf-gain", type=float, default=None)
    parser.add_argument("--yaw-lr-gain", type=float, default=None)
    parser.add_argument("--yaw-rf-gain", type=float, default=None)
    parser.add_argument("--yaw-rr-gain", type=float, default=None)
    parser.add_argument(
        "--yaw-port-gain",
        type=float,
        default=None,
        help="Convenience override for yaw_lf and yaw_lr direct gain.",
    )
    parser.add_argument(
        "--yaw-starboard-gain",
        type=float,
        default=None,
        help="Convenience override for yaw_rf and yaw_rr direct gain.",
    )
    parser.add_argument("--horizontal-z-offset", type=float, default=None)
    parser.add_argument("--vertical-x-scale", type=float, default=None)
    parser.add_argument("--pool-xy-scale", type=float, default=None)
    parser.add_argument("--thruster-voltage", type=float, default=None)
    parser.add_argument("--fluid-extra-scale", default=None)
    parser.add_argument("--fluid-model", default="current", choices=("current", "legacy"))
    parser.add_argument("--profile", default="current")
    parser.add_argument(
        "--profile-source",
        type=Path,
        default=None,
        help=(
            "Temporary sim_profiles.json source for this case. The runner copies "
            "the file into config/sim_profiles.json before launch and restores the "
            "original file in finally, so SVD/HAN candidate profiles can be replayed "
            "without leaving the workspace profile mutated."
        ),
    )
    parser.add_argument(
        "--thruster-params-source",
        type=Path,
        default=None,
        help=(
            "Temporary thruster_params.json source for this case. The runner copies "
            "the file into config/thruster_params.json before launch and restores "
            "the original file in finally, so profile/thruster candidate pairs can "
            "be replayed without leaving the workspace mutated."
        ),
    )
    parser.add_argument("--buoyancy-scale", type=float, default=None)
    parser.add_argument("--cob-x-offset", type=float, default=None)
    parser.add_argument("--cob-z-offset", type=float, default=None)
    parser.add_argument("--cob-torque-scale", type=float, default=None)
    parser.add_argument("--hydro-pitch-moment-coeff", type=float, default=None)
    parser.add_argument("--hydro-vertical-lift-coeff", type=float, default=None)
    parser.add_argument("--heave-extra-damping", type=float, default=None)
    parser.add_argument("--yaw-torque-scale", type=float, default=None)
    parser.add_argument(
        "--hydrostatic-restoring-active",
        choices=("0", "1"),
        default=None,
        help="Override UUV_HYDROSTATIC_RESTORING_ACTIVE for plant replay A/B checks.",
    )
    parser.add_argument("--hydrostatic-roll-stiffness", type=float, default=None)
    parser.add_argument("--hydrostatic-pitch-stiffness", type=float, default=None)
    parser.add_argument("--hydrostatic-roll-trim-rad", type=float, default=None)
    parser.add_argument("--hydrostatic-pitch-trim-rad", type=float, default=None)
    parser.add_argument(
        "--hydrostatic-trim-from-real-start",
        choices=("0", "1"),
        default=None,
        help="Override UUV_HYDROSTATIC_RESTORING_TRIM_FROM_REAL_START.",
    )
    parser.add_argument(
        "--launch-mode",
        choices=("plant_only", "native_sitl"),
        default="plant_only",
        help=(
            "plant_only launches MuJoCo/ROS only and injects recorded RCOU directly into the plant. "
            "native_sitl preserves the old host-SITL launch path for A/B checks."
        ),
    )
    parser.add_argument("--conda-env", default="ros2_h311")
    parser.add_argument(
        "--skip-overlay",
        action="store_true",
        help="Skip sensor overlay plotting. Use for high-volume sensitivity perturbation runs.",
    )
    parser.add_argument(
        "--skip-audit",
        action="store_true",
        help="Skip plant contract audit. Keep disabled for baselines and enabled only for perturbation speed runs.",
    )
    args = parser.parse_args()
    real_csv = args.real_csv.resolve()
    start_s, end_s = resolve_replay_window(real_csv, args.start, args.end)
    _, _, real_rows_for_contract = csv_time_extent(real_csv)
    plot_start = float(start_s if args.plot_start is None else args.plot_start)
    plot_end = float(end_s if args.plot_end is None else args.plot_end)
    if plot_start < start_s - 1.0e-9:
        raise SystemExit("--plot-start must be greater than or equal to --start")
    if plot_end > end_s + 1.0e-9:
        raise SystemExit("--plot-end must be less than or equal to --end")
    if plot_end <= plot_start:
        raise SystemExit("--plot-end must be greater than --plot-start")

    out_dir = args.out_dir.resolve()
    input_csv = (args.input_csv.resolve() if args.input_csv is not None else real_csv)
    profile_config_path = ROOT / "config/sim_profiles.json"
    thruster_params_config_path = ROOT / "config/thruster_params.json"
    profile_source_path: Path | None = None
    profile_source_bytes: bytes | None = None
    profile_restore_bytes: bytes | None = None
    thruster_params_source_path: Path | None = None
    thruster_params_source_bytes: bytes | None = None
    thruster_params_restore_bytes: bytes | None = None
    if args.profile_source is not None:
        profile_source_path = args.profile_source.expanduser().resolve()
        if not profile_source_path.exists():
            raise SystemExit(f"--profile-source does not exist: {profile_source_path}")
        profile_source_bytes = profile_source_path.read_bytes()
        try:
            profile_source_json = json.loads(profile_source_bytes.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise SystemExit(f"--profile-source is not valid JSON: {profile_source_path}: {exc}") from exc
        if str(args.profile) not in profile_source_json:
            raise SystemExit(
                f"--profile-source has no profile {args.profile!r}: {profile_source_path}"
            )
    if args.thruster_params_source is not None:
        thruster_params_source_path = args.thruster_params_source.expanduser().resolve()
        if not thruster_params_source_path.exists():
            raise SystemExit(f"--thruster-params-source does not exist: {thruster_params_source_path}")
        thruster_params_source_bytes = thruster_params_source_path.read_bytes()
        try:
            json.loads(thruster_params_source_bytes.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise SystemExit(
                f"--thruster-params-source is not valid JSON: {thruster_params_source_path}: {exc}"
            ) from exc
    effective_input_lag_s = float(args.input_lag_s)
    if (
        looks_like_real_rcout_telemetry(input_csv, real_csv)
        and abs(effective_input_lag_s) < 1.0e-12
        and os.environ.get("UUV_PLANT_REPLAY_ALLOW_ZERO_TELEMETRY_LAG", "").strip() != "1"
    ):
        raise SystemExit(
            "refusing plant replay with real /mavros/rc/out telemetry and zero input lag. "
            "This CSV is a low-rate SERVO_OUTPUT_RAW observation, not the exact high-rate "
            "actuator history. Omit --input-lag-s to use the runner's telemetry advance "
            "default, or set UUV_PLANT_REPLAY_ALLOW_ZERO_TELEMETRY_LAG=1 for an explicit "
            "diagnostic A/B run."
        )
    out_dir.mkdir(parents=True, exist_ok=True)
    seed_history_meta: dict[str, object] | None = None
    seed_history_s = max(0.0, float(args.seed_release_thruster_history_s))
    if seed_history_s > 0.0:
        seed_history_params_path = thruster_params_source_path or thruster_params_config_path
        seed_history_meta = estimate_release_thruster_history_seed(
            input_csv=input_csv,
            input_time_column=str(args.input_time_column),
            source_t_s=float(start_s) - effective_input_lag_s,
            history_s=seed_history_s,
            thruster_params_path=seed_history_params_path,
            scope=str(args.seed_release_thruster_history_scope),
            scale=float(args.seed_release_thruster_history_scale),
        )
        (out_dir / "release_thruster_history_seed.json").write_text(
            json.dumps(seed_history_meta, indent=2, sort_keys=True, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(
            "[case] release thruster history seed computed: "
            f"history={seed_history_s:.3f}s "
            f"scope={args.seed_release_thruster_history_scope} "
            f"scale={float(args.seed_release_thruster_history_scale):.3f} "
            f"events={seed_history_meta.get('history_event_count')} "
            f"explicit_thrusters={len(seed_history_meta.get('state', {}))} "
            f"source_t={float(seed_history_meta.get('source_t_s', math.nan)):.3f}s",
            flush=True,
        )
    start_contract = real_start_state_contract(
        real_csv,
        start_s,
        args.start_velocity_source,
        dvl_z_blend=float(args.start_velocity_dvl_z_blend),
        pose_fd_window_s=float(args.start_velocity_pose_fd_window_s),
    )
    validate_real_start_state_contract(start_contract, start_s)
    (out_dir / "real_start_state_contract.json").write_text(
        json.dumps(start_contract, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(
        "[case] real start-state contract OK: "
        f"t={finite(start_contract.get('source_t_s')):.3f}s "
        f"mode={start_contract.get('mode')} armed={bool(start_contract.get('armed'))} "
        f"depth_source={start_contract.get('depth_source')} "
        f"attitude_source={start_contract.get('attitude_source')} "
        f"velocity_source={start_contract.get('velocity_source')}",
        flush=True,
    )
    sim_csv = out_dir / "sim_sensor_replay.csv"
    thruster_csv = out_dir / "thruster_debug.csv"
    sim_log = out_dir / "sim.log"
    overlay_dir = out_dir / "sensor_overlay"
    ros_log_dir = out_dir / "ros_log"
    ros_log_dir.mkdir(parents=True, exist_ok=True)

    env = os.environ.copy()
    env.update(
        {
            "UUV_RUN_MODE": "plant_replay",
            "UUV_REAL_START_STATE": "1",
            "UUV_REAL_START_STATE_CSV": str(real_csv),
            "UUV_REAL_START_STATE_T_S": str(start_s),
            "UUV_REAL_START_GEOMETRY_DEPTH_SOURCE": str(args.geometry_depth_source),
            "UUV_REAL_START_VELOCITY_SOURCE": str(args.start_velocity_source),
            "UUV_REAL_START_DVL_Z_BLEND": str(float(args.start_velocity_dvl_z_blend)),
            "UUV_REAL_START_POSE_FD_WINDOW_S": str(float(args.start_velocity_pose_fd_window_s)),
            "UUV_MJ_THRUSTER_DEBUG_CSV": str(thruster_csv),
            "ROS_LOG_DIR": str(ros_log_dir),
            "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE": "1",
            "UUV_REAL_START_STATE_HOLD_UNTIL_RELEASE": "1",
            "UUV_REAL_START_STATE_AUTO_RELEASE": "0",
            "UUV_REAL_START_SEED_THRUSTER_STATE": (
                "1" if seed_history_meta is not None else str(args.seed_release_thruster_state)
            ),
            "UUV_REAL_START_THRUSTER_STATE_SEED_SCALE": str(float(args.seed_release_thruster_state_scale)),
            "SITL_SCHED_LOOP_RATE": "400",
            "SITL_SENSOR_HZ_DEFAULT": "400",
            "SITL_THRUSTER_LOOP_HZ_DEFAULT": "400",
            "UUV_ROS2_SENSOR_HZ": "400",
            "UUV_THRUSTER_LOOP_HZ": "400",
        }
    )
    if seed_history_meta is not None:
        env["UUV_REAL_START_THRUSTER_STATE_SEED_JSON"] = json.dumps(
            seed_history_meta["state"],
            sort_keys=True,
            separators=(",", ":"),
        )
    if args.seed_release_thruster_vertical_scale is not None:
        env["UUV_REAL_START_THRUSTER_STATE_SEED_VERTICAL_SCALE"] = str(
            float(args.seed_release_thruster_vertical_scale)
        )
    if args.seed_release_thruster_horizontal_scale is not None:
        env["UUV_REAL_START_THRUSTER_STATE_SEED_HORIZONTAL_SCALE"] = str(
            float(args.seed_release_thruster_horizontal_scale)
        )
    if args.seed_release_thruster_forward_scale is not None:
        env["UUV_REAL_START_THRUSTER_STATE_SEED_FORWARD_SCALE"] = str(
            float(args.seed_release_thruster_forward_scale)
        )
    if args.seed_release_thruster_sway_scale is not None:
        env["UUV_REAL_START_THRUSTER_STATE_SEED_SWAY_SCALE"] = str(
            float(args.seed_release_thruster_sway_scale)
        )
    if args.seed_release_thruster_yaw_scale is not None:
        env["UUV_REAL_START_THRUSTER_STATE_SEED_YAW_SCALE"] = str(
            float(args.seed_release_thruster_yaw_scale)
        )
    if sys.platform == "darwin":
        env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    ros_imu_accel_z_scale = infer_ros_imu_accel_z_scale(real_rows_for_contract, start_s)
    if ros_imu_accel_z_scale is not None:
        env.setdefault("ROS2_UUV_ROS_IMU_ACCEL_Z_SCALE", f"{ros_imu_accel_z_scale:.9f}")
        print(
            "[case] ROS IMU accel surface z-scale inferred from real start sample: "
            f"{ros_imu_accel_z_scale:.9f}",
            flush=True,
        )
    if args.horizontal_gain is not None:
        env["UUV_HORIZONTAL_DIRECT_GAIN_SCALE"] = str(float(args.horizontal_gain))
    if args.vertical_gain is not None:
        env["UUV_VERTICAL_DIRECT_GAIN_SCALE"] = str(float(args.vertical_gain))
    yaw_gain_overrides = {
        "UUV_YAW_LF_DIRECT_GAIN_SCALE": args.yaw_lf_gain,
        "UUV_YAW_LR_DIRECT_GAIN_SCALE": args.yaw_lr_gain,
        "UUV_YAW_RF_DIRECT_GAIN_SCALE": args.yaw_rf_gain,
        "UUV_YAW_RR_DIRECT_GAIN_SCALE": args.yaw_rr_gain,
    }
    if args.yaw_port_gain is not None:
        yaw_gain_overrides["UUV_YAW_LF_DIRECT_GAIN_SCALE"] = args.yaw_port_gain
        yaw_gain_overrides["UUV_YAW_LR_DIRECT_GAIN_SCALE"] = args.yaw_port_gain
    if args.yaw_starboard_gain is not None:
        yaw_gain_overrides["UUV_YAW_RF_DIRECT_GAIN_SCALE"] = args.yaw_starboard_gain
        yaw_gain_overrides["UUV_YAW_RR_DIRECT_GAIN_SCALE"] = args.yaw_starboard_gain
    for env_name, value in yaw_gain_overrides.items():
        if value is not None:
            env[env_name] = str(float(value))
    if args.horizontal_z_offset is not None:
        env["UUV_HORIZONTAL_THRUSTER_Z_OFFSET_M"] = str(float(args.horizontal_z_offset))
    if args.vertical_x_scale is not None:
        env["UUV_VERTICAL_THRUSTER_X_SCALE"] = str(float(args.vertical_x_scale))
    if args.pool_xy_scale is not None:
        env["UUV_POOL_XY_SCALE"] = str(float(args.pool_xy_scale))
    if args.fluid_extra_scale:
        env["UUV_MJ_FLUIDCOEF_EXTRA_SCALE"] = str(args.fluid_extra_scale)
    if args.cob_x_offset is not None:
        env["UUV_COB_X_OFFSET_M"] = str(float(args.cob_x_offset))
    if args.cob_z_offset is not None:
        env["UUV_COB_Z_OFFSET_M"] = str(float(args.cob_z_offset))
    if args.cob_torque_scale is not None:
        env["UUV_COB_TORQUE_SCALE"] = str(float(args.cob_torque_scale))
    if args.hydro_pitch_moment_coeff is not None:
        env["UUV_HYDRO_PITCH_MOMENT_COEFF"] = str(float(args.hydro_pitch_moment_coeff))
    if args.hydro_vertical_lift_coeff is not None:
        env["UUV_HYDRO_VERTICAL_LIFT_COEFF"] = str(float(args.hydro_vertical_lift_coeff))
    if args.heave_extra_damping is not None:
        env["UUV_HEAVE_EXTRA_DAMPING_N_PER_MPS"] = str(float(args.heave_extra_damping))
    if args.yaw_torque_scale is not None:
        env["UUV_YAW_TORQUE_SCALE"] = str(float(args.yaw_torque_scale))
    if args.hydrostatic_restoring_active is not None:
        env["UUV_HYDROSTATIC_RESTORING_ACTIVE"] = str(args.hydrostatic_restoring_active)
    if args.hydrostatic_roll_stiffness is not None:
        env["UUV_HYDROSTATIC_RESTORING_ROLL_NM_PER_RAD"] = str(
            float(args.hydrostatic_roll_stiffness)
        )
    if args.hydrostatic_pitch_stiffness is not None:
        env["UUV_HYDROSTATIC_RESTORING_PITCH_NM_PER_RAD"] = str(
            float(args.hydrostatic_pitch_stiffness)
        )
    if args.hydrostatic_roll_trim_rad is not None:
        env["UUV_HYDROSTATIC_RESTORING_ROLL_TRIM_RAD"] = str(
            float(args.hydrostatic_roll_trim_rad)
        )
    if args.hydrostatic_pitch_trim_rad is not None:
        env["UUV_HYDROSTATIC_RESTORING_PITCH_TRIM_RAD"] = str(
            float(args.hydrostatic_pitch_trim_rad)
        )
    if args.hydrostatic_trim_from_real_start is not None:
        env["UUV_HYDROSTATIC_RESTORING_TRIM_FROM_REAL_START"] = str(
            args.hydrostatic_trim_from_real_start
        )

    if args.launch_mode == "plant_only":
        launch_cmd = [
            "./launch_uuv_sim.sh",
            "--ros2",
            "--force-clean",
            "--headless",
            "--no-qgc-video",
            "--tank-549x274x132",
            "--fluid-model",
            str(args.fluid_model),
            "--thruster-perf-direct",
        ]
    else:
        launch_cmd = [
            "./start_sitl_mujoco_mj311.sh",
            "--ros2",
            "--direct-mavlink",
            "--sitl-no-rebuild",
            "--",
            "--headless",
            "--no-qgc-video",
            "--tank-549x274x132",
            "--fluid-model",
            str(args.fluid_model),
        ]
    if args.profile:
        launch_cmd.extend(["--profile", str(args.profile)])
    if args.buoyancy_scale is not None:
        launch_cmd.extend(["--buoyancy-scale", str(float(args.buoyancy_scale))])
    if args.thruster_voltage is not None:
        launch_cmd.extend(["--thruster-voltage", str(float(args.thruster_voltage))])
    probe_args = [
        "debug/controller_parity_412/plant_replay_sensor_probe.py",
        "--real-csv",
        str(real_csv),
        "--input-csv",
        str(input_csv),
        "--input-time-column",
        str(args.input_time_column),
        "--out",
        str(sim_csv),
        "--start",
        str(start_s),
        "--end",
        str(end_s),
        "--publish-hz",
        str(float(args.publish_hz)),
        "--sample-hz",
        str(float(args.sample_hz)),
        "--input-lag-s",
        str(effective_input_lag_s),
        "--time-base",
        str(args.time_base),
        "--prehold-s",
        str(float(args.prehold_s)),
        "--input-precondition-s",
        str(float(args.input_precondition_s)),
        "--service-timeout-s",
        str(float(args.service_timeout_s)),
        "--sensor-ready-timeout-s",
        str(float(args.sensor_ready_timeout_s)),
        "--release-snapshot-timeout-s",
        str(float(args.release_snapshot_timeout_s)),
    ]
    ros_workspace_candidates = [
        ROOT / "rospkg/install/setup.bash",
        ROOT.parents[1] / "rospkg/install/setup.bash",
    ]
    ros_workspace_setup = next((path for path in ros_workspace_candidates if path.exists()), None)
    if ros_workspace_setup is not None:
        probe_shell = (
            f"source {shlex.quote(str(ros_workspace_setup))} "
            + "&& exec python "
            + " ".join(shlex.quote(part) for part in probe_args)
        )
        probe_cmd = ["conda", "run", "-n", args.conda_env, "bash", "-lc", probe_shell]
    else:
        probe_cmd = ["conda", "run", "-n", args.conda_env, "python", *probe_args]
    plot_cmd = [
        "python3",
        "debug/controller_parity_412/plot_plant_sensor_overlay.py",
        "--real-csv",
        str(real_csv),
        "--sim-csv",
        str(sim_csv),
        "--out-dir",
        str(overlay_dir),
        "--real-start",
        str(plot_start),
        "--real-end",
        str(plot_end),
        "--dt",
        "0.05",
    ]
    audit_cmd = [
        "python3",
        "debug/controller_parity_412/audit_plant_sensor_contract.py",
        "--real-csv",
        str(real_csv),
        "--plant-input-csv",
        str(input_csv),
        "--plant-input-time-column",
        str(args.input_time_column),
        "--sim-csv",
        str(sim_csv),
        "--thruster-debug-csv",
        str(thruster_csv),
        "--out-dir",
        str(overlay_dir),
        "--real-start",
        str(plot_start),
        "--real-end",
        str(plot_end),
        "--dt",
        "0.05",
    ]

    if profile_source_path is not None and profile_source_bytes is not None:
        profile_restore_bytes = profile_config_path.read_bytes()
        profile_config_path.write_bytes(profile_source_bytes)
        (out_dir / "profile_source_meta.json").write_text(
            json.dumps(
                {
                    "profile": str(args.profile),
                    "profile_config_path": str(profile_config_path),
                    "profile_source": str(profile_source_path),
                    "restored_after_run": True,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(f"[case] applied temporary profile source: {profile_source_path}", flush=True)
    if thruster_params_source_path is not None and thruster_params_source_bytes is not None:
        thruster_params_restore_bytes = thruster_params_config_path.read_bytes()
        thruster_params_config_path.write_bytes(thruster_params_source_bytes)
        (out_dir / "thruster_params_source_meta.json").write_text(
            json.dumps(
                {
                    "thruster_params_config_path": str(thruster_params_config_path),
                    "thruster_params_source": str(thruster_params_source_path),
                    "restored_after_run": True,
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        print(
            f"[case] applied temporary thruster params source: {thruster_params_source_path}",
            flush=True,
        )

    print(f"[case] output: {out_dir}", flush=True)
    print(f"[case] sim log: {sim_log}", flush=True)
    try:
        with sim_log.open("wb") as log_handle:
            proc = subprocess.Popen(
                launch_cmd,
                cwd=str(ROOT),
                env=env,
                stdout=log_handle,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid,
            )
            try:
                print(f"[case] waiting {float(args.startup_wait_s):.1f}s for simulator bootstrap", flush=True)
                time.sleep(max(0.0, float(args.startup_wait_s)))
                if proc.poll() is not None:
                    raise RuntimeError(f"simulator exited early with code {proc.returncode}; see {sim_log}")
                run_checked(probe_cmd, cwd=ROOT, env=env.copy())
                if not args.skip_overlay:
                    run_checked(plot_cmd, cwd=ROOT)
                if not args.skip_audit:
                    run_checked(audit_cmd, cwd=ROOT)
                converter = shutil.which("rsvg-convert")
                if converter and not args.skip_overlay and (overlay_dir / "plant_sensor_overlay.svg").exists():
                    run_checked(
                        [
                            converter,
                            "-w",
                            "1280",
                            "-o",
                            str(overlay_dir / "plant_sensor_overlay.png"),
                            str(overlay_dir / "plant_sensor_overlay.svg"),
                        ],
                        cwd=ROOT,
                    )
            finally:
                stop_process_group(proc)
                reset_sim_stack()
    finally:
        if thruster_params_restore_bytes is not None:
            thruster_params_config_path.write_bytes(thruster_params_restore_bytes)
            print(f"[case] restored thruster params config: {thruster_params_config_path}", flush=True)
        if profile_restore_bytes is not None:
            profile_config_path.write_bytes(profile_restore_bytes)
            print(f"[case] restored profile config: {profile_config_path}", flush=True)
    print(f"[case] wrote {sim_csv}", flush=True)
    if not args.skip_overlay:
        print(f"[case] wrote {overlay_dir / 'metrics.json'}", flush=True)
        print(f"[case] wrote {overlay_dir / 'plant_sensor_overlay.svg'}", flush=True)
    if not args.skip_audit:
        print(f"[case] wrote {overlay_dir / 'plant_sensor_contract_audit.json'}", flush=True)
        print(f"[case] wrote {overlay_dir / 'plant_sensor_contract_audit.md'}", flush=True)
    if (overlay_dir / "plant_sensor_overlay.png").exists():
        print(f"[case] wrote {overlay_dir / 'plant_sensor_overlay.png'}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
