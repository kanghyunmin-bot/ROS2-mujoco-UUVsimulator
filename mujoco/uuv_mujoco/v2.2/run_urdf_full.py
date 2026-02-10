"""Main runtime for the UUV MuJoCo simulator.

The script keeps simulation, controls, calibration, validation, and optional
ROS2 publishing in a single entrypoint so that model tuning is reproducible.
"""

import argparse
import csv
import glob
import json
import os
import struct
import sys
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

MODEL_PATH = Path(__file__).resolve().parent / "urdf_full_scene.xml"
PROFILE_PATH = Path(__file__).resolve().parent / "sim_profiles.json"
VALIDATION_THRESH_PATH = Path(__file__).resolve().parent / "validation_thresholds.json"


def main() -> None:
    """Parse CLI options, initialize runtime state, and execute selected mode."""
    parser = argparse.ArgumentParser(description="UUV MuJoCo runner")
    parser.add_argument(
        "--scene",
        type=str,
        default=str(MODEL_PATH),
        help="MJCF scene path (default: urdf_full_scene.xml)",
    )
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Run step-response validation headless and exit",
    )
    parser.add_argument(
        "--validation-dir",
        type=str,
        default=str(Path(__file__).resolve().parent / "validation"),
        help="Output directory for validation CSV/JSON",
    )
    parser.add_argument(
        "--validate-thrusters",
        action="store_true",
        help="Run single-thruster direction validation headless and exit",
    )
    parser.add_argument(
        "--validation-thresholds",
        type=str,
        default=str(VALIDATION_THRESH_PATH),
        help="Validation threshold JSON path",
    )
    parser.add_argument(
        "--strict-validation",
        action="store_true",
        help="Return non-zero exit code when validation result is fail",
    )
    parser.add_argument(
        "--calibrate-joystick",
        action="store_true",
        help="Interactive joystick axis/sign/deadzone calibration and save to joystick_map.json",
    )
    parser.add_argument(
        "--calibrate-thrusters",
        action="store_true",
        help="Calibrate per-thruster gain_scale from single-thruster response and save to thruster_params.json",
    )
    parser.add_argument(
        "--calibrate-thresholds",
        action="store_true",
        help="Auto-generate validation thresholds from current model response",
    )
    parser.add_argument(
        "--calibration-dir",
        type=str,
        default=str(Path(__file__).resolve().parent / "validation"),
        help="Output directory for calibration logs",
    )
    parser.add_argument(
        "--joystick-calib-seconds",
        type=float,
        default=1.8,
        help="Sampling seconds per joystick calibration step",
    )
    parser.add_argument(
        "--imu-stabilize",
        action="store_true",
        help="Apply IMU-based roll/pitch stabilization torque to keep upright",
    )
    parser.add_argument(
        "--imu-stab-kp",
        type=float,
        default=8.0,
        help="IMU stabilization proportional gain",
    )
    parser.add_argument(
        "--imu-stab-kd",
        type=float,
        default=2.0,
        help="IMU stabilization derivative gain",
    )
    parser.add_argument(
        "--imu-stab-max",
        type=float,
        default=10.0,
        help="Max stabilization torque (N*m) per axis",
    )
    parser.add_argument(
        "--imu-stab-mode",
        type=str,
        default="thrusters",
        choices=("thrusters", "torque", "both"),
        help="IMU stabilization mode: thrusters | torque | both",
    )
    parser.add_argument(
        "--imu-stab-ki",
        type=float,
        default=0.6,
        help="IMU stabilization integral gain",
    )
    parser.add_argument(
        "--imu-stab-int-max",
        type=float,
        default=0.6,
        help="Max integral term (rad*s)",
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="sim_real",
        help="Simulation profile name (see --list-profiles)",
    )
    parser.add_argument(
        "--profile-file",
        type=str,
        default=str(PROFILE_PATH),
        help="Simulation profile JSON path",
    )
    parser.add_argument(
        "--list-profiles",
        action="store_true",
        help="Print available simulation profiles and exit",
    )
    parser.add_argument(
        "--ros2",
        action="store_true",
        help="Enable ROS2 bridge (/cmd_vel input + IMU/DVL output)",
    )
    parser.add_argument(
        "--ros2-images",
        action="store_true",
        help="Publish stereo camera images and camera_info over ROS2",
    )
    parser.add_argument(
        "--ros2-image-width",
        type=int,
        default=640,
        help="Stereo image width for ROS2 image topics",
    )
    parser.add_argument(
        "--ros2-image-height",
        type=int,
        default=360,
        help="Stereo image height for ROS2 image topics",
    )
    parser.add_argument(
        "--ros2-sensor-hz",
        type=float,
        default=50.0,
        help="ROS2 IMU/DVL publish rate (Hz)",
    )
    parser.add_argument(
        "--ros2-image-hz",
        type=float,
        default=10.0,
        help="ROS2 stereo image publish rate (Hz)",
    )
    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Enable ArduPilot SITL JSON bridge (sends IMU/Pose)",
    )
    parser.add_argument(
        "--sitl-ip",
        type=str,
        default="127.0.0.1",
        help="ArduPilot SITL JSON interface IP",
    )
    parser.add_argument(
        "--sitl-port",
        type=int,
        default=9002,
        help="ArduPilot SITL JSON interface Port",
    )
    args = parser.parse_args()

    # Profile values are externalized so users can retune dynamics without code edits.
    default_profiles = {
        "sim_fast": {
            "half_height": 0.147,
            "buoyancy_scale": 1.0,
            "cob_torque_scale": 1.0,
            "buoyancy_point_blend": 1.0,
            "align_com_to_thruster_plane": False,
            "cob_y_offset": 0.0,
            "thruster_force_max": 65.0,
            "linear_drag": 0.95,
            "angular_drag": 0.10,
            "spin_gain": 26.0,
            "validation_timestep": 0.004,
            "validation_iterations": 20,
            "validation_ls_iterations": 8,
            "validation_step_start": 0.5,
            "validation_step_end": 1.4,
            "validation_total_time": 3.2,
            "validation_step_amp_ratio": 0.35,
        },
        "sim_real": {
            "half_height": 0.147,
            "buoyancy_scale": 1.0,
            "cob_torque_scale": 1.0,
            "buoyancy_point_blend": 1.0,
            "align_com_to_thruster_plane": False,
            "cob_y_offset": 0.0,
            "thruster_force_max": 50.0,
            "linear_drag": 1.20,
            "angular_drag": 0.12,
            "spin_gain": 22.0,
            "validation_timestep": 0.005,
            "validation_iterations": 20,
            "validation_ls_iterations": 8,
            "validation_step_start": 0.5,
            "validation_step_end": 1.4,
            "validation_total_time": 3.2,
            "validation_step_amp_ratio": 0.30,
        },
    }
    default_validation_thresholds = {
        "forward_step": {
            "overshoot_pct_max": 45.0,
            "settling_time_s_max": 1.8,
            "steady_state_drift_max": 0.12,
            "target_ss_abs_min": 0.04,
        },
        "heave_step": {
            "overshoot_pct_max": 55.0,
            "settling_time_s_max": 2.2,
            "steady_state_drift_max": 0.10,
            "target_ss_abs_min": 0.03,
        },
        "yaw_step": {
            "overshoot_pct_max": 60.0,
            "settling_time_s_max": 2.2,
            "steady_state_drift_max": 0.15,
            "target_ss_abs_min": 0.08,
        },
        "thruster_single": {
            "alignment_dot_min": 0.35,
            "min_speed": 0.04,
            "min_pass_ratio": 1.0,
        },
    }

    profile_path = Path(args.profile_file).expanduser()
    if not profile_path.exists():
        profile_path.write_text(json.dumps(default_profiles, indent=2))
    profiles = dict(default_profiles)
    try:
        payload = json.loads(profile_path.read_text())
        if isinstance(payload, dict):
            for name, cfg in payload.items():
                if not isinstance(cfg, dict):
                    continue
                merged = dict(profiles.get(name, {}))
                merged.update(cfg)
                profiles[name] = merged
    except json.JSONDecodeError:
        print(f"[profile] invalid json: {profile_path}, using built-in defaults", flush=True)

    if args.list_profiles:
        print("Available profiles:", flush=True)
        for name in sorted(profiles):
            print(f"  - {name}", flush=True)
        return

    if args.profile not in profiles:
        print(f"[profile] unknown profile: {args.profile}", flush=True)
        print("Available profiles:", flush=True)
        for name in sorted(profiles):
            print(f"  - {name}", flush=True)
        raise SystemExit(2)

    sim_profile = dict(profiles[args.profile])
    print(f"[profile] using '{args.profile}' from {profile_path}", flush=True)

    validation_threshold_path = Path(args.validation_thresholds).expanduser()
    if not validation_threshold_path.exists():
        validation_threshold_path.write_text(json.dumps(default_validation_thresholds, indent=2))
    validation_thresholds = dict(default_validation_thresholds)
    try:
        payload = json.loads(validation_threshold_path.read_text())
        if isinstance(payload, dict):
            for key, cfg in payload.items():
                if isinstance(cfg, dict):
                    merged = dict(validation_thresholds.get(key, {}))
                    merged.update(cfg)
                    validation_thresholds[key] = merged
    except json.JSONDecodeError:
        print(
            f"[validation] invalid threshold json: {validation_threshold_path}, using built-in defaults",
            flush=True,
        )

    # Load model/state once and reuse for runtime, validation, and calibration paths.
    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)

    # Identify base body
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    # Actuator indices
    act = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
        for i in range(model.nu)
    }
    ctrlrange = model.actuator_ctrlrange.copy()
    tune_path = Path(__file__).resolve().parent / "thruster_tune.json"
    joy_map_path = Path(__file__).resolve().parent / "joystick_map.json"
    thruster_params_path = Path(__file__).resolve().parent / "thruster_params.json"

    # Shared command state. Terminal, joystick, and ROS2 all write into this state.
    cmd = {"forward": 0.0, "heave": 0.0, "yaw": 0.0, "sway": 0.0}
    state = {"step": 2.0, "max": 15.0}
    cmd_lock = threading.Lock()
    stop_event = threading.Event()
    paused_flag = {"value": False}
    viewer_control_mode = {"value": True}
    show_thruster_labels = {"value": True}
    follow_camera = {"value": False}
    follow_camera_init = {"value": False}
    show_sensor_overlay = {"value": True}
    camera_mode = {"value": "free"}  # free | follow | stereo_left | stereo_right
    heave_sign = -1.0
    forward_sign = 1.0
    yaw_sign = 1.0
    sway_sign = -1.0

    joy_map = {
        "axes": {
            "sway": 0,
            "forward": 1,
            "yaw": [3, 2],
            "heave": [4, 5, 3],
        },
        "axis_sign": {
            "sway": 1.0,
            "forward": -1.0,
            "yaw": 1.0,
            "heave": -1.0,
        },
        "deadzone": 0.12,
        "yaw_deadzone": 0.18,
        "buttons": {
            "stop": 0,
            "pause": 7,
            "mode": 6,
        },
    }

    def clamp(value: float, max_val: float) -> float:
        return max(-max_val, min(max_val, value))

    def print_status() -> None:
        with cmd_lock:
            forward = cmd["forward"]
            yaw = cmd["yaw"]
            heave = cmd["heave"]
            step = state["step"]
        paused = paused_flag["value"]
        mode = "viewer" if viewer_control_mode["value"] else "terminal"
        labels = "on" if show_thruster_labels["value"] else "off"
        sensors = "on" if show_sensor_overlay["value"] else "off"
        cam_mode = camera_mode["value"]
        sway = cmd["sway"]
        print(
            f"[cmd] fwd={forward:+.1f} sway={sway:+.1f} yaw={yaw:+.1f} heave={heave:+.1f} step={step:.1f} paused={paused} mode={mode} labels={labels} sensors={sensors} cam={cam_mode}",
            flush=True,
        )

    def apply_key(ch: str) -> None:
        with cmd_lock:
            step = state["step"]
            max_val = state["max"]
            if ch == "w":
                cmd["forward"] = clamp(cmd["forward"] + step, max_val)
            elif ch == "s":
                cmd["forward"] = clamp(cmd["forward"] - step, max_val)
            elif ch == "a":
                cmd["sway"] = clamp(cmd["sway"] + step, max_val)
            elif ch == "d":
                cmd["sway"] = clamp(cmd["sway"] - step, max_val)
            elif ch == "q":
                cmd["yaw"] = clamp(cmd["yaw"] + step, max_val)
            elif ch == "e":
                cmd["yaw"] = clamp(cmd["yaw"] - step, max_val)
            elif ch == "r":
                cmd["heave"] = clamp(cmd["heave"] + step, max_val)
            elif ch == "f":
                cmd["heave"] = clamp(cmd["heave"] - step, max_val)
            elif ch == "x":
                cmd["forward"] = 0.0
                cmd["heave"] = 0.0
                cmd["yaw"] = 0.0
                cmd["sway"] = 0.0

    def toggle_pause() -> None:
        paused_flag["value"] = not paused_flag["value"]

    def toggle_mode() -> None:
        viewer_control_mode["value"] = not viewer_control_mode["value"]

    def toggle_thruster_labels() -> None:
        show_thruster_labels["value"] = not show_thruster_labels["value"]

    def toggle_follow_camera() -> None:
        follow_camera["value"] = not follow_camera["value"]
        # Re-apply follow defaults each time follow mode is enabled.
        if follow_camera["value"]:
            follow_camera_init["value"] = False
            camera_mode["value"] = "follow"
        else:
            camera_mode["value"] = "free"

    def toggle_sensor_overlay() -> None:
        show_sensor_overlay["value"] = not show_sensor_overlay["value"]

    def apply_ros_cmd(forward: float, sway: float, yaw: float, heave: float) -> None:
        with cmd_lock:
            max_val = state["max"]
            cmd["forward"] = clamp(forward, max_val)
            cmd["sway"] = clamp(sway, max_val)
            cmd["yaw"] = clamp(yaw, max_val)
            cmd["heave"] = clamp(heave, max_val)
        # ROS2 command stream should directly drive the robot.
        viewer_control_mode["value"] = False

    def load_joystick_map() -> None:
        nonlocal joy_map
        if not joy_map_path.exists():
            joy_map_path.write_text(json.dumps(joy_map, indent=2))
            print(f"[joy] created default mapping: {joy_map_path}", flush=True)
            return
        try:
            payload = json.loads(joy_map_path.read_text())
        except json.JSONDecodeError:
            print(f"[joy] invalid mapping JSON: {joy_map_path}", flush=True)
            return
        if isinstance(payload, dict):
            for key in ("axes", "axis_sign", "buttons"):
                if isinstance(payload.get(key), dict):
                    joy_map[key].update(payload[key])
            if isinstance(payload.get("deadzone"), (int, float)):
                joy_map["deadzone"] = float(payload["deadzone"])
            if isinstance(payload.get("yaw_deadzone"), (int, float)):
                joy_map["yaw_deadzone"] = float(payload["yaw_deadzone"])

    def apply_joystick_axes(axes: dict[int, float], neutral: dict[int, float]) -> None:
        """Convert raw joystick axes into normalized motion commands."""
        deadzone = float(joy_map["deadzone"])
        yaw_deadzone = float(joy_map.get("yaw_deadzone", deadzone))

        def shape_axis(raw: float, dz: float | None = None) -> float:
            threshold = deadzone if dz is None else max(0.0, min(0.95, float(dz)))
            if abs(raw) <= threshold:
                return 0.0
            if raw > 0.0:
                return (raw - threshold) / (1.0 - threshold)
            return (raw + threshold) / (1.0 - threshold)

        def normalize_axis(idx: int) -> float:
            raw = float(axes.get(idx, 0.0))
            center = float(neutral.get(idx, 0.0))
            # Handles both sticks (center ~0) and triggers (center ~-1 or +1).
            span = max(abs(1.0 - center), abs(-1.0 - center), 1e-6)
            v = (raw - center) / span
            return float(np.clip(v, -1.0, 1.0))

        def axis_value(key: str) -> float:
            spec = axis_id.get(key)
            if isinstance(spec, list):
                best = 0.0
                for idx in spec:
                    v = normalize_axis(int(idx))
                    if abs(v) > abs(best):
                        best = v
                return best
            if isinstance(spec, (int, float)):
                return normalize_axis(int(spec))
            return 0.0

        axis_id = joy_map["axes"]
        axis_sign = joy_map["axis_sign"]
        # Backward compatibility for older mapping files.
        if "yaw" not in axis_id and ("yaw_primary" in axis_id or "yaw_alt" in axis_id):
            axis_id["yaw"] = [axis_id.get("yaw_primary", 3), axis_id.get("yaw_alt", 2)]
        if "heave" not in axis_id:
            axis_id["heave"] = [4, 5, 3]

        yaw_axis = axis_value("yaw")
        heave_axis = axis_value("heave")
        with cmd_lock:
            max_val = state["max"]
            cmd["forward"] = clamp(
                float(axis_sign["forward"]) * shape_axis(axis_value("forward")) * max_val,
                max_val,
            )
            cmd["sway"] = clamp(
                float(axis_sign["sway"]) * shape_axis(axis_value("sway")) * max_val,
                max_val,
            )
            cmd["yaw"] = clamp(
                float(axis_sign["yaw"]) * shape_axis(yaw_axis, yaw_deadzone) * max_val,
                max_val,
            )
            cmd["heave"] = clamp(
                float(axis_sign["heave"]) * shape_axis(heave_axis) * max_val,
                max_val,
            )

    def joystick_loop() -> None:
        """Read Linux joystick events and stream commands into the control state."""
        candidates = sorted(glob.glob("/dev/input/js*"))
        if not candidates:
            print("[joy] no joystick found (/dev/input/js*)", flush=True)
            return

        dev = candidates[0]
        try:
            fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            print(f"[joy] failed to open {dev}: {exc}", flush=True)
            return

        print(f"[joy] connected: {dev}", flush=True)
        load_joystick_map()
        print(f"[joy] mapping file: {joy_map_path}", flush=True)
        # Joystick should directly drive thrusters, so use terminal control mode.
        viewer_control_mode["value"] = False
        axes: dict[int, float] = {}
        axis_neutral: dict[int, float] = {}
        event_size = struct.calcsize("IhBB")
        JS_EVENT_BUTTON = 0x01
        JS_EVENT_AXIS = 0x02
        JS_EVENT_INIT = 0x80

        try:
            while not stop_event.is_set():
                try:
                    packet = os.read(fd, event_size * 32)
                except BlockingIOError:
                    time.sleep(0.01)
                    continue
                except OSError as exc:
                    print(f"[joy] read error: {exc}", flush=True)
                    break

                if not packet:
                    time.sleep(0.01)
                    continue

                for i in range(0, len(packet) - event_size + 1, event_size):
                    _, value, etype, number = struct.unpack("IhBB", packet[i : i + event_size])
                    etype_no_init = etype & ~JS_EVENT_INIT
                    if etype_no_init == JS_EVENT_AXIS:
                        raw = float(value) / 32767.0
                        axes[number] = raw
                        if (etype & JS_EVENT_INIT) or (number not in axis_neutral):
                            axis_neutral[number] = raw
                    elif etype_no_init == JS_EVENT_BUTTON and value == 1:
                        print(f"[joy] button pressed: {number}", flush=True)
                        btn = joy_map["buttons"]
                        if number == int(btn["stop"]):
                            print("[joy] action: stop", flush=True)
                            apply_key("x")
                        elif number == int(btn["pause"]):
                            print("[joy] action: pause", flush=True)
                            toggle_pause()
                        elif number == int(btn["mode"]):
                            print("[joy] action: mode toggle", flush=True)
                            toggle_mode()
                if axes:
                    apply_joystick_axes(axes, axis_neutral)
        finally:
            os.close(fd)

    def collect_axis_samples(
        fd: int,
        duration_s: float,
        neutral: dict[int, float],
    ) -> dict[int, dict[str, float]]:
        event_size = struct.calcsize("IhBB")
        JS_EVENT_AXIS = 0x02
        JS_EVENT_INIT = 0x80
        samples: dict[int, list[float]] = {}
        t_end = time.time() + max(0.2, float(duration_s))
        while time.time() < t_end:
            try:
                packet = os.read(fd, event_size * 64)
            except BlockingIOError:
                time.sleep(0.005)
                continue
            except OSError:
                break

            if not packet:
                time.sleep(0.005)
                continue

            for i in range(0, len(packet) - event_size + 1, event_size):
                _, value, etype, number = struct.unpack("IhBB", packet[i : i + event_size])
                etype_no_init = etype & ~JS_EVENT_INIT
                if etype_no_init != JS_EVENT_AXIS:
                    continue
                raw = float(value) / 32767.0
                if (etype & JS_EVENT_INIT) or (number not in neutral):
                    neutral[number] = raw
                delta = raw - float(neutral[number])
                samples.setdefault(int(number), []).append(delta)

        stats: dict[int, dict[str, float]] = {}
        for idx, vals in samples.items():
            arr = np.array(vals, dtype=np.float64)
            if arr.size == 0:
                continue
            stats[int(idx)] = {
                "peak": float(np.max(np.abs(arr))),
                "mean": float(np.mean(arr)),
                "std": float(np.std(arr)),
            }
        return stats

    def calibrate_joystick_mapping() -> bool:
        nonlocal joy_map
        candidates = sorted(glob.glob("/dev/input/js*"))
        if not candidates:
            print("[joy-cal] no joystick found (/dev/input/js*)", flush=True)
            return False
        dev = candidates[0]
        try:
            fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            print(f"[joy-cal] failed to open {dev}: {exc}", flush=True)
            return False

        def wait_ready(message: str) -> None:
            print(message, flush=True)
            if sys.stdin.isatty():
                try:
                    input("Press Enter to continue... ")
                except EOFError:
                    pass
            else:
                time.sleep(0.6)

        try:
            load_joystick_map()
            print(f"[joy-cal] connected: {dev}", flush=True)
            neutral: dict[int, float] = {}
            wait_ready("[joy-cal] Release all sticks/triggers (neutral capture)")
            idle_stats = collect_axis_samples(fd, args.joystick_calib_seconds, neutral)
            noise_peak = 0.0
            for st in idle_stats.values():
                noise_peak = max(noise_peak, float(st.get("peak", 0.0)))
            deadzone = float(np.clip(noise_peak * 2.5 + 0.02, 0.05, 0.30))
            yaw_deadzone = float(np.clip(deadzone + 0.05, 0.08, 0.35))

            prompts = [
                ("sway", "Move LEFT stick LEFT and hold"),
                ("forward", "Move LEFT stick UP and hold"),
                ("yaw", "Move RIGHT stick RIGHT and hold"),
                ("heave", "Move RIGHT stick UP and hold"),
            ]
            used_axes: set[int] = set()
            axis_ids: dict[str, int] = {}
            axis_signs: dict[str, float] = {}

            for key, prompt in prompts:
                wait_ready(f"[joy-cal] {prompt}")
                stats = collect_axis_samples(fd, args.joystick_calib_seconds, neutral)
                ranked = sorted(stats.items(), key=lambda item: item[1].get("peak", 0.0), reverse=True)
                chosen_axis = None
                chosen_stat = None
                for idx, st in ranked:
                    if idx in used_axes:
                        continue
                    if float(st.get("peak", 0.0)) < 0.10:
                        continue
                    chosen_axis = int(idx)
                    chosen_stat = st
                    break
                if chosen_axis is None:
                    print(f"[joy-cal] failed to detect axis for '{key}'", flush=True)
                    return False

                mean_delta = float(chosen_stat.get("mean", 0.0))
                sign = 1.0 if mean_delta >= 0.0 else -1.0
                axis_ids[key] = chosen_axis
                axis_signs[key] = sign
                used_axes.add(chosen_axis)
                print(
                    f"[joy-cal] {key}: axis={chosen_axis}, sign={sign:+.1f}, peak={chosen_stat.get('peak', 0.0):.3f}",
                    flush=True,
                )

            calibrated_map = {
                "axes": {
                    "sway": int(axis_ids["sway"]),
                    "forward": int(axis_ids["forward"]),
                    "yaw": [int(axis_ids["yaw"])],
                    "heave": [int(axis_ids["heave"])],
                },
                "axis_sign": {
                    "sway": float(axis_signs["sway"]),
                    "forward": float(axis_signs["forward"]),
                    "yaw": float(axis_signs["yaw"]),
                    "heave": float(axis_signs["heave"]),
                },
                "deadzone": round(deadzone, 3),
                "yaw_deadzone": round(yaw_deadzone, 3),
                "buttons": dict(joy_map.get("buttons", {"stop": 0, "pause": 7, "mode": 6})),
            }
            joy_map_path.write_text(json.dumps(calibrated_map, indent=2))
            joy_map = calibrated_map
            print(f"[joy-cal] saved: {joy_map_path}", flush=True)
            return True
        finally:
            os.close(fd)

    def input_loop() -> None:
        """Terminal command loop used as fallback/manual override input."""
        print(
            "Terminal control ready.\n"
            "Keys: w/s fwd, a/d sway, q/e yaw, r/f heave, x stop, p status, m mode, l labels, i sensors, c follow, 1/2 stereo cam, 0 free cam, space pause, q quit\n"
            "Joystick: left stick=fwd/sway, right stick=yaw/heave, A=stop, Start=pause, Back=mode\n"
            "Step: +/- to change step size (default 2.0)",
            flush=True,
        )

        # Prefer raw single-key input when attached to a TTY.
        if sys.stdin.isatty():
            try:
                import select
                import termios
                import tty
            except ImportError:
                select = None
                termios = None
                tty = None

            if select and termios and tty:
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setcbreak(fd)
                    while not stop_event.is_set():
                        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                        if not rlist:
                            continue
                        ch = sys.stdin.read(1).lower()
                        if ch in ("q",):
                            stop_event.set()
                            break
                        if ch in ("p",):
                            print_status()
                            continue
                        if ch in ("m",):
                            toggle_mode()
                            print_status()
                            continue
                        if ch in ("l",):
                            toggle_thruster_labels()
                            print_status()
                            continue
                        if ch in ("c",):
                            toggle_follow_camera()
                            print_status()
                            continue
                        if ch in ("i",):
                            toggle_sensor_overlay()
                            print_status()
                            continue
                        if ch in ("1",):
                            camera_mode["value"] = "stereo_left"
                            follow_camera["value"] = False
                            print_status()
                            continue
                        if ch in ("2",):
                            camera_mode["value"] = "stereo_right"
                            follow_camera["value"] = False
                            print_status()
                            continue
                        if ch in ("0",):
                            camera_mode["value"] = "free"
                            follow_camera["value"] = False
                            print_status()
                            continue
                        if ch == " ":
                            toggle_pause()
                            print_status()
                            continue
                        if ch in ("+", "="):
                            with cmd_lock:
                                state["step"] = min(10.0, state["step"] + 0.5)
                            print_status()
                            continue
                        if ch in ("-", "_"):
                            with cmd_lock:
                                state["step"] = max(0.5, state["step"] - 0.5)
                            print_status()
                            continue
                        if ch in ("w", "s", "a", "d", "q", "e", "r", "f", "x"):
                            apply_key(ch)
                            print_status()
                    return
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        # Fallback: line-based input
        while not stop_event.is_set():
            line = sys.stdin.readline()
            if line == "":
                break
            text = line.strip().lower()
            if not text:
                continue
            parts = text.split()
            head = parts[0]
            if head in ("q", "quit", "exit"):
                stop_event.set()
                break
            if head == "p":
                print_status()
                continue
            if head == "m":
                toggle_mode()
                print_status()
                continue
            if head in ("l", "label", "labels"):
                toggle_thruster_labels()
                print_status()
                continue
            if head in ("c", "cam", "camera", "follow"):
                toggle_follow_camera()
                print_status()
                continue
            if head in ("i", "imu", "sensor", "sensors"):
                toggle_sensor_overlay()
                print_status()
                continue
            if head in ("caml", "leftcam", "left"):
                camera_mode["value"] = "stereo_left"
                follow_camera["value"] = False
                print_status()
                continue
            if head in ("camr", "rightcam", "right"):
                camera_mode["value"] = "stereo_right"
                follow_camera["value"] = False
                print_status()
                continue
            if head in ("cam0", "camfree", "freecam"):
                camera_mode["value"] = "free"
                follow_camera["value"] = False
                print_status()
                continue
            if head == "pause":
                toggle_pause()
                print_status()
                continue
            if head == "x":
                apply_key("x")
                print_status()
                continue
            if head == "step" and len(parts) >= 2:
                try:
                    value = float(parts[1])
                except ValueError:
                    print("Invalid step value.", flush=True)
                    continue
                with cmd_lock:
                    state["step"] = max(0.1, abs(value))
                print_status()
                continue
            if head == "set" and len(parts) >= 3:
                target = parts[1]
                try:
                    value = float(parts[2])
                except ValueError:
                    print("Invalid set value.", flush=True)
                    continue
                with cmd_lock:
                    max_val = state["max"]
                    if target in ("fwd", "forward"):
                        cmd["forward"] = clamp(value, max_val)
                    elif target in ("yaw",):
                        cmd["yaw"] = clamp(value, max_val)
                    elif target in ("heave", "z"):
                        cmd["heave"] = clamp(value, max_val)
                    else:
                        print("Unknown set target. Use fwd, yaw, heave.", flush=True)
                        continue
                print_status()
                continue
            for token in parts:
                for ch in token:
                    if ch in ("w", "s", "a", "d", "q", "e", "r", "f", "x"):
                        apply_key(ch)
            print_status()

    ros_bridge = None
    if args.ros2:
        try:
            from ros2_bridge import Ros2Bridge

            ros_bridge = Ros2Bridge(
                model=model,
                command_callback=apply_ros_cmd,
                cmd_limit=state["max"],
                publish_images=args.ros2_images,
                image_width=args.ros2_image_width,
                image_height=args.ros2_image_height,
                sensor_hz=args.ros2_sensor_hz,
                image_hz=args.ros2_image_hz,
                enable_mavros=True,
                enable_sitl=args.sitl,
                sitl_ip=args.sitl_ip,
                sitl_port=args.sitl_port,
            )
            viewer_control_mode["value"] = False
            print(
                "[ros2] bridge enabled: /cmd_vel, /mavros/rc/override -> control, /imu/data, /dvl/velocity, /dvl/odometry, /dvl/altitude, /mujoco/ground_truth/pose"
                + (", /stereo/*" if args.ros2_images else ""),
                flush=True,
            )
        except Exception as exc:
            print(f"[ros2] bridge init failed: {exc}", flush=True)
            ros_bridge = None


    if not (
        args.validate
        or args.validate_thrusters
        or args.calibrate_joystick
        or args.calibrate_thrusters
        or args.calibrate_thresholds
    ):
        input_thread = threading.Thread(target=input_loop, daemon=True)
        input_thread.start()
        joystick_thread = threading.Thread(target=joystick_loop, daemon=True)
        joystick_thread.start()

    # Thruster sets
    yaw_names = ["yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr"]
    # Ordering for horizontal thrusters: 1=rf, 2=lf, 3=rr, 4=lr
    horiz_order = ["yaw_rf", "yaw_lf", "yaw_rr", "yaw_lr"]
    ver_names = ["ver_lf", "ver_lr", "ver_rf", "ver_rr"]

    def normalize(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            return v
        return v / n

    sensor_names = ["imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude"]
    sensor_ids = {}
    for sname in sensor_names:
        sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sname)
        if sid >= 0:
            sensor_ids[sname] = sid

    camera_ids = {}
    for cname in ("stereo_left", "stereo_right"):
        cid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cname)
        if cid >= 0:
            camera_ids[cname] = cid

    sensor_site_ids = {
        "imu": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_site"),
        "dvl": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "dvl_site"),
        "cam_left": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cam_left_site"),
        "cam_right": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cam_right_site"),
    }

    def sensor_value(name: str) -> np.ndarray | None:
        sid = sensor_ids.get(name, -1)
        if sid < 0:
            return None
        adr = int(model.sensor_adr[sid])
        dim = int(model.sensor_dim[sid])
        return data.sensordata[adr : adr + dim].copy()

    def update_stabilization(dt: float) -> None:
        """Estimate roll/pitch error and build stabilization torques."""
        if not imu_stabilize_active["value"]:
            imu_stab_cache["tau_body"][:] = 0.0
            imu_stab_cache["tau_world"][:] = 0.0
            return
        base_rot = data.xmat[base_id].reshape(3, 3)
        body_up = base_rot @ np.array([0.0, 1.0, 0.0], dtype=np.float64)
        world_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        dot_up = float(np.dot(body_up, world_up))
        roll = float(np.arctan2(base_rot[2, 1], base_rot[2, 2]))
        pitch = float(np.arcsin(np.clip(-base_rot[2, 0], -1.0, 1.0)))
        if dot_up < -0.95:
            roll = float(np.pi) * (1.0 if base_rot[0, 1] >= 0.0 else -1.0)
            pitch = 0.0
        err = np.array([roll, pitch], dtype=np.float64)
        imu_stab_int[:] = np.clip(imu_stab_int + err * max(1e-6, dt), -imu_stab_int_max, imu_stab_int_max)
        ang_vel_world = data.cvel[base_id, 0:3]
        ang_vel_body = base_rot.T @ ang_vel_world
        tau_body = np.array(
            [
                -imu_stab_kp * err[0] - imu_stab_kd * ang_vel_body[0] - imu_stab_ki * imu_stab_int[0],
                -imu_stab_kp * err[1] - imu_stab_kd * ang_vel_body[1] - imu_stab_ki * imu_stab_int[1],
                0.0,
            ],
            dtype=np.float64,
        )
        tau_body = np.clip(tau_body, -imu_stab_max, imu_stab_max)
        imu_stab_cache["tau_body"] = tau_body
        imu_stab_cache["tau_world"] = base_rot @ tau_body

    imu_stabilize_active = {"value": bool(args.imu_stabilize)}
    imu_stab_kp = float(args.imu_stab_kp)
    imu_stab_kd = float(args.imu_stab_kd)
    imu_stab_max = float(args.imu_stab_max)
    imu_stab_mode = str(args.imu_stab_mode)
    imu_stab_ki = float(args.imu_stab_ki)
    imu_stab_int_max = float(args.imu_stab_int_max)
    imu_stab_int = np.zeros(2, dtype=np.float64)
    imu_stab_cache = {"tau_body": np.zeros(3, dtype=np.float64), "tau_world": np.zeros(3, dtype=np.float64)}

    def ensure_tune_file() -> None:
        if tune_path.exists():
            return
        gears = {}
        for name in ver_names + yaw_names:
            gears[name] = model.actuator_gear[act[name], :3].tolist()
        payload = {"gears": gears}
        tune_path.write_text(json.dumps(payload, indent=2))

    def load_tuning() -> bool:
        if not tune_path.exists():
            return False
        try:
            payload = json.loads(tune_path.read_text())
        except json.JSONDecodeError:
            return False
        gears = payload.get("gears", {})
        changed = False
        for name, vec in gears.items():
            if name not in act:
                continue
            if not isinstance(vec, list) or len(vec) != 3:
                continue
            v = np.array(vec, dtype=np.float64)
            v = normalize(v)
            if np.linalg.norm(v) < 1e-9:
                continue
            if not np.allclose(model.actuator_gear[act[name], :3], v, atol=1e-6):
                model.actuator_gear[act[name], :3] = v
                changed = True
        return changed

    ensure_tune_file()
    last_tune_mtime = tune_path.stat().st_mtime if tune_path.exists() else 0.0
    load_tuning()
    cob_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cob_site")
    align_com_to_thruster_plane = bool(sim_profile.get("align_com_to_thruster_plane", False))
    cob_y_offset = float(sim_profile.get("cob_y_offset", 0.0))

    def align_com_to_horizontal_thruster_plane() -> None:
        ys = []
        for name in horiz_order:
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            if sid >= 0:
                ys.append(float(model.site_pos[sid][1]))
        if len(ys) < 2:
            return
        target_y = float(np.mean(ys))
        old_y = float(model.body_ipos[base_id][1])
        if abs(target_y - old_y) < 1e-5:
            return
        model.body_ipos[base_id][1] = target_y
        mujoco.mj_setConst(model, data)
        print(f"[model] aligned CoM.y: {old_y:.4f} -> {target_y:.4f}", flush=True)

    def align_cob_to_com_with_offset() -> None:
        if cob_site_id < 0:
            return
        target_y = float(model.body_ipos[base_id][1] + cob_y_offset)
        old_y = float(model.site_pos[cob_site_id][1])
        if abs(target_y - old_y) < 1e-6:
            return
        model.site_pos[cob_site_id][1] = target_y
        print(f"[model] aligned CoB.y: {old_y:.4f} -> {target_y:.4f} (offset={cob_y_offset:+.4f})", flush=True)

    if align_com_to_thruster_plane:
        align_com_to_horizontal_thruster_plane()
    align_cob_to_com_with_offset()

    # Geometry-aware horizontal allocator:
    # maps desired [forward, sway, yaw] in normalized units to 4 horizontal thruster commands.
    horiz_pinv = np.zeros((len(horiz_order), 3), dtype=np.float64)

    def build_horizontal_allocator() -> None:
        nonlocal horiz_pinv
        com_body = model.body_ipos[base_id].copy()
        alloc = np.zeros((3, len(horiz_order)), dtype=np.float64)
        for i, name in enumerate(horiz_order):
            aid = act[name]
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            fdir = normalize(model.actuator_gear[aid, :3].copy())
            r = model.site_pos[sid].copy() - com_body
            tau = np.cross(r, fdir)
            # [Fx, Fz, My] in body frame
            alloc[:, i] = np.array([fdir[0], fdir[2], tau[1]], dtype=np.float64)

        row_scale = np.sum(np.abs(alloc), axis=1)
        row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
        horiz_pinv = np.linalg.pinv(alloc / row_scale[:, None])

    def mix_horizontal_thrusters(fwd_cmd: float, sway_cmd: float, yaw_cmd: float) -> np.ndarray:
        """Map normalized [forward, sway, yaw] wrench into 4 horizontal thrusters."""
        wrench_cmd = np.array([fwd_cmd, sway_cmd, yaw_cmd], dtype=np.float64)
        u = horiz_pinv @ wrench_cmd
        max_abs = float(np.max(np.abs(u)))
        if max_abs > 1.0:
            u /= max_abs
        return np.clip(u, -1.0, 1.0)

    build_horizontal_allocator()

    # Water/fluid basics
    water_surface_z = 0.0
    rho = float(model.opt.density)
    g = abs(float(model.opt.gravity[2]))
    total_mass = float(np.sum(model.body_mass[1:]))
    neutral_volume = total_mass / max(rho, 1e-6)

    all_thruster_names = ver_names + yaw_names
    thr_state = {name: 0.0 for name in all_thruster_names}
    thr_target = {name: 0.0 for name in all_thruster_names}
    thruster_scale = {name: 1.0 for name in all_thruster_names}
    prop_phase = {name: 0.0 for name in all_thruster_names}
    prop_qpos_adr = {}
    prop_dof_adr = {}
    prop_spin_sign = {}
    for name in all_thruster_names:
        jname = f"prop_{name}_j"
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            prop_qpos_adr[name] = int(model.jnt_qposadr[jid])
            prop_dof_adr[name] = int(model.jnt_dofadr[jid])
            # Alternate rotation direction for a more natural visual.
            prop_spin_sign[name] = 1.0 if name.endswith(("lf", "rr")) else -1.0

    def ensure_thruster_params_file() -> None:
        if thruster_params_path.exists():
            return
        payload = {
            "global": {
                "deadzone": 0.05,
                "tau_up": 0.12,
                "tau_down": 0.18,
                "reverse_asymmetry": 0.75,
                "forward_poly": [0.0, 3.5, 7.0, 12.0],
                "reverse_poly": [0.0, 2.8, 5.5, 9.5],
                "command_limit": 0.65,
            },
            "per_thruster": {name: {"gain_scale": 1.0} for name in all_thruster_names},
        }
        thruster_params_path.write_text(json.dumps(payload, indent=2))

    def load_thruster_params() -> bool:
        for name in all_thruster_names:
            thruster_scale[name] = 1.0
        if not thruster_params_path.exists():
            return False
        try:
            payload = json.loads(thruster_params_path.read_text())
        except json.JSONDecodeError:
            return False
        changed = False
        per_thruster = payload.get("per_thruster", {})
        if not isinstance(per_thruster, dict):
            return False
        for name, cfg in per_thruster.items():
            if name not in thruster_scale:
                continue
            if not isinstance(cfg, dict):
                continue
            gain = cfg.get("gain_scale", 1.0)
            if not isinstance(gain, (int, float)):
                continue
            new_gain = float(np.clip(float(gain), 0.4, 1.8))
            if abs(new_gain - thruster_scale[name]) > 1e-8:
                changed = True
            thruster_scale[name] = new_gain
        return changed

    ensure_thruster_params_file()
    load_thruster_params()

    # Simple underwater model (v1.1-like): no current/waves/turbulence,
    # direct thruster commands, buoyancy + linear damping only.
    half_height = float(sim_profile.get("half_height", 0.147))
    buoyancy_scale = float(sim_profile.get("buoyancy_scale", 1.0))
    cob_torque_scale = float(sim_profile.get("cob_torque_scale", 1.0))
    buoyancy_point_blend = float(np.clip(sim_profile.get("buoyancy_point_blend", 1.0), 0.0, 1.0))
    thruster_force_max = float(sim_profile.get("thruster_force_max", 50.0))
    linear_drag = float(sim_profile.get("linear_drag", 1.2))
    angular_drag = float(sim_profile.get("angular_drag", 0.12))
    spin_gain = float(sim_profile.get("spin_gain", 22.0))

    # Vertical allocator for roll/pitch stabilization using vertical thrusters.
    vert_pinv = np.zeros((len(ver_names), 2), dtype=np.float64)

    def build_vertical_allocator() -> None:
        nonlocal vert_pinv
        com_body = model.body_ipos[base_id].copy()
        alloc = np.zeros((2, len(ver_names)), dtype=np.float64)
        for i, name in enumerate(ver_names):
            aid = act[name]
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            if sid < 0:
                continue
            fdir = normalize(model.actuator_gear[aid, :3].copy())
            r = model.site_pos[sid].copy() - com_body
            tau = np.cross(r, fdir * thruster_force_max)
            # Map roll (x) and pitch (z) torque.
            alloc[:, i] = np.array([tau[0], tau[1]], dtype=np.float64)
        if alloc.shape[1] > 0:
            vert_pinv = np.linalg.pinv(alloc)

    build_vertical_allocator()

    def apply_stabilization_thrusters() -> None:
        if not (imu_stabilize_active["value"] and imu_stab_mode in ("thrusters", "both")):
            return
        tau_cmd = np.array([imu_stab_cache["tau_body"][0], imu_stab_cache["tau_body"][1]], dtype=np.float64)
        u = vert_pinv @ tau_cmd if vert_pinv.size else np.zeros(len(ver_names))
        u = np.clip(u, -0.5, 0.5)
        for i, name in enumerate(ver_names):
            thr_target[name] = float(np.clip(thr_target[name] + u[i], -1.0, 1.0))

    # Debug/validation cache
    last_flow_world = np.zeros(3, dtype=np.float64)
    last_buoy_force = np.zeros(3, dtype=np.float64)
    last_buoy_point = np.zeros(3, dtype=np.float64)

    def update_thruster_forces(_dt: float) -> None:
        for name in all_thruster_names:
            thr_state[name] = thr_target[name]
            aid = act[name]
            lo, hi = ctrlrange[aid]
            gain = float(thruster_scale.get(name, 1.0))
            data.ctrl[aid] = float(np.clip(thr_target[name] * thruster_force_max * gain, lo, hi))

    def update_propeller_visuals(dt: float) -> None:
        # Visual-only propeller spin (no reaction torque applied to vehicle).
        for name in all_thruster_names:
            qadr = prop_qpos_adr.get(name)
            dadr = prop_dof_adr.get(name)
            if qadr is None or dadr is None:
                continue
            omega = prop_spin_sign.get(name, 1.0) * float(data.ctrl[act[name]]) * spin_gain
            if dt > 0.0:
                prop_phase[name] += omega * dt
            data.qpos[qadr] = prop_phase[name]
            data.qvel[dadr] = 0.0

    def apply_underwater_wrench(_dt: float) -> None:
        """Apply simplified underwater forces: buoyancy + damping + optional torque."""
        nonlocal last_flow_world, last_buoy_force, last_buoy_point

        data.xfrc_applied[base_id, :] = 0.0
        com = data.xipos[base_id].copy()
        cob = data.site_xpos[cob_site_id].copy() if cob_site_id >= 0 else com
        buoy_point = ((1.0 - buoyancy_point_blend) * com) + (buoyancy_point_blend * cob)

        # Partial-submersion buoyancy based on base COM depth.
        depth = water_surface_z - float(com[2])
        frac = np.clip((depth + half_height) / (2.0 * half_height), 0.0, 1.0)
        buoy = rho * g * neutral_volume * frac * buoyancy_scale
        buoy_force_world = np.array([0.0, 0.0, buoy], dtype=np.float64)

        # MuJoCo xfrc_applied layout: [force_xyz, torque_xyz].
        data.xfrc_applied[base_id, 0:3] += buoy_force_world
        if abs(cob_torque_scale) > 1e-9:
            r = buoy_point - com
            buoy_tau_world = np.cross(r, buoy_force_world) * cob_torque_scale
            data.xfrc_applied[base_id, 3:6] += buoy_tau_world

        # Mild global damping for stable, less exaggerated motion.
        ang_vel = data.cvel[base_id, 0:3]
        lin_vel = data.cvel[base_id, 3:6]
        data.xfrc_applied[base_id, 0:3] -= linear_drag * lin_vel
        data.xfrc_applied[base_id, 3:6] -= angular_drag * ang_vel

        if imu_stabilize_active["value"] and imu_stab_mode in ("torque", "both"):
            data.xfrc_applied[base_id, 3:6] += imu_stab_cache["tau_world"]

        last_flow_world = np.zeros(3, dtype=np.float64)
        last_buoy_force = buoy_force_world
        last_buoy_point = buoy_point

    def viewer_key_callback(keycode):
        # GLFW keycodes for SPACE is 32
        if keycode == 32:
            paused_flag["value"] = not paused_flag["value"]
        if keycode in (77, 109):  # M or m
            viewer_control_mode["value"] = not viewer_control_mode["value"]
        if keycode in (76, 108):  # L or l
            show_thruster_labels["value"] = not show_thruster_labels["value"]
        if keycode in (73, 105):  # I or i
            toggle_sensor_overlay()
        if keycode in (67, 99):  # C or c
            toggle_follow_camera()
        if keycode == 49:  # 1
            camera_mode["value"] = "stereo_left"
            follow_camera["value"] = False
        if keycode == 50:  # 2
            camera_mode["value"] = "stereo_right"
            follow_camera["value"] = False
        if keycode == 48:  # 0
            camera_mode["value"] = "free"
            follow_camera["value"] = False

    def reset_dynamic_states() -> None:
        nonlocal last_flow_world, last_buoy_force, last_buoy_point
        last_flow_world = np.zeros(3, dtype=np.float64)
        last_buoy_force = np.zeros(3, dtype=np.float64)
        last_buoy_point = np.zeros(3, dtype=np.float64)
        for name in all_thruster_names:
            thr_state[name] = 0.0
            thr_target[name] = 0.0

    def run_validation(out_dir: Path) -> dict:
        """Run step-response validation for forward/heave/yaw axes."""
        prev_stab = imu_stabilize_active["value"]
        imu_stabilize_active["value"] = False
        tests = [
            ("forward_step", "forward"),
            ("heave_step", "heave"),
            ("yaw_step", "yaw"),
        ]
        step_start = float(sim_profile.get("validation_step_start", 0.5))
        step_end = float(sim_profile.get("validation_step_end", 1.4))
        total_time = float(sim_profile.get("validation_total_time", 3.2))
        step_amp = float(sim_profile.get("validation_step_amp_ratio", 0.3)) * state["max"]
        summary = {}
        orig_timestep = float(model.opt.timestep)
        orig_iterations = int(model.opt.iterations)
        orig_ls_iterations = int(model.opt.ls_iterations)
        orig_disableflags = int(model.opt.disableflags)
        model.opt.timestep = float(sim_profile.get("validation_timestep", 0.005))
        model.opt.iterations = int(sim_profile.get("validation_iterations", 20))
        model.opt.ls_iterations = int(sim_profile.get("validation_ls_iterations", 8))
        # Validation focuses on free-space dynamics; disable contact for speed.
        model.opt.disableflags = orig_disableflags | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

        try:
            for test_name, axis in tests:
                mujoco.mj_resetData(model, data)
                mujoco.mj_forward(model, data)
                reset_dynamic_states()
                csv_path = out_dir / f"{test_name}.csv"
                rows = []
                unstable = False
                max_steps = int(total_time / model.opt.timestep) + 50

                for _ in range(max_steps):
                    if data.time >= total_time:
                        break
                    t = float(data.time)
                    cmd_f = 0.0
                    cmd_h = 0.0
                    cmd_y = 0.0
                    cmd_s = 0.0
                    if step_start <= t <= step_end:
                        if axis == "forward":
                            cmd_f = step_amp
                        elif axis == "heave":
                            cmd_h = step_amp
                        elif axis == "yaw":
                            cmd_y = step_amp

                    forward = forward_sign * cmd_f
                    heave = heave_sign * cmd_h
                    yaw = yaw_sign * cmd_y
                    sway = sway_sign * cmd_s
                    cmd_scale = max(state["max"], 1e-6)
                    fwd_cmd = np.clip(forward / cmd_scale, -1.0, 1.0)
                    sway_cmd = np.clip(sway / cmd_scale, -1.0, 1.0)
                    yaw_cmd = np.clip(yaw / cmd_scale, -1.0, 1.0)
                    heave_cmd = np.clip(heave / cmd_scale, -1.0, 1.0)

                    horiz_cmd = mix_horizontal_thrusters(fwd_cmd, sway_cmd, yaw_cmd)

                    for name in all_thruster_names:
                        thr_target[name] = 0.0
                    for name in ver_names:
                        thr_target[name] = heave_cmd
                    update_stabilization(model.opt.timestep)
                    for i, name in enumerate(horiz_order):
                        thr_target[name] = float(horiz_cmd[i])
                    apply_stabilization_thrusters()

                    update_thruster_forces(model.opt.timestep)
                    update_propeller_visuals(model.opt.timestep)
                    apply_underwater_wrench(model.opt.timestep)
                    mujoco.mj_step(model, data)
                    t_next = float(data.time)
                    if (not np.isfinite(t_next)) or (t_next <= t + 1e-12):
                        unstable = True
                        break

                    base_rot = data.xmat[base_id].reshape(3, 3)
                    lin_body = base_rot.T @ data.cvel[base_id, 3:6]
                    ang_body = base_rot.T @ data.cvel[base_id, 0:3]
                    if axis == "forward":
                        resp = float(lin_body[0])
                        cmd_sig = float(fwd_cmd)
                    elif axis == "heave":
                        resp = float(lin_body[1])
                        cmd_sig = float(heave_cmd)
                    else:
                        resp = float(ang_body[1])
                        cmd_sig = float(yaw_cmd)

                    rows.append(
                        {
                            "time": t,
                            "cmd": cmd_sig,
                            "resp": resp,
                            "x": float(data.xpos[base_id][0]),
                            "y": float(data.xpos[base_id][1]),
                            "z": float(data.xpos[base_id][2]),
                        }
                    )

                if unstable:
                    print(f"[validation] {test_name}: unstable dynamics detected, truncated early", flush=True)

                with csv_path.open("w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=["time", "cmd", "resp", "x", "y", "z"])
                    writer.writeheader()
                    writer.writerows(rows)

                if not rows:
                    summary[test_name] = {
                        "target_ss": None,
                        "peak": None,
                        "overshoot_pct": None,
                        "settling_time_s": None,
                        "steady_state_drift": None,
                        "status": "unstable",
                        "log_csv": str(csv_path),
                    }
                    continue

                times = np.array([r["time"] for r in rows], dtype=np.float64)
                resp = np.array([r["resp"] for r in rows], dtype=np.float64)
                step_dur = max(1e-6, step_end - step_start)
                ss_start = step_start + 0.6 * step_dur
                ss_end = step_end - 0.05 * step_dur
                step_mask = (times >= ss_start) & (times <= ss_end)
                target_ss = float(np.mean(resp[step_mask])) if np.any(step_mask) else float(np.mean(resp))

                active_mask = (times >= step_start) & (times <= step_end)
                active_resp = resp[active_mask] if np.any(active_mask) else resp
                if target_ss >= 0.0:
                    peak = float(np.max(active_resp))
                    overshoot_val = peak - target_ss
                else:
                    peak = float(np.min(active_resp))
                    overshoot_val = target_ss - peak

                if abs(target_ss) > 1e-6:
                    overshoot_pct = max(0.0, overshoot_val / abs(target_ss) * 100.0)
                    tol = 0.05 * abs(target_ss) + 0.01
                else:
                    overshoot_pct = None
                    tol = 0.02

                settling_time = None
                check_idx = np.where(active_mask)[0]
                if check_idx.size > 0:
                    end_idx = int(check_idx[-1]) + 1
                    for idx in check_idx:
                        if np.all(np.abs(resp[idx:end_idx] - target_ss) <= tol):
                            settling_time = float(times[idx] - step_start)
                            break

                drift_mask = times >= (step_end + 0.25 * step_dur)
                # Drift is residual motion after command release (target is zero).
                steady_state_drift = float(np.mean(np.abs(resp[drift_mask]))) if np.any(drift_mask) else 0.0
                summary[test_name] = {
                    "target_ss": target_ss,
                    "peak": peak,
                    "overshoot_pct": overshoot_pct,
                    "settling_time_s": settling_time,
                    "steady_state_drift": steady_state_drift,
                    "status": "unstable" if unstable else "ok",
                    "log_csv": str(csv_path),
                }
        finally:
            model.opt.timestep = orig_timestep
            model.opt.iterations = orig_iterations
            model.opt.ls_iterations = orig_ls_iterations
            model.opt.disableflags = orig_disableflags
            imu_stabilize_active["value"] = prev_stab

        all_pass = True
        for test_name, metrics in summary.items():
            if test_name.startswith("_"):
                continue
            cfg = validation_thresholds.get(test_name, {})
            reasons = []
            if metrics.get("status") != "ok":
                reasons.append("status != ok")

            target_ss = metrics.get("target_ss")
            min_ss = cfg.get("target_ss_abs_min")
            if isinstance(min_ss, (int, float)):
                if target_ss is None or abs(float(target_ss)) < float(min_ss):
                    reasons.append(f"|target_ss| < {float(min_ss):.4f}")

            overshoot_pct = metrics.get("overshoot_pct")
            max_overshoot = cfg.get("overshoot_pct_max")
            if isinstance(max_overshoot, (int, float)) and overshoot_pct is not None:
                if float(overshoot_pct) > float(max_overshoot):
                    reasons.append(f"overshoot_pct > {float(max_overshoot):.3f}")

            settling_time = metrics.get("settling_time_s")
            max_settle = cfg.get("settling_time_s_max")
            if isinstance(max_settle, (int, float)):
                if settling_time is None or float(settling_time) > float(max_settle):
                    reasons.append(f"settling_time_s > {float(max_settle):.3f}")

            steady_state_drift = metrics.get("steady_state_drift")
            max_drift = cfg.get("steady_state_drift_max")
            if isinstance(max_drift, (int, float)):
                if steady_state_drift is None or float(steady_state_drift) > float(max_drift):
                    reasons.append(f"steady_state_drift > {float(max_drift):.3f}")

            passed = len(reasons) == 0
            metrics["pass"] = passed
            metrics["fail_reasons"] = reasons
            metrics["thresholds"] = cfg
            all_pass = all_pass and passed

        summary["_overall"] = {"pass": all_pass}
        summary_path = out_dir / "summary.json"
        summary_path.write_text(json.dumps(summary, indent=2))
        print(f"[validation] saved: {summary_path}", flush=True)
        return summary

    def run_thruster_validation(out_dir: Path) -> dict:
        """Validate each thruster's direction and minimum effective speed."""
        prev_stab = imu_stabilize_active["value"]
        imu_stabilize_active["value"] = False
        csv_path = out_dir / "thruster_single.csv"
        summary_path = out_dir / "thruster_summary.json"
        cfg = validation_thresholds.get("thruster_single", {})
        min_dot = float(cfg.get("alignment_dot_min", 0.35))
        min_speed = float(cfg.get("min_speed", 0.04))
        min_pass_ratio = float(cfg.get("min_pass_ratio", 1.0))

        orig_timestep = float(model.opt.timestep)
        orig_iterations = int(model.opt.iterations)
        orig_ls_iterations = int(model.opt.ls_iterations)
        orig_disableflags = int(model.opt.disableflags)
        model.opt.timestep = float(sim_profile.get("validation_timestep", 0.005))
        model.opt.iterations = int(sim_profile.get("validation_iterations", 20))
        model.opt.ls_iterations = int(sim_profile.get("validation_ls_iterations", 8))
        model.opt.disableflags = orig_disableflags | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)

        cases = []
        cmd_norm = 0.40
        total_time = 1.2
        sample_start = 0.35
        max_steps = int(total_time / model.opt.timestep) + 50

        try:
            for name in all_thruster_names:
                for sign in (1.0, -1.0):
                    mujoco.mj_resetData(model, data)
                    mujoco.mj_forward(model, data)
                    reset_dynamic_states()
                    vel_samples = []

                    for _ in range(max_steps):
                        if data.time >= total_time:
                            break
                        for thr_name in all_thruster_names:
                            thr_target[thr_name] = 0.0
                        thr_target[name] = float(sign * cmd_norm)
                        update_thruster_forces(model.opt.timestep)
                        update_propeller_visuals(model.opt.timestep)
                        apply_underwater_wrench(model.opt.timestep)
                        mujoco.mj_step(model, data)

                        if float(data.time) >= sample_start:
                            base_rot = data.xmat[base_id].reshape(3, 3)
                            lin_body = base_rot.T @ data.cvel[base_id, 3:6]
                            vel_samples.append(lin_body.copy())

                    if vel_samples:
                        avg_v = np.mean(np.array(vel_samples), axis=0)
                    else:
                        avg_v = np.zeros(3, dtype=np.float64)
                    speed = float(np.linalg.norm(avg_v))
                    expected = sign * normalize(model.actuator_gear[act[name], :3].copy())
                    if speed > 1e-9:
                        alignment_dot = float(np.dot(avg_v / speed, expected))
                    else:
                        alignment_dot = 0.0
                    passed = (alignment_dot >= min_dot) and (speed >= min_speed)
                    cases.append(
                        {
                            "thruster": name,
                            "sign": int(sign),
                            "avg_vx": float(avg_v[0]),
                            "avg_vy": float(avg_v[1]),
                            "avg_vz": float(avg_v[2]),
                            "speed": speed,
                            "expected_x": float(expected[0]),
                            "expected_y": float(expected[1]),
                            "expected_z": float(expected[2]),
                            "alignment_dot": alignment_dot,
                            "pass": passed,
                        }
                    )
        finally:
            model.opt.timestep = orig_timestep
            model.opt.iterations = orig_iterations
            model.opt.ls_iterations = orig_ls_iterations
            model.opt.disableflags = orig_disableflags
            imu_stabilize_active["value"] = prev_stab

        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=[
                    "thruster",
                    "sign",
                    "avg_vx",
                    "avg_vy",
                    "avg_vz",
                    "speed",
                    "expected_x",
                    "expected_y",
                    "expected_z",
                    "alignment_dot",
                    "pass",
                ],
            )
            writer.writeheader()
            writer.writerows(cases)

        pass_count = sum(1 for c in cases if c["pass"])
        total_count = len(cases)
        pass_ratio = float(pass_count) / max(total_count, 1)
        overall_pass = pass_ratio >= min_pass_ratio
        summary = {
            "thresholds": {
                "alignment_dot_min": min_dot,
                "min_speed": min_speed,
                "min_pass_ratio": min_pass_ratio,
            },
            "overall": {
                "pass": overall_pass,
                "pass_ratio": pass_ratio,
                "pass_count": pass_count,
                "total_count": total_count,
            },
            "cases": cases,
            "log_csv": str(csv_path),
        }
        summary_path.write_text(json.dumps(summary, indent=2))
        print(f"[validation] saved: {summary_path}", flush=True)
        return summary

    def calibrate_thruster_gains(out_dir: Path) -> dict:
        """Fit per-thruster gain_scale from measured single-thruster responses."""
        summary = run_thruster_validation(out_dir)
        per_plus = [c for c in summary.get("cases", []) if int(c.get("sign", 0)) == 1]
        speeds = np.array([float(c.get("speed", 0.0)) for c in per_plus], dtype=np.float64)
        speeds = speeds[speeds > 1e-6]
        median_speed = float(np.median(speeds)) if speeds.size > 0 else 0.0
        if median_speed <= 1e-6:
            return {"updated": False, "reason": "no valid speed samples"}

        try:
            payload = json.loads(thruster_params_path.read_text()) if thruster_params_path.exists() else {}
        except json.JSONDecodeError:
            payload = {}
        if not isinstance(payload, dict):
            payload = {}
        payload.setdefault("global", {})
        payload.setdefault("per_thruster", {})
        if not isinstance(payload["global"], dict):
            payload["global"] = {}
        if not isinstance(payload["per_thruster"], dict):
            payload["per_thruster"] = {}

        updated = {}
        for c in per_plus:
            name = str(c.get("thruster"))
            if name not in thruster_scale:
                continue
            speed = max(1e-6, float(c.get("speed", 0.0)))
            gain_scale = float(np.clip(median_speed / speed, 0.70, 1.30))
            cfg = payload["per_thruster"].get(name, {})
            if not isinstance(cfg, dict):
                cfg = {}
            cfg["gain_scale"] = round(gain_scale, 4)
            cfg["speed_sample"] = round(speed, 4)
            payload["per_thruster"][name] = cfg
            updated[name] = cfg["gain_scale"]

        payload["global"]["last_gain_calibration_profile"] = args.profile
        payload["global"]["last_gain_calibration_scene"] = args.scene
        payload["global"]["last_gain_calibration_time"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        thruster_params_path.write_text(json.dumps(payload, indent=2))
        load_thruster_params()

        report = {
            "updated": True,
            "median_speed": median_speed,
            "gains": updated,
            "path": str(thruster_params_path),
        }
        report_path = out_dir / "thruster_calibration_report.json"
        report_path.write_text(json.dumps(report, indent=2))
        print(f"[cal] saved: {report_path}", flush=True)
        return report

    def calibrate_validation_thresholds(out_dir: Path) -> dict:
        """Rebuild pass/fail thresholds from current measured validation responses."""
        step_summary = run_validation(out_dir)
        thr_summary = run_thruster_validation(out_dir)
        calibrated = {}

        for name in ("forward_step", "heave_step", "yaw_step"):
            m = step_summary.get(name, {})
            if not isinstance(m, dict):
                continue
            target_ss = abs(float(m.get("target_ss", 0.0)))
            overshoot = float(m.get("overshoot_pct", 0.0) or 0.0)
            settle = float(m.get("settling_time_s", 0.0) or 0.0)
            drift = float(m.get("steady_state_drift", 0.0) or 0.0)
            calibrated[name] = {
                "overshoot_pct_max": round(max(5.0, overshoot * 1.8 + 2.0), 4),
                "settling_time_s_max": round(max(0.2, settle * 1.8 + 0.15), 4),
                "steady_state_drift_max": round(max(0.02, drift * 1.6 + 0.03), 4),
                "target_ss_abs_min": round(max(0.01, target_ss * 0.45), 4),
            }

        cases = thr_summary.get("cases", [])
        dots = np.array([float(c.get("alignment_dot", 0.0)) for c in cases], dtype=np.float64)
        speeds = np.array([float(c.get("speed", 0.0)) for c in cases], dtype=np.float64)
        dots = dots[np.isfinite(dots)]
        speeds = speeds[np.isfinite(speeds)]
        min_dot = float(np.min(dots)) if dots.size else 0.35
        min_speed = float(np.min(speeds)) if speeds.size else 0.04
        calibrated["thruster_single"] = {
            "alignment_dot_min": round(max(0.20, min_dot * 0.90), 4),
            "min_speed": round(max(0.01, min_speed * 0.80), 4),
            "min_pass_ratio": 1.0,
        }

        validation_threshold_path.write_text(json.dumps(calibrated, indent=2))
        validation_thresholds.clear()
        validation_thresholds.update(calibrated)
        report = {
            "updated": True,
            "path": str(validation_threshold_path),
            "thresholds": calibrated,
        }
        report_path = out_dir / "threshold_calibration_report.json"
        report_path.write_text(json.dumps(report, indent=2))
        print(f"[cal] saved: {report_path}", flush=True)
        return report

    def run_all_validations() -> bool:
        """Execute requested validation passes and write aggregate report JSON."""
        out_dir = Path(args.validation_dir)
        out_dir.mkdir(parents=True, exist_ok=True)

        step_summary = run_validation(out_dir) if args.validate else None
        thruster_summary = run_thruster_validation(out_dir) if args.validate_thrusters else None

        overall_pass = True
        if step_summary is not None:
            overall_pass = overall_pass and bool(step_summary.get("_overall", {}).get("pass", False))
        if thruster_summary is not None:
            overall_pass = overall_pass and bool(thruster_summary.get("overall", {}).get("pass", False))

        report = {
            "profile": args.profile,
            "scene": args.scene,
            "step_validation": step_summary,
            "thruster_validation": thruster_summary,
            "overall_pass": overall_pass,
        }
        report_path = out_dir / "validation_report.json"
        report_path.write_text(json.dumps(report, indent=2))
        print(f"[validation] saved: {report_path}", flush=True)
        return overall_pass

    def run_calibrations() -> bool:
        """Execute selected calibration tasks and persist calibration report."""
        out_dir = Path(args.calibration_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        ok = True
        report = {
            "profile": args.profile,
            "scene": args.scene,
            "joystick": None,
            "thrusters": None,
            "thresholds": None,
        }

        if args.calibrate_joystick:
            ok_joy = calibrate_joystick_mapping()
            report["joystick"] = {"success": bool(ok_joy), "path": str(joy_map_path)}
            ok = ok and bool(ok_joy)

        if args.calibrate_thrusters:
            thr_report = calibrate_thruster_gains(out_dir)
            report["thrusters"] = thr_report
            ok = ok and bool(thr_report.get("updated", False))

        if args.calibrate_thresholds:
            th_report = calibrate_validation_thresholds(out_dir)
            report["thresholds"] = th_report
            ok = ok and bool(th_report.get("updated", False))

        report["success"] = ok
        report_path = out_dir / "calibration_report.json"
        report_path.write_text(json.dumps(report, indent=2))
        print(f"[cal] saved: {report_path}", flush=True)
        return ok

    if args.validate or args.validate_thrusters:
        passed = run_all_validations()
        if ros_bridge is not None:
            ros_bridge.shutdown()
        stop_event.set()
        if args.strict_validation and not passed:
            raise SystemExit(2)
        return

    if args.calibrate_joystick or args.calibrate_thrusters or args.calibrate_thresholds:
        success = run_calibrations()
        if ros_bridge is not None:
            ros_bridge.shutdown()
        stop_event.set()
        if not success:
            raise SystemExit(2)
        return

    with mujoco.viewer.launch_passive(model, data, key_callback=viewer_key_callback) as viewer:
        has_set_texts = hasattr(viewer, "set_texts")
        while viewer.is_running() and not stop_event.is_set():
            if ros_bridge is not None:
                try:
                    ros_bridge.spin_once()
                except Exception as exc:
                    print(f"[ros2] spin_once failed, disabling bridge: {exc}", flush=True)
                    ros_bridge.shutdown()
                    ros_bridge = None

            # Respect GUI pause in passive viewer
            paused = False
            if hasattr(viewer, "is_paused"):
                flag = viewer.is_paused
                paused = flag() if callable(flag) else bool(flag)
            is_paused = paused_flag["value"] or paused

            if not viewer_control_mode["value"]:
                # Manual commands
                with cmd_lock:
                    forward = forward_sign * cmd["forward"]
                    heave = heave_sign * cmd["heave"]
                    yaw = yaw_sign * cmd["yaw"]
                    sway = sway_sign * cmd["sway"]

                # Reload tuning if file changed
                if tune_path.exists():
                    mtime = tune_path.stat().st_mtime
                    if mtime > last_tune_mtime:
                        if load_tuning():
                            build_horizontal_allocator()
                        last_tune_mtime = mtime

                # Terminal commands are normalized to [-1, 1] before thruster dynamics.
                cmd_scale = max(state["max"], 1e-6)
                fwd_cmd = np.clip(forward / cmd_scale, -1.0, 1.0)
                sway_cmd = np.clip(sway / cmd_scale, -1.0, 1.0)
                yaw_cmd = np.clip(yaw / cmd_scale, -1.0, 1.0)
                heave_cmd = np.clip(heave / cmd_scale, -1.0, 1.0)

                horiz_cmd = mix_horizontal_thrusters(fwd_cmd, sway_cmd, yaw_cmd)

                for name in all_thruster_names:
                    thr_target[name] = 0.0
                for name in ver_names:
                    thr_target[name] = heave_cmd
                for i, name in enumerate(horiz_order):
                    thr_target[name] = float(horiz_cmd[i])
                apply_stabilization_thrusters()

                update_thruster_forces(model.opt.timestep)
                update_propeller_visuals(model.opt.timestep if not is_paused else 0.0)
            else:
                forward = 0.0
                yaw = 0.0
                heave = 0.0
                sway = 0.0
                if imu_stabilize_active["value"]:
                    update_stabilization(model.opt.timestep)
                if imu_stabilize_active["value"] and imu_stab_mode in ("thrusters", "both"):
                    for name in all_thruster_names:
                        thr_target[name] = 0.0
                    apply_stabilization_thrusters()
                    update_thruster_forces(model.opt.timestep)
                update_propeller_visuals(model.opt.timestep if not is_paused else 0.0)

            apply_underwater_wrench(model.opt.timestep if not is_paused else 0.0)

            # Debug arrows for thruster directions (world frame)
            with viewer.lock():
                user_scn = viewer.user_scn
                user_scn.ngeom = 0

                base_rot = data.xmat[base_id].reshape(3, 3)
                com = data.xipos[base_id].copy()

                # Camera control: fixed stereo, follow, or free.
                if camera_mode["value"] in camera_ids:
                    cam = viewer.cam
                    cam.type = int(mujoco.mjtCamera.mjCAMERA_FIXED)
                    cam.fixedcamid = int(camera_ids[camera_mode["value"]])
                    cam.trackbodyid = -1
                elif follow_camera["value"]:
                    cam = viewer.cam
                    cam.type = int(mujoco.mjtCamera.mjCAMERA_TRACKING)
                    cam.trackbodyid = int(base_id)
                    if not follow_camera_init["value"]:
                        cam.distance = 2.2
                        cam.elevation = -20.0
                        cam.azimuth = 135.0
                        follow_camera_init["value"] = True
                else:
                    if int(viewer.cam.type) in (
                        int(mujoco.mjtCamera.mjCAMERA_TRACKING),
                        int(mujoco.mjtCamera.mjCAMERA_FIXED),
                    ):
                        viewer.cam.type = int(mujoco.mjtCamera.mjCAMERA_FREE)
                        viewer.cam.fixedcamid = -1
                        viewer.cam.trackbodyid = -1

                def add_arrow(start, direction, magnitude, rgba, thickness=0.02):
                    if user_scn.ngeom >= user_scn.maxgeom:
                        return
                    length = 0.08 + 0.01 * abs(magnitude)
                    end = start + direction * length
                    geom = user_scn.geoms[user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_ARROW,
                        np.zeros(3),
                        np.zeros(3),
                        np.eye(3).flatten(),
                        np.array(rgba, dtype=np.float32),
                    )
                    mujoco.mjv_connector(
                        geom,
                        mujoco.mjtGeom.mjGEOM_ARROW,
                        thickness,
                        start,
                        end,
                    )
                    user_scn.ngeom += 1

                def add_sphere(position, radius, rgba):
                    if user_scn.ngeom >= user_scn.maxgeom:
                        return
                    geom = user_scn.geoms[user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_SPHERE,
                        np.array([radius, 0.0, 0.0]),
                        position,
                        np.eye(3).flatten(),
                        np.array(rgba, dtype=np.float32),
                    )
                    user_scn.ngeom += 1

                def add_bubble_stream(start, exhaust_dir, thrust_mag):
                    if thrust_mag < 2.0:
                        return
                    d = normalize(exhaust_dir)
                    up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
                    if abs(float(np.dot(d, up))) > 0.9:
                        up = np.array([1.0, 0.0, 0.0], dtype=np.float64)
                    s1 = normalize(np.cross(d, up))
                    s2 = normalize(np.cross(d, s1))
                    strength = min(1.0, thrust_mag / max(thruster_force_max, 1e-6))
                    count = 3 + int(3 * strength)
                    for k in range(count):
                        phase = (float(data.time) * 2.4 + k * 0.31) % 1.0
                        dist = 0.04 + phase * (0.22 + 0.08 * strength)
                        swirl = 0.008 * (1.0 - phase) * (0.6 + 0.4 * strength)
                        wobble = np.sin(2.0 * np.pi * (phase + 0.17 * k))
                        wobble2 = np.cos(2.0 * np.pi * (phase + 0.11 * k))
                        pos = start + d * dist + s1 * (swirl * wobble) + s2 * (swirl * wobble2)
                        radius = 0.004 + 0.003 * strength * (1.0 - 0.5 * phase)
                        alpha = 0.35 * (1.0 - phase)
                        add_sphere(pos, radius, (0.82, 0.93, 1.0, alpha))

                def add_label(text, position, rgba):
                    if user_scn.ngeom >= user_scn.maxgeom:
                        return
                    geom = user_scn.geoms[user_scn.ngeom]
                    mujoco.mjv_initGeom(
                        geom,
                        mujoco.mjtGeom.mjGEOM_LABEL,
                        np.zeros(3),
                        position,
                        np.eye(3).flatten(),
                        np.array(rgba, dtype=np.float32),
                    )
                    geom.size[:] = np.array([0.03, 0.03, 0.03])
                    geom.label = text
                    user_scn.ngeom += 1

                # Thruster force vectors (blue): direction includes the force sign.
                for name in ver_names + yaw_names:
                    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
                    start = data.site_xpos[sid].copy()
                    force = float(data.ctrl[act[name]])
                    fdir = model.actuator_gear[act[name], :3]
                    fdir = fdir / (np.linalg.norm(fdir) + 1e-9)
                    world_dir = base_rot @ fdir
                    draw_dir = world_dir if force >= 0.0 else -world_dir
                    add_arrow(start, draw_dir, abs(force), (0.2, 0.6, 1.0, 1.0))
                    add_bubble_stream(start, -draw_dir, abs(force))
                    if show_thruster_labels["value"]:
                        add_label(
                            f"{name}:{force:+.1f}",
                            start + np.array([0.0, 0.03, 0.0]),
                            (0.8, 0.9, 1.0, 1.0),
                        )

                # Sensor and camera markers
                if show_sensor_overlay["value"]:
                    imu_id = sensor_site_ids.get("imu", -1)
                    dvl_id = sensor_site_ids.get("dvl", -1)
                    cam_l_id = sensor_site_ids.get("cam_left", -1)
                    cam_r_id = sensor_site_ids.get("cam_right", -1)
                    if imu_id >= 0:
                        pos = data.site_xpos[imu_id].copy()
                        add_sphere(pos, 0.025, (1.0, 1.0, 0.1, 1.0))
                        add_label("IMU", pos + np.array([0.0, 0.03, 0.0]), (1.0, 1.0, 0.3, 1.0))
                    if dvl_id >= 0:
                        pos = data.site_xpos[dvl_id].copy()
                        add_sphere(pos, 0.025, (0.1, 1.0, 1.0, 1.0))
                        add_label("DVL", pos + np.array([0.0, 0.03, 0.0]), (0.3, 1.0, 1.0, 1.0))
                    if cam_l_id >= 0:
                        pos = data.site_xpos[cam_l_id].copy()
                        add_sphere(pos, 0.022, (1.0, 0.1, 1.0, 1.0))
                        add_label("CAM_L", pos + np.array([0.0, 0.03, 0.0]), (1.0, 0.3, 1.0, 1.0))
                    if cam_r_id >= 0:
                        pos = data.site_xpos[cam_r_id].copy()
                        add_sphere(pos, 0.022, (1.0, 0.5, 0.1, 1.0))
                        add_label("CAM_R", pos + np.array([0.0, 0.03, 0.0]), (1.0, 0.6, 0.2, 1.0))

                # Net thrust (red) and buoyancy (green) at CoB
                net_force = np.zeros(3)
                for name in ver_names + yaw_names:
                    fdir = model.actuator_gear[act[name], :3]
                    fdir = fdir / (np.linalg.norm(fdir) + 1e-9)
                    net_force += (base_rot @ fdir) * data.ctrl[act[name]]
                net_mag = float(np.linalg.norm(net_force))
                if net_mag > 1e-6:
                    add_arrow(com, net_force / net_mag, net_mag, (0.95, 0.95, 0.95, 1.0))
                    add_label("NET", com + np.array([0.0, 0.05, 0.0]), (0.95, 0.95, 0.95, 1.0))

                buoy_mag = float(np.linalg.norm(last_buoy_force))
                if buoy_mag > 1e-6:
                    add_arrow(last_buoy_point, np.array([0.0, 0.0, 1.0]), buoy_mag * 0.05, (0.2, 1.0, 0.2, 1.0))
                    add_label("BUOY", last_buoy_point + np.array([0.0, 0.08, 0.0]), (0.6, 1.0, 0.6, 1.0))

            # On-screen help (if available)
            if has_set_texts:
                overlay_line = "Space: pause, M: mode, C: follow, 1/2: stereo cam, 0: free, I: sensors"
                if show_sensor_overlay["value"]:
                    imu_g = sensor_value("imu_gyro")
                    imu_a = sensor_value("imu_acc")
                    dvl_v = sensor_value("dvl_vel_body")
                    dvl_alt = sensor_value("dvl_altitude")
                    if imu_g is not None and imu_a is not None and dvl_v is not None and dvl_alt is not None:
                        overlay_line = (
                            f"IMU gyro[{imu_g[0]:+.2f},{imu_g[1]:+.2f},{imu_g[2]:+.2f}] "
                            f"acc[{imu_a[0]:+.2f},{imu_a[1]:+.2f},{imu_a[2]:+.2f}] "
                            f"DVL vel[{dvl_v[0]:+.2f},{dvl_v[1]:+.2f},{dvl_v[2]:+.2f}] alt={dvl_alt[0]:.2f}m"
                        )
                viewer.set_texts([
                    (
                        None,
                        None,
                        f"Mode: {'viewer' if viewer_control_mode['value'] else 'terminal'} | fwd {forward:+.1f} sway {sway:+.1f} yaw {yaw:+.1f} heave {heave:+.1f} | Cam: {camera_mode['value']}",
                        overlay_line,
                    )
                ])

            if not is_paused:
                mujoco.mj_step(model, data)
            if ros_bridge is not None:
                try:
                    ros_bridge.publish(data)
                except Exception as exc:
                    print(f"[ros2] publish failed, disabling bridge: {exc}", flush=True)
                    ros_bridge.shutdown()
                    ros_bridge = None
            viewer.sync()
            time.sleep(model.opt.timestep)

    if ros_bridge is not None:
        ros_bridge.shutdown()
    stop_event.set()


if __name__ == "__main__":
    main()
