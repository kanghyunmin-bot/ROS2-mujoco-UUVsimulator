"""Main runtime for the UUV MuJoCo simulator.

The script keeps simulation, controls, calibration, validation, and optional
ROS2 publishing in a single entrypoint so that model tuning is reproducible.
"""

import argparse
import fnmatch
import json
import math
import os
import platform
import threading
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from bridge.qgc_video_stream import QgcVideoStreamer
from physics.hydrodynamics_helpers import (
    added_mass_coriolis,
    first_order_response,
    scaled_polynomial_force,
    shape_thruster_command,
    submerged_fraction,
)
from physics.sim_profile_helpers import (
    build_hydrodynamics_config,
    build_sim_profile,
    canonical_profile_name,
    load_sim_profiles,
)
from physics.thruster_mapping import (
    ARDUSUB_VECTORED_6DOF_SERVO_MAP,
    ARDUSUB_VECTORED_6DOF_SERVO_SIGNS,
    ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER,
    PHYSICAL_VERTICAL_THRUSTERS,
    PHYSICAL_YAW_THRUSTERS,
)

BASE_DIR = Path(__file__).resolve().parent
SCENES_DIR = BASE_DIR / "scenes"
CONFIG_DIR = BASE_DIR / "config"

MODEL_PATH = SCENES_DIR / "tank_current_scene.xml"
PROFILE_PATH = CONFIG_DIR / "sim_profiles.json"
THRUSTER_PERF_PATH = CONFIG_DIR / "thruster_performance.json"


def _env_float(name: str, default: float) -> float:
    try:
        return float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return float(default)


def main() -> None:
    """Parse CLI options, initialize runtime state, and execute selected mode."""
    parser = argparse.ArgumentParser(description="UUV MuJoCo runner")
    parser.add_argument(
        "--scene",
        type=str,
        default=str(SCENES_DIR / "tank_current_scene.xml"),
        help="MJCF scene path (default: scenes/tank_current_scene.xml)",
    )
    parser.add_argument(
        "--profile",
        type=str,
        default="current",
        help="Simulation profile name (see --list-profiles)",
    )
    parser.add_argument(
        "--profile-file",
        type=str,
        default=str(PROFILE_PATH),
        help="Simulation profile JSON path",
    )
    parser.add_argument(
        "--thruster-perf-file",
        type=str,
        default=str(THRUSTER_PERF_PATH),
        help="PWM-thrust performance curve JSON file",
    )
    parser.add_argument(
        "--thruster-voltage",
        type=float,
        default=None,
        help="Select nearest thrust curve voltage from performance file (ex. 10,12,14,16,18,20). If omitted, use profile value.",
    )
    parser.add_argument(
        "--buoyancy-scale",
        type=float,
        default=None,
        help="Override profile buoyancy scale at runtime (ex. 0.98 for weaker buoyancy).",
    )
    parser.add_argument(
        "--disable-thruster-perf",
        action="store_true",
        help="Force linear thruster mapping and ignore performance curve JSON",
    )
    parser.add_argument(
        "--thruster-perf-direct",
        action="store_true",
        help=(
            "When the performance curve is active, map raw normalized PWM directly "
            "to force and bypass legacy command shaping/gain scaling."
        ),
    )
    parser.add_argument(
        "--fluid-model",
        type=str,
        default="current",
        help=(
            "Hydrodynamic model selection: "
            "legacy/custom=previous Python 6-DOF baseline, "
            "current/ellipsoid=MuJoCo built-in ellipsoid current model"
        ),
    )
    parser.add_argument(
        "--list-profiles",
        action="store_true",
        help="Print available simulation profiles and exit",
    )
    parser.add_argument(
        "--ros2",
        action="store_true",
        help="Enable ROS2 transport topics (/cmd_vel as TwistStamped, /imu/data, /dvl/*)",
    )
    parser.add_argument(
        "--ros2-real-pkg-compat",
        action="store_true",
        help="Keep ROS2 sensor topics but reduce the simulator MAVROS surface to compat-only (/mavros/vfr_hud) for external MAVROS + kmu26_auv.",
    )
    parser.add_argument(
        "--ros2-images",
        action="store_true",
        help="Legacy no-op. Stereo ROS2 image publishing is disabled in the lightweight real-robot bridge.",
    )
    parser.add_argument(
        "--no-ping360",
        action="store_true",
        help="Disable the synthetic Ping360 sonar ROS2 topics.",
    )
    parser.add_argument(
        "--ping360-config",
        type=str,
        default=str(CONFIG_DIR / "ping360.json"),
        help="Ping360 JSON configuration path.",
    )
    parser.add_argument(
        "--ping360-range-m",
        type=float,
        default=None,
        help="Override requested Ping360 range in meters; firmware model clamps to 0.75-50.0 m.",
    )
    parser.add_argument(
        "--ping360-num-steps",
        type=int,
        default=None,
        help="Override Ping360 motor steps per ping, 1-10 gradians.",
    )
    parser.add_argument(
        "--ping360-interface",
        choices=("usb", "ethernet", "rs485"),
        default=None,
        help="Override Ping360 communications interface for scan-time estimation.",
    )
    parser.add_argument(
        "--ping360-gain",
        type=int,
        default=None,
        help="Override Ping360 gain setting, 0=low, 1=normal, 2=high.",
    )
    parser.add_argument(
        "--qgc-video",
        action="store_true",
        help="Stream stereo_left camera directly to QGroundControl over UDP H264",
    )
    parser.add_argument(
        "--qgc-video-host",
        type=str,
        default="127.0.0.1",
        help="QGroundControl video UDP target host",
    )
    parser.add_argument(
        "--qgc-video-port",
        type=int,
        default=5600,
        help="QGroundControl video UDP target port",
    )
    parser.add_argument(
        "--qgc-video-fps",
        type=float,
        default=15.0,
        help="QGC video output FPS",
    )
    parser.add_argument(
        "--qgc-video-width",
        type=int,
        default=640,
        help="QGC video output width",
    )
    parser.add_argument(
        "--qgc-video-height",
        type=int,
        default=360,
        help="QGC video output height",
    )
    parser.add_argument(
        "--qgc-video-bitrate-kbps",
        type=int,
        default=2600,
        help="QGC video H264 bitrate in kbps",
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
        default=_env_float("UUV_ROS2_SENSOR_HZ", 60.0),
        help="ROS2 IMU/DVL publish rate (Hz)",
    )
    parser.add_argument(
        "--ros2-image-hz",
        type=float,
        default=10.0,
        help="ROS2 stereo image publish rate (Hz)",
    )
    parser.add_argument(
        "--ros2-camera-calib-left",
        type=str,
        default="",
        help="Path to left camera calibration YAML (camera_info format)",
    )
    parser.add_argument(
        "--ros2-camera-calib-right",
        type=str,
        default="",
        help="Path to right camera calibration YAML (camera_info format)",
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
        help="ArduPilot SITL JSON servo recv/listen Port (ArduPilot sends servo packets here)",
    )
    parser.add_argument(
        "--sitl-send-port",
        type=int,
        default=9003,
        help="ArduPilot SITL JSON sensor send Port (ArduPilot expects JSON sensor packets here)",
    )
    parser.add_argument(
        "--sitl-mavlink-endpoint",
        type=str,
        default="udpin:0.0.0.0:14660",
        help="MAVLink endpoint to receive SERVO_OUTPUT_RAW for SITL thruster commands. Use 'none' to disable MAVLink servo input and rely on JSON UDP control only.",
    )
    parser.add_argument(
        "--sitl-mavlink-servo-hz",
        type=float,
        default=_env_float("UUV_SITL_MAVLINK_SERVO_HZ", 25.0),
        help="Requested SERVO_OUTPUT_RAW rate over MAVLink (Hz).",
    )
    parser.add_argument(
        "--sitl-mavlink-target-sysid",
        type=int,
        default=0,
        help="Target vehicle sysid expected for SERVO_OUTPUT_RAW (0=auto from heartbeat).",
    )
    parser.add_argument(
        "--sitl-mavlink-target-compid",
        type=int,
        default=0,
        help="Target vehicle compid expected for SERVO_OUTPUT_RAW (0=auto from heartbeat).",
    )
    parser.add_argument(
        "--sitl-mavlink-source-sysid",
        type=int,
        default=255,
        help="Source sysid for MuJoCo MAVLink control messages. ArduSub accepts pilot input only from SYSID_MYGCS.",
    )
    parser.add_argument(
        "--sitl-mavlink-source-compid",
        type=int,
        default=190,
        help="Source component id for MuJoCo MAVLink listener.",
    )
    parser.add_argument(
        "--sitl-servo-scale",
        type=float,
        # Legacy polynomial/gain tuned mode used default=0.58.
        default=1.0,
        help=(
            "Scale applied to direct-thruster normalized command from SITL PWM "
            "(default: 1.0; legacy polynomial/gain mode used 0.58)."
        ),
    )
    parser.add_argument(
        "--thruster-loop-hz",
        type=float,
        default=_env_float("UUV_THRUSTER_LOOP_HZ", 80.0),
        help="Thruster force update rate (Hz), decoupled from physics timestep.",
    )
    parser.add_argument(
        "--initial-depth-m",
        type=float,
        default=None,
        help=(
            "Set initial base_link depth relative to the water surface before runtime starts. "
            "Positive is underwater; negative starts above the water and lets gravity drop the vehicle."
        ),
    )
    parser.add_argument(
        "--initial-rpy-rad",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=None,
        help=(
            "Set the initial base_link attitude as ROS/ENU roll, pitch, yaw in radians. "
            "Useful for rosbag replays that start mid-run instead of from a level vehicle."
        ),
    )
    parser.add_argument(
        "--initial-depth-hold-target-m",
        type=float,
        default=None,
        help=(
            "Deprecated compatibility value. Runtime depth target switching is disabled; "
            "use --initial-depth-m to set the starting pose before sensors are published."
        ),
    )
    parser.add_argument(
        "--hold-initial-depth-until-release",
        action="store_true",
        help="Pin the vehicle at the initial/target depth until /mujoco/release_initial_depth_hold is called.",
    )
    parser.add_argument(
        "--release-linear-velocity-body",
        type=float,
        nargs=3,
        metavar=("VX", "VY", "VZ"),
        default=None,
        help=(
            "Body-frame linear velocity [m/s] applied when the artificial "
            "initial-depth hold is released. Use this for rosbag replays that "
            "start while the real vehicle is already moving."
        ),
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Run real-time simulation loop without GLFW viewer",
    )
    parser.add_argument(
        "--viewer-fps",
        type=float,
        default=_env_float("UUV_MUJOCO_VIEWER_FPS", 60.0),
        help="Maximum passive MuJoCo viewer refresh rate in Hz.",
    )
    parser.add_argument(
        "--viewer-debug",
        action="store_true",
        help="Draw heavy viewer debug overlays such as thruster arrows, bubbles, labels, and sensor markers",
    )
    parser.add_argument(
        "--enable-viewer-pause",
        action="store_true",
        help="Allow the MuJoCo viewer spacebar/pause state to stop the simulation loop",
    )
    parser.add_argument(
        "--viewer-camera-mode",
        type=str,
        default=os.environ.get("UUV_MUJOCO_VIEWER_CAMERA_MODE", ""),
        choices=("", "free", "follow", "stereo_left", "stereo_right"),
        help=(
            "Initial MuJoCo viewer camera. Empty defaults to follow in SITL and free otherwise. "
            "Use stereo_left/stereo_right for fixed onboard robot cameras."
        ),
    )
    args = parser.parse_args()

    fluid_model_aliases = {
        "legacy": "legacy",
        "custom": "legacy",
        "current": "current",
        "ellipsoid": "current",
        "builtin-ellipsoid": "current",
    }
    fluid_model = fluid_model_aliases.get(str(args.fluid_model).strip().lower())
    if fluid_model is None:
        print(f"[physics] unknown fluid model: {args.fluid_model}", flush=True)
        print("Available fluid models:", flush=True)
        for name in ("legacy", "current"):
            print(f"  - {name}", flush=True)
        raise SystemExit(2)
    args.fluid_model = fluid_model

    profile_path = Path(args.profile_file).expanduser()
    profiles, profile_warning = load_sim_profiles(profile_path)
    if profile_warning is not None:
        print(profile_warning, flush=True)

    if args.list_profiles:
        print("Available profiles:", flush=True)
        for name in sorted(profiles):
            print(f"  - {name}", flush=True)
        print("Aliases: custom->legacy, ellipsoid->current", flush=True)
        return

    resolved_profile_name = canonical_profile_name(args.profile)
    if resolved_profile_name not in profiles:
        print(f"[profile] unknown profile: {args.profile}", flush=True)
        print("Available profiles:", flush=True)
        for name in sorted(profiles):
            print(f"  - {name}", flush=True)
        raise SystemExit(2)

    sim_profile = build_sim_profile(
        profiles,
        resolved_profile_name,
        buoyancy_scale_override=args.buoyancy_scale,
    )
    if resolved_profile_name != args.profile:
        print(f"[profile] alias '{args.profile}' -> '{resolved_profile_name}'", flush=True)
    print(f"[profile] using '{resolved_profile_name}' from {profile_path}", flush=True)
    if args.buoyancy_scale is not None:
        print(
            f"[profile] override buoyancy_scale={sim_profile['buoyancy_scale']:.3f}",
            flush=True,
        )
    profile_thruster_voltage = float(sim_profile.get("thruster_voltage", 20.0))
    if args.thruster_voltage is None:
        active_thruster_voltage = profile_thruster_voltage
        print(
            f"[profile] using thruster_voltage={active_thruster_voltage:.1f}V from profile",
            flush=True,
        )
    else:
        active_thruster_voltage = float(args.thruster_voltage)
        print(
            f"[profile] override thruster_voltage={active_thruster_voltage:.1f}V (profile {profile_thruster_voltage:.1f}V)",
            flush=True,
        )

    # Optional thruster PWM->force profile.
    perf_cfg = {
        "active": False,
        "direct": bool(args.thruster_perf_direct),
        "requested_voltage": float(active_thruster_voltage),
        "selected_voltage": None,
        "pwm": np.array([], dtype=np.float64),
        "force": np.array([], dtype=np.float64),
    }

    def _to_float_array(values) -> np.ndarray | None:
        if not isinstance(values, list) or not values:
            return None
        out = []
        for value in values:
            try:
                out.append(float(value))
            except (TypeError, ValueError):
                return None
        return np.array(out, dtype=np.float64)

    def _normalize_thruster_perf_voltage(value: float | str | None) -> float:
        try:
            return float(value)
        except (TypeError, ValueError):
            return float(active_thruster_voltage)

    def _load_thruster_performance(path: Path) -> None:
        perf_path = path.expanduser()
        if not perf_path.exists():
            print(f"[thruster perf] file not found: {perf_path}", flush=True)
            return
        try:
            payload = json.loads(perf_path.read_text())
        except (OSError, json.JSONDecodeError):
            print(f"[thruster perf] invalid json: {perf_path}", flush=True)
            return

        curves_raw = payload.get("curves") if isinstance(payload, dict) else None
        if not isinstance(curves_raw, list):
            print(f"[thruster perf] missing curves in: {perf_path}", flush=True)
            return

        candidates = []
        for curve in curves_raw:
            if not isinstance(curve, dict):
                continue
            voltage = curve.get("voltage_v")
            pwm = _to_float_array(curve.get("pwm_us"))
            force = _to_float_array(curve.get("force_n"))
            if voltage is None or pwm is None or force is None:
                continue
            if pwm.size != force.size:
                continue
            if pwm.size < 2:
                continue
            order = np.argsort(pwm)
            pwm = pwm[order]
            force = force[order]
            valid = np.isfinite(pwm) & np.isfinite(force)
            if not np.any(valid):
                continue
            candidates.append(
                {
                    "voltage": float(voltage),
                    "pwm": pwm[valid],
                    "force": force[valid],
                }
            )

        if not candidates:
            print(f"[thruster perf] no usable curve in: {perf_path}", flush=True)
            return

        requested = _normalize_thruster_perf_voltage(active_thruster_voltage)
        selected = min(candidates, key=lambda item: abs(item["voltage"] - requested))
        perf_cfg.update(
            {
                "active": True,
                "requested_voltage": requested,
                "selected_voltage": float(selected["voltage"]),
                "pwm": selected["pwm"],
                "force": selected["force"],
            }
        )
        print(
            f"[thruster perf] loaded curve {selected['voltage']}V from {perf_path} "
            f"(requested {requested}V)",
            flush=True,
        )
        if perf_cfg.get("direct"):
            print(
                "[thruster perf] direct mode: raw PWM command -> T200 curve; "
                "legacy polynomial shaping and thruster gain scaling bypassed",
                flush=True,
            )

    if not args.disable_thruster_perf:
        _load_thruster_performance(Path(args.thruster_perf_file).expanduser())

    def pwm_to_force_from_perf(norm_cmd: float) -> float:
        pwm = float(np.clip(norm_cmd, -1.0, 1.0) * 400.0 + 1500.0)
        return float(np.interp(pwm, perf_cfg["pwm"], perf_cfg["force"]))

    # Load model/state once and reuse for runtime, validation, and calibration paths.
    model = mujoco.MjModel.from_xml_path(args.scene)
    data = mujoco.MjData(model)
    scene_fluid_density = float(model.opt.density)
    scene_fluid_viscosity = float(model.opt.viscosity)

    fluidcoef_scale = _to_float_array(sim_profile.get("mujoco_fluidcoef_scale"))
    fluid_geom_mask = (model.geom_fluid[:, 0] > 0.5) & np.any(
        np.abs(model.geom_fluid[:, 1:6]) > 1e-12,
        axis=1,
    )
    fluid_geom_ids = np.flatnonzero(fluid_geom_mask)

    def _fluid_geom_name(geom_id: int) -> str:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(geom_id))
        return name or f"geom_{geom_id}"

    if fluidcoef_scale is not None:
        if fluidcoef_scale.size != 5:
            print(
                "[physics] ignoring mujoco_fluidcoef_scale: expected 5 values "
                "(blunt, slender, angular, Kutta, Magnus)",
                flush=True,
            )
        else:
            fluidcoef_scale = np.clip(fluidcoef_scale.astype(np.float64, copy=False), 0.0, 10.0)
            if fluid_geom_ids.size:
                model.geom_fluid[fluid_geom_mask, 1:6] *= fluidcoef_scale.reshape(1, 5)
                print(
                    "[physics] MuJoCo fluidcoef scale applied: "
                    f"count={int(fluid_geom_ids.size)}, "
                    "coeff=(blunt, slender, angular, Kutta, Magnus)="
                    f"{np.array2string(fluidcoef_scale, precision=3)}",
                    flush=True,
                )

    fluidcoef_geom_scales = sim_profile.get("mujoco_fluidcoef_geom_scales")
    if isinstance(fluidcoef_geom_scales, dict) and fluid_geom_ids.size:
        fluid_geom_names = {int(geom_id): _fluid_geom_name(int(geom_id)) for geom_id in fluid_geom_ids}
        for pattern, raw_scale in fluidcoef_geom_scales.items():
            geom_scale = _to_float_array(raw_scale)
            if geom_scale is None or geom_scale.size != 5:
                print(
                    "[physics] ignoring mujoco_fluidcoef_geom_scales "
                    f"for {pattern!r}: expected 5 values "
                    "(blunt, slender, angular, Kutta, Magnus)",
                    flush=True,
                )
                continue
            geom_scale = np.clip(geom_scale.astype(np.float64, copy=False), 0.0, 10.0)
            matching_geom_ids = [
                geom_id
                for geom_id, geom_name in fluid_geom_names.items()
                if fnmatch.fnmatchcase(geom_name, str(pattern))
            ]
            if not matching_geom_ids:
                print(
                    "[physics] warning: mujoco_fluidcoef_geom_scales pattern "
                    f"{pattern!r} matched no fluid geoms",
                    flush=True,
                )
                continue
            model.geom_fluid[np.array(matching_geom_ids, dtype=np.int32), 1:6] *= geom_scale.reshape(1, 5)
            print(
                "[physics] MuJoCo fluidcoef per-geom scale applied: "
                f"pattern={pattern!r}, count={len(matching_geom_ids)}, "
                f"geoms={', '.join(fluid_geom_names[geom_id] for geom_id in matching_geom_ids)}, "
                "coeff=(blunt, slender, angular, Kutta, Magnus)="
                f"{np.array2string(geom_scale, precision=3)}",
                flush=True,
            )

    # Initialize derived state once before runtime loops so launch start poses
    # and sensor readings use valid base position.
    mujoco.mj_forward(model, data)

    # Identify base body
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    if base_id < 0:
        raise SystemExit("[runtime] base_link body not found in model")
    world_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "world_joint")
    if world_joint_id < 0:
        raise SystemExit("[runtime] world_joint free joint not found in model")
    world_qpos_adr = int(model.jnt_qposadr[world_joint_id])
    world_qvel_adr = int(model.jnt_dofadr[world_joint_id])
    water_surface_z = float(_env_float("UUV_WATER_SURFACE_Z", 0.0))

    def quat_wxyz_from_rpy_rad(roll: float, pitch: float, yaw: float) -> np.ndarray:
        cr = math.cos(0.5 * float(roll))
        sr = math.sin(0.5 * float(roll))
        cp = math.cos(0.5 * float(pitch))
        sp = math.sin(0.5 * float(pitch))
        cy = math.cos(0.5 * float(yaw))
        sy = math.sin(0.5 * float(yaw))
        quat = np.array(
            [
                cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
            ],
            dtype=np.float64,
        )
        norm = float(np.linalg.norm(quat))
        if norm <= 0.0 or not np.isfinite(norm):
            return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        return quat / norm

    def base_origin_world() -> np.ndarray:
        """Shared rigid-body reference used by SITL and buoyancy/depth helpers."""
        return data.xpos[base_id].copy()

    def set_base_depth(depth_m: float, reset_velocity: bool = True) -> None:
        """Move the free body origin to a given positive-down depth."""
        data.qpos[world_qpos_adr + 2] = water_surface_z - float(depth_m)
        if reset_velocity:
            data.qvel[world_qvel_adr : world_qvel_adr + 6] = 0.0
            data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
        mujoco.mj_forward(model, data)

    def set_base_attitude_rpy(roll: float, pitch: float, yaw: float, reset_velocity: bool = True) -> None:
        """Set the free body attitude from ROS/ENU roll, pitch, yaw."""
        data.qpos[world_qpos_adr + 3 : world_qpos_adr + 7] = quat_wxyz_from_rpy_rad(roll, pitch, yaw)
        if reset_velocity:
            data.qvel[world_qvel_adr + 3 : world_qvel_adr + 6] = 0.0
            data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
        mujoco.mj_forward(model, data)

    drop_start_enabled_raw = os.environ.get("UUV_SITL_DROP_START_ABOVE_WATER", "1").strip().lower()
    drop_start_enabled = drop_start_enabled_raw in {"1", "true", "yes", "on", "enable", "enabled"}
    if args.sitl and args.initial_depth_m is None and drop_start_enabled:
        drop_height_m = float(max(_env_float("UUV_SITL_DROP_HEIGHT_M", 0.35), 0.0))
        args.initial_depth_m = -drop_height_m

    initial_depth_hold = {
        "active": bool(args.hold_initial_depth_until_release),
        "depth_m": float(args.initial_depth_m) if args.initial_depth_m is not None else None,
        "release_linear_velocity_body": (
            np.asarray(args.release_linear_velocity_body, dtype=np.float64)
            if args.release_linear_velocity_body is not None
            else None
        ),
        "pose_qpos": None,
    }

    def capture_initial_depth_hold_pose(depth_m: float | None = None) -> None:
        pose_qpos = data.qpos[world_qpos_adr : world_qpos_adr + 7].copy()
        if depth_m is not None:
            pose_qpos[2] = water_surface_z - float(depth_m)
        initial_depth_hold["pose_qpos"] = pose_qpos

    if args.initial_depth_m is not None:
        set_base_depth(float(args.initial_depth_m))
        initial_depth_value = float(args.initial_depth_m)
        if initial_depth_value < 0.0:
            start_msg = f"drop start: base_link={-initial_depth_value:.3f} m above water"
        else:
            start_msg = f"initial depth set: {initial_depth_value:.3f} m"
        print(
            "[runtime] "
            + start_msg
            + (
                " with hold enabled"
                if initial_depth_hold["active"]
                else ""
            ),
            flush=True,
        )
    if args.initial_rpy_rad is not None:
        initial_rpy = np.asarray(args.initial_rpy_rad, dtype=np.float64)
        if initial_rpy.shape == (3,) and np.all(np.isfinite(initial_rpy)):
            set_base_attitude_rpy(float(initial_rpy[0]), float(initial_rpy[1]), float(initial_rpy[2]))
            print(
                "[runtime] initial attitude set: "
                f"roll={initial_rpy[0]:+.4f} pitch={initial_rpy[1]:+.4f} yaw={initial_rpy[2]:+.4f} rad",
                flush=True,
            )
    if args.initial_depth_m is not None:
        capture_initial_depth_hold_pose(float(args.initial_depth_m))

    def apply_initial_depth_hold() -> None:
        if not initial_depth_hold["active"]:
            return
        depth_m = initial_depth_hold["depth_m"]
        if depth_m is None:
            return
        pose_qpos = initial_depth_hold.get("pose_qpos")
        if pose_qpos is not None:
            hold_pose = np.asarray(pose_qpos, dtype=np.float64).copy()
            if hold_pose.shape == (7,) and np.all(np.isfinite(hold_pose)):
                hold_pose[2] = water_surface_z - float(depth_m)
                data.qpos[world_qpos_adr : world_qpos_adr + 7] = hold_pose
                data.qvel[world_qvel_adr : world_qvel_adr + 6] = 0.0
                data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
                mujoco.mj_forward(model, data)
                return
        set_base_depth(float(depth_m), reset_velocity=True)

    def apply_release_linear_velocity() -> None:
        velocity_body = initial_depth_hold.get("release_linear_velocity_body")
        if velocity_body is None:
            return
        velocity_body = np.asarray(velocity_body, dtype=np.float64)
        if velocity_body.shape != (3,) or not np.all(np.isfinite(velocity_body)):
            return
        base_rot = data.xmat[base_id].reshape(3, 3)
        data.qvel[world_qvel_adr : world_qvel_adr + 3] = base_rot @ velocity_body
        data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
        mujoco.mj_forward(model, data)
        print(
            "[runtime] release linear velocity body set: "
            f"{np.array2string(velocity_body, precision=4)} m/s",
            flush=True,
        )

    def reset_initial_depth_release_state() -> None:
        data.qvel[world_qvel_adr : world_qvel_adr + 6] = 0.0
        data.qacc[world_qvel_adr : world_qvel_adr + 6] = 0.0
        mujoco.mj_forward(model, data)

    # Build body-child table once so subtree mass can be computed robustly.
    body_children = [[] for _ in range(model.nbody)]
    for body_idx in range(1, model.nbody):
        parent_idx = int(model.body_parentid[body_idx])
        if 0 <= parent_idx < model.nbody:
            body_children[parent_idx].append(body_idx)

    def body_subtree_mass(root_body_id: int) -> float:
        """Return total mass of the given body and all descendants."""
        if not (0 <= root_body_id < model.nbody):
            return 0.0
        total = 0.0
        stack = [int(root_body_id)]
        while stack:
            bid = stack.pop()
            total += float(model.body_mass[bid])
            stack.extend(body_children[bid])
        return total

    # Actuator indices
    act = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
        for i in range(model.nu)
    }
    ctrlrange = model.actuator_ctrlrange.copy()
    thruster_params_path = CONFIG_DIR / "thruster_params.json"

    # Shared command state. ROS2 and SITL handlers write into this state.
    cmd = {"forward": 0.0, "heave": 0.0, "yaw": 0.0, "sway": 0.0}
    state = {"step": 2.0, "max": 15.0}
    ros_cmd_last_wall = {"value": -1.0}
    ros_cmd_timeout_s = float(
        np.clip(
            float(os.getenv("ROS2_UUV_CMD_TIMEOUT_S", "0.45")),
            0.1,
            2.0,
        )
    )

    def env_flag(name: str, default: bool = False) -> bool:
        value = os.environ.get(name)
        if value is None or value == "":
            return bool(default)
        return value.strip().lower() in {"1", "true", "yes", "on", "enable", "enabled"}

    sitl_allow_direct_cmd = env_flag("ROS2_UUV_SITL_ALLOW_DIRECT_CMD", False)
    if args.sitl and not sitl_allow_direct_cmd:
        print(
            "[control] SITL closed-loop authority: direct MuJoCo command fallback is disabled. "
            "Set ROS2_UUV_SITL_ALLOW_DIRECT_CMD=1 only for smoke/debug.",
            flush=True,
        )
    cmd_lock = threading.Lock()
    stop_event = threading.Event()
    paused_flag = {"value": False}
    show_viewer_debug = {"value": bool(args.viewer_debug)}
    show_thruster_labels = {"value": bool(args.viewer_debug)}
    initial_viewer_camera_mode = str(args.viewer_camera_mode or "").strip().lower()
    if not initial_viewer_camera_mode:
        initial_viewer_camera_mode = "follow" if args.sitl else "free"
    if initial_viewer_camera_mode not in {"free", "follow", "stereo_left", "stereo_right"}:
        initial_viewer_camera_mode = "follow" if args.sitl else "free"
    follow_camera = {"value": initial_viewer_camera_mode == "follow"}
    follow_camera_init = {"value": False}
    show_sensor_overlay = {"value": bool(args.viewer_debug)}
    camera_mode = {"value": initial_viewer_camera_mode}  # free | follow | stereo_left | stereo_right
    follow_camera_distance = float(np.clip(_env_float("UUV_VIEWER_FOLLOW_DISTANCE", 2.2), 0.2, 20.0))
    follow_camera_elevation = float(np.clip(_env_float("UUV_VIEWER_FOLLOW_ELEVATION_DEG", -20.0), -89.0, 89.0))
    follow_camera_azimuth = float(_env_float("UUV_VIEWER_FOLLOW_AZIMUTH_DEG", 135.0))

    def clamp(value: float, max_val: float) -> float:
        return max(-max_val, min(max_val, value))

    def toggle_thruster_labels() -> None:
        show_thruster_labels["value"] = not show_thruster_labels["value"]
        if show_thruster_labels["value"]:
            show_viewer_debug["value"] = True

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
        if show_sensor_overlay["value"]:
            show_viewer_debug["value"] = True

    def apply_ros_cmd(forward: float, sway: float, yaw: float, heave: float) -> None:
        if args.sitl and not sitl_allow_direct_cmd:
            return
        with cmd_lock:
            max_val = state["max"]
            cmd["forward"] = clamp(forward, max_val)
            cmd["sway"] = clamp(sway, max_val)
            cmd["yaw"] = clamp(yaw, max_val)
            cmd["heave"] = clamp(heave, max_val)
        ros_cmd_last_wall["value"] = time.monotonic()

    ros_bridge = None
    try:
        from bridge.ros2_bridge import Ros2Bridge

        enable_ros2 = bool(args.ros2)
        ping360_overrides = {
            "requested_range_m": args.ping360_range_m,
            "num_steps": args.ping360_num_steps,
            "interface_mode": args.ping360_interface,
            "gain_setting": args.ping360_gain,
        }

        if args.sitl or enable_ros2:
            ros_bridge = Ros2Bridge(
                model=model,
                command_callback=apply_ros_cmd,
                cmd_limit=state["max"],
                publish_images=args.ros2_images,
                image_width=args.ros2_image_width,
                image_height=args.ros2_image_height,
                sensor_hz=args.ros2_sensor_hz,
                image_hz=args.ros2_image_hz,
                enable_sitl=args.sitl,
                sitl_ip=args.sitl_ip,
                sitl_port=args.sitl_port,
                sitl_send_port=args.sitl_send_port,
                sitl_mavlink_endpoint=str(args.sitl_mavlink_endpoint),
                sitl_mavlink_servo_hz=float(args.sitl_mavlink_servo_hz),
                sitl_mavlink_target_sysid=int(args.sitl_mavlink_target_sysid),
                sitl_mavlink_target_compid=int(args.sitl_mavlink_target_compid),
                sitl_mavlink_source_sysid=int(args.sitl_mavlink_source_sysid),
                sitl_mavlink_source_compid=int(args.sitl_mavlink_source_compid),
                camera_calib_left=args.ros2_camera_calib_left,
                camera_calib_right=args.ros2_camera_calib_right,
                enable_ros=enable_ros2,
                enable_mavros_surface=not args.ros2_real_pkg_compat,
                enable_ping360=not args.no_ping360,
                ping360_config_path=args.ping360_config,
                ping360_overrides=ping360_overrides,
            )
            if enable_ros2:
                bridge_topics = (
                    "[bridge] enabled: /cmd_vel(TwistStamped) -> control, /imu/data, "
                    "/dvl/velocity, /dvl/twist, /dvl/odometry, /dvl/altitude, /dvl/data, /dvl/position, "
                    "/depth, /depth/pose, /bar30/pressure_pa, /rovio/odometry, /mujoco/ground_truth/pose, "
                    "/tf, /tf_static, /robot_description"
                )
                if not args.no_ping360:
                    bridge_topics += (
                        ", /ping360/image, /ping360/scan_image, /ping360/scan, /ping360/scan_echo, "
                        "/ping360/echo, /ping360/status, /ping360/config"
                    )
                if args.ros2_real_pkg_compat:
                    bridge_topics += ", mavros_surface=compat-only, /mavros/vfr_hud"
                else:
                    bridge_topics += (
                        ", /mavros/state, /mavros/imu/*, /mavros/vfr_hud, "
                        "/mavros/local_position/pose, /mavros/local_position/velocity_local, "
                        "/mavros/local_position/odom, /mavros/vision_pose/pose, "
                        "/mavros/battery, /mavros/rc/in, /mavros/rc/override, "
                        "/mavros/setpoint_raw/local, /mavros/cmd/arming, /mavros/set_mode, "
                        "/mavros/cmd/command"
                    )
                print(bridge_topics, flush=True)
                if args.ros2_images:
                    print(
                        "[bridge] --ros2-images is ignored by the lightweight real-robot bridge. "
                        "Use --qgc-video if you need the stereo_left video stream.",
                        flush=True,
                    )
                if args.ros2_real_pkg_compat and args.sitl:
                    print(
                        "[bridge] real package compat: launch external MAVROS with "
                        "fcu_url:=udp://:14551@127.0.0.1:14551 "
                        "(ArduSub MAVLink output on port 14551)",
                        flush=True,
                    )
            elif args.sitl:
                sitl_servo_mode = "mavlink"
                sitl_mavlink_endpoint_value = str(args.sitl_mavlink_endpoint or "").strip().lower()
                if sitl_mavlink_endpoint_value in {"", "none", "off", "disabled", "disable"}:
                    sitl_servo_mode = "json"
                print(
                    f"[bridge] SITL transport enabled (sensor=UDP JSON, servo={sitl_servo_mode}, ROS2 disabled).",
                    flush=True,
                )
    except Exception as exc:
        if args.sitl:
            raise SystemExit(
                f"[runtime] --sitl initialization failed: {exc}"
            )
        if args.ros2:
            print(f"[ros2] bridge init failed: {exc}", flush=True)

    initial_depth_services = []
    if (
        initial_depth_hold["active"]
        and ros_bridge is not None
        and getattr(ros_bridge, "node", None) is not None
    ):
        try:
            from std_srvs.srv import Trigger

            def _release_initial_depth_hold(_request, response):
                initial_depth_hold["active"] = False
                reset_initial_depth_release_state()
                apply_release_linear_velocity()
                mujoco.mj_forward(model, data)
                response.success = True
                response.message = "initial depth hold released"
                print("[runtime] initial depth hold released", flush=True)
                return response

            initial_depth_services.append(
                ros_bridge.node.create_service(
                    Trigger,
                    "/mujoco/release_initial_depth_hold",
                    _release_initial_depth_hold,
                )
            )
            print(
                "[runtime] initial depth hold service enabled: "
                "/mujoco/release_initial_depth_hold",
                flush=True,
            )
        except Exception as exc:
            print(f"[ros2] initial depth hold service unavailable: {exc}", flush=True)

    fluid_model = str(args.fluid_model)
    if args.sitl:
        if fluid_model != "legacy":
            print(
                "[physics] SITL free-surface contract: forcing fluid_model=legacy "
                "and disabling MuJoCo built-in global fluid",
                flush=True,
            )
        fluid_model = "legacy"
    use_custom_hydrodynamics = fluid_model == "legacy"
    if use_custom_hydrodynamics:
        model.opt.density = 0.0
        model.opt.viscosity = 0.0
        print(
            "[physics] fluid model: legacy/custom free-surface "
            f"(MuJoCo built-in fluid disabled, scene rho={scene_fluid_density:.1f}, "
            f"viscosity={scene_fluid_viscosity:.6f})",
            flush=True,
        )
    else:
        print(
            "[physics] fluid model: current MuJoCo built-in ellipsoid "
            f"(rho={scene_fluid_density:.1f}, viscosity={scene_fluid_viscosity:.6f})",
            flush=True,
        )
        if not any(token in Path(args.scene).name for token in ("current", "ellipsoid")):
            print(
                "[physics] warning: current ellipsoid mode selected but the scene path "
                "does not look like an ellipsoid proxy scene.",
                flush=True,
            )


    if args.sitl:
        print("[runtime] Local manual control path removed in SITL mode (QGC remote control only).", flush=True)
    else:
        print("[runtime] Manual keyboard/joystick path removed. Use ROS2 /cmd_vel or SITL/QGC control.", flush=True)
    if args.sitl and args.ros2:
        print(
            "[runtime] SITL RC override/manual-control topics are forwarded to "
            "ArduSub; direct MuJoCo override is disabled unless explicitly enabled.",
            flush=True,
        )

    # Thruster sets
    yaw_names = list(PHYSICAL_YAW_THRUSTERS)
    # Horizontal allocation follows ArduSub vectored_6dof channel order.
    horiz_order = list(ARDUSUB_VECTORED_6DOF_YAW_CHANNEL_ORDER)
    ver_names = list(PHYSICAL_VERTICAL_THRUSTERS)

    def normalize(v: np.ndarray) -> np.ndarray:
        n = float(np.linalg.norm(v))
        if n < 1e-9:
            return v
        return v / n

    sensor_names = ["imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude", "depth_pos"]
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

    qgc_video_renderer = None
    qgc_video_streamer = None
    qgc_video_share_bridge = False
    qgc_video_dt = 1.0 / max(float(args.qgc_video_fps), 1.0)
    qgc_video_next_t = 0.0
    if args.qgc_video:
        if "stereo_left" not in camera_ids:
            print("[qgc_video] stereo_left camera not found; video disabled", flush=True)
        elif not QgcVideoStreamer.is_available():
            print("[qgc_video] ffmpeg not found; video disabled", flush=True)
        else:
            qgc_video_share_bridge = bool(
                ros_bridge is not None
                and hasattr(ros_bridge, "can_share_camera_renderer")
                and ros_bridge.can_share_camera_renderer(
                    "stereo_left",
                    int(args.qgc_video_width),
                    int(args.qgc_video_height),
                )
            )
            if not qgc_video_share_bridge:
                qgc_video_renderer = mujoco.Renderer(
                    model,
                    height=int(args.qgc_video_height),
                    width=int(args.qgc_video_width),
                )
            qgc_video_streamer = QgcVideoStreamer(
                host=args.qgc_video_host,
                port=int(args.qgc_video_port),
                width=int(args.qgc_video_width),
                height=int(args.qgc_video_height),
                fps=float(args.qgc_video_fps),
                bitrate_kbps=int(args.qgc_video_bitrate_kbps),
            )
            print(
                f"[qgc_video] direct UDP stream enabled: stereo_left -> "
                f"rtp://{args.qgc_video_host}:{args.qgc_video_port} "
                f"({args.qgc_video_width}x{args.qgc_video_height}@{args.qgc_video_fps:.1f}fps)",
                flush=True,
            )
            if qgc_video_share_bridge:
                print("[qgc_video] sharing stereo_left renderer with ROS2 image bridge", flush=True)

    sensor_site_ids = {
        "imu": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "imu_site"),
        "bar30": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "bar30_site"),
        "dvl": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "dvl_site"),
        "ping360": mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ping360_site"),
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

    perf_force_max = None
    if perf_cfg.get("active") and perf_cfg.get("force").size > 0:
        perf_max = float(np.max(np.abs(perf_cfg["force"])))
        if perf_max > 0.0:
            perf_force_max = perf_max
    hydro_cfg = build_hydrodynamics_config(
        sim_profile,
        perf_force_max=perf_force_max,
        fluid_density=scene_fluid_density,
    )

    def component_self_inertia_diag(component) -> np.ndarray:
        a, b, c = component.size.astype(np.float64, copy=False)
        if component.shape == "box":
            return np.array(
                [
                    component.mass * (b * b + c * c) / 3.0,
                    component.mass * (a * a + c * c) / 3.0,
                    component.mass * (a * a + b * b) / 3.0,
                ],
                dtype=np.float64,
            )
        return np.array(
            [
                component.mass * (b * b + c * c) / 5.0,
                component.mass * (a * a + c * c) / 5.0,
                component.mass * (a * a + b * b) / 5.0,
            ],
            dtype=np.float64,
        )

    def apply_body_component_distribution() -> None:
        components = hydro_cfg.body_components
        if not components:
            return
        total_mass = float(sum(component.mass for component in components))
        if total_mass <= 1e-9:
            return
        inertia_scale = np.array(
            sim_profile.get("body_inertia_scale_xyz", [1.0, 1.0, 1.0]),
            dtype=np.float64,
        )
        if inertia_scale.shape != (3,) or not np.all(np.isfinite(inertia_scale)):
            inertia_scale = np.ones(3, dtype=np.float64)
        inertia_scale = np.clip(inertia_scale, 1e-6, 100.0)

        composite_com = sum(
            component.mass * component.mass_pos for component in components
        ) / total_mass
        composite_inertia = np.zeros(3, dtype=np.float64)
        for component in components:
            offset = component.mass_pos - composite_com
            parallel_axis = component.mass * np.array(
                [
                    offset[1] * offset[1] + offset[2] * offset[2],
                    offset[0] * offset[0] + offset[2] * offset[2],
                    offset[0] * offset[0] + offset[1] * offset[1],
                ],
                dtype=np.float64,
            )
            composite_inertia += component_self_inertia_diag(component) + parallel_axis
        composite_inertia *= inertia_scale

        old_mass = float(model.body_mass[base_id])
        old_com = model.body_ipos[base_id].copy()
        old_inertia = model.body_inertia[base_id].copy()

        model.body_mass[base_id] = total_mass
        model.body_ipos[base_id, :] = composite_com
        model.body_inertia[base_id, :] = np.maximum(composite_inertia, 1e-6)
        if hasattr(mujoco, "mj_setConst"):
            mujoco.mj_setConst(model, data)
        mujoco.mj_forward(model, data)

        print(
            "[model] distributed body components: "
            f"{len(components)} parts, mass {old_mass:.3f} -> {total_mass:.3f} kg, "
            f"CoM {np.array2string(old_com, precision=4)} -> "
            f"{np.array2string(composite_com, precision=4)}, "
            f"inertia {np.array2string(old_inertia, precision=4)} -> "
            f"{np.array2string(composite_inertia, precision=4)} "
            f"(scale={np.array2string(inertia_scale, precision=4)})",
            flush=True,
        )
        for component in components:
            print(
                "[model]   component "
                f"{component.name}: mass={component.mass:.3f}kg, "
                f"mass_pos={np.array2string(component.mass_pos, precision=4)}, "
                f"buoyancy_pos={np.array2string(component.buoyancy_pos, precision=4)}, "
                f"share={component.buoyancy_share:.3f}",
                flush=True,
            )

    apply_body_component_distribution()

    cob_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "cob_site")
    cob_longitudinal_offset = float(sim_profile.get("cob_x_offset", 0.0))
    cob_vertical_offset = float(sim_profile.get("cob_z_offset", 0.0))

    def align_cob_to_com_with_offset() -> None:
        if cob_site_id < 0:
            return
        target_x = float(model.body_ipos[base_id][0] + cob_longitudinal_offset)
        target_z = float(model.body_ipos[base_id][2] + cob_vertical_offset)
        old_x = float(model.site_pos[cob_site_id][0])
        old_z = float(model.site_pos[cob_site_id][2])
        if abs(target_x - old_x) < 1e-6 and abs(target_z - old_z) < 1e-6:
            return
        model.site_pos[cob_site_id][0] = target_x
        model.site_pos[cob_site_id][2] = target_z
        print(
            f"[model] aligned CoB: x {old_x:.4f} -> {target_x:.4f} "
            f"(offset={cob_longitudinal_offset:+.4f}), "
            f"z {old_z:.4f} -> {target_z:.4f} "
            f"(offset={cob_vertical_offset:+.4f})",
            flush=True,
        )

    align_cob_to_com_with_offset()
    if hydro_cfg.buoyancy_points:
        print(
            f"[physics] buoyancy application: distributed 4-point model "
            f"({len(hydro_cfg.buoyancy_points)} points)",
            flush=True,
        )
    elif hydro_cfg.body_components:
        print(
            f"[physics] buoyancy application: distributed component model "
            f"({len(hydro_cfg.body_components)} points)",
            flush=True,
        )
    else:
        print("[physics] buoyancy application: single-point CoB/center model", flush=True)

    # Geometry-aware horizontal allocator:
    # maps desired [forward, sway(left+), yaw(ccw+)] in normalized units
    # to 4 horizontal thruster commands in body FLU.
    horiz_pinv = np.zeros((len(horiz_order), 3), dtype=np.float64)
    horiz_alloc = np.zeros((3, len(horiz_order)), dtype=np.float64)
    horiz_stab_yaw_scale = {"value": 40.0}

    def build_horizontal_allocator() -> None:
        nonlocal horiz_pinv, horiz_alloc
        com_body = model.body_ipos[base_id].copy()
        alloc = np.zeros((3, len(horiz_order)), dtype=np.float64)
        for i, name in enumerate(horiz_order):
            aid = act[name]
            sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
            fdir = normalize(model.actuator_gear[aid, :3].copy())
            r = model.site_pos[sid].copy() - com_body
            tau = np.cross(r, fdir)
            # [Fx, Fy, Mz] in body frame
            alloc[:, i] = np.array([fdir[0], fdir[1], tau[2]], dtype=np.float64)

        row_scale = np.sum(np.abs(alloc), axis=1)
        row_scale = np.where(row_scale < 1e-6, 1.0, row_scale)
        horiz_pinv = np.linalg.pinv(alloc / row_scale[:, None])
        horiz_alloc = alloc

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
    # Keep the same waterline used by initial pose/depth and Bar30 contracts.
    rho = scene_fluid_density
    g = abs(float(model.opt.gravity[2]))
    total_mass_all = float(np.sum(model.body_mass[1:]))
    vehicle_mass = body_subtree_mass(base_id)
    if vehicle_mass <= 1e-9:
        vehicle_mass = float(model.body_mass[base_id])
    neutral_volume = vehicle_mass / max(rho, 1e-6)
    if total_mass_all > vehicle_mass * 1.2:
        print(
            "[physics] buoyancy mass reference: "
            f"vehicle_subtree={vehicle_mass:.3f}kg (all_nonworld={total_mass_all:.3f}kg)",
            flush=True,
        )

    all_thruster_names = list(PHYSICAL_VERTICAL_THRUSTERS + PHYSICAL_YAW_THRUSTERS)
    thruster_site_ids = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"thr_{name}")
        for name in all_thruster_names
    }
    thr_state = {name: 0.0 for name in all_thruster_names}
    thr_target = {name: 0.0 for name in all_thruster_names}
    thruster_scale = {name: 1.0 for name in all_thruster_names}
    thruster_direct_scale = {name: 1.0 for name in all_thruster_names}
    thruster_reverse_asymmetry = {name: None for name in all_thruster_names}
    thruster_tau_up = {name: None for name in all_thruster_names}
    thruster_tau_down = {name: None for name in all_thruster_names}
    thruster_force_cmd = {name: 0.0 for name in all_thruster_names}
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

    thruster_global = {
        "deadzone": 0.05,
        "tau_up": 0.12,
        "tau_down": 0.18,
        "reverse_asymmetry": 0.75,
        "command_limit": 0.65,
        "reaction_torque_gain": 0.012,
        "gain_scale_all": 1.0,
        "forward_poly": [0.0, 3.5, 7.0, 12.0],
        "reverse_poly": [0.0, 2.8, 5.5, 9.5],
    }

    sitl_servo_cmd_norm = {name: 0.0 for name in all_thruster_names}
    sitl_servo_pwm_values = [1500] * 8
    sitl_servo_last_wall = {"value": -1.0}
    sitl_servo_timeout_s = 0.8
    sitl_servo_scale = float(np.clip(args.sitl_servo_scale, 0.0, 2.0))

    def sitl_pwm_to_norm(pwm: int) -> float:
        # ArduSub bidirectional motor range: 1100..1900, neutral=1500.
        if pwm <= 0 or pwm == 65535:
            return 0.0
        return float(np.clip((float(pwm) - 1500.0) / 400.0, -1.0, 1.0))

    if args.sitl:
        raw_map = list(ARDUSUB_VECTORED_6DOF_SERVO_MAP)
        servo_signs = list(ARDUSUB_VECTORED_6DOF_SERVO_SIGNS)

        def on_sitl_servo_packet(pwm_values: list[int]) -> None:
            for idx in range(len(sitl_servo_pwm_values)):
                sitl_servo_pwm_values[idx] = int(pwm_values[idx]) if idx < len(pwm_values) else 1500
            for thr_name in all_thruster_names:
                sitl_servo_cmd_norm[thr_name] = 0.0
            packet_commands = {thr_name: 0.0 for thr_name in all_thruster_names}
            for idx, thr_name in enumerate(raw_map):
                if idx >= len(pwm_values):
                    break
                norm = sitl_pwm_to_norm(int(pwm_values[idx])) * servo_signs[idx]
                packet_commands[thr_name] = float(np.clip(norm, -1.0, 1.0))
            for thr_name, norm in packet_commands.items():
                sitl_servo_cmd_norm[thr_name] = float(np.clip(norm, -1.0, 1.0))
            sitl_servo_last_wall["value"] = time.monotonic()

        if ros_bridge is not None:
            ros_bridge.set_sitl_servo_handler(on_sitl_servo_packet)
        print(
            "[sitl] direct thruster mode enabled: "
            + ", ".join(
                f"ch{idx + 1}->{thr_name}*{servo_signs[idx]:+0.0f}"
                for idx, thr_name in enumerate(raw_map)
            )
            + f", servo-scale={sitl_servo_scale:.2f}",
            flush=True,
        )

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
                "reaction_torque_gain": 0.012,
                "gain_scale_all": 1.0,
                "direct_gain_scale_all": 1.0,
            },
            "per_thruster": {
                name: {
                    "gain_scale": 1.0,
                    "direct_gain_scale": 1.0,
                    "reverse_asymmetry": None,
                    "tau_up": None,
                    "tau_down": None,
                }
                for name in all_thruster_names
            },
        }
        thruster_params_path.write_text(json.dumps(payload, indent=2))

    def load_thruster_params() -> bool:
        for name in all_thruster_names:
            thruster_scale[name] = 1.0
            thruster_direct_scale[name] = 1.0
            thruster_reverse_asymmetry[name] = None
            thruster_tau_up[name] = None
            thruster_tau_down[name] = None
        if not thruster_params_path.exists():
            return False
        try:
            payload = json.loads(thruster_params_path.read_text())
        except json.JSONDecodeError:
            return False
        changed = False
        global_cfg = payload.get("global", {})
        global_gain_scale = 1.0
        global_direct_gain_scale = 1.0
        if isinstance(global_cfg, dict):
            for key in (
                "deadzone",
                "tau_up",
                "tau_down",
                "reverse_asymmetry",
                "command_limit",
                "reaction_torque_gain",
            ):
                value = global_cfg.get(key)
                if isinstance(value, (int, float)):
                    thruster_global[key] = float(value)
            scale_all = global_cfg.get("gain_scale_all")
            if isinstance(scale_all, (int, float)):
                global_gain_scale = float(np.clip(float(scale_all), 0.1, 20.0))
                thruster_global["gain_scale_all"] = global_gain_scale
            direct_scale_all = global_cfg.get("direct_gain_scale_all")
            if isinstance(direct_scale_all, (int, float)):
                global_direct_gain_scale = float(np.clip(float(direct_scale_all), 0.1, 20.0))
                thruster_global["direct_gain_scale_all"] = global_direct_gain_scale
            for key in ("forward_poly", "reverse_poly"):
                coeffs = global_cfg.get(key)
                if isinstance(coeffs, list) and coeffs:
                    try:
                        thruster_global[key] = [float(item) for item in coeffs]
                    except (TypeError, ValueError):
                        pass
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
            new_gain = float(
                np.clip(float(gain), 0.1, 20.0)
                * np.clip(global_gain_scale, 0.1, 20.0)
            )
            new_gain = float(np.clip(new_gain, 0.1, 20.0))
            if abs(new_gain - thruster_scale[name]) > 1e-8:
                changed = True
            thruster_scale[name] = new_gain
            direct_gain = cfg.get("direct_gain_scale", 1.0)
            if isinstance(direct_gain, (int, float)):
                new_direct_gain = float(
                    np.clip(float(direct_gain), 0.1, 20.0)
                    * np.clip(global_direct_gain_scale, 0.1, 20.0)
                )
                new_direct_gain = float(np.clip(new_direct_gain, 0.1, 20.0))
                if abs(new_direct_gain - thruster_direct_scale[name]) > 1e-8:
                    changed = True
                thruster_direct_scale[name] = new_direct_gain
            reverse_asym = cfg.get("reverse_asymmetry")
            if isinstance(reverse_asym, (int, float)):
                thruster_reverse_asymmetry[name] = float(np.clip(float(reverse_asym), 0.1, 1.5))
            tau_up = cfg.get("tau_up")
            if isinstance(tau_up, (int, float)):
                thruster_tau_up[name] = float(np.clip(float(tau_up), 1.0e-4, 2.0))
            tau_down = cfg.get("tau_down")
            if isinstance(tau_down, (int, float)):
                thruster_tau_down[name] = float(np.clip(float(tau_down), 1.0e-4, 2.0))
        return changed

    ensure_thruster_params_file()
    load_thruster_params()
    print(
        "[thruster] dynamics: "
        f"deadzone={thruster_global['deadzone']:.3f}, "
        f"tau_up={thruster_global['tau_up']:.3f}s, "
        f"tau_down={thruster_global['tau_down']:.3f}s, "
        f"reverse_asym={thruster_global['reverse_asymmetry']:.3f}, "
        f"command_limit={thruster_global['command_limit']:.3f}, "
        f"reaction_tau_gain={thruster_global['reaction_torque_gain']:.4f}, "
        f"gain_scale_all={thruster_global['gain_scale_all']:.3f}, "
        f"direct_gain_scale_all={thruster_global['direct_gain_scale_all']:.3f}",
        flush=True,
    )
    if perf_cfg.get("active") and perf_cfg.get("direct"):
        direct_scale_overrides = [
            f"{name}={float(thruster_direct_scale[name]):.3f}"
            for name in all_thruster_names
            if abs(float(thruster_direct_scale[name]) - 1.0) > 1e-8
        ]
        if direct_scale_overrides:
            print(
                "[thruster perf] direct curve gain overrides: "
                + ", ".join(direct_scale_overrides),
                flush=True,
            )
    yaw_reverse_asym = [
        (
            thruster_reverse_asymmetry[name]
            if thruster_reverse_asymmetry[name] is not None
            else thruster_global["reverse_asymmetry"]
        )
        for name in PHYSICAL_YAW_THRUSTERS
    ]
    if len(set(round(float(value), 6) for value in yaw_reverse_asym)) > 1 or abs(
        float(yaw_reverse_asym[0]) - float(thruster_global["reverse_asymmetry"])
    ) > 1e-8:
        print(
            "[thruster] yaw reverse asym override: "
            + ", ".join(
                f"{name}={float(value):.3f}"
                for name, value in zip(PHYSICAL_YAW_THRUSTERS, yaw_reverse_asym)
            ),
            flush=True,
        )
    yaw_tau_pairs = [
        (
            thruster_tau_up[name] if thruster_tau_up[name] is not None else thruster_global["tau_up"],
            thruster_tau_down[name] if thruster_tau_down[name] is not None else thruster_global["tau_down"],
        )
        for name in PHYSICAL_YAW_THRUSTERS
    ]
    if any(
        abs(float(up) - float(thruster_global["tau_up"])) > 1e-8
        or abs(float(down) - float(thruster_global["tau_down"])) > 1e-8
        for up, down in yaw_tau_pairs
    ):
        print(
            "[thruster] yaw dynamics override: "
            + ", ".join(
                f"{name}=up{float(up):.3f}/down{float(down):.3f}s"
                for name, (up, down) in zip(PHYSICAL_YAW_THRUSTERS, yaw_tau_pairs)
            ),
            flush=True,
        )

    # Upgraded underwater model: 6-DOF damping, added mass, current-relative flow,
    # and thruster dynamics layered on top of MuJoCo rigid-body physics.
    half_height = hydro_cfg.half_height
    buoyancy_model = hydro_cfg.buoyancy_model
    buoyancy_scale = hydro_cfg.buoyancy_scale
    buoyancy_slope_scale = hydro_cfg.buoyancy_slope_scale
    surface_heave_damping = hydro_cfg.surface_heave_damping
    heave_damping_scale = hydro_cfg.heave_damping_scale
    full_heave_damping = surface_heave_damping * heave_damping_scale
    cob_torque_scale = hydro_cfg.cob_torque_scale
    buoyancy_point_blend = hydro_cfg.buoyancy_point_blend
    thruster_force_max = hydro_cfg.thruster_force_max
    linear_drag = hydro_cfg.linear_drag
    angular_drag = hydro_cfg.angular_drag
    air_linear_drag = hydro_cfg.air_linear_drag
    air_angular_drag = hydro_cfg.air_angular_drag
    spin_gain = hydro_cfg.spin_gain
    yaw_torque_scale_config = float(max(hydro_cfg.yaw_torque_scale, 0.0))
    if perf_cfg.get("active") and perf_cfg.get("direct"):
        yaw_torque_scale = 1.0
    else:
        yaw_torque_scale = yaw_torque_scale_config
    added_mass_diag = hydro_cfg.added_mass_diag.astype(np.float64, copy=True)
    linear_damping_diag = hydro_cfg.linear_damping_diag.astype(np.float64, copy=True)
    quadratic_damping_diag = hydro_cfg.quadratic_damping_diag.astype(np.float64, copy=True)
    air_linear_damping_diag = hydro_cfg.air_linear_damping_diag.astype(np.float64, copy=True)
    water_current_world = hydro_cfg.water_current_world.astype(np.float64, copy=True)
    body_components = hydro_cfg.body_components
    buoyancy_points = hydro_cfg.buoyancy_points
    thruster_air_force_scale = float(np.clip(_env_float("UUV_THRUSTER_AIR_FORCE_SCALE", 0.0), 0.0, 1.0))
    thruster_immersion_half_height_m = float(
        max(_env_float("UUV_THRUSTER_IMMERSION_HALF_HEIGHT_M", 0.045), 1.0e-4)
    )
    if hydro_cfg.displaced_volume is not None and hydro_cfg.displaced_volume > 0.0:
        neutral_volume = float(hydro_cfg.displaced_volume)
    thruster_loop_hz = float(np.clip(args.thruster_loop_hz, 1.0, 500.0))
    thruster_loop_dt = 1.0 / thruster_loop_hz
    next_thruster_sim_time = {"value": -1.0}

    def thruster_update_due() -> bool:
        sim_t = float(data.time)
        if next_thruster_sim_time["value"] < 0.0:
            next_thruster_sim_time["value"] = sim_t
        if sim_t + 1e-9 < next_thruster_sim_time["value"]:
            return False
        while sim_t + 1e-9 >= next_thruster_sim_time["value"]:
            next_thruster_sim_time["value"] += thruster_loop_dt
        return True

    print(
        f"[runtime] thruster loop rate: {thruster_loop_hz:.1f} Hz "
        f"(physics dt={float(model.opt.timestep):.4f}s)",
        flush=True,
    )
    print(
        "[physics] buoyancy setup: "
        f"scale={buoyancy_scale:.3f}, mass={vehicle_mass:.3f}kg, "
        f"rho={rho:.1f}, neutral_volume={neutral_volume:.5f}m^3, "
        f"half_height={half_height:.3f}, model={buoyancy_model}, "
        f"slope_scale={buoyancy_slope_scale:.2f}, "
        f"surface_heave_damping={surface_heave_damping:.2f}, "
        f"heave_damping_scale={heave_damping_scale:.2f}, "
        f"full_heave_damping={full_heave_damping:.2f}, "
        f"water_surface_z={water_surface_z:.3f}",
        flush=True,
    )
    print(
        "[physics] thruster immersion force scale: "
        f"air_scale={thruster_air_force_scale:.3f}, "
        f"half_height={thruster_immersion_half_height_m:.3f}m",
        flush=True,
    )
    if abs(yaw_torque_scale - yaw_torque_scale_config) > 1e-9:
        print(
            f"[physics] yaw torque scale: {yaw_torque_scale:.3f} "
            f"(direct T200 mode bypassed configured {yaw_torque_scale_config:.3f})",
            flush=True,
        )
    else:
        print(f"[physics] yaw torque scale: {yaw_torque_scale:.3f}", flush=True)
    if use_custom_hydrodynamics:
        print(
            "[physics] hydro 6dof: "
            f"source={hydro_cfg.model_source}, "
            f"added_mass={np.array2string(added_mass_diag, precision=3)}, "
            f"lin_damp={np.array2string(linear_damping_diag, precision=3)}, "
            f"quad_damp={np.array2string(quadratic_damping_diag, precision=3)}, "
            f"current_world={np.array2string(water_current_world, precision=3)}",
            flush=True,
        )
        if hydro_cfg.ellipsoid_semi_axes is not None:
            print(
                "[physics] ellipsoid baseline: "
                f"semi_axes={np.array2string(hydro_cfg.ellipsoid_semi_axes, precision=3)}m, "
                f"shape_volume={neutral_volume:.5f}m^3",
                flush=True,
            )
    else:
        print(
            "[physics] current mode uses MuJoCo built-in fluidcoef; "
            "profile 6DOF added_mass/linear_damping/quadratic_damping are inactive. "
            "Active profile knobs here are hydrostatic buoyancy, CoB torque, "
            "full_heave_damping, thruster tuning, mujoco_fluidcoef_scale, "
            "and mujoco_fluidcoef_geom_scales.",
            flush=True,
        )

    # Vertical allocator for roll/pitch control via vertical thrusters.
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
            # Map roll (x) and pitch (y) torque in FLU body axes.
            alloc[:, i] = np.array([tau[0], tau[1]], dtype=np.float64)
        if alloc.shape[1] > 0:
            vert_pinv = np.linalg.pinv(alloc)

    build_vertical_allocator()

    def update_horizontal_stab_gain() -> None:
        """Recompute normalized yaw correction gain from allocator and thrust limit."""
        if not horiz_alloc.size or not horiz_order:
            horiz_stab_yaw_scale["value"] = 1.0
            return
        yaw_basis = np.array([0.0, 0.0, 1.0], dtype=np.float64)
        u = horiz_pinv @ yaw_basis
        tau_per_unit = float(np.dot(horiz_alloc[2], u) * thruster_force_max)
        if abs(tau_per_unit) < 1e-6:
            tau_per_unit = 1.0
        horiz_stab_yaw_scale["value"] = abs(tau_per_unit)

    update_horizontal_stab_gain()

    # Debug/validation cache
    last_flow_world = np.zeros(3, dtype=np.float64)
    last_buoy_force = np.zeros(3, dtype=np.float64)
    last_buoy_point = np.zeros(3, dtype=np.float64)
    prev_rel_nu_body = np.zeros(6, dtype=np.float64)
    thruster_reaction_torque_world = np.zeros(3, dtype=np.float64)
    last_thruster_force_body = np.zeros(3, dtype=np.float64)
    last_thruster_torque_body = np.zeros(3, dtype=np.float64)
    descent_guard_enabled = bool(args.sitl and env_flag("UUV_MJ_DESCENT_CONTRACT_GUARD", True))
    descent_guard_fail_fast = env_flag("UUV_MJ_DESCENT_CONTRACT_FAIL_FAST", False)
    descent_guard_vz_down_mps = float(
        np.clip(float(os.getenv("UUV_MJ_DESCENT_CONTRACT_VZ_DOWN_MPS", "0.05")), 0.005, 1.0)
    )
    descent_guard_start_s = float(
        np.clip(float(os.getenv("UUV_MJ_DESCENT_CONTRACT_START_S", "2.0")), 0.0, 60.0)
    )
    descent_guard_last_warn_wall = {"value": -1.0}
    thruster_debug_path = os.environ.get("UUV_MJ_THRUSTER_DEBUG_CSV", "").strip()
    thruster_debug_file = None
    thruster_debug_next_t = {"value": 0.0}
    if thruster_debug_path:
        try:
            debug_path = Path(thruster_debug_path).expanduser()
            debug_path.parent.mkdir(parents=True, exist_ok=True)
            thruster_debug_file = debug_path.open("w", encoding="utf-8", buffering=1)
            header = [
                "wall_mono_s",
                "sim_time",
                "lin_vel_body_x",
                "lin_vel_body_y",
                "lin_vel_body_z",
                "ang_vel_body_x",
                "ang_vel_body_y",
                "ang_vel_body_z",
                "base_depth_m",
                "base_vz_down_mps",
                "buoy_force_world_z",
                "weight_force_world_z",
                "net_static_force_world_z",
                "initial_depth_hold_active",
                "thr_force_body_x",
                "thr_force_body_y",
                "thr_force_body_z",
                "thr_torque_body_x",
                "thr_torque_body_y",
                "thr_torque_body_z",
            ]
            header.extend(f"sitl_ch{idx}_pwm" for idx in range(1, 9))
            for thr_name in all_thruster_names:
                header.extend(
                    [
                        f"{thr_name}_servo_norm",
                        f"{thr_name}_target",
                        f"{thr_name}_state",
                        f"{thr_name}_force",
                    ]
                )
            thruster_debug_file.write(",".join(header) + "\n")
            print(f"[debug] thruster CSV: {debug_path}", flush=True)
        except OSError as exc:
            print(f"[debug] failed to open thruster CSV {thruster_debug_path}: {exc}", flush=True)
            thruster_debug_file = None

    def body_velocity_local() -> tuple[np.ndarray, np.ndarray]:
        vel6 = np.zeros(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            model,
            data,
            mujoco.mjtObj.mjOBJ_BODY,
            int(base_id),
            vel6,
            1,
        )
        ang_local = vel6[:3].copy()
        lin_local = vel6[3:].copy()
        return lin_local, ang_local

    def emit_thruster_debug() -> None:
        if thruster_debug_file is None:
            return
        sim_t = float(data.time)
        if sim_t + 1e-9 < thruster_debug_next_t["value"]:
            return
        while sim_t + 1e-9 >= thruster_debug_next_t["value"]:
            thruster_debug_next_t["value"] += 0.05
        lin_vel_body, ang_vel_body = body_velocity_local()
        base_depth_m = water_surface_z - float(base_origin_world()[2])
        base_vz_down_mps = -float(data.qvel[world_qvel_adr + 2])
        weight_force_world_z = -float(vehicle_mass * g)
        net_static_force_world_z = float(last_buoy_force[2] + weight_force_world_z)
        values: list[float] = [
            time.monotonic(),
            sim_t,
            *lin_vel_body.tolist(),
            *ang_vel_body.tolist(),
            base_depth_m,
            base_vz_down_mps,
            float(last_buoy_force[2]),
            weight_force_world_z,
            net_static_force_world_z,
            1.0 if initial_depth_hold["active"] else 0.0,
            *last_thruster_force_body.tolist(),
            *last_thruster_torque_body.tolist(),
        ]
        values.extend(float(value) for value in sitl_servo_pwm_values[:8])
        for thr_name in all_thruster_names:
            values.extend(
                [
                    float(sitl_servo_cmd_norm.get(thr_name, 0.0)),
                    float(thr_target.get(thr_name, 0.0)),
                    float(thr_state.get(thr_name, 0.0)),
                    float(thruster_force_cmd.get(thr_name, 0.0)),
                ]
            )
        thruster_debug_file.write(",".join(f"{value:.9g}" for value in values) + "\n")

    def enforce_descent_contract() -> None:
        if not descent_guard_enabled:
            return
        if initial_depth_hold["active"] or float(data.time) < descent_guard_start_s:
            return
        base_vz_down_mps = -float(data.qvel[world_qvel_adr + 2])
        if base_vz_down_mps < descent_guard_vz_down_mps:
            return

        vertical_pwm = [int(v) for v in sitl_servo_pwm_values[4:8]]
        vertical_pwm_delta = max((abs(v - 1500) for v in vertical_pwm), default=0)
        base_rot = data.xmat[base_id].reshape(3, 3)
        thruster_force_world_z = float((base_rot @ last_thruster_force_body)[2])
        weight_force_world_z = -float(vehicle_mass * g)
        net_static_force_world_z = float(last_buoy_force[2] + weight_force_world_z)

        if vertical_pwm_delta <= 12 and abs(thruster_force_world_z) <= 1.0:
            if net_static_force_world_z < -1.0:
                cause = "physics_negative_buoyancy_or_partial_submergence"
            else:
                cause = "neutral_pwm_with_existing_down_velocity"
        elif thruster_force_world_z < -1.0:
            cause = "controller_or_mapping_is_commanding_down_force"
        else:
            cause = "controller_is_braking_or_force_sign_needs_review"

        now_wall = time.monotonic()
        if now_wall - descent_guard_last_warn_wall["value"] < 1.0:
            return
        descent_guard_last_warn_wall["value"] = now_wall
        message = (
            "[descent-contract] "
            f"cause={cause} "
            f"depth={water_surface_z - float(base_origin_world()[2]):.3f}m "
            f"vz_down={base_vz_down_mps:+.3f}m/s "
            f"json_pwm5_8={tuple(vertical_pwm)} "
            f"thruster_force_z={thruster_force_world_z:+.3f}N "
            f"buoy_z={float(last_buoy_force[2]):+.3f}N "
            f"weight_z={weight_force_world_z:+.3f}N "
            f"net_static_z={net_static_force_world_z:+.3f}N"
        )
        if descent_guard_fail_fast:
            raise RuntimeError(message)
        print(message, flush=True)

    def force_from_shaped_command(name: str, command_shaped: float, gain: float) -> float:
        if abs(command_shaped) <= 1e-9:
            return 0.0
        if perf_cfg.get("active") and perf_cfg.get("force").size > 0:
            if perf_cfg.get("direct"):
                return float(
                    pwm_to_force_from_perf(command_shaped)
                    * float(thruster_direct_scale.get(name, 1.0))
                )
            return float(pwm_to_force_from_perf(command_shaped) * gain)

        magnitude = abs(command_shaped)
        if command_shaped >= 0.0:
            force_mag = scaled_polynomial_force(
                magnitude,
                thruster_global["forward_poly"],
                thruster_force_max,
            )
            return float(force_mag * gain)

        reverse_asymmetry = thruster_reverse_asymmetry.get(name)
        if reverse_asymmetry is None:
            reverse_asymmetry = thruster_global["reverse_asymmetry"]
        reverse_force_max = thruster_force_max * float(np.clip(reverse_asymmetry, 0.1, 1.5))
        force_mag = scaled_polynomial_force(
            magnitude,
            thruster_global["reverse_poly"],
            reverse_force_max,
        )
        return float(-force_mag * gain)

    def thruster_force_immersion_scale(thr_name: str) -> float:
        """Scale plant force by the actual thruster site water immersion."""
        sid = thruster_site_ids.get(thr_name, -1)
        if sid < 0:
            return 1.0

        site_z = float(data.site_xpos[sid, 2])
        site_depth_m = float(water_surface_z - site_z)
        water_fraction = submerged_fraction(
            site_depth_m,
            thruster_immersion_half_height_m,
            buoyancy_model,
        )
        water_fraction = float(np.clip(water_fraction, 0.0, 1.0))
        return float(thruster_air_force_scale + (1.0 - thruster_air_force_scale) * water_fraction)

    def update_thruster_forces(_dt: float) -> None:
        nonlocal thruster_reaction_torque_world, last_thruster_force_body, last_thruster_torque_body

        thruster_reaction_torque_world = np.zeros(3, dtype=np.float64)
        last_thruster_force_body = np.zeros(3, dtype=np.float64)
        last_thruster_torque_body = np.zeros(3, dtype=np.float64)
        base_rot = data.xmat[base_id].reshape(3, 3)
        com_body = model.body_ipos[base_id].copy()
        deadzone = float(np.clip(thruster_global["deadzone"], 0.0, 0.95))
        global_tau_up = float(max(thruster_global["tau_up"], 1e-4))
        global_tau_down = float(max(thruster_global["tau_down"], 1e-4))
        command_limit = float(np.clip(thruster_global["command_limit"], deadzone + 1e-3, 1.0))
        reaction_torque_gain = float(max(thruster_global["reaction_torque_gain"], 0.0))

        for name in all_thruster_names:
            aid = act[name]
            lo, hi = ctrlrange[aid]
            gain = float(thruster_scale.get(name, 1.0))
            target_norm = float(np.clip(thr_target[name], -1.0, 1.0))
            tau_up = float(thruster_tau_up[name] if thruster_tau_up[name] is not None else global_tau_up)
            tau_down = float(thruster_tau_down[name] if thruster_tau_down[name] is not None else global_tau_down)
            thr_state[name] = first_order_response(thr_state[name], target_norm, _dt, tau_up, tau_down)
            if perf_cfg.get("active") and perf_cfg.get("direct"):
                shaped_cmd = float(np.clip(thr_state[name], -1.0, 1.0))
            else:
                shaped_cmd = shape_thruster_command(thr_state[name], deadzone, command_limit)
            force = force_from_shaped_command(name, shaped_cmd, gain)
            force *= thruster_force_immersion_scale(name)
            force = float(np.clip(force, lo, hi))
            data.ctrl[aid] = force
            thruster_force_cmd[name] = force

            fdir = model.actuator_gear[aid, :3]
            fdir = fdir / (np.linalg.norm(fdir) + 1e-9)
            sid = thruster_site_ids.get(name, -1)
            r_body = model.site_pos[sid].copy() - com_body if sid >= 0 else np.zeros(3, dtype=np.float64)
            force_body = fdir * force
            last_thruster_force_body += force_body
            last_thruster_torque_body += np.cross(r_body, force_body)
            world_dir = base_rot @ fdir
            thruster_reaction_torque_world += (
                -prop_spin_sign.get(name, 1.0) * world_dir * force * reaction_torque_gain
            )
            if yaw_torque_scale > 1.0 and name in PHYSICAL_YAW_THRUSTERS:
                if sid >= 0:
                    tau_body = np.cross(r_body, fdir * force)
                    extra_tau_body = np.array(
                        [0.0, 0.0, tau_body[2] * (yaw_torque_scale - 1.0)],
                        dtype=np.float64,
                    )
                    thruster_reaction_torque_world += base_rot @ extra_tau_body

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

    def apply_direct_command_targets() -> tuple[float, float, float, float]:
        """Convert recent ROS bridge commands into thruster targets."""
        with cmd_lock:
            forward = cmd["forward"]
            heave = cmd["heave"]
            yaw = cmd["yaw"]
            sway = cmd["sway"]

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
        return forward, sway, yaw, heave

    def apply_underwater_wrench(_dt: float) -> None:
        """Apply buoyancy plus 6-DOF underwater hydrodynamics in body coordinates."""
        nonlocal last_flow_world, last_buoy_force, last_buoy_point, prev_rel_nu_body

        data.xfrc_applied[base_id, :] = 0.0
        com = data.xipos[base_id].copy()
        base_rot = data.xmat[base_id].reshape(3, 3)
        base_origin = base_origin_world()
        lin_vel_body, ang_vel_body = body_velocity_local()
        current_body = base_rot.T @ water_current_world
        rel_lin_vel_body = lin_vel_body - current_body
        rel_lin_vel_world = base_rot @ rel_lin_vel_body
        cob = data.site_xpos[cob_site_id].copy() if cob_site_id >= 0 else com
        depth = water_surface_z - float(base_origin[2])
        submerged = submerged_fraction(depth, half_height, buoyancy_model)
        buoyancy_submerged = submerged_fraction(depth * buoyancy_slope_scale, half_height, buoyancy_model)
        buoy_tau_world = np.zeros(3, dtype=np.float64)
        buoy_force_world = np.zeros(3, dtype=np.float64)
        buoy_point = cob.copy()

        if buoyancy_points:
            total_share = float(sum(point.share for point in buoyancy_points))
            if total_share <= 1e-9:
                total_share = float(len(buoyancy_points))
            weighted_submerged = 0.0
            weighted_buoyancy_submerged = 0.0
            weighted_point = np.zeros(3, dtype=np.float64)
            for point in buoyancy_points:
                share = point.share / max(total_share, 1e-9)
                volume_point_local = point.pos.copy()
                volume_point_world = base_origin + base_rot @ volume_point_local
                force_point_local = point.pos.copy()
                force_point_local[0] += cob_longitudinal_offset
                force_point_local[2] += cob_vertical_offset
                force_point_world = base_origin + base_rot @ force_point_local
                # CoB offsets are restoring-torque tuning offsets.  They should
                # not move the volume sample used to decide how much water is
                # displaced; otherwise a large restoring offset can incorrectly
                # turn buoyancy off near the surface.
                point_depth = water_surface_z - float(volume_point_world[2])
                point_submerged = submerged_fraction(point_depth, point.half_height, buoyancy_model)
                point_buoyancy_submerged = submerged_fraction(
                    point_depth * buoyancy_slope_scale,
                    point.half_height,
                    buoyancy_model,
                )
                point_buoyancy = (
                    rho
                    * g
                    * neutral_volume
                    * share
                    * point_buoyancy_submerged
                    * buoyancy_scale
                )
                point_force_world = np.array([0.0, 0.0, point_buoyancy], dtype=np.float64)
                buoy_force_world += point_force_world
                weighted_submerged += share * point_submerged
                weighted_buoyancy_submerged += share * point_buoyancy_submerged
                weighted_point += point_buoyancy * force_point_world
                if abs(cob_torque_scale) > 1e-9:
                    buoy_tau_world += np.cross(force_point_world - com, point_force_world) * cob_torque_scale
            submerged = float(np.clip(weighted_submerged, 0.0, 1.0))
            buoyancy_submerged = float(np.clip(weighted_buoyancy_submerged, 0.0, 1.0))
            total_buoyancy = float(np.linalg.norm(buoy_force_world))
            if total_buoyancy > 1e-9:
                buoy_point = weighted_point / total_buoyancy
            else:
                buoy_point = cob.copy()
        elif body_components:
            total_share = float(sum(component.buoyancy_share for component in body_components))
            if total_share <= 1e-9:
                total_share = float(sum(component.mass for component in body_components))
            weighted_submerged = 0.0
            weighted_buoyancy_submerged = 0.0
            weighted_point = np.zeros(3, dtype=np.float64)
            for component in body_components:
                share = component.buoyancy_share / max(total_share, 1e-9)
                volume_point_local = component.buoyancy_pos.copy()
                volume_point_world = base_origin + base_rot @ volume_point_local
                force_point_local = component.buoyancy_pos.copy()
                force_point_local[0] += cob_longitudinal_offset
                force_point_local[2] += cob_vertical_offset
                force_point_world = base_origin + base_rot @ force_point_local
                component_half_height = float(max(component.size[2], 1e-4))
                component_depth = water_surface_z - float(volume_point_world[2])
                component_submerged = submerged_fraction(component_depth, component_half_height, buoyancy_model)
                component_buoyancy_submerged = submerged_fraction(
                    component_depth * buoyancy_slope_scale,
                    component_half_height,
                    buoyancy_model,
                )
                component_buoyancy = (
                    rho
                    * g
                    * neutral_volume
                    * share
                    * component_buoyancy_submerged
                    * buoyancy_scale
                )
                component_force_world = np.array([0.0, 0.0, component_buoyancy], dtype=np.float64)
                buoy_force_world += component_force_world
                weighted_submerged += share * component_submerged
                weighted_buoyancy_submerged += share * component_buoyancy_submerged
                weighted_point += component_buoyancy * force_point_world
                if abs(cob_torque_scale) > 1e-9:
                    buoy_tau_world += np.cross(force_point_world - com, component_force_world) * cob_torque_scale
            submerged = float(np.clip(weighted_submerged, 0.0, 1.0))
            buoyancy_submerged = float(np.clip(weighted_buoyancy_submerged, 0.0, 1.0))
            total_buoyancy = float(np.linalg.norm(buoy_force_world))
            if total_buoyancy > 1e-9:
                buoy_point = weighted_point / total_buoyancy
            else:
                buoy_point = cob.copy()
        else:
            buoyancy_blend = buoyancy_point_blend * buoyancy_submerged
            buoy_point = ((1.0 - buoyancy_blend) * com) + (buoyancy_blend * cob)
            buoy = rho * g * neutral_volume * buoyancy_submerged * buoyancy_scale
            buoy_force_world = np.array([0.0, 0.0, buoy], dtype=np.float64)
            if abs(cob_torque_scale) > 1e-9:
                buoy_tau_world = np.cross(buoy_point - com, buoy_force_world) * cob_torque_scale

        # MuJoCo xfrc_applied layout: [force_xyz, torque_xyz].
        data.xfrc_applied[base_id, 0:3] += buoy_force_world
        data.xfrc_applied[base_id, 3:6] += buoy_tau_world

        surface_weight = max(0.0, 4.0 * submerged * (1.0 - submerged))
        if surface_heave_damping > 1e-9 and surface_weight > 1e-9:
            surface_force_world = np.array(
                [0.0, 0.0, -surface_heave_damping * surface_weight * float(rel_lin_vel_world[2])],
                dtype=np.float64,
            )
            data.xfrc_applied[base_id, 0:3] += surface_force_world

        if (not use_custom_hydrodynamics) and full_heave_damping > 1e-9 and submerged > 1e-9:
            full_heave_force_world = np.array(
                [0.0, 0.0, -full_heave_damping * submerged * float(rel_lin_vel_world[2])],
                dtype=np.float64,
            )
            data.xfrc_applied[base_id, 0:3] += full_heave_force_world

        if use_custom_hydrodynamics:
            rel_flow_world = water_current_world - (base_rot @ lin_vel_body)
            nu_rel_body = np.concatenate((rel_lin_vel_body, ang_vel_body))

            if _dt > 0.0:
                rel_acc_body = (nu_rel_body - prev_rel_nu_body) / max(_dt, 1e-6)
            else:
                rel_acc_body = np.zeros(6, dtype=np.float64)
            prev_rel_nu_body = nu_rel_body.copy()

            immersed_added_mass = added_mass_diag * submerged
            immersed_linear_damping = linear_damping_diag * submerged
            immersed_quadratic_damping = quadratic_damping_diag * submerged
            immersed_linear_damping[2] *= heave_damping_scale
            immersed_quadratic_damping[2] *= heave_damping_scale

            hydro_wrench_body = np.zeros(6, dtype=np.float64)
            if np.any(immersed_added_mass > 1e-9):
                hydro_wrench_body -= immersed_added_mass * rel_acc_body
                hydro_wrench_body -= added_mass_coriolis(immersed_added_mass, nu_rel_body) @ nu_rel_body

            hydro_wrench_body -= immersed_linear_damping * nu_rel_body
            hydro_wrench_body -= immersed_quadratic_damping * np.abs(nu_rel_body) * nu_rel_body

            data.xfrc_applied[base_id, 0:3] += base_rot @ hydro_wrench_body[:3]
            data.xfrc_applied[base_id, 3:6] += base_rot @ hydro_wrench_body[3:]
            last_flow_world = rel_flow_world
        else:
            prev_rel_nu_body[:] = 0.0
            last_flow_world = np.zeros(3, dtype=np.float64)

        data.xfrc_applied[base_id, 3:6] += thruster_reaction_torque_world

        last_buoy_force = buoy_force_world
        last_buoy_point = buoy_point

    def viewer_key_callback(keycode):
        if args.enable_viewer_pause and keycode == 32:
            paused_flag["value"] = not paused_flag["value"]
        if keycode in (76, 108):  # L or l
            show_thruster_labels["value"] = not show_thruster_labels["value"]
            if show_thruster_labels["value"]:
                show_viewer_debug["value"] = True
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

    def publish_ros_once() -> None:
        nonlocal ros_bridge
        if ros_bridge is None:
            return
        try:
            if hasattr(ros_bridge, "set_sitl_initial_depth_hold_active"):
                ros_bridge.set_sitl_initial_depth_hold_active(bool(initial_depth_hold["active"]))
            ros_bridge.publish(data)
        except Exception as exc:
            print(f"[ros2] publish failed, disabling bridge: {exc}", flush=True)
            ros_bridge.shutdown()
            ros_bridge = None

    def publish_qgc_video_once() -> None:
        nonlocal qgc_video_next_t, qgc_video_streamer, qgc_video_renderer, qgc_video_share_bridge
        if qgc_video_streamer is None:
            return
        if data.time + 1e-9 < qgc_video_next_t:
            return
        qgc_video_next_t = data.time + qgc_video_dt
        try:
            rgb = None
            if qgc_video_share_bridge and ros_bridge is not None:
                try:
                    rgb = ros_bridge.render_camera_rgb("stereo_left", data)
                except Exception as exc:
                    print(
                        f"[qgc_video] shared renderer failed, falling back to dedicated renderer: {exc}",
                        flush=True,
                    )
                    qgc_video_share_bridge = False
            if rgb is None:
                if qgc_video_renderer is None:
                    qgc_video_renderer = mujoco.Renderer(
                        model,
                        height=int(args.qgc_video_height),
                        width=int(args.qgc_video_width),
                    )
                qgc_video_renderer.update_scene(data, camera="stereo_left")
                rgb = qgc_video_renderer.render()
            if rgb is not None:
                qgc_video_streamer.write(np.ascontiguousarray(rgb))
        except Exception as exc:
            print(f"[qgc_video] stream failed, disabling video: {exc}", flush=True)
            qgc_video_streamer.close()
            qgc_video_streamer = None

    def run_step(is_paused: bool, publish_ros: bool = True) -> tuple[float, float, float, float]:
        """Run one control + physics + publish cycle."""
        nonlocal ros_bridge
        if ros_bridge is not None:
            try:
                ros_bridge.spin_once()
            except Exception as exc:
                print(f"[ros2] spin_once failed, disabling bridge: {exc}", flush=True)
                ros_bridge.shutdown()
                ros_bridge = None

        thruster_due = thruster_update_due()

        if args.sitl:
            return run_sitl_step(is_paused, publish_ros, thruster_due)

        forward, sway, yaw, heave = apply_direct_command_targets()
        thr_dt = model.opt.timestep if not is_paused else 0.0
        if thruster_due:
            update_thruster_forces(thr_dt)
        update_propeller_visuals(thr_dt)

        apply_initial_depth_hold()
        apply_underwater_wrench(model.opt.timestep if not is_paused else 0.0)
        emit_thruster_debug()

        if not is_paused:
            mujoco.mj_step(model, data)
            apply_initial_depth_hold()
        if publish_ros:
            publish_ros_once()
        return forward, sway, yaw, heave

    def run_sitl_step(
        is_paused: bool,
        publish_ros: bool,
        thruster_due: bool,
    ) -> tuple[float, float, float, float]:
        """Run the lean SITL path where ArduPilot owns stabilization and depth hold."""
        if args.sitl:
            now = time.monotonic()
            ros_cmd_active = (
                sitl_allow_direct_cmd
                and ros_cmd_last_wall["value"] > 0.0
                and (now - ros_cmd_last_wall["value"]) <= ros_cmd_timeout_s
            )
            if ros_cmd_active:
                forward, sway, yaw, heave = apply_direct_command_targets()
            else:
                stale = (
                    sitl_servo_last_wall["value"] <= 0.0
                    or (now - sitl_servo_last_wall["value"]) > sitl_servo_timeout_s
                )
                if stale:
                    for name in all_thruster_names:
                        thr_target[name] = 0.0
                else:
                    for name in all_thruster_names:
                        thr_target[name] = float(
                            np.clip(sitl_servo_cmd_norm.get(name, 0.0) * sitl_servo_scale, -1.0, 1.0)
                        )
                forward = 0.0
                sway = 0.0
                yaw = 0.0
                heave = 0.0
            thr_dt = model.opt.timestep if not is_paused else 0.0
            if thruster_due:
                update_thruster_forces(thr_dt)
            update_propeller_visuals(thr_dt)
            apply_initial_depth_hold()
            apply_underwater_wrench(model.opt.timestep if not is_paused else 0.0)
            emit_thruster_debug()
            if not is_paused:
                mujoco.mj_step(model, data)
                apply_initial_depth_hold()
            if publish_ros:
                publish_ros_once()
            publish_qgc_video_once()
            return forward, sway, yaw, heave
        raise AssertionError("run_sitl_step called outside SITL mode")

    if args.headless:
        print("[runtime] headless mode enabled: running without GLFW viewer", flush=True)
        target_dt = float(max(model.opt.timestep, 1e-6))
        next_wall = time.perf_counter()
        while not stop_event.is_set():
            is_paused = paused_flag["value"] if args.enable_viewer_pause else False
            run_step(is_paused)
            # Keep real-time pacing without accumulating extra delay from
            # compute time (important for SITL stabilization responsiveness).
            next_wall += target_dt
            now_wall = time.perf_counter()
            sleep_s = next_wall - now_wall
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_wall = now_wall
        if ros_bridge is not None:
            ros_bridge.shutdown()
        if qgc_video_streamer is not None:
            qgc_video_streamer.close()
        if qgc_video_renderer is not None:
            try:
                qgc_video_renderer.close()
            except Exception:
                pass
        stop_event.set()
        return

    with mujoco.viewer.launch_passive(model, data, key_callback=viewer_key_callback) as viewer:
        has_set_texts = hasattr(viewer, "set_texts")
        target_dt = float(max(model.opt.timestep, 1e-6))
        next_step_wall = time.perf_counter()
        sensor_hz = float(max(args.ros2_sensor_hz, 1.0))
        sensor_dt = 1.0 / sensor_hz
        next_sensor_wall = time.perf_counter()
        viewer_fps = float(np.clip(args.viewer_fps, 10.0, 240.0))
        viewer_dt = 1.0 / viewer_fps
        next_viewer_wall = time.perf_counter()
        max_catchup_steps = max(4, int(round(0.10 / target_dt)))
        max_sensor_catchup = max(2, int(round(0.10 / sensor_dt)))
        forward = 0.0
        sway = 0.0
        yaw = 0.0
        heave = 0.0
        while viewer.is_running() and not stop_event.is_set():
            paused = False
            if args.enable_viewer_pause and hasattr(viewer, "is_paused"):
                flag = viewer.is_paused
                paused = flag() if callable(flag) else bool(flag)
            is_paused = bool(args.enable_viewer_pause and (paused_flag["value"] or paused))
            now_wall = time.perf_counter()
            if is_paused:
                forward, sway, yaw, heave = run_step(True, publish_ros=False)
                next_step_wall = now_wall + target_dt
                next_sensor_wall = now_wall + sensor_dt
            else:
                step_count = 0
                while now_wall >= next_step_wall and step_count < max_catchup_steps:
                    forward, sway, yaw, heave = run_step(False, publish_ros=False)
                    next_step_wall += target_dt
                    step_count += 1
                    now_wall = time.perf_counter()
                if step_count >= max_catchup_steps and now_wall >= next_step_wall:
                    next_step_wall = now_wall + target_dt

                sensor_count = 0
                while ros_bridge is not None and now_wall >= next_sensor_wall and sensor_count < max_sensor_catchup:
                    publish_ros_once()
                    next_sensor_wall += sensor_dt
                    sensor_count += 1
                    now_wall = time.perf_counter()
                if sensor_count >= max_sensor_catchup and now_wall >= next_sensor_wall:
                    next_sensor_wall = now_wall + sensor_dt

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
                        cam.distance = follow_camera_distance
                        cam.elevation = follow_camera_elevation
                        cam.azimuth = follow_camera_azimuth
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
                    up = np.array([0.0, 0.0, 1.0], dtype=np.float64)
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

                # Heavy debug geometry is opt-in; drawing it every viewer frame
                # dominates runtime on many Ubuntu desktops.
                if show_viewer_debug["value"] or show_thruster_labels["value"]:
                    for name in ver_names + yaw_names:
                        sid = thruster_site_ids.get(name, -1)
                        if sid < 0:
                            continue
                        start = data.site_xpos[sid].copy()
                        force = float(data.ctrl[act[name]])
                        fdir = model.actuator_gear[act[name], :3]
                        fdir = fdir / (np.linalg.norm(fdir) + 1e-9)
                        world_dir = base_rot @ fdir
                        draw_dir = world_dir if force >= 0.0 else -world_dir
                        if show_viewer_debug["value"]:
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
                    bar30_id = sensor_site_ids.get("bar30", -1)
                    dvl_id = sensor_site_ids.get("dvl", -1)
                    ping360_id = sensor_site_ids.get("ping360", -1)
                    cam_l_id = sensor_site_ids.get("cam_left", -1)
                    cam_r_id = sensor_site_ids.get("cam_right", -1)
                    if imu_id >= 0:
                        pos = data.site_xpos[imu_id].copy()
                        add_sphere(pos, 0.025, (1.0, 1.0, 0.1, 1.0))
                        add_label("IMU", pos + np.array([0.0, 0.03, 0.0]), (1.0, 1.0, 0.3, 1.0))
                    if bar30_id >= 0:
                        pos = data.site_xpos[bar30_id].copy()
                        add_sphere(pos, 0.022, (0.2, 0.8, 1.0, 1.0))
                        add_label("BAR30", pos + np.array([0.0, 0.03, 0.0]), (0.3, 0.9, 1.0, 1.0))
                    if dvl_id >= 0:
                        pos = data.site_xpos[dvl_id].copy()
                        add_sphere(pos, 0.025, (0.1, 1.0, 1.0, 1.0))
                        add_label("DVL", pos + np.array([0.0, 0.03, 0.0]), (0.3, 1.0, 1.0, 1.0))
                    if ping360_id >= 0:
                        pos = data.site_xpos[ping360_id].copy()
                        add_sphere(pos, 0.022, (0.2, 0.6, 1.0, 1.0))
                        add_label("PING360", pos + np.array([0.0, 0.03, 0.0]), (0.3, 0.7, 1.0, 1.0))
                    if cam_l_id >= 0:
                        pos = data.site_xpos[cam_l_id].copy()
                        add_sphere(pos, 0.022, (1.0, 0.1, 1.0, 1.0))
                        add_label("CAM_L", pos + np.array([0.0, 0.03, 0.0]), (1.0, 0.3, 1.0, 1.0))
                    if cam_r_id >= 0:
                        pos = data.site_xpos[cam_r_id].copy()
                        add_sphere(pos, 0.022, (1.0, 0.5, 0.1, 1.0))
                        add_label("CAM_R", pos + np.array([0.0, 0.03, 0.0]), (1.0, 0.6, 0.2, 1.0))

                if show_viewer_debug["value"]:
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
                overlay_line = "C: follow, 1/2: stereo cam, 0: free, I: sensors, L: thruster labels"
                if args.enable_viewer_pause:
                    overlay_line = "Space: pause, " + overlay_line
                if show_sensor_overlay["value"]:
                    imu_g = sensor_value("imu_gyro")
                    imu_a = sensor_value("imu_acc")
                    dvl_v = sensor_value("dvl_vel_body")
                    dvl_alt = sensor_value("dvl_altitude")
                    depth_pos = sensor_value("depth_pos")
                    if imu_g is not None and imu_a is not None and dvl_v is not None and dvl_alt is not None:
                        depth_suffix = ""
                        if depth_pos is not None and len(depth_pos) >= 3:
                            depth_suffix = f" depth_z={depth_pos[2]:+.2f}"
                        overlay_line = (
                            f"IMU gyro[{imu_g[0]:+.2f},{imu_g[1]:+.2f},{imu_g[2]:+.2f}] "
                            f"acc[{imu_a[0]:+.2f},{imu_a[1]:+.2f},{imu_a[2]:+.2f}] "
                            f"DVL vel[{dvl_v[0]:+.2f},{dvl_v[1]:+.2f},{dvl_v[2]:+.2f}] alt={dvl_alt[0]:.2f}m"
                            f"{depth_suffix}"
                        )
                viewer.set_texts([
                    (
                        None,
                        None,
                        f"Cmd fwd {forward:+.1f} sway {sway:+.1f} yaw {yaw:+.1f} heave {heave:+.1f} | Cam: {camera_mode['value']}",
                        overlay_line,
                    )
                ])

            viewer.sync()
            next_viewer_wall += viewer_dt
            now_wall = time.perf_counter()
            sleep_s = next_viewer_wall - now_wall
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_viewer_wall = now_wall

    if ros_bridge is not None:
        ros_bridge.shutdown()
    if qgc_video_streamer is not None:
        qgc_video_streamer.close()
    if qgc_video_renderer is not None:
        try:
            qgc_video_renderer.close()
        except Exception:
            pass
    stop_event.set()


if __name__ == "__main__":
    main()
    
