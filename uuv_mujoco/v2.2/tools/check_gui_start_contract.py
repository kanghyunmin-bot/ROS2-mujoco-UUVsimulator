#!/usr/bin/env python3
"""Smoke checks for GUI Start simulator-stack contracts."""

from __future__ import annotations

import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.config_paths import SIM_STACK_DIR  # noqa: E402
import gui.control_pilot_release as pilot_release  # noqa: E402
from gui.control_update_command_ready import command_ready_for_gui_stack, external_stack_blocks_command_ready  # noqa: E402
import gui.sim_stack_reset_worker as reset_worker  # noqa: E402
import gui.sim_stack_start_runtime as start_runtime  # noqa: E402
import gui.sim_stack_start_state as start_state  # noqa: E402
from gui.sim_stack_env_contract import build_gui_sim_stack_env  # noqa: E402
from gui.sim_stack_initial_depth_args import build_initial_depth_args  # noqa: E402
from gui.sim_stack_launch_command import build_sim_stack_launch_command  # noqa: E402
from gui.sim_stack_launch_target import resolve_sim_stack_launch_target, sim_stack_start_script_error  # noqa: E402
from gui.process_env import default_sim_stack_backend  # noqa: E402
from gui.sim_stack_start_process import StartedSimStackProcess  # noqa: E402
from gui.sim_stack_viewer_args import apply_viewer_args  # noqa: E402


def _assert_equal(actual: object, expected: object, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def _assert_key(env: dict[str, str], key: str, expected: str) -> None:
    _assert_equal(env.get(key), expected, key)


def check_default_gui_start_env() -> None:
    env = build_gui_sim_stack_env({}, backend="docker", sim_stack_dir=SIM_STACK_DIR)
    _assert_key(env, "UUV_RUN_MODE", "closed_loop")
    _assert_key(env, "UUV_EKF_CONTRACT", "althold_baro")
    _assert_key(env, "SITL_EKF3_EXTNAV", "0")
    _assert_key(env, "ROS2_UUV_SITL_EXTNAV_ENABLE", "0")
    _assert_key(env, "ROS2_UUV_REQUIRE_EXTNAV_TX", "0")
    _assert_key(env, "ROS2_UUV_SITL_AUTO_READY", "0")
    _assert_key(env, "ROS2_UUV_SITL_AUTO_READY_MODE", "MANUAL")
    _assert_key(env, "ROS2_UUV_MAVROS_FORWARD_ARM_MODE", "1")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK", "0")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND", "rc_channels_override")
    _assert_key(env, "UUV_GUI_PILOT_CONTROL_MODE", "rc_override")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_PWM_SPAN", "300")
    _assert_key(env, "UUV_GUI_RC_PWM_SPAN", "300")
    _assert_key(env, "ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE", "0")
    _assert_key(env, "ROS2_UUV_SITL_JSON_SERVO_FALLBACK", "1")
    _assert_key(env, "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE", "0")
    _assert_key(env, "UUV_REAL_START_STATE", "0")
    _assert_key(env, "SITL_DEDICATED_COMMAND_MAVLINK", "0")
    _assert_key(env, "ROS2_UUV_COMMAND_LINK_TELEMETRY", "0")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "same")
    _assert_key(env, "ROS2_UUV_SPIN_HZ", "400")
    _assert_key(env, "ROS2_UUV_SITL_MAVLINK_POLL_HZ", "200")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_POLL_HZ", "400")
    _assert_key(env, "SITL_SCHED_LOOP_RATE", "400")
    _assert_key(env, "UUV_ROS2_SENSOR_HZ", "60")
    _assert_key(env, "UUV_THRUSTER_LOOP_HZ", "80")
    _assert_key(env, "UUV_MUJOCO_VIEWER_FPS", "30")
    if "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE" in env:
        raise AssertionError("default GUI Start must not inject initial-depth hold")


def check_native_gui_start_env() -> None:
    env = build_gui_sim_stack_env({}, backend="native", sim_stack_dir=SIM_STACK_DIR)
    _assert_key(env, "UUV_RUN_MODE", "closed_loop")
    _assert_key(env, "UUV_EKF_CONTRACT", "althold_baro")
    _assert_key(env, "SITL_EKF3_EXTNAV", "0")
    _assert_key(env, "SITL_DIRECT_MAVLINK", "1")
    _assert_key(env, "SITL_QGC_OUTPUT_ENABLE", "1")
    _assert_key(env, "SITL_QGC_DIRECT_SERIAL_ENABLE", "1")
    _assert_key(env, "SITL_SERIAL0_UDPCLIENT", "0")
    _assert_key(env, "SITL_DEDICATED_COMMAND_MAVLINK", "1")
    _assert_key(env, "SITL_NO_EXTRA_PORTS", "1")
    _assert_key(env, "SITL_PARAM_COMPAT_FILTER", "1")
    _assert_key(env, "SITL_USE_REAL_PARAM_FILE", "0")
    _assert_key(env, "SITL_WIPE_EEPROM", "1")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "udpin:0.0.0.0:14661")
    _assert_key(env, "ROS2_UUV_SPIN_HZ", "400")
    _assert_key(env, "ROS2_UUV_SITL_MAVLINK_POLL_HZ", "200")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_POLL_HZ", "400")
    _assert_key(env, "SITL_SCHED_LOOP_RATE", "400")
    _assert_key(env, "UUV_ROS2_SENSOR_HZ", "60")
    _assert_key(env, "UUV_THRUSTER_LOOP_HZ", "80")
    _assert_key(env, "UUV_MUJOCO_VIEWER_FPS", "30")
    expected_ardupilot = str(SIM_STACK_DIR.resolve().parents[1] / "ardupilot_sub_stable")
    _assert_key(env, "ARDUPILOT_DIR", expected_ardupilot)


def check_gui_default_backend_is_native() -> None:
    _assert_equal(default_sim_stack_backend(), "native", "default GUI SITL backend")


def check_launcher_mavlink_source_matches_gcs() -> None:
    launch_script = SIM_STACK_DIR / "launch_uuv_sim.sh"
    text = launch_script.read_text(encoding="utf-8")
    if 'SITL_MAVLINK_SOURCE_SYSID="${SITL_MAVLINK_SOURCE_SYSID:-254}"' not in text:
        raise AssertionError("launcher MAVLink source sysid must be bridge-owned by default")
    if 'SITL_MAVLINK_SOURCE_COMPID="${SITL_MAVLINK_SOURCE_COMPID:-240}"' not in text:
        raise AssertionError("launcher MAVLink source compid must be bridge-owned by default")
    if "SITL_MAVLINK_TARGET_COMPID=1" not in text:
        raise AssertionError("launcher MAVLink target compid must address ArduSub autopilot component 1 for arm/mode commands")
    if "append_extra_arg_if_missing \"--sitl-mavlink-source-sysid\"" not in text:
        raise AssertionError("launcher must pass the MAVLink source sysid into run_urdf_full.py")


def check_ardusub_rc_override_heave_trim() -> None:
    start_script = SIM_STACK_DIR / "start_ardusub_sitl_mj311.sh"
    text = start_script.read_text(encoding="utf-8")
    docker_text = (SIM_STACK_DIR / "start_docker_sitl_mujoco_mj311.sh").read_text(encoding="utf-8")
    expected_gcs = 'append_param_if_not_overridden "SYSID_MYGCS" "${SITL_SYSID_MYGCS:-${SITL_MAVLINK_SOURCE_SYSID:-254}}"'
    if expected_gcs not in text:
        raise AssertionError("Docker/QGC startup must bind ArduSub pilot authority to bridge sysid by default")
    expected = 'append_param_if_not_overridden "RC3_TRIM" "${SITL_RC3_TRIM:-1100}"'
    if expected not in text:
        raise AssertionError("Docker/QGC RC override heave requires default RC3_TRIM=1100 for neutral 1500 PWM")
    if "sitl_real_params_filtered_" not in text or "filtered sim-forced duplicate params" not in text:
        raise AssertionError("Docker/QGC startup must filter real-param keys that SITL forcibly overrides")
    if "RC3_TRIM|RC4_DZ|RC5_DZ|RC6_DZ|THR_DZ|JS_GAIN_DEFAULT|JS_GAIN_MIN|JS_GAIN_MAX|JS_GAIN_STEPS|JS_THR_GAIN" not in text:
        raise AssertionError("Docker/QGC pilot-input params must override the real-robot replay dump")
    expected = 'append_param_if_not_overridden "JS_GAIN_DEFAULT" "${SITL_JS_GAIN_DEFAULT:-0.5}"'
    if expected not in text:
        raise AssertionError("GUI/QGC MANUAL_CONTROL must use ArduSub's live-control joystick gain")
    expected = 'append_param_if_not_overridden "JS_GAIN_MAX" "${SITL_JS_GAIN_MAX:-1.0}"'
    if expected not in text:
        raise AssertionError("GUI/QGC MANUAL_CONTROL must not inherit replay-only JS_GAIN_MAX=2.0")
    expected = 'append_param_if_not_overridden "JS_GAIN_STEPS" "${SITL_JS_GAIN_STEPS:-1}"'
    if expected not in text:
        raise AssertionError("GUI/QGC MANUAL_CONTROL gain buttons must not shift live-control scale")
    expected = 'append_param_if_not_overridden "ARMING_CHECK" "${SITL_ARMING_CHECK:-0}"'
    if expected not in text:
        raise AssertionError("Docker/QGC arm path must not inherit real-vehicle ARMING_CHECK prearm blocks")
    expected = 'append_param_if_not_overridden "FS_GCS_ENABLE" "${SITL_FS_GCS_ENABLE:-0}"'
    if expected not in text:
        raise AssertionError("Docker/QGC arm path must disable real-vehicle GCS failsafe disarm")
    expected = 'append_param_if_not_overridden "FS_PILOT_INPUT" "${SITL_FS_PILOT_INPUT:-0}"'
    if expected not in text:
        raise AssertionError("Docker/QGC arm path must disable real-vehicle pilot-input failsafe disarm")
    expected = 'append_param_if_not_overridden "BRD_SAFETYENABLE" "${SITL_BRD_SAFETYENABLE:-0}"'
    if expected not in text:
        raise AssertionError("Docker/QGC arm path must force the SITL safety switch off")
    expected = 'export SITL_DIRECT_MAVLINK="${SITL_DIRECT_MAVLINK:-0}"'
    if expected not in docker_text:
        raise AssertionError("Docker/QGC must use dist-compatible MAVProxy fan-out by default")
    expected = 'export ROS2_UUV_QGC_MAVLINK_RELAY_ENABLE="${ROS2_UUV_QGC_MAVLINK_RELAY_ENABLE:-0}"'
    if expected not in docker_text:
        raise AssertionError("Docker/QGC must not duplicate MAVProxy QGC fan-out through the host bridge relay by default")
    expected = 'export SITL_QGC_DIRECT_SERIAL_ENABLE="${SITL_QGC_DIRECT_SERIAL_ENABLE:-0}"'
    if expected not in docker_text:
        raise AssertionError("Docker/QGC direct serial1 must be opt-in, not the default")
    expected = 'export SITL_QGC_CONTROL_AUTHORITY="${SITL_QGC_CONTROL_AUTHORITY:-1}"'
    if expected not in docker_text:
        raise AssertionError("Docker/QGC must accept QGC joystick/arm commands by default")
    if 'QGC_LINK="MAVProxy fan-out ${SITL_QGC_HOST}:${SITL_QGC_PORT}"' not in docker_text:
        raise AssertionError("Docker/QGC startup handoff must report the default MAVProxy fan-out QGC link")
    expected = 'append_param_if_not_overridden "SERIAL1_PROTOCOL" "2"'
    if expected not in text:
        raise AssertionError("direct mode must still be able to expose QGC on serial1 when explicitly enabled")
    expected = 'append_param_if_not_overridden "SR1_PARAMS" "${SITL_QGC_SR1_PARAMS:-2}"'
    if expected not in text:
        raise AssertionError("QGC direct serial1 stream must expose parameter traffic")
    expected = 'append_param_if_not_overridden "SR2_PARAMS" "${SITL_MUJOCO_SR2_PARAMS:-0}"'
    if expected not in text:
        raise AssertionError("MuJoCo serial2 stream must keep parameter traffic off by default")
    if 'SERIAL_ARGS+=" --serial1=udpclient:${SITL_QGC_HOST}:${SITL_QGC_PORT}"' not in text:
        raise AssertionError("direct serial1 QGC path must be wired when enabled")
    if 'append_param_if_not_overridden "SERIAL1_PROTOCOL" "-1"' not in text:
        raise AssertionError("default QGC path must not leave an unwired serial1 MAVLink port active")
    relay_path = SIM_STACK_DIR / "bridge" / "qgc_mavlink_relay.py"
    relay_text = relay_path.read_text(encoding="utf-8")
    if "relay_ap_message_to_qgc" not in relay_text or "drain_qgc_mavlink_relay" not in relay_text:
        raise AssertionError("QGC MAVLink relay must be two-way: ArduSub telemetry out, QGC requests back")
    expected = 'append_param_if_not_overridden "SERIAL2_PROTOCOL" "2"'
    if expected not in text:
        raise AssertionError("MuJoCo command/RC override MAVLink must stay on a dedicated serial2 link")
    expected = '--sitl-mavlink-endpoint "udpin:0.0.0.0:${SITL_MUJOCO_MAV_PORT:-14660}"'
    if expected not in docker_text:
        raise AssertionError("MuJoCo bridge must keep listening on the serial2 MAVLink port")


def check_qgc_default_sim_profile_contract() -> None:
    profile_path = SIM_STACK_DIR / "config" / "sim_profiles.json"
    payload = json.loads(profile_path.read_text(encoding="utf-8"))
    current = payload.get("current", {})
    heave_damping = float(current.get("heave_extra_damping_n_per_mps", 0.0))
    if abs(heave_damping) > 1.0e-9:
        raise AssertionError("QGC default must not include replay-only heave extra damping")
    vertical_lift = float(current.get("hydro_vertical_lift_coeff", 0.0))
    if abs(vertical_lift) > 1.0e-9:
        raise AssertionError("QGC default must not include replay-only vertical lift")
    yawrate_heave_pos = float(current.get("hydro_yawrate_heave_pos_coeff", 0.0))
    yawrate_heave_neg = float(current.get("hydro_yawrate_heave_neg_coeff", 0.0))
    if abs(yawrate_heave_pos) > 1.0e-9 or abs(yawrate_heave_neg) > 1.0e-9:
        raise AssertionError("QGC default must not include replay-only yaw-rate/heave coupling")

    yaw_scale = float(current.get("yaw_torque_scale", 1.0))
    if abs(yaw_scale - 1.0) > 1.0e-9:
        raise AssertionError("QGC default must not include hidden yaw torque amplification")

    yaw_thruster_scales = current.get("yaw_torque_thruster_scales", {})
    if isinstance(yaw_thruster_scales, dict):
        bad = {
            name: value
            for name, value in yaw_thruster_scales.items()
            if str(name).startswith("yaw_") and abs(float(value) - 1.0) > 1.0e-9
        }
        if bad:
            raise AssertionError(f"QGC default yaw torque scales must be symmetric unity: {bad}")


def check_command_link_same_opt_out() -> None:
    env = build_gui_sim_stack_env(
        {"SITL_DEDICATED_COMMAND_MAVLINK": "0"},
        backend="docker",
        sim_stack_dir=SIM_STACK_DIR,
    )
    _assert_key(env, "SITL_DEDICATED_COMMAND_MAVLINK", "0")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "same")


def check_dedicated_command_link_opt_in() -> None:
    env = build_gui_sim_stack_env(
        {"SITL_DEDICATED_COMMAND_MAVLINK": "1"},
        backend="docker",
        sim_stack_dir=SIM_STACK_DIR,
    )
    _assert_key(env, "SITL_DEDICATED_COMMAND_MAVLINK", "1")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "udpin:0.0.0.0:14661")


def check_default_initial_depth_args() -> None:
    initial_depth = build_initial_depth_args({}, launch_extra_args=())
    _assert_equal(initial_depth.args, ("--initial-bar30-depth-m", "auto"), "default initial-depth args")
    _assert_equal(initial_depth.events, ("sim initial depth: bar30=auto m",), "default initial-depth event")
    if "--hold-initial-depth-until-release" in initial_depth.args:
        raise AssertionError("default GUI Start must not hold initial depth until release")


def check_default_qgc_video_args() -> None:
    args: list[str] = []
    events: list[str] = []
    apply_viewer_args(args, events, {}, platform_name="darwin")
    if "--no-qgc-video" not in args:
        raise AssertionError("GUI Start must keep QGC video off by default")

    args = []
    events = []
    apply_viewer_args(args, events, {"UUV_GUI_QGC_VIDEO": "1"}, platform_name="darwin")
    if "--qgc-video" not in args:
        raise AssertionError("UUV_GUI_QGC_VIDEO=1 must keep the explicit video-on opt-in")

    args = []
    events = []
    apply_viewer_args(args, events, {"UUV_GUI_QGC_VIDEO": "0"}, platform_name="darwin")
    if "--no-qgc-video" not in args:
        raise AssertionError("UUV_GUI_QGC_VIDEO=0 must keep the explicit video-off opt-out")


def check_launch_targets_exist() -> None:
    docker_target = resolve_sim_stack_launch_target("docker")
    native_target = resolve_sim_stack_launch_target("native")
    if sim_stack_start_script_error(docker_target.start_script) is not None:
        raise AssertionError(f"Docker start script unavailable: {docker_target.start_script}")
    if sim_stack_start_script_error(native_target.start_script) is not None:
        raise AssertionError(f"native start script unavailable: {native_target.start_script}")


class _Owner:
    def __init__(self, *, tracked: bool, external_cached: bool) -> None:
        self._tracked = tracked
        self._external_sim_stack_running_cached = external_cached

    def _tracked_sim_stack_running(self) -> bool:
        return self._tracked


class _Node:
    def __init__(self, calls: list[str]) -> None:
        self._calls = calls

    def push_event(self, event: str) -> None:
        self._calls.append(f"event:{event}")

    def publish_rc_release(self) -> None:
        self._calls.append("publish_rc_release")

    def publish_rc_override(self, **_kwargs: object) -> None:
        self._calls.append("publish_rc_override")

    def publish_manual_control(self, **_kwargs: object) -> None:
        self._calls.append("publish_manual_control")


class _Var:
    def __init__(self, value: object = None) -> None:
        self.value = value

    def get(self) -> object:
        return self.value

    def set(self, value: object) -> None:
        self.value = value


class _LaunchOwner:
    def __init__(self, env: dict[str, str] | None = None) -> None:
        self.env = env or {}
        self.calls: list[str] = []
        self.node = _Node(self.calls)

    def _append_mavros_surface_args(self, _cmd: list[str]) -> None:
        return None

    def _env_flag(self, key: str, default: bool) -> bool:
        raw = self.env.get(key)
        if raw is None:
            return default
        return str(raw).lower() in {"1", "true", "yes", "on", "enable", "enabled"}

    def _arg_present(self, args: list[str], needle: str) -> bool:
        return needle in args

    def _normalized_sim_extra_args(self, extra_args: list[str] | None) -> list[str]:
        return list(extra_args or [])

    def _append_initial_depth_args(self, _cmd: list[str], _extra_args: list[str]) -> None:
        return None


class _StartProc:
    def poll(self) -> None:
        return None


class _StartOwner:
    def __init__(self, *, backend: str = "native") -> None:
        self.calls: list[str] = []
        self.node = _Node(self.calls)
        self.backend = backend
        self.rc_override_enabled = _Var(True)
        self._sim_stack_process = None
        self._sim_stack_log_path = None
        self._sim_stack_thread = None
        self._sim_stack_owned_by_gui = False
        self._external_sim_stack_running_cached = False

    def _tracked_sim_stack_running(self) -> bool:
        return False

    def _external_sim_stack_running(self) -> bool:
        return False

    def _sim_stack_backend(self) -> str:
        return self.backend

    def _gui_sim_stack_env(self) -> dict[str, str]:
        self.calls.append("build_env")
        return {"UUV_RUN_MODE": "closed_loop"}

    def _rc_replay_running(self) -> bool:
        return True

    def _stop_rc_replay(self) -> None:
        self.calls.append("stop_rc_replay")

    def _set_sim_stack_status(self, text: str) -> None:
        self.calls.append(f"status:{text}")

    def _refresh_sim_stack_controls(self) -> None:
        self.calls.append("refresh_controls")


class _ResetOwner:
    def __init__(self, *, backend: str = "docker") -> None:
        self.calls: list[str] = []
        self.node = _Node(self.calls)
        self.backend = backend

    def _sim_stack_backend(self) -> str:
        return self.backend

    def _stop_docker_sitl_blocking(self, *, timeout_s: float) -> None:
        self.calls.append(f"docker_stop:{timeout_s:.1f}")

    def _set_sim_stack_status(self, text: str) -> None:
        self.calls.append(f"status:{text}")

    def _wait_for_external_sim_stack_exit(self, *, timeout_s: float) -> bool:
        self.calls.append(f"wait_external_exit:{timeout_s:.1f}")
        return True


class _FakeProc:
    stdout = ["[reset] done\n"]

    def wait(self) -> int:
        return 0


def check_external_stack_blocks_command_ready() -> None:
    if external_stack_blocks_command_ready(tracked=False, external_cached=True):
        raise AssertionError("external non-GUI stack must not block command-ready")
    if external_stack_blocks_command_ready(tracked=True, external_cached=True):
        raise AssertionError("GUI-tracked stack must not be blocked as external")
    text, style = command_ready_for_gui_stack(
        _Owner(tracked=False, external_cached=True),
        "READY: command",
        "Ready.TLabel",
    )
    _assert_equal(text, "READY: command", "external-stack command-ready text")
    _assert_equal(style, "Ready.TLabel", "external-stack command-ready style")


def check_gui_start_uses_dist_like_transport_default() -> None:
    native_cmd = build_sim_stack_launch_command(
        _LaunchOwner(),
        start_script=SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh",
        backend="native",
        extra_args=[],
    )
    if "--sitl-no-rebuild" not in native_cmd:
        raise AssertionError("native GUI Start must skip rebuild and use the existing stable ArduSub binary")
    if "--direct-mavlink" in native_cmd:
        raise AssertionError("native GUI Start must get direct MAVLink from env, not duplicate wrapper args")

    docker_cmd = build_sim_stack_launch_command(
        _LaunchOwner(),
        start_script=SIM_STACK_DIR / "start_docker_sitl_mujoco_mj311.sh",
        backend="docker",
        extra_args=[],
    )
    if "--direct-mavlink" in docker_cmd:
        raise AssertionError("Docker GUI Start must not force direct MAVLink by default")

    opt_in = build_sim_stack_launch_command(
        _LaunchOwner({"UUV_GUI_SITL_DIRECT_MAVLINK": "1"}),
        start_script=SIM_STACK_DIR / "start_docker_sitl_mujoco_mj311.sh",
        backend="docker",
        extra_args=[],
    )
    if "--direct-mavlink" not in opt_in:
        raise AssertionError("explicit UUV_GUI_SITL_DIRECT_MAVLINK=1 must still opt into direct MAVLink")


def check_docker_reset_stops_docker_first() -> None:
    owner = _ResetOwner()
    original_exists = reset_worker.reset_script_exists
    original_open = reset_worker.open_reset_sim_stack_process
    original_refresh = reset_worker.refresh_sim_stack_controls_async
    try:
        reset_worker.reset_script_exists = lambda: True
        reset_worker.open_reset_sim_stack_process = lambda: _FakeProc()
        reset_worker.refresh_sim_stack_controls_async = lambda gui: gui.calls.append("refresh_async")
        reset_worker.run_sim_stack_reset(owner)
    finally:
        reset_worker.reset_script_exists = original_exists
        reset_worker.open_reset_sim_stack_process = original_open
        reset_worker.refresh_sim_stack_controls_async = original_refresh

    expected_prefix = ["event:docker SITL stop requested", "docker_stop:20.0"]
    _assert_equal(owner.calls[:2], expected_prefix, "docker reset sequence")
    if "status:sim: stopped/reset" not in owner.calls:
        raise AssertionError("successful reset must mark sim stopped/reset")


def check_native_reset_uses_reset_script_only() -> None:
    owner = _ResetOwner(backend="native")
    original_exists = reset_worker.reset_script_exists
    original_open = reset_worker.open_reset_sim_stack_process
    original_refresh = reset_worker.refresh_sim_stack_controls_async
    try:
        reset_worker.reset_script_exists = lambda: True
        reset_worker.open_reset_sim_stack_process = lambda: _FakeProc()
        reset_worker.refresh_sim_stack_controls_async = lambda gui: gui.calls.append("refresh_async")
        reset_worker.run_sim_stack_reset(owner)
    finally:
        reset_worker.reset_script_exists = original_exists
        reset_worker.open_reset_sim_stack_process = original_open
        reset_worker.refresh_sim_stack_controls_async = original_refresh

    if any(call.startswith("docker_stop:") for call in owner.calls):
        raise AssertionError("native reset must not stop Docker before running reset script")
    if owner.calls[:2] != ["event:[reset] done", "status:sim: [reset] done"]:
        raise AssertionError(f"native reset sequence must stream reset script output, got {owner.calls[:2]}")
    if "status:sim: stopped/reset" not in owner.calls:
        raise AssertionError("successful native reset must mark sim stopped/reset")


def check_reset_script_cleans_docker_runtime() -> None:
    reset_text = (SIM_STACK_DIR / "reset_uuv_sim.sh").read_text(encoding="utf-8")
    expected = 'kill_pattern "Docker start wrapper" "start_docker_sitl_mujoco_mj311.sh"'
    if expected not in reset_text:
        raise AssertionError("reset must stop stale GUI Docker start wrappers")
    if 'docker compose -f "$COMPOSE_FILE" down --remove-orphans' not in reset_text:
        raise AssertionError("reset must stop Docker SITL containers, not only local host processes")
    if "CURRENT_LINEAGE_PIDS" not in reset_text:
        raise AssertionError("reset must not kill the wrapper process that is currently invoking it")
    if "local_endpoint = endpoint.split(\"->\", 1)[0]" not in reset_text:
        raise AssertionError("reset UDP report must only show local bound ports, not Docker remote target sockets")


def check_native_wrappers_default_to_stable_ardupilot() -> None:
    expected = "ardupilot_sub_stable"
    for script_name in (
        "start_sitl_mujoco_mj311.sh",
        "start_ardusub_sitl_mj311.sh",
        "reset_uuv_sim.sh",
    ):
        text = (SIM_STACK_DIR / script_name).read_text(encoding="utf-8")
        if expected not in text:
            raise AssertionError(f"{script_name} must prefer ardupilot_sub_stable for native SITL")


def check_native_one_shot_uses_dedicated_command_link() -> None:
    start_text = (SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh").read_text(encoding="utf-8")
    ardusub_text = (SIM_STACK_DIR / "start_ardusub_sitl_mj311.sh").read_text(encoding="utf-8")
    launch_text = (SIM_STACK_DIR / "launch_uuv_sim.sh").read_text(encoding="utf-8")
    expected_default = 'SITL_DEDICATED_COMMAND_MAVLINK="${SITL_DEDICATED_COMMAND_MAVLINK:-1}"'
    if expected_default not in start_text or expected_default not in ardusub_text:
        raise AssertionError("native one-shot SITL must default to the dedicated serial4 command link")
    expected_endpoint = 'ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT="${ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT:-udpin:0.0.0.0:${SITL_COMMAND_MAV_PORT}}"'
    if expected_endpoint not in start_text or expected_endpoint not in launch_text:
        raise AssertionError("native one-shot MuJoCo bridge must listen on the dedicated command endpoint")
    if "Command  -> serial4 udpclient" not in ardusub_text:
        raise AssertionError("ArduSub direct startup must wire serial4 to the command endpoint")


def check_gui_start_button_owns_fresh_stack() -> None:
    owner = _StartOwner()
    original_start = start_runtime.start_gui_sim_stack_process
    original_watcher = start_state.start_sim_stack_watcher
    try:
        start_runtime.start_gui_sim_stack_process = lambda owner, target, extra_args, env: StartedSimStackProcess(
            proc=_StartProc(),
            log_path=Path("/tmp/gui_start_stack_fake.log"),
        )
        start_state.start_sim_stack_watcher = lambda owner, proc, log_path: "watcher"
        start_runtime._start_sim_stack(owner)
    finally:
        start_runtime.start_gui_sim_stack_process = original_start
        start_state.start_sim_stack_watcher = original_watcher

    if owner.rc_override_enabled.get() is not False:
        raise AssertionError("GUI Start must disable existing pilot input before launch")
    if owner._sim_stack_process is None:
        raise AssertionError("GUI Start must record the started simulator process")
    if owner._sim_stack_thread != "watcher":
        raise AssertionError("GUI Start must start the simulator log watcher")
    if not owner._sim_stack_owned_by_gui:
        raise AssertionError("GUI Start must mark the new stack as GUI-owned")
    expected_prefix = ["stop_rc_replay", "publish_rc_release", "build_env"]
    _assert_equal(owner.calls[:3], expected_prefix, "GUI Start setup sequence")
    if not any(call.startswith("status:sim: starting SITL/MuJoCo") for call in owner.calls):
        raise AssertionError("GUI Start must show SITL/MuJoCo startup status")


def check_rc_override_release_does_not_send_manual_control() -> None:
    class _ReleaseOwner:
        def __init__(self) -> None:
            self.calls: list[str] = []
            self.node = _Node(self.calls)
            self._pilot_input_release_requested = True
            self._rc_override_prev = True

    owner = _ReleaseOwner()
    pilot_release.publish_rc_release_and_neutral(owner)
    _assert_equal(
        owner.calls,
        ["publish_rc_override", "publish_rc_release"],
        "RC override release sequence",
    )


def main() -> int:
    check_default_gui_start_env()
    check_native_gui_start_env()
    check_gui_default_backend_is_native()
    check_launcher_mavlink_source_matches_gcs()
    check_ardusub_rc_override_heave_trim()
    check_qgc_default_sim_profile_contract()
    check_command_link_same_opt_out()
    check_dedicated_command_link_opt_in()
    check_default_initial_depth_args()
    check_default_qgc_video_args()
    check_launch_targets_exist()
    check_external_stack_blocks_command_ready()
    check_gui_start_uses_dist_like_transport_default()
    check_docker_reset_stops_docker_first()
    check_native_reset_uses_reset_script_only()
    check_reset_script_cleans_docker_runtime()
    check_native_wrappers_default_to_stable_ardupilot()
    check_native_one_shot_uses_dedicated_command_link()
    check_gui_start_button_owns_fresh_stack()
    check_rc_override_release_does_not_send_manual_control()
    print("gui_start_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
