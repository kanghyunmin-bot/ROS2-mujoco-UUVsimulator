#!/usr/bin/env python3
"""Smoke checks for GUI Start simulator-stack contracts."""

from __future__ import annotations

import io
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.config_paths import SIM_STACK_DIR  # noqa: E402
import gui.control_pilot_release as pilot_release  # noqa: E402
from gui.homing_direction_body_adapter import world_vector_to_body  # noqa: E402
from gui.control_update_command_ready import command_ready_for_gui_stack, external_stack_blocks_command_ready  # noqa: E402
import gui.sim_stack_reset_worker as reset_worker  # noqa: E402
import gui.sim_stack_start_runtime as start_runtime  # noqa: E402
import gui.sim_stack_start_state as start_state  # noqa: E402
import gui.web_process_manager as web_process_manager  # noqa: E402
from gui.sim_stack_env_contract import build_gui_sim_stack_env  # noqa: E402
from gui.sim_stack_env_defaults import profile_defaults  # noqa: E402
from gui.sim_stack_initial_depth_args import build_initial_depth_args  # noqa: E402
from gui.sim_stack_launch_command import build_sim_stack_launch_command  # noqa: E402
from gui.sim_stack_launch_target import resolve_sim_stack_launch_target, sim_stack_start_script_error  # noqa: E402
from gui.process_env import default_sim_stack_backend  # noqa: E402
from gui.pinger_sim_profile import (  # noqa: E402
    normalize_sim_start_purpose,
    pinger_sim_environment,
    pinger_sim_launch_args,
)
from gui.ros_package_stack import mavros_launch_command  # noqa: E402
from gui.ros_workspace_setup_paths import workspace_setup_candidates  # noqa: E402
from gui.sim_stack_start_process import StartedSimStackProcess  # noqa: E402
from gui.sim_stack_viewer_args import apply_viewer_args  # noqa: E402
from sim.runtime.simulation_loop_cadence import build_viewer_loop_cadence  # noqa: E402


def _assert_equal(actual: object, expected: object, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def _assert_key(env: dict[str, str], key: str, expected: str) -> None:
    _assert_equal(env.get(key), expected, key)


def check_default_gui_start_env() -> None:
    env = build_gui_sim_stack_env({}, backend="docker", sim_stack_dir=SIM_STACK_DIR)
    _assert_key(env, "UUV_RUN_MODE", "closed_loop")
    _assert_key(env, "UUV_EKF_CONTRACT", "althold_baro")
    _assert_key(env, "SITL_AHRS_EKF_TYPE", "3")
    _assert_key(env, "ROS2_UUV_SITL_JSON_TIMING_MODE", "lockstep")
    _assert_key(env, "SITL_EKF3_EXTNAV", "0")
    _assert_key(env, "ROS2_UUV_SITL_EXTNAV_ENABLE", "0")
    _assert_key(env, "ROS2_UUV_REQUIRE_EXTNAV_TX", "0")
    _assert_key(env, "ROS2_UUV_SITL_AUTO_READY", "0")
    _assert_key(env, "ROS2_UUV_SITL_AUTO_READY_MODE", "MANUAL")
    _assert_key(env, "ROS2_UUV_MAVROS_FORWARD_ARM_MODE", "1")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK", "0")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND", "rc_override")
    _assert_key(env, "UUV_GUI_PILOT_CONTROL_MODE", "rc_override")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_PWM_SPAN", "400")
    _assert_key(env, "UUV_GUI_RC_PWM_SPAN", "400")
    _assert_key(env, "ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE", "0")
    _assert_key(env, "ROS2_UUV_SITL_JSON_SERVO_FALLBACK", "1")
    _assert_key(env, "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE", "0")
    _assert_key(env, "UUV_REAL_START_STATE", "0")
    _assert_key(env, "SITL_DEDICATED_COMMAND_MAVLINK", "0")
    _assert_key(env, "ROS2_UUV_COMMAND_LINK_TELEMETRY", "0")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "same")
    _assert_key(env, "UUV_RUNTIME_PROFILE", "balanced")
    _assert_key(env, "ROS2_UUV_SPIN_HZ", "60")
    _assert_key(env, "ROS2_UUV_SPIN_TIMEOUT_S", "0.001")
    _assert_key(env, "ROS2_UUV_SITL_MAVLINK_POLL_HZ", "40")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_POLL_HZ", "80")
    _assert_key(env, "ROS2_UUV_SITL_POLL_THREAD_HZ", "200")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ", "120")
    _assert_key(env, "SITL_SCHED_LOOP_RATE", "400")
    _assert_key(env, "SITL_MAVLINK_SERVO_HZ_DEFAULT", "30")
    _assert_key(env, "SITL_SERVO_SCALE_DEFAULT", "1.0")
    _assert_key(env, "UUV_HORIZONTAL_DIRECT_GAIN_SCALE", "1.0")
    _assert_key(env, "UUV_VERTICAL_DIRECT_GAIN_SCALE", "1.0")
    _assert_key(env, "SITL_SPEEDUP_DEFAULT", "1")
    _assert_key(env, "SITL_SENSOR_HZ_DEFAULT", "100")
    _assert_key(env, "SITL_THRUSTER_LOOP_HZ_DEFAULT", "100")
    _assert_key(env, "UUV_ROS2_SENSOR_HZ", "100")
    _assert_key(env, "UUV_THRUSTER_LOOP_HZ", "100")
    _assert_key(env, "UUV_MUJOCO_TIMESTEP", "0.0025")
    _assert_key(env, "UUV_COURSE_BUOY_TIMESTEP_GUARD", "1")
    _assert_key(env, "UUV_COURSE_BUOY_UPDATE_HZ", "10")
    _assert_key(env, "UUV_COURSE_BUOY_TRACK_CSV_ENABLE", "0")
    _assert_key(env, "UUV_MUJOCO_VIEWER_FPS", "30")
    _assert_key(env, "UUV_MUJOCO_VIEWER_WIDTH", "1280")
    _assert_key(env, "UUV_MUJOCO_VIEWER_HEIGHT", "720")
    _assert_key(env, "UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC", "1")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_SAMPLE_RATE_HZ", "96000")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_AUDIO_HZ", "23.4375")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_SYNC_HZ", "50")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_STATUS_HZ", "5")
    _assert_key(env, "UUV_MUJOCO_CATCHUP_WINDOW_S", "0.040")
    _assert_key(env, "UUV_MUJOCO_SENSOR_CATCHUP_WINDOW_S", "0.067")
    _assert_key(env, "UUV_MUJOCO_MAX_STEP_LAG_S", "0.040")
    _assert_key(env, "UUV_MUJOCO_MAX_SENSOR_LAG_S", "0.067")
    _assert_key(env, "UUV_MUJOCO_MAX_SLEEP_S", "0.001")
    _assert_key(env, "UUV_MUJOCO_DROP_EXCESS_STEP_LAG", "0")
    if "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE" in env:
        raise AssertionError("default GUI Start must not inject initial-depth hold")


def check_native_gui_start_env() -> None:
    env = build_gui_sim_stack_env({}, backend="native", sim_stack_dir=SIM_STACK_DIR)
    _assert_key(env, "UUV_RUN_MODE", "closed_loop")
    _assert_key(env, "UUV_EKF_CONTRACT", "althold_baro")
    _assert_key(env, "SITL_AHRS_EKF_TYPE", "3")
    _assert_key(env, "ROS2_UUV_SITL_JSON_TIMING_MODE", "lockstep")
    _assert_key(env, "SITL_EKF3_EXTNAV", "0")
    _assert_key(env, "SITL_DIRECT_MAVLINK", "0")
    _assert_key(env, "SITL_QGC_OUTPUT_ENABLE", "1")
    _assert_key(env, "SITL_QGC_DIRECT_SERIAL_ENABLE", "0")
    _assert_key(env, "SITL_SERIAL0_UDPCLIENT", "0")
    _assert_key(env, "SITL_DEDICATED_COMMAND_MAVLINK", "1")
    _assert_key(env, "SITL_NO_EXTRA_PORTS", "1")
    _assert_key(env, "SITL_PARAM_COMPAT_FILTER", "1")
    _assert_key(env, "SITL_USE_REAL_PARAM_FILE", "1")
    _assert_key(env, "SITL_WIPE_EEPROM", "1")
    _assert_key(env, "ROS2_UUV_SITL_JSON_SERVO_FALLBACK", "1")
    _assert_key(env, "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE", "0")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "udpin:0.0.0.0:14661")
    _assert_key(env, "UUV_RUNTIME_PROFILE", "balanced")
    _assert_key(env, "ROS2_UUV_SPIN_HZ", "60")
    _assert_key(env, "ROS2_UUV_SPIN_TIMEOUT_S", "0.001")
    _assert_key(env, "ROS2_UUV_SITL_MAVLINK_POLL_HZ", "40")
    _assert_key(env, "ROS2_UUV_SITL_COMMAND_POLL_HZ", "80")
    _assert_key(env, "ROS2_UUV_SITL_POLL_THREAD_HZ", "200")
    _assert_key(env, "ROS2_UUV_MAVROS_RC_OVERRIDE_FORWARD_HZ", "120")
    _assert_key(env, "SITL_SCHED_LOOP_RATE", "400")
    _assert_key(env, "SITL_MAVLINK_SERVO_HZ_DEFAULT", "30")
    _assert_key(env, "SITL_SERVO_SCALE_DEFAULT", "1.0")
    _assert_key(env, "UUV_HORIZONTAL_DIRECT_GAIN_SCALE", "1.0")
    _assert_key(env, "UUV_VERTICAL_DIRECT_GAIN_SCALE", "1.0")
    _assert_key(env, "SITL_SPEEDUP_DEFAULT", "1")
    _assert_key(env, "SITL_SENSOR_HZ_DEFAULT", "100")
    _assert_key(env, "SITL_THRUSTER_LOOP_HZ_DEFAULT", "100")
    _assert_key(env, "UUV_ROS2_SENSOR_HZ", "100")
    _assert_key(env, "UUV_THRUSTER_LOOP_HZ", "100")
    _assert_key(env, "UUV_MUJOCO_TIMESTEP", "0.0025")
    _assert_key(env, "UUV_COURSE_BUOY_TIMESTEP_GUARD", "1")
    _assert_key(env, "UUV_COURSE_BUOY_UPDATE_HZ", "10")
    _assert_key(env, "UUV_COURSE_BUOY_TRACK_CSV_ENABLE", "0")
    _assert_key(env, "UUV_MUJOCO_VIEWER_FPS", "30")
    _assert_key(env, "UUV_MUJOCO_VIEWER_WIDTH", "1280")
    _assert_key(env, "UUV_MUJOCO_VIEWER_HEIGHT", "720")
    _assert_key(env, "UUV_MUJOCO_VIEWER_STATE_ONLY_SYNC", "1")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_SAMPLE_RATE_HZ", "96000")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_AUDIO_HZ", "23.4375")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_SYNC_HZ", "50")
    _assert_key(env, "ROS2_UUV_HYDROPHONE_STATUS_HZ", "5")
    _assert_key(env, "UUV_MUJOCO_CATCHUP_WINDOW_S", "0.040")
    _assert_key(env, "UUV_MUJOCO_SENSOR_CATCHUP_WINDOW_S", "0.067")
    _assert_key(env, "UUV_MUJOCO_MAX_STEP_LAG_S", "0.040")
    _assert_key(env, "UUV_MUJOCO_MAX_SENSOR_LAG_S", "0.067")
    _assert_key(env, "UUV_MUJOCO_MAX_SLEEP_S", "0.001")
    _assert_key(env, "UUV_MUJOCO_DROP_EXCESS_STEP_LAG", "0")
    workspace = SIM_STACK_DIR.resolve().parents[1]
    stable_ardupilot = workspace / "ardupilot_sub_stable"
    expected_ardupilot = str(stable_ardupilot if stable_ardupilot.exists() else workspace / "ardupilot")
    _assert_key(env, "ARDUPILOT_DIR", expected_ardupilot)


def check_native_gui_env_precedence() -> None:
    """Explicit env beats GUI contract, which beats native defaults."""

    default_env = build_gui_sim_stack_env({}, backend="native", sim_stack_dir=SIM_STACK_DIR)
    _assert_key(default_env, "SITL_USE_REAL_PARAM_FILE", "1")

    explicit_env = build_gui_sim_stack_env(
        {
            "SITL_USE_REAL_PARAM_FILE": "0",
            "ROS2_UUV_SITL_JSON_SERVO_FALLBACK": "0",
            "ROS2_UUV_SITL_JSON_TIMING_MODE": "async",
        },
        backend="native",
        sim_stack_dir=SIM_STACK_DIR,
    )
    _assert_key(explicit_env, "SITL_USE_REAL_PARAM_FILE", "0")
    _assert_key(explicit_env, "ROS2_UUV_SITL_JSON_SERVO_FALLBACK", "0")
    _assert_key(explicit_env, "ROS2_UUV_SITL_JSON_TIMING_MODE", "async")


def check_gui_truth_ahrs_is_forbidden() -> None:
    """Keep GUI starts on EKF3 instead of ArduPilot's perfect SITL AHRS."""

    source = (ROOT / "gui" / "sim_stack_env_forced_ekf.py").read_text(
        encoding="utf-8"
    )
    forbidden_default = '"SITL_AHRS_EKF_TYPE": "10"'
    if forbidden_default in source:
        raise AssertionError(
            "GUI sensor contract must not select AHRS_EKF_TYPE=10 (SITL truth AHRS)"
        )

    for backend in ("docker", "native"):
        for ekf_contract in ("althold_baro", "poshold_extnav"):
            resolved = build_gui_sim_stack_env(
                {
                    "UUV_EKF_CONTRACT": ekf_contract,
                    "UUV_GUI_USE_EXTERNAL_MAVROS": "1",
                },
                backend=backend,
                sim_stack_dir=SIM_STACK_DIR,
            )
            _assert_key(resolved, "SITL_AHRS_EKF_TYPE", "3")


def check_low_profile_closed_loop_cadence_floor() -> None:
    """Low-load mode may reduce rendering, not the ArduSub feedback loop."""

    defaults = profile_defaults({"UUV_RUNTIME_PROFILE": "low"}, "400")
    for key in (
        "SITL_SENSOR_HZ_DEFAULT",
        "SITL_THRUSTER_LOOP_HZ_DEFAULT",
        "UUV_ROS2_SENSOR_HZ",
        "UUV_THRUSTER_LOOP_HZ",
    ):
        _assert_key(defaults, key, "100")


def check_gui_default_backend_is_native() -> None:
    _assert_equal(default_sim_stack_backend(), "native", "default GUI SITL backend")


def check_launcher_mavlink_source_matches_gcs() -> None:
    launch_script = SIM_STACK_DIR / "launch_uuv_sim.sh"
    text = launch_script.read_text(encoding="utf-8")
    if 'SITL_MAVLINK_SOURCE_SYSID="${SITL_MAVLINK_SOURCE_SYSID:-255}"' not in text:
        raise AssertionError("launcher MAVLink source sysid must match ArduSub GCS authority by default")
    if 'SITL_MAVLINK_SOURCE_COMPID="${SITL_MAVLINK_SOURCE_COMPID:-190}"' not in text:
        raise AssertionError("launcher MAVLink source compid must use MAV_COMP_ID_MISSIONPLANNER by default")
    if 'elif [[ "${SITL_DIRECT_MAVLINK:-0}" == "1" ]]' not in text:
        raise AssertionError("native/direct closed-loop must use a dedicated plant input branch")
    if "closed_loop/native-direct: MAVLink SERVO_OUTPUT_RAW is authoritative plant input" not in text:
        raise AssertionError("native/direct closed-loop must use MAVLink SERVO_OUTPUT_RAW as plant input")
    if "SITL_MAVLINK_TARGET_COMPID=1" not in text:
        raise AssertionError("launcher MAVLink target compid must address ArduSub autopilot component 1 for arm/mode commands")
    if "append_extra_arg_if_missing \"--sitl-mavlink-source-sysid\"" not in text:
        raise AssertionError("launcher must pass the MAVLink source sysid into run_urdf_full.py")


def check_ardusub_rc_override_heave_trim() -> None:
    start_script = SIM_STACK_DIR / "start_ardusub_sitl_mj311.sh"
    text = start_script.read_text(encoding="utf-8")
    docker_text = (SIM_STACK_DIR / "start_docker_sitl_mujoco_mj311.sh").read_text(encoding="utf-8")
    expected_gcs = 'append_param_if_not_overridden "MAV_GCS_SYSID" "$SITL_GCS_SYSID"'
    if expected_gcs not in text:
        raise AssertionError("startup must bind ArduSub 4.8 pilot authority to bridge sysid by default")
    expected_gcs_legacy = 'append_param_if_not_overridden "SYSID_MYGCS" "$SITL_GCS_SYSID"'
    if expected_gcs_legacy not in text:
        raise AssertionError("startup must keep ArduSub 4.1 pilot authority compatibility")
    expected = (
        'append_param_if_not_overridden "RC3_TRIM" '
        '"${SITL_RC3_TRIM:-$SITL_DEFAULT_RC3_TRIM}"'
    )
    if expected not in text:
        raise AssertionError(
            "ArduSub RC3 trim must follow the selected firmware's throttle formula"
        )
    if 'SITL_DEFAULT_RC3_TRIM="$(resolve_default_rc3_trim "$ARDUSUB_FIRMWARE_VERSION")"' not in text:
        raise AssertionError("startup must resolve RC3 trim from the selected ArduSub version")
    expected = 'append_param_if_not_overridden "RC_OPTIONS" "${SITL_RC_OPTIONS:-32}"'
    if expected not in text:
        raise AssertionError(
            "ArduSub startup must retain RC_OPTIONS bit 5; ArduSub 4.1.2 arms "
            "at low RC3 before switching to the separate 1500 zero-thrust input"
        )
    if "SIM_BARO_RND|SIM_BARO_DRIFT|SIM_BARO_GLITCH|SIM_BARO_DELAY|SIM_BAR2_RND" not in text:
        raise AssertionError(
            "ArduSub 4.1 metadata filtering must preserve the binary-supported SITL barometer noise overrides"
        )
    expected = 'append_param_if_not_overridden "FRAME_CONFIG" "${SITL_FRAME_CONFIG:-2}"'
    if expected not in text:
        raise AssertionError("GUI/QGC forward RC requires the 8-thruster 6DOF frame by default with a SITL_FRAME_CONFIG override for diagnostics")
    for expected_rcmap in (
        'append_param_if_not_overridden "RCMAP_ROLL" "2"',
        'append_param_if_not_overridden "RCMAP_PITCH" "1"',
        'append_param_if_not_overridden "RCMAP_THROTTLE" "3"',
        'append_param_if_not_overridden "RCMAP_YAW" "4"',
        'append_param_if_not_overridden "RCMAP_FORWARD" "5"',
        'append_param_if_not_overridden "RCMAP_LATERAL" "6"',
    ):
        if expected_rcmap not in text:
            raise AssertionError("GUI/QGC RC override must force ArduSub Sub channel mapping")
    if "sitl_real_params_filtered_" not in text or "filtered sim-forced duplicate params" not in text:
        raise AssertionError("Docker/QGC startup must filter real-param keys that SITL forcibly overrides")
    if "FRAME_CONFIG|RCMAP_ROLL|RCMAP_PITCH|RCMAP_THROTTLE|RCMAP_YAW|RCMAP_FORWARD|RCMAP_LATERAL|RC1_DZ|RC2_DZ|RC3_DZ|RC3_MIN|RC3_MAX|RC3_TRIM|RC4_DZ|RC5_DZ|RC6_DZ|THR_DZ|JS_GAIN_DEFAULT|JS_GAIN_MIN|JS_GAIN_MAX|JS_GAIN_STEPS|JS_THR_GAIN" not in text:
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
    expected = 'append_param_if_not_overridden "ARMING_CHECK" "${SITL_ARMING_CHECK:-194}"'
    if expected not in text:
        raise AssertionError(
            "GUI/QGC arm path must retain the real Bar30, RC, and voltage prearm checks"
        )
    expected = 'append_param_if_not_overridden "FS_GCS_ENABLE" "${SITL_FS_GCS_ENABLE:-2}"'
    if expected not in text:
        raise AssertionError("Docker/QGC arm path must retain real-vehicle GCS failsafe action")
    expected = 'append_param_if_not_overridden "FS_PILOT_INPUT" "${SITL_FS_PILOT_INPUT:-2}"'
    if expected not in text:
        raise AssertionError("Docker/QGC arm path must retain real-vehicle pilot-input failsafe action")
    expected = 'append_param_if_not_overridden "FS_PILOT_TIMEOUT" "${SITL_FS_PILOT_TIMEOUT:-3.0}"'
    if expected not in text:
        raise AssertionError("Docker/QGC pilot-input failsafe timeout must match the real vehicle")
    expected = 'append_param_if_not_overridden "BRD_SAFETYENABLE" "${SITL_BRD_SAFETYENABLE:-0}"'
    if expected not in text:
        raise AssertionError("Docker/QGC arm path must force the SITL safety switch off")
    expected = 'append_param_if_not_overridden "MOT_FV_CPLNG_K" "${SITL_MOT_FV_CPLNG_K:-0.0}"'
    if expected not in text:
        raise AssertionError("GUI/QGC MANUAL forward RC must not be clamped by forward/vertical coupling by default")
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


def check_wayland_viewer_prefers_xwayland() -> None:
    env = build_gui_sim_stack_env(
        {
            "DISPLAY": ":0",
            "WAYLAND_DISPLAY": "wayland-0",
            "XDG_SESSION_TYPE": "wayland",
        },
        backend="docker",
        sim_stack_dir=SIM_STACK_DIR,
    )
    _assert_key(env, "DISPLAY", ":0")
    if "WAYLAND_DISPLAY" in env:
        raise AssertionError("GUI MuJoCo viewer must force XWayland by dropping WAYLAND_DISPLAY")
    _assert_key(env, "GLFW_PLATFORM", "x11")
    _assert_key(env, "QT_QPA_PLATFORM", "xcb")
    _assert_key(env, "GDK_BACKEND", "x11")
    _assert_key(env, "SDL_VIDEODRIVER", "x11")
    if not env.get("PYGLFW_LIBRARY"):
        raise AssertionError("GUI MuJoCo viewer must prefer the system X11 GLFW library on Wayland sessions")


def check_wayland_viewer_native_opt_out() -> None:
    env = build_gui_sim_stack_env(
        {
            "DISPLAY": ":0",
            "WAYLAND_DISPLAY": "wayland-0",
            "UUV_GUI_MUJOCO_XWAYLAND": "0",
        },
        backend="docker",
        sim_stack_dir=SIM_STACK_DIR,
    )
    _assert_key(env, "WAYLAND_DISPLAY", "wayland-0")
    if env.get("GLFW_PLATFORM") == "x11":
        raise AssertionError("UUV_GUI_MUJOCO_XWAYLAND=0 must not force GLFW_PLATFORM=x11")


def check_default_initial_depth_args() -> None:
    initial_depth = build_initial_depth_args({}, launch_extra_args=())
    _assert_equal(initial_depth.args, ("--initial-bar30-depth-m", "auto"), "default initial-depth args")
    _assert_equal(initial_depth.events, ("sim initial depth: bar30=auto m",), "default initial-depth event")
    if "--hold-initial-depth-until-release" in initial_depth.args:
        raise AssertionError("default GUI Start must not hold initial depth until release")


def check_default_qgc_video_args() -> None:
    args: list[str] = []
    events: list[str] = []
    apply_viewer_args(args, events, {"DISPLAY": ":0"}, platform_name="linux")
    if "--headless" in args:
        raise AssertionError("GUI Start must show the MuJoCo viewer by default when a display is available")
    if not any("GLFW viewer enabled" in event for event in events):
        raise AssertionError("GUI Start must report the default MuJoCo viewer")
    if "--no-qgc-video" not in args:
        raise AssertionError("GUI Start must keep QGC video off by default")

    args = []
    events = []
    apply_viewer_args(args, events, {}, platform_name="linux")
    if "--headless" not in args:
        raise AssertionError("GUI Start must fall back to headless MuJoCo when no display is available")

    args = []
    events = []
    apply_viewer_args(args, events, {"DISPLAY": ":0", "UUV_GUI_MUJOCO_VIEWER": "0"}, platform_name="linux")
    if "--headless" not in args:
        raise AssertionError("UUV_GUI_MUJOCO_VIEWER=0 must force headless MuJoCo")

    args = []
    events = []
    apply_viewer_args(args, events, {"UUV_GUI_MUJOCO_VIEWER": "1"}, platform_name="linux")
    if "--headless" in args:
        raise AssertionError("UUV_GUI_MUJOCO_VIEWER=1 must keep explicit GLFW viewer opt-in")
    if not any("GLFW viewer enabled" in event for event in events):
        raise AssertionError("UUV_GUI_MUJOCO_VIEWER=1 must report the viewer opt-in")

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


def check_sim_loop_catchup_not_tied_to_frame_time() -> None:
    cadence = build_viewer_loop_cadence(timestep=0.002, ros2_sensor_hz=60.0, viewer_fps=30.0)
    if cadence.max_step_lag_s < 0.200:
        raise AssertionError("simulation loop must tolerate more than a few dropped render frames before losing physics time")
    if cadence.max_catchup_steps < 100:
        raise AssertionError("simulation loop must catch up at least 200ms of 2ms physics steps by default")
    if cadence.max_sleep_s > 0.001:
        raise AssertionError("simulation loop sleep quantum must stay small for low-latency catch-up")


def check_launch_targets_exist() -> None:
    docker_target = resolve_sim_stack_launch_target("docker")
    native_target = resolve_sim_stack_launch_target("native")
    if sim_stack_start_script_error(docker_target.start_script) is not None:
        raise AssertionError(f"Docker start script unavailable: {docker_target.start_script}")
    if sim_stack_start_script_error(native_target.start_script) is not None:
        raise AssertionError(f"native start script unavailable: {native_target.start_script}")


def check_start_uses_complete_real_ros_stack() -> None:
    command = mavros_launch_command("udp://0.0.0.0:14551@")
    required = (
        "ros2 launch hit25_auv_ros2 rov_start.launch.py",
        "fcu_url:=udp://0.0.0.0:14551@",
        "use_sim_time:=true",
        "use_dvl:=true",
        "dvl_ip:=127.0.0.1",
        "configure_dvl_acoustic_on_startup:=true",
        "request_dvl_config_on_startup:=true",
        "use_joy2mavros:=false",
        "use_battery_bridge:=false",
        "use_odom2mavros:=false",
        "publish_static_tf:=false",
        "use_localization:=true",
        "use_ekf:=true",
        "surface_pressure_pa:=101640.0",
        "depth_zero_at_start:=false",
        "depth_offset_m:=-0.0536",
        "use_web_gui:=false",
        "use_rviz:=false",
        "use_mission_rviz_visualizer:=false",
    )
    for token in required:
        if token not in command:
            raise AssertionError(f"GUI Start real ROS stack is missing {token!r}: {command}")
    if "mavros apm.launch" in command:
        raise AssertionError("GUI Start must not launch MAVROS without the real sensor/EKF stack")

    workspace_root = ROOT.parents[1]
    _assert_equal(
        workspace_setup_candidates(),
        [
            workspace_root / "rospkg" / "install" / "setup.bash",
        ],
        "GUI ROS workspace overlay order",
    )


def check_homing_world_to_body_rotation() -> None:
    half = 2.0 ** -0.5
    body = world_vector_to_body((1.0, 0.0, 0.0), (0.0, 0.0, half, half))
    expected = (0.0, -1.0, 0.0)
    for actual, wanted in zip(body, expected):
        if abs(actual - wanted) > 1.0e-6:
            raise AssertionError(f"homing world-to-body rotation: expected {expected}, got {body}")


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

    def publish_rc_neutral_then_release(self) -> None:
        self._calls.extend(["publish_rc_override", "publish_rc_release"])

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
        return any(arg == needle or arg.startswith(f"{needle}=") for arg in args)

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
    if "--ros2-images" not in native_cmd:
        raise AssertionError("native GUI Start must enable ROS2 stereo camera image topics")
    _assert_equal(native_cmd[native_cmd.index("--ros2-image-width") + 1], "640", "default stereo image width")
    _assert_equal(native_cmd[native_cmd.index("--ros2-image-height") + 1], "360", "default stereo image height")
    _assert_equal(native_cmd[native_cmd.index("--ros2-image-hz") + 1], "4", "default stereo image Hz")
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

    disabled = build_sim_stack_launch_command(
        _LaunchOwner({"UUV_GUI_STEREO_CAMERA_ENABLE": "0"}),
        start_script=SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh",
        backend="native",
        extra_args=[],
    )
    if "--ros2-images" in disabled:
        raise AssertionError("UUV_GUI_STEREO_CAMERA_ENABLE=0 must disable default stereo image topics")

    purpose_disabled = build_sim_stack_launch_command(
        _LaunchOwner(),
        start_script=SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh",
        backend="native",
        extra_args=[],
        enable_stereo_camera=False,
    )
    if "--ros2-images" in purpose_disabled:
        raise AssertionError("purpose-specific sim start must be able to disable ROS2 image rendering")

    env_size = build_sim_stack_launch_command(
        _LaunchOwner(
            {
                "UUV_GUI_CAMERA_WIDTH": "800",
                "UUV_GUI_CAMERA_HEIGHT": "450",
                "UUV_GUI_CAMERA_HZ": "3",
            }
        ),
        start_script=SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh",
        backend="native",
        extra_args=[],
    )
    _assert_equal(env_size[env_size.index("--ros2-image-width") + 1], "800", "env stereo image width")
    _assert_equal(env_size[env_size.index("--ros2-image-height") + 1], "450", "env stereo image height")
    _assert_equal(env_size[env_size.index("--ros2-image-hz") + 1], "3", "env stereo image Hz")

    selected_owner = _LaunchOwner()
    selected_owner._camera_config = {"preset_id": "hd720_realtime"}
    selected_size = build_sim_stack_launch_command(
        selected_owner,
        start_script=SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh",
        backend="native",
        extra_args=[],
    )
    _assert_equal(selected_size[selected_size.index("--ros2-image-width") + 1], "1280", "GUI stereo image width")
    _assert_equal(selected_size[selected_size.index("--ros2-image-height") + 1], "720", "GUI stereo image height")
    _assert_equal(selected_size[selected_size.index("--ros2-image-hz") + 1], "30", "GUI stereo image Hz")

    explicit_size = build_sim_stack_launch_command(
        _LaunchOwner(),
        start_script=SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh",
        backend="native",
        extra_args=["--ros2-image-width=640", "--ros2-image-height", "360", "--ros2-image-hz", "12"],
    )
    if "--ros2-image-width" in explicit_size:
        raise AssertionError("explicit --ros2-image-width must not be duplicated")
    _assert_equal(
        explicit_size[explicit_size.index("--ros2-image-height") + 1],
        "360",
        "explicit stereo image height",
    )
    _assert_equal(
        explicit_size[explicit_size.index("--ros2-image-hz") + 1],
        "12",
        "explicit stereo image Hz",
    )


def check_pinger_gui_start_profile() -> None:
    _assert_equal(normalize_sim_start_purpose(None), "default", "default sim purpose")
    _assert_equal(
        normalize_sim_start_purpose("pinger_homing"),
        "pinger_homing",
        "pinger sim purpose",
    )
    try:
        normalize_sim_start_purpose("unknown")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown simulator start purpose must be rejected")

    profile = pinger_sim_environment()
    expected = {
        "UUV_EKF_CONTRACT": "althold_baro",
        "UUV_MUJOCO_TIMESTEP": "0.005",
        "UUV_COURSE_BUOY_TIMESTEP_GUARD": "1",
        "UUV_COURSE_BUOYS_ENABLE": "1",
        "UUV_MUJOCO_VIEWER_FPS": "12",
        "UUV_MUJOCO_VIEWER_WIDTH": "960",
        "UUV_MUJOCO_VIEWER_HEIGHT": "540",
        "UUV_ROS2_SENSOR_HZ": "100",
        "UUV_THRUSTER_LOOP_HZ": "100",
        "SITL_SENSOR_HZ_DEFAULT": "100",
        "SITL_THRUSTER_LOOP_HZ_DEFAULT": "100",
        "UUV_REAL_PKG_CAMERA_ENABLE": "0",
        "ROS2_UUV_HYDROPHONE_AUDIO_HZ": "23.4375",
        "ROS2_UUV_HYDROPHONE_SYNC_HZ": "50",
    }
    for key, expected_value in expected.items():
        _assert_equal(profile.get(key), expected_value, f"pinger profile {key}")

    resolved = build_gui_sim_stack_env(profile, backend="native", sim_stack_dir=SIM_STACK_DIR)
    _assert_key(resolved, "UUV_EKF_CONTRACT", "althold_baro")
    _assert_key(resolved, "SITL_EKF3_EXTNAV", "0")
    _assert_key(resolved, "ROS2_UUV_SITL_EXTNAV_ENABLE", "0")
    _assert_key(resolved, "ROS2_UUV_REQUIRE_EXTNAV_TX", "0")
    _assert_key(resolved, "SITL_AHRS_EKF_TYPE", "3")
    _assert_key(resolved, "ROS2_UUV_SITL_JSON_TIMING_MODE", "lockstep")
    _assert_key(resolved, "UUV_MUJOCO_TIMESTEP", "0.005")
    _assert_key(resolved, "UUV_COURSE_BUOYS_ENABLE", "1")
    _assert_key(resolved, "UUV_ROS2_SENSOR_HZ", "100")
    _assert_key(resolved, "UUV_THRUSTER_LOOP_HZ", "100")
    _assert_key(resolved, "SITL_SENSOR_HZ_DEFAULT", "100")
    _assert_key(resolved, "SITL_THRUSTER_LOOP_HZ_DEFAULT", "100")

    args = pinger_sim_launch_args()
    for option, value in (
        ("--initial-bar30-depth-m", "auto"),
        ("--ros2-sensor-hz", "100"),
        ("--thruster-loop-hz", "100"),
        ("--viewer-fps", "12"),
        ("--profile", "current"),
    ):
        _assert_equal(args[args.index(option) + 1], value, f"pinger launch {option}")

    launcher_text = (SIM_STACK_DIR / "tools" / "start_pinger_homing_sim.sh").read_text(
        encoding="utf-8"
    )
    for expected_line in (
        'export UUV_ROS2_SENSOR_HZ="100"',
        'export UUV_THRUSTER_LOOP_HZ="100"',
        'export SITL_SENSOR_HZ_DEFAULT="100"',
        'export SITL_THRUSTER_LOOP_HZ_DEFAULT="100"',
        "--ros2-sensor-hz 100",
        "--thruster-loop-hz 100",
    ):
        if expected_line not in launcher_text:
            raise AssertionError(f"pinger shell launcher missing {expected_line!r}")


def check_pinger_production_child_env_contract() -> None:
    """Exercise WebProcessManager's real Pinger-purpose env/command merge.

    This stops at the Popen boundary, where ``spawn_sim_stack_process`` would
    hand the environment to the simulator child.  In particular, an inherited
    stale ``UUV_COURSE_BUOYS_ENABLE=0`` must be replaced by the Pinger profile
    before that boundary; disabling the image/YOLO load must not disable the
    course-buoy runtime.
    """

    calls: list[str] = []
    manager = web_process_manager.WebProcessManager(_Node(calls))
    captured: dict[str, object] = {}
    fake_proc = _StartProc()

    patched_names = (
        "prepare_active_course_runtime",
        "resolve_sim_stack_launch_target",
        "sim_stack_start_script_error",
        "open_gui_sim_stack_log",
        "spawn_sim_stack_process",
    )
    originals = {name: getattr(web_process_manager, name) for name in patched_names}
    old_env = {
        key: os.environ.get(key)
        for key in (
            "UUV_COURSE_BUOYS_ENABLE",
            "UUV_REAL_PKG_CAMERA_ENABLE",
            "UUV_GUI_YOLO_ENABLE",
            "UUV_GUI_USE_EXTERNAL_MAVROS",
        )
    }
    try:
        # Reproduce the regression-prone parent environment: the Pinger
        # purpose must win over an old lightweight setting that disabled the
        # entire buoy runtime.
        os.environ["UUV_COURSE_BUOYS_ENABLE"] = "0"
        os.environ["UUV_REAL_PKG_CAMERA_ENABLE"] = "1"
        os.environ.pop("UUV_GUI_YOLO_ENABLE", None)
        os.environ["UUV_GUI_USE_EXTERNAL_MAVROS"] = "0"

        web_process_manager.prepare_active_course_runtime = lambda **_kwargs: SimpleNamespace(
            pinger_site_name="competition_pinger",
            mode="competition",
            scene_path=SIM_STACK_DIR / "scenes" / "tank_current_scene.xml",
        )
        web_process_manager.resolve_sim_stack_launch_target = lambda _backend: SimpleNamespace(
            backend="native",
            start_script=SIM_STACK_DIR / "start_sitl_mujoco_mj311.sh",
        )
        web_process_manager.sim_stack_start_script_error = lambda _path: None
        web_process_manager.open_gui_sim_stack_log = lambda: (
            Path("/tmp/gui_pinger_child_env_contract.log"),
            io.StringIO(),
        )

        def capture_spawn(*, cmd: list[str], env: dict[str, str], log_file):
            captured["cmd"] = list(cmd)
            captured["env"] = dict(env)
            captured["log_file"] = log_file
            return fake_proc

        web_process_manager.spawn_sim_stack_process = capture_spawn
        manager._start_process_watcher = lambda **_kwargs: None
        manager._schedule_external_mavros_after_sim_reset = lambda **_kwargs: None

        result = manager.start_sim_stack(purpose="pinger_homing")
    finally:
        for name, value in originals.items():
            setattr(web_process_manager, name, value)
        for key, value in old_env.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value

    _assert_equal(result.get("purpose"), "pinger_homing", "Pinger production start purpose")
    child_env = captured.get("env")
    if not isinstance(child_env, dict):
        raise AssertionError("Pinger production start did not reach simulator child Popen boundary")
    _assert_key(child_env, "UUV_COURSE_BUOYS_ENABLE", "1")
    _assert_key(child_env, "UUV_REAL_PKG_CAMERA_ENABLE", "0")
    _assert_key(child_env, "UUV_GUI_YOLO_ENABLE", "0")
    _assert_key(child_env, "UUV_COURSE_BUOY_TIMESTEP_GUARD", "1")

    child_cmd = captured.get("cmd")
    if not isinstance(child_cmd, list):
        raise AssertionError("Pinger production start did not build a simulator child command")
    if "--ros2-images" in child_cmd:
        raise AssertionError("Pinger production child must keep ROS camera rendering disabled")
    for option in ("--scene", "--fluid-model", "--viewer-fps", "--profile"):
        if option not in child_cmd:
            raise AssertionError(f"Pinger production child command missing {option}")

    # None of the shell wrappers between Popen and run_urdf_full.py may turn
    # the profile's explicit value back off.
    for script_name in (
        "start_sitl_mujoco_mj311.sh",
        "start_ardusub_sitl_mj311.sh",
        "launch_uuv_sim.sh",
    ):
        script = (SIM_STACK_DIR / script_name).read_text(encoding="utf-8")
        for line in script.splitlines():
            stripped = line.strip()
            if stripped.startswith("UUV_COURSE_BUOYS_ENABLE=") or stripped.startswith(
                "export UUV_COURSE_BUOYS_ENABLE="
            ):
                raise AssertionError(
                    f"{script_name} must inherit, not overwrite, UUV_COURSE_BUOYS_ENABLE"
                )


def check_stale_stopped_sim_is_not_available() -> None:
    manager = web_process_manager.WebProcessManager.__new__(
        web_process_manager.WebProcessManager
    )
    manager._sim_process = None

    class _SnapshotNode:
        def __init__(self, snapshot: SimpleNamespace) -> None:
            self._snapshot = snapshot

        def snapshot(self) -> SimpleNamespace:
            return self._snapshot

    fresh = SimpleNamespace(
        connected=True,
        sitl_mavlink_active=True,
        imu_age_s=0.01,
        state_age_s=0.02,
        sitl_mavlink_status_age_s=0.03,
    )
    manager.node = _SnapshotNode(fresh)
    if not manager._simulation_runtime_available():
        raise AssertionError("fresh simulator telemetry must be recognized")

    stale_state = SimpleNamespace(**vars(fresh))
    stale_state.state_age_s = 30.0
    manager.node = _SnapshotNode(stale_state)
    if manager._simulation_runtime_available():
        raise AssertionError("stale MAVROS state must not keep a stopped sim alive")

    stale_sitl = SimpleNamespace(**vars(fresh))
    stale_sitl.sitl_mavlink_status_age_s = 30.0
    manager.node = _SnapshotNode(stale_sitl)
    if manager._simulation_runtime_available():
        raise AssertionError("stale SITL diagnostics must not keep a stopped sim alive")


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
    check_native_gui_env_precedence()
    check_gui_truth_ahrs_is_forbidden()
    check_low_profile_closed_loop_cadence_floor()
    check_gui_default_backend_is_native()
    check_launcher_mavlink_source_matches_gcs()
    check_ardusub_rc_override_heave_trim()
    check_qgc_default_sim_profile_contract()
    check_command_link_same_opt_out()
    check_dedicated_command_link_opt_in()
    check_wayland_viewer_prefers_xwayland()
    check_wayland_viewer_native_opt_out()
    check_default_initial_depth_args()
    check_default_qgc_video_args()
    check_sim_loop_catchup_not_tied_to_frame_time()
    check_launch_targets_exist()
    check_start_uses_complete_real_ros_stack()
    check_homing_world_to_body_rotation()
    check_external_stack_blocks_command_ready()
    check_gui_start_uses_dist_like_transport_default()
    check_pinger_gui_start_profile()
    check_pinger_production_child_env_contract()
    check_stale_stopped_sim_is_not_available()
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
