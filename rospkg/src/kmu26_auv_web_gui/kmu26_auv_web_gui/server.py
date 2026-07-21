import argparse
import asyncio
import math
import os
import time
from pathlib import Path

import uvicorn
from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI
from fastapi import HTTPException
from fastapi import Request
from fastapi import Response
from fastapi import WebSocket
from fastapi import WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from kmu26_auv_web_gui.bag_analyzer import analyze_bag
from kmu26_auv_web_gui.bag_analyzer import write_analysis_artifacts
from kmu26_auv_web_gui.ekf_config import read_process_noise_covariance
from kmu26_auv_web_gui.ekf_config import write_process_noise_covariance
from kmu26_auv_web_gui.process_manager import ProcessManager
from kmu26_auv_web_gui.ros_interface import ATTITUDE_MAX_TILT_DEG
from kmu26_auv_web_gui.ros_interface import READY_MODES
from kmu26_auv_web_gui.ros_interface import STILLNESS_MAX_SPEED_MPS
from kmu26_auv_web_gui.ros_interface import TopicConfig
from kmu26_auv_web_gui.ros_interface import RosInterface


DEFAULT_BAG_TOPICS = [
    "/joy",
    "/battery",
    "/dvl/command/response",
    "/dvl/config/command",
    "/dvl/config/status",
    "/dvl/data",
    "/dvl/odometry",
    "/dvl/position",
    "/dvl/twist",
    "/depth/pose",
    "/mavros/imu/data",
    "/mavros/state",
    "/odometry/filtered",
    "/audio",
    "/audio_phase_estimator/delta_range_m",
    "/audio_phase_estimator/iq_magnitude",
    "/pinger_homing/status",
    "/pinger_homing/phase_peak_candidates",
    "/pinger_homing/select_frequency_hz",
    "/pinger_homing/selected_frequency_hz",
    "/homing/direction",
    "/pinger_homing/direction_body",
    "/control/pinger/rc_override",
    "/control/rc_override_mux/status",
    "/localization/path",
    "/tf",
    "/tf_static",
]

OPTIONAL_BAG_TOPICS = [
    "/joy/set_feedback",
]

ALLOWED_DVL_COMMANDS = {
    "calibrate_gyro",
    "get_config",
    "reset_dead_reckoning",
    "set_config",
}

ALLOWED_DVL_PARAMETERS = {
    "",
    "acoustic_enabled",
    "dark_mode_enabled",
    "mounting_rotation_offset",
    "mountig_rotation_offset",
    "range_mode",
    "speed_of_sound",
}

ALLOWED_CONTROL_MODES = {
    "MANUAL",
    "STABILIZE",
    "ALT_HOLD",
    "POSHOLD",
    "GUIDED",
}

VISION_YOLO_LAUNCH_ARGS = {
    "image_topic",
    "bbox_topic",
    "annotated_image_topic",
    "publish_annotated_image",
    "annotated_jpeg_quality",
    "model_path",
    "target_class_name",
    "target_class_id",
    "confidence_threshold",
    "device",
    "imgsz",
    "show_preview",
    "preview_window_name",
    "publish_per_class",
}

VISION_MISSION_LAUNCH_ARGS = {
    "bbox_topic",
    "depth_topic",
    "depth_pose_topic",
    "depth_pose_scale",
    "depth_pose_offset_m",
    "enable_topic",
    "state_topic",
    "rc_override_topic",
    "rc_monitor_topic",
    "control_rate_hz",
    "throttle_channel",
    "yaw_channel",
    "forward_channel",
    "neutral_pwm",
    "min_pwm",
    "max_pwm",
    "max_yaw_delta",
    "forward_pwm",
    "yaw_invert",
    "vertical_positive_is_up",
    "work_depth_m",
    "surface_depth_m",
    "max_depth_m",
    "buoy_class_id",
    "stick_class_id",
    "approach_area_ratio",
    "fork_target_x",
    "fork_target_y",
    "stick_deadband_x",
    "stick_deadband_y",
    "align_stable_sec",
    "insert_pwm",
    "insert_duration_sec",
    "detach_pwm",
    "detach_duration_sec",
    "backoff_pwm",
    "backoff_duration_sec",
    "search_timeout_sec",
    "area_verify_sec",
}
ALLOWED_PINGER_MODES = {"MANUAL", "STABILIZE", "ALT_HOLD", "POSHOLD", "GUIDED"}

REAL_VEHICLE_TOPIC_TYPES = {
    "odom": "nav_msgs/msg/Odometry",
    "imu": "sensor_msgs/msg/Imu",
    "depth": "geometry_msgs/msg/PoseWithCovarianceStamped",
    "mavros_state": "mavros_msgs/msg/State",
    "audio": "audio_common_msgs/msg/AudioData",
}


def create_app(
    robot_package: str,
    robot_launch: str,
    start_dronecan_allocator: bool = True,
    dronecan_can_interface: str = "can0",
    dronecan_allocator_node_id: int = 126,
    dronecan_allocator_db: str = "",
    dronecan_python: str = "",
    pinger_package: str = "kmu26_pinger_homing",
    pinger_launch: str = "pinger_homing_real.launch.py",
    topic_config: TopicConfig | None = None,
    auto_start_phase_peak_scan: bool = False,
    phase_peak_scan_args: dict[str, str] | None = None,
) -> FastAPI:
    app = FastAPI(title="AUV Localization Test GUI")
    process_manager = ProcessManager(
        robot_package=robot_package,
        robot_launch=robot_launch,
        start_dronecan_allocator=start_dronecan_allocator,
        dronecan_can_interface=dronecan_can_interface,
        dronecan_allocator_node_id=dronecan_allocator_node_id,
        dronecan_allocator_db=dronecan_allocator_db,
        dronecan_python=dronecan_python,
        pinger_package=pinger_package,
        pinger_launch=pinger_launch,
    )
    ros_interface = RosInterface(topic_config=topic_config)
    bag_selection: dict[str, object] = {
        "record_all": False,
        "topics": list(DEFAULT_BAG_TOPICS),
    }
    phase_peak_selection_lock = asyncio.Lock()
    web_dir_override = os.environ.get("KMU26_WEB_GUI_WEB_DIR")
    if web_dir_override:
        web_dir = Path(web_dir_override)
    else:
        package_share = Path(get_package_share_directory("kmu26_auv_web_gui"))
        web_dir = package_share / "web"

    app.mount("/static", StaticFiles(directory=web_dir, follow_symlink=True), name="static")

    @app.on_event("startup")
    def on_startup() -> None:
        process_manager.start_dronecan_allocator()
        ros_interface.start()
        if auto_start_phase_peak_scan:
            try:
                process_manager.start_phase_peak_scan(phase_peak_scan_args)
            except (OSError, RuntimeError) as exc:
                # Peak discovery is a pre-start convenience. Keep the GUI alive
                # so the operator can fix audio settings or confirm Hz manually.
                process_manager.logs.append(
                    f"[phase_peak_scanner] auto-start failed: {exc}"
                )

    @app.on_event("shutdown")
    def on_shutdown() -> None:
        process_manager.stop_all()
        ros_interface.stop()

    @app.get("/")
    def index() -> FileResponse:
        return FileResponse(web_dir / "index.html")

    @app.get("/favicon.ico", include_in_schema=False)
    def favicon() -> Response:
        return Response(status_code=204)

    @app.get("/api/status")
    def status() -> dict:
        return _status(process_manager, ros_interface)

    @app.post("/api/stack/start")
    async def start_stack(request: Request) -> dict:
        body = await _json_or_empty(request)
        requested_launch_args = body.get("launch_args", {})
        if not isinstance(requested_launch_args, dict):
            raise HTTPException(status_code=400, detail="launch_args must be an object")
        launch_args = {
            "joy_rc_output_topic": "/control/joystick/rc_override",
            "joy_release_when_idle": "true",
        }
        launch_args.update({str(key): str(value) for key, value in requested_launch_args.items()})
        try:
            process_manager.start_stack(launch_args)
        except (OSError, RuntimeError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/stack/stop")
    def stop_stack() -> dict:
        process_manager.stop_stack()
        return _status(process_manager, ros_interface)

    @app.get("/api/pinger/phase-peaks")
    def phase_peaks() -> dict:
        status = _status(process_manager, ros_interface)
        return {
            "scanner_running": bool(
                status.get("process", {}).get("phase_peak_scanner_running", False)
            ),
            **status.get("ros", {}).get("phase_peak_selection", {}),
        }

    @app.post("/api/pinger/phase-peaks/scan")
    async def start_phase_peak_scan(request: Request) -> dict:
        body = await _json_or_empty(request)
        if process_manager.status().get("pinger_running", False):
            raise HTTPException(
                status_code=400,
                detail="stop pinger homing before scanning phase peaks",
            )
        topics = ros_interface.topic_config
        min_frequency_hz, max_frequency_hz = _validated_phase_scan_band(body)
        launch_args = {
            "use_audio_capture": (
                "true" if bool(body.get("use_audio_capture", False)) else "false"
            ),
            "audio_device": str(body.get("audio_device", "")).strip(),
            "audio_topic": str(body.get("audio_topic", topics.audio)).strip()
            or topics.audio,
            "use_stamped_audio": (
                "true" if bool(body.get("use_stamped_audio", True)) else "false"
            ),
            "audio_stamped_topic": str(
                body.get("audio_stamped_topic", "/audio_stamped")
            ).strip(),
            "audio_info_topic": str(
                body.get("audio_info_topic", "/audio_info")
            ).strip(),
            "audio_sample_rate": str(
                _bounded_int(body, "audio_sample_rate", 96000, 8000, 192000)
            ),
            "min_frequency_hz": str(min_frequency_hz),
            "max_frequency_hz": str(max_frequency_hz),
        }
        try:
            process_manager.start_phase_peak_scan(launch_args)
        except (OSError, RuntimeError) as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        ros_interface.clear_phase_frequency_selection()
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/phase-peaks/stop")
    def stop_phase_peak_scan() -> dict:
        process_manager.stop_phase_peak_scan()
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/phase-peaks/select")
    async def select_phase_peak(request: Request) -> dict:
        process_status = process_manager.status()
        if process_status.get("pinger_running", False):
            raise HTTPException(
                status_code=400,
                detail="stop pinger homing before changing the phase frequency",
            )
        body = await _json_or_empty(request)
        frequency_hz = _bounded_float(
            body, "frequency_hz", 21164.0, 15000.0, 25000.0
        )
        source = str(body.get("source", "candidate")).strip().lower()
        if source not in {"candidate", "manual"}:
            raise HTTPException(
                status_code=400,
                detail="phase frequency source must be candidate or manual",
            )
        rank: int | None = None
        if source == "candidate":
            selection = ros_interface.status().get("phase_peak_selection", {})
            candidates = selection.get("candidates", [])
            matched = next(
                (
                    item
                    for item in candidates
                    if isinstance(item, dict)
                    and _frequency_matches(item.get("frequency_hz"), frequency_hz)
                ),
                None,
            )
            if matched is None:
                raise HTTPException(
                    status_code=400,
                    detail="selected phase peak is not in the current candidate set",
                )
            if not bool(matched.get("selectable", False)):
                raise HTTPException(
                    status_code=400,
                    detail="selected phase peak is not stable enough yet; keep scanning",
                )
            rank = int(matched.get("rank", 0) or 0) or None
            async with phase_peak_selection_lock:
                request_sequence = ros_interface.request_phase_frequency(
                    frequency_hz, source, rank
                )
                accepted = await asyncio.to_thread(
                    ros_interface.wait_for_phase_frequency_selection,
                    request_sequence,
                )
                if not accepted:
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            "phase peak scanner did not accept the candidate; "
                            "keep scanning and select a stable candidate"
                        ),
                    )
                acknowledged_hz = _finite_float(
                    ros_interface.status()
                    .get("phase_peak_selection", {})
                    .get("selected_frequency_hz")
                )
                if acknowledged_hz is None:
                    raise HTTPException(
                        status_code=409,
                        detail=(
                            "phase peak scanner acknowledgement contained no frequency"
                        ),
                    )
                # The scanner may snap a selection within its tolerance to the
                # stabilized peak. Return and launch that acknowledged exact Hz.
                frequency_hz = acknowledged_hz
        else:
            if process_status.get("phase_peak_scanner_running", False):
                raise HTTPException(
                    status_code=409,
                    detail=(
                        "stop Phase peak scanning before using the explicit "
                        "manual-frequency fallback"
                    ),
                )
            async with phase_peak_selection_lock:
                ros_interface.confirm_manual_phase_frequency(frequency_hz)
        status = _status(process_manager, ros_interface)
        return {
            "accepted": True,
            "frequency_hz": frequency_hz,
            "source": source,
            "rank": rank,
            **status,
        }

    @app.post("/api/pinger/start")
    async def start_pinger(request: Request) -> dict:
        body = await _json_or_empty(request)
        dry_run = bool(body.get("dry_run", True))
        if not dry_run and not bool(body.get("confirm_live", False)):
            raise HTTPException(
                status_code=400,
                detail="live pinger homing requires confirm_live=true",
            )
        amplitude_constant = _bounded_float(
            body, "amplitude_range_constant", 0.0, 0.0, 10.0
        )
        success_range = _bounded_float(body, "success_range_m", 0.0, 0.0, 20.0)
        if success_range > 0.0 and amplitude_constant <= 0.0:
            raise HTTPException(
                status_code=400,
                detail=(
                    "success_range_m requires a calibrated positive "
                    "amplitude_range_constant"
                ),
            )

        topics = ros_interface.topic_config
        estimator_mode = str(body.get("estimator_mode", "phase")).strip().lower()
        if estimator_mode not in {"phase", "snr"}:
            raise HTTPException(
                status_code=400,
                detail="estimator_mode must be phase or snr",
            )
        navigation_mode = str(
            body.get("navigation_mode", "odometry")
        ).strip().lower()
        if navigation_mode not in {"odometry", "no_odom_phase"}:
            raise HTTPException(
                status_code=400,
                detail="navigation_mode must be odometry or no_odom_phase",
            )
        if navigation_mode == "no_odom_phase" and estimator_mode != "phase":
            raise HTTPException(
                status_code=400,
                detail="NO_ODOM_PHASE requires estimator_mode=phase",
            )
        reference_frequency_hz = _bounded_float(
            body, "reference_frequency_hz", 21164.0, 15000.0, 25000.0
        )
        ros_status = ros_interface.status()
        frequency_confirmation = _phase_frequency_confirmation_check(ros_status, body)
        if not frequency_confirmation["ok"]:
            raise HTTPException(status_code=400, detail=frequency_confirmation["detail"])
        if not dry_run:
            preflight = _pinger_live_preflight(ros_status, body)
            if not preflight["ok"]:
                failed = "; ".join(
                    check["detail"] for check in preflight["checks"] if not check["ok"]
                )
                raise HTTPException(
                    status_code=400,
                    detail=f"pinger live preflight failed: {failed}",
                )
        forward_max = _bounded_float(body, "forward_max", 0.48, 0.05, 0.8)
        probe_pwm_delta = _bounded_int(
            body, "probe_pwm_delta", 20, 15, 25
        )
        approach_pwm_delta = _bounded_int(
            body, "approach_pwm_delta", 120, 20, 200
        )
        approach_pwm_cap = forward_max * 400.0
        if approach_pwm_delta > approach_pwm_cap + 1.0e-6:
            raise HTTPException(
                status_code=400,
                detail=(
                    f"approach_pwm_delta {approach_pwm_delta:.1f} exceeds the "
                    f"Forward max cap {approach_pwm_cap:.1f} (400 us RC span)"
                ),
            )
        approach_duration_s = _bounded_float(
            body, "approach_duration_s", 4.0, 0.5, 20.0
        )
        launch_args = {
            "dry_run": "true" if dry_run else "false",
            "mode": "STABILIZE",
            "auto_mode": "false" if dry_run else "true",
            "use_audio_capture": "true" if bool(body.get("use_audio_capture", False)) else "false",
            "use_hydrophone_estimator": (
                "true" if bool(body.get("use_hydrophone_estimator", True)) else "false"
            ),
            "use_rc_mux": "true",
            "estimator_mode": estimator_mode,
            "navigation_mode": navigation_mode,
            "controller_profile": "real",
            "audio_device": str(body.get("audio_device", "")).strip(),
            "audio_topic": str(body.get("audio_topic", topics.audio)).strip() or topics.audio,
            "reference_frequency_hz": str(reference_frequency_hz),
            "no_odom_probe_pwm_delta": str(
                probe_pwm_delta
            ),
            "no_odom_approach_pwm_delta": str(
                approach_pwm_delta
            ),
            "no_odom_forward_duration_s": str(
                approach_duration_s
            ),
            "odometry_topic": (
                "/pinger_homing/disabled/odometry"
                if navigation_mode == "no_odom_phase"
                else str(body.get("odometry_topic", topics.odom)).strip()
            ),
            "imu_topic": str(body.get("imu_topic", "/mavros/imu/data")).strip(),
            "depth_topic": str(body.get("depth_topic", topics.depth)).strip(),
            "state_topic": str(body.get("state_topic", topics.mavros_state)).strip(),
            "direction_topic": str(
                body.get("direction_topic", topics.hydrophone_direction)
            ).strip(),
            "status_topic": topics.pinger_homing_status,
            "rate_hz": str(_bounded_float(body, "rate_hz", 30.0, 1.0, 120.0)),
            "forward_max": str(forward_max),
            "yaw_gain": str(_bounded_float(body, "yaw_gain", 0.85, 0.1, 2.0)),
            "yaw_command_limit": str(
                _bounded_float(body, "yaw_command_limit", 0.42, 0.05, 0.7)
            ),
            "tank_max_depth_m": str(
                _bounded_float(body, "tank_max_depth_m", 11.0, 0.5, 50.0)
            ),
            "success_range_m": str(success_range),
            "success_hold_s": str(
                _bounded_float(body, "success_hold_s", 0.8, 0.1, 10.0)
            ),
            "arrival_radius_m": str(
                _bounded_float(body, "arrival_radius_m", 1.5, 0.2, 20.0)
            ),
            "arrival_hold_s": str(
                _bounded_float(body, "arrival_hold_s", 1.0, 0.1, 10.0)
            ),
            "max_runtime_s": str(
                _bounded_float(body, "max_runtime_s", 180.0, 5.0, 3600.0)
            ),
            "amplitude_range_constant": str(amplitude_constant),
        }
        try:
            process_manager.stop_phase_peak_scan()
            process_manager.start_pinger(launch_args)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/stop")
    def stop_pinger() -> dict:
        process_manager.stop_pinger()
        return _status(process_manager, ros_interface)

    @app.post("/api/pinger/preflight")
    async def pinger_preflight(request: Request) -> dict:
        return _pinger_live_preflight(ros_interface.status(), await _json_or_empty(request))

    @app.post("/api/pinger/arm")
    async def pinger_arm(request: Request) -> dict:
        body = await _json_or_empty(request)
        armed = bool(body.get("armed", False))
        if armed and process_manager.status().get("pinger_running", False):
            raise HTTPException(
                status_code=400,
                detail="stop pinger homing before changing arm state",
            )
        accepted = ros_interface.set_armed(armed)
        if not accepted:
            raise HTTPException(status_code=503, detail="MAVROS arming service unavailable")
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = True
        return payload

    @app.post("/api/pinger/mode")
    async def pinger_mode(request: Request) -> dict:
        body = await _json_or_empty(request)
        mode = str(body.get("mode", "")).upper()
        if mode not in ALLOWED_PINGER_MODES:
            raise HTTPException(status_code=400, detail=f"unsupported mode: {mode}")
        if process_manager.status().get("pinger_running", False):
            raise HTTPException(
                status_code=400,
                detail="stop pinger homing before changing vehicle mode",
            )
        accepted = ros_interface.set_mode(mode)
        if not accepted:
            raise HTTPException(status_code=503, detail="MAVROS set-mode service unavailable")
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = True
        return payload

    @app.post("/api/dvl/command")
    async def run_dvl_command(request: Request) -> dict:
        body = await _json_or_empty(request)
        command = str(body.get("command", ""))
        parameter_name = str(body.get("parameter_name", ""))
        parameter_value = str(body.get("parameter_value", ""))
        if command not in ALLOWED_DVL_COMMANDS:
            raise HTTPException(status_code=400, detail=f"unsupported DVL command: {command}")
        if parameter_name not in ALLOWED_DVL_PARAMETERS:
            raise HTTPException(status_code=400, detail=f"unsupported DVL parameter: {parameter_name}")
        if command != "set_config" and (parameter_name or parameter_value):
            raise HTTPException(status_code=400, detail=f"{command} does not accept a parameter")
        if command == "set_config" and not parameter_name:
            raise HTTPException(status_code=400, detail="set_config requires a parameter_name")

        ros_interface.publish_dvl_command(command, parameter_name, parameter_value)
        return _status(process_manager, ros_interface)

    @app.post("/api/dvl/reset_dr")
    def reset_dvl() -> dict:
        ros_interface.reset_dvl_dead_reckoning()
        return _status(process_manager, ros_interface)

    @app.post("/api/path/clear")
    def clear_path() -> dict:
        ros_interface.clear_path()
        return _status(process_manager, ros_interface)

    @app.post("/api/localization/set_origin")
    def set_localization_origin() -> dict:
        previous_pose = ros_interface.status().get("pose", {})
        set_pose = None
        try:
            restart = process_manager.restart_localization_filter()
            fresh_since = time.monotonic()
            odom_refreshed = ros_interface.wait_for_odom_after(fresh_since, timeout=5.0)
            if odom_refreshed:
                set_pose = ros_interface.set_localization_origin()
            ros_interface.clear_path()
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["origin"] = {
            "method": "restart_robot_localization_then_set_pose",
            "previous_pose": previous_pose,
            "restart": restart,
            "set_pose": set_pose,
            "odom_refreshed": odom_refreshed,
        }
        return payload

    @app.post("/api/test/start")
    async def start_localization_test(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            test = await asyncio.to_thread(
                _run_localization_test,
                process_manager,
                ros_interface,
                body,
            )
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["test"] = test
        return payload

    @app.post("/api/test/stop")
    def stop_localization_test() -> dict:
        try:
            process_manager.stop_bag()
            process_manager.stop_stack()
            ros_interface.clear_path()
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["test"] = {
            "message": "Idle",
            "steps": [
                {"name": "bag", "status": "ok", "detail": "stopped"},
                {"name": "stack", "status": "ok", "detail": "stopped"},
                {"name": "path", "status": "ok", "detail": "cleared"},
            ],
        }
        return payload

    @app.post("/api/control/enable")
    async def set_control_enabled(request: Request) -> dict:
        body = await _json_or_empty(request)
        ros_interface.set_web_control_enabled(bool(body.get("enabled", False)))
        return _status(process_manager, ros_interface)

    @app.post("/api/control/command")
    async def set_control_command(request: Request) -> dict:
        body = await _json_or_empty(request)
        axes = body.get("axes", {})
        if not isinstance(axes, dict):
            raise HTTPException(status_code=400, detail="axes must be an object")
        ros_interface.update_web_control_command(axes, bool(body.get("active", False)))
        return _status(process_manager, ros_interface)

    @app.post("/api/control/arm")
    async def set_control_arm(request: Request) -> dict:
        body = await _json_or_empty(request)
        accepted = ros_interface.set_armed(bool(body.get("armed", False)))
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = accepted
        return payload

    @app.post("/api/control/mode")
    async def set_control_mode(request: Request) -> dict:
        body = await _json_or_empty(request)
        mode = str(body.get("mode", ""))
        if mode not in ALLOWED_CONTROL_MODES:
            raise HTTPException(status_code=400, detail=f"unsupported mode: {mode}")
        accepted = ros_interface.set_mode(mode)
        payload = _status(process_manager, ros_interface)
        payload["accepted"] = accepted
        return payload

    @app.post("/api/vision/yolo/start")
    async def start_vision_yolo(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            launch_args = _validated_launch_args(body, VISION_YOLO_LAUNCH_ARGS)
            if not launch_args.get("model_path"):
                raise RuntimeError("model_path is required to start YOLO")
            launch_args.setdefault("show_preview", "false")
            process_manager.start_vision_yolo(launch_args)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/vision/yolo/stop")
    def stop_vision_yolo() -> dict:
        ros_interface.release_vision_control()
        process_manager.stop_vision_yolo()
        return _status(process_manager, ros_interface)

    @app.post("/api/vision/mission/start")
    async def start_vision_mission(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            process_manager.start_vision_mission(
                _validated_launch_args(body, VISION_MISSION_LAUNCH_ARGS)
            )
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        return _status(process_manager, ros_interface)

    @app.post("/api/vision/mission/stop")
    def stop_vision_mission() -> dict:
        ros_interface.release_vision_control()
        time.sleep(0.15)
        process_manager.stop_vision_mission()
        return _status(process_manager, ros_interface)

    @app.post("/api/vision/control")
    async def set_vision_control(request: Request) -> dict:
        body = await _json_or_empty(request)
        enabled = _parse_bool(body.get("enabled", False))
        if enabled and ros_interface.topic_publisher_count("/mission/state") <= 0:
            raise HTTPException(
                status_code=400,
                detail="mission controller is not available; start it before enabling control",
            )
        ros_interface.set_vision_control_enabled(enabled)
        return _status(process_manager, ros_interface)

    @app.post("/api/vision/emergency_stop")
    def stop_vision_control() -> dict:
        ros_interface.release_vision_control()
        return _status(process_manager, ros_interface)

    @app.post("/api/vision/stop")
    def stop_vision_all() -> dict:
        ros_interface.release_vision_control()
        time.sleep(0.15)
        process_manager.stop_vision()
        return _status(process_manager, ros_interface)

    @app.get("/api/vision/frame")
    def vision_frame(after: int = 0) -> Response:
        data, content_type, sequence = ros_interface.latest_vision_frame()
        if not data or sequence <= after:
            return Response(status_code=204)
        return Response(
            content=data,
            media_type=content_type,
            headers={
                "Cache-Control": "no-store, max-age=0",
                "X-Vision-Frame-Sequence": str(sequence),
            },
        )

    @app.get("/api/ekf/process_noise")
    def get_process_noise() -> dict:
        return read_process_noise_covariance()

    @app.post("/api/ekf/process_noise")
    async def set_process_noise(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            return write_process_noise_covariance(body.get("values", []))
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @app.post("/api/bag/start")
    async def start_bag(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            topics, record_all = _bag_record_options(body)
            _save_bag_selection(bag_selection, topics, record_all)
            output_dir = process_manager.start_bag(topics, record_all=record_all)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        payload = _status(process_manager, ros_interface)
        payload["bag_output"] = output_dir
        return payload

    @app.post("/api/bag/stop")
    def stop_bag() -> dict:
        process_manager.stop_bag()
        return _status(process_manager, ros_interface)

    @app.get("/api/bag/topics")
    def bag_topics() -> dict:
        try:
            active_topics = process_manager.list_topics()
        except RuntimeError as exc:
            raise HTTPException(status_code=500, detail=str(exc)) from exc
        topics = sorted(
            set(DEFAULT_BAG_TOPICS)
            | set(OPTIONAL_BAG_TOPICS)
            | set(active_topics)
            | set(bag_selection["topics"])
        )
        return {
            "default_topics": DEFAULT_BAG_TOPICS,
            "optional_topics": OPTIONAL_BAG_TOPICS,
            "active_topics": active_topics,
            "selected_topics": list(bag_selection["topics"]),
            "record_all": bool(bag_selection["record_all"]),
            "topics": topics,
        }

    @app.post("/api/bag/selection")
    async def save_bag_selection(request: Request) -> dict:
        body = await _json_or_empty(request)
        try:
            topics, record_all = _bag_record_options(body)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        _save_bag_selection(bag_selection, topics, record_all)
        return {
            "record_all": bool(bag_selection["record_all"]),
            "topics": list(bag_selection["topics"]),
        }

    @app.post("/api/bag/analyze")
    async def run_bag_analysis(request: Request) -> dict:
        body = await _json_or_empty(request)
        process_status = process_manager.status()
        if process_status.get("bag_running"):
            raise HTTPException(status_code=400, detail="stop bag recording before analysis")

        bag_path = str(body.get("path") or process_status.get("bag_output") or "")
        if not bag_path:
            raise HTTPException(status_code=400, detail="no bag output path is available")

        try:
            analysis = await asyncio.to_thread(_analyze_and_write_bag, bag_path)
        except RuntimeError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

        payload = _status(process_manager, ros_interface)
        payload["analysis"] = analysis
        return payload

    @app.websocket("/ws/status")
    async def status_ws(websocket: WebSocket) -> None:
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(_status(process_manager, ros_interface))
                await asyncio.sleep(0.2)
        except WebSocketDisconnect:
            return

    return app


async def _json_or_empty(request: Request) -> dict:
    try:
        return await request.json()
    except Exception:
        return {}


def _validated_launch_args(body: dict, allowed: set[str]) -> dict[str, str]:
    raw_args = body.get("launch_args", body)
    if not isinstance(raw_args, dict):
        raise RuntimeError("launch_args must be an object")
    unknown = sorted(str(key) for key in raw_args if str(key) not in allowed)
    if unknown:
        raise RuntimeError(f"unsupported launch argument(s): {', '.join(unknown)}")
    return {
        str(key): str(value).strip()
        for key, value in raw_args.items()
        if value is not None and str(value).strip() != ""
    }


def _status(process_manager: ProcessManager, ros_interface: RosInterface) -> dict:
    return {
        "process": process_manager.status(),
        "ros": ros_interface.status(),
    }


def _bounded_float(
    body: dict, key: str, default: float, minimum: float, maximum: float
) -> float:
    try:
        value = float(body.get(key, default))
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"{key} must be numeric") from exc
    if not minimum <= value <= maximum:
        raise HTTPException(
            status_code=400,
            detail=f"{key} must be between {minimum} and {maximum}",
        )
    return value


def _bounded_int(
    body: dict, key: str, default: int, minimum: int, maximum: int
) -> int:
    try:
        numeric = float(body.get(key, default))
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"{key} must be an integer") from exc
    if not math.isfinite(numeric) or not numeric.is_integer():
        raise HTTPException(status_code=400, detail=f"{key} must be an integer")
    value = int(numeric)
    if not minimum <= value <= maximum:
        raise HTTPException(
            status_code=400,
            detail=f"{key} must be between {minimum} and {maximum}",
        )
    return value


def _validated_phase_scan_band(body: dict) -> tuple[float, float]:
    minimum_hz = _bounded_float(
        body, "min_frequency_hz", 15000.0, 15000.0, 25000.0
    )
    maximum_hz = _bounded_float(
        body, "max_frequency_hz", 25000.0, 15000.0, 25000.0
    )
    if maximum_hz <= minimum_hz:
        raise HTTPException(
            status_code=400,
            detail="max_frequency_hz must be greater than min_frequency_hz",
        )
    return minimum_hz, maximum_hz


def _frequency_matches(left: object, right: object, tolerance_hz: float = 1.0) -> bool:
    left_hz = _finite_float(left)
    right_hz = _finite_float(right)
    return (
        left_hz is not None
        and right_hz is not None
        and abs(left_hz - right_hz) <= tolerance_hz
    )


def _confirmed_frequency_matches(left: object, right: object) -> bool:
    left_hz = _finite_float(left)
    right_hz = _finite_float(right)
    if left_hz is None or right_hz is None:
        return False
    tolerance_hz = max(1.0e-6, 1.0e-9 * max(abs(left_hz), abs(right_hz)))
    return abs(left_hz - right_hz) <= tolerance_hz


def _phase_frequency_confirmation_check(ros_status: dict, body: dict) -> dict:
    estimator_mode = str(body.get("estimator_mode", "phase")).strip().lower()
    if estimator_mode != "phase":
        return {
            "name": "phase_frequency",
            "ok": True,
            "detail": "phase frequency confirmation is not required for SNR mode",
        }

    selection = (
        ros_status.get("phase_peak_selection", {})
        if isinstance(ros_status, dict)
        else {}
    )
    requested_hz = _finite_float(body.get("reference_frequency_hz", 21164.0))
    selected_hz = _finite_float(selection.get("selected_frequency_hz"))
    client_confirmed = bool(body.get("phase_peak_confirmed", False))
    bridge_confirmed = bool(selection.get("confirmed", False))
    source = str(selection.get("confirmation_source", "") or "")

    if not client_confirmed:
        detail = "select a detected Phase peak or explicitly confirm the manual frequency"
        ok = False
    elif not bridge_confirmed or selected_hz is None:
        detail = "the Phase frequency has not been confirmed through the ROS selector"
        ok = False
    elif requested_hz is None or not _confirmed_frequency_matches(
        selected_hz, requested_hz
    ):
        detail = (
            f"selected Phase frequency {selected_hz:.1f} Hz does not match "
            f"requested {requested_hz:.1f} Hz"
            if requested_hz is not None
            else "reference_frequency_hz must be finite"
        )
        ok = False
    else:
        detail = f"Phase frequency {selected_hz:.1f} Hz confirmed ({source or 'selector'})"
        ok = True
    return {
        "name": "phase_frequency",
        "ok": ok,
        "detail": detail,
        "selected_frequency_hz": selected_hz,
        "requested_frequency_hz": requested_hz,
        "source": source,
    }


def _pinger_live_preflight(ros_status: dict, body: dict) -> dict:
    topics = ros_status.get("topics", {}) if isinstance(ros_status, dict) else {}
    mavros = ros_status.get("mavros_state", {}) if isinstance(ros_status, dict) else {}
    graph = ros_status.get("graph", {}) if isinstance(ros_status, dict) else {}
    frames = ros_status.get("frames", {}) if isinstance(ros_status, dict) else {}
    pose = ros_status.get("pose", {}) if isinstance(ros_status, dict) else {}
    depth = ros_status.get("depth", {}) if isinstance(ros_status, dict) else {}
    topic_types = graph.get("topic_types", {}) if isinstance(graph, dict) else {}
    services = graph.get("services", {}) if isinstance(graph, dict) else {}
    checks: list[dict] = []

    def add(name: str, ok: bool, detail: str) -> None:
        checks.append({"name": name, "ok": bool(ok), "detail": detail})

    odom_alive = bool(topics.get("odom", {}).get("alive", False))
    depth_alive = bool(topics.get("depth", {}).get("alive", False))
    state_alive = bool(topics.get("mavros_state", {}).get("alive", False))
    connected = bool(mavros.get("connected", False))
    armed = bool(mavros.get("armed", False))
    mode = str(mavros.get("mode", "")).upper()
    rc_publishers = int(graph.get("rc_output_publishers", 0) or 0)
    audio_publishers = int(graph.get("audio_publishers", 0) or 0)
    use_capture = bool(body.get("use_audio_capture", False))
    use_estimator = bool(body.get("use_hydrophone_estimator", True))
    navigation_mode = str(body.get("navigation_mode", "odometry")).strip().lower()
    no_odom_phase = navigation_mode == "no_odom_phase"

    odom_frame = _normalized_frame(frames.get("odom", {}).get("frame_id"))
    odom_child_frame = _normalized_frame(
        frames.get("odom", {}).get("child_frame_id")
    )
    if no_odom_phase:
        imu_alive = bool(topics.get("imu", {}).get("alive", False))
        add("odometry", True, "NO_ODOM_PHASE explicitly bypasses /odometry/filtered")
        add("imu", imu_alive, "IMU is fresh" if imu_alive else "/mavros/imu/data is stale")
        _add_topic_type_check(checks, topic_types, "imu")
    else:
        add("odometry", odom_alive, "odometry is fresh" if odom_alive else "/odometry/filtered is stale")
        _add_topic_type_check(checks, topic_types, "odom")
        add(
            "odometry_frames",
            odom_alive and odom_frame == "odom" and odom_child_frame == "base_link",
            (
                "odometry frames are odom -> base_link"
                if odom_alive and odom_frame == "odom" and odom_child_frame == "base_link"
                else f"expected odom -> base_link, got {odom_frame or '--'} -> {odom_child_frame or '--'}"
            ),
        )
    add("depth", depth_alive, "depth pose is fresh" if depth_alive else "/depth/pose is stale")
    _add_topic_type_check(checks, topic_types, "depth")
    depth_frame = _normalized_frame(frames.get("depth", {}).get("frame_id"))
    add(
        "depth_frame",
        depth_alive and depth_frame == "odom",
        (
            "depth frame is odom"
            if depth_alive and depth_frame == "odom"
            else f"expected depth frame odom, got {depth_frame or '--'}"
        ),
    )
    odom_z = _finite_float(pose.get("z"))
    depth_z = _finite_float(depth.get("z"))
    depth_sign_ok = (
        depth_alive and depth_z is not None and depth_z <= 0.10
        if no_odom_phase
        else (
            odom_alive
            and depth_alive
            and odom_z is not None
            and depth_z is not None
            and odom_z <= 0.10
            and depth_z <= 0.10
            and (
                abs(odom_z) < 0.10
                or abs(depth_z) < 0.10
                or odom_z * depth_z >= 0.0
            )
        )
    )
    add(
        "depth_sign",
        depth_sign_ok,
        (
            (
                f"NO_ODOM_PHASE depth z-up contract is consistent (depth z={depth_z:.2f})"
                if no_odom_phase
                else f"z-up contract is consistent (odom z={odom_z:.2f}, depth z={depth_z:.2f})"
            )
            if depth_sign_ok
            else (
                "depth must use z-up (underwater z is negative)"
                if no_odom_phase
                else "odometry and depth must both use z-up (underwater z is negative)"
            )
        ),
    )
    add(
        "mavros",
        state_alive and connected,
        "MAVROS is connected" if state_alive and connected else "/mavros/state is stale or disconnected",
    )
    _add_topic_type_check(checks, topic_types, "mavros_state")
    add(
        "mavros_services",
        bool(services.get("arming", False)) and bool(services.get("set_mode", False)),
        (
            "MAVROS arming and set-mode services are ready"
            if services.get("arming", False) and services.get("set_mode", False)
            else "MAVROS /mavros/cmd/arming or /mavros/set_mode service is unavailable"
        ),
    )
    add("armed", armed, "vehicle is armed" if armed else "vehicle is not armed")
    add(
        "mode",
        mode in ALLOWED_PINGER_MODES,
        f"vehicle mode is {mode}" if mode else "vehicle mode is unavailable",
    )
    add(
        "rc_owner",
        rc_publishers == 0,
        (
            "RC output has no existing publisher"
            if rc_publishers == 0
            else (
                f"/mavros/rc/override already has {rc_publishers} publisher(s): "
                f"{_publisher_node_summary(graph.get('rc_output_publisher_nodes', []))}"
            )
        ),
    )
    audio_types = set(topic_types.get("audio", []))
    expected_audio_type = REAL_VEHICLE_TOPIC_TYPES["audio"]
    audio_type_ok = use_capture or expected_audio_type in audio_types
    add(
        "audio_type",
        audio_type_ok,
        (
            "bundled audio capture will provide audio_common_msgs/msg/AudioData"
            if use_capture
            else (
                "audio type is audio_common_msgs/msg/AudioData"
                if audio_type_ok
                else f"expected {expected_audio_type}, got {', '.join(sorted(audio_types)) or '--'}"
            )
        ),
    )
    add(
        "estimator",
        use_estimator,
        "hydrophone estimator enabled" if use_estimator else "hydrophone estimator is disabled",
    )
    frequency_confirmation = _phase_frequency_confirmation_check(ros_status, body)
    add(
        frequency_confirmation["name"],
        frequency_confirmation["ok"],
        frequency_confirmation["detail"],
    )
    add(
        "audio",
        use_capture or audio_publishers > 0,
        (
            "audio capture will start"
            if use_capture
            else (
                f"audio source publishers: {audio_publishers}"
                if audio_publishers > 0
                else "/audio has no publisher; enable capture or start the hydrophone input"
            )
        ),
    )
    try:
        max_runtime_s = float(body.get("max_runtime_s", 180.0))
    except (TypeError, ValueError):
        max_runtime_s = 0.0
    try:
        arrival_radius_m = float(body.get("arrival_radius_m", 1.5))
    except (TypeError, ValueError):
        arrival_radius_m = 0.0
    add(
        "runtime_limit",
        max_runtime_s >= 5.0,
        f"runtime limit {max_runtime_s:.1f} s" if max_runtime_s >= 5.0 else "max_runtime_s must be at least 5 s",
    )
    add(
        "arrival_stop",
        arrival_radius_m >= 0.2,
        f"arrival radius {arrival_radius_m:.2f} m" if arrival_radius_m >= 0.2 else "arrival_radius_m must be at least 0.2 m",
    )
    return {"ok": all(check["ok"] for check in checks), "checks": checks}


def _add_topic_type_check(checks: list[dict], topic_types: dict, key: str) -> None:
    expected = REAL_VEHICLE_TOPIC_TYPES[key]
    discovered = set(topic_types.get(key, []))
    ok = expected in discovered
    checks.append(
        {
            "name": f"{key}_type",
            "ok": ok,
            "detail": (
                f"{key} type is {expected}"
                if ok
                else f"expected {expected}, got {', '.join(sorted(discovered)) or '--'}"
            ),
        }
    )


def _normalized_frame(value: object) -> str:
    return str(value or "").strip().lstrip("/")


def _finite_float(value: object) -> float | None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return None
    return numeric if math.isfinite(numeric) else None


def _publisher_node_summary(nodes: object) -> str:
    if not isinstance(nodes, list):
        return "unknown"
    labels = []
    for item in nodes:
        if not isinstance(item, dict):
            continue
        namespace = str(item.get("namespace", "/")).rstrip("/")
        name = str(item.get("name", "unknown")).lstrip("/")
        labels.append(f"{namespace}/{name}" if namespace else f"/{name}")
    return ", ".join(labels) or "unknown"


def _run_localization_test(
    process_manager: ProcessManager,
    ros_interface: RosInterface,
    body: dict,
) -> dict:
    launch_args = body.get("launch_args", {})
    if not isinstance(launch_args, dict):
        raise RuntimeError("launch_args must be an object")
    launch_args = {str(key): str(value) for key, value in launch_args.items()}

    steps: list[dict[str, str]] = []

    if process_manager.stack_running:
        _append_test_step(steps, "stack", "skipped", "already running")
    else:
        process_manager.start_stack(launch_args)
        _append_test_step(steps, "stack", "ok", "started")
        time.sleep(1.0)

    publisher_count = _wait_for_topic_publisher(ros_interface, "/dvl/data", timeout=8.0)
    if publisher_count <= 0:
        raise RuntimeError("DVL data publisher was not discovered")
    _append_test_step(steps, "dvl_data", "ok", f"{publisher_count} publisher(s)")

    subscriber_count = _wait_for_dvl_command_subscriber(ros_interface, timeout=4.0)
    if subscriber_count <= 0:
        raise RuntimeError("DVL command subscriber was not discovered")
    _append_test_step(steps, "dvl_command", "ok", f"{subscriber_count} subscriber(s)")

    ros_interface.neutralize_web_control()
    _append_test_step(steps, "web_control", "ok", "neutral burst sent")
    time.sleep(0.3)

    _send_dvl_command_step(
        ros_interface,
        steps,
        "set_config",
        "acoustic_enabled",
        "true",
    )
    time.sleep(0.25)
    _send_dvl_command_step(
        ros_interface,
        steps,
        "set_config",
        "range_mode",
        "auto",
    )
    time.sleep(0.25)
    _send_dvl_command_step(ros_interface, steps, "get_config")
    time.sleep(0.25)

    _require_mode_ready(ros_interface, steps)
    _require_attitude_level(ros_interface, steps)
    _require_dvl_good(ros_interface, steps, "dvl_good_before_reset")
    _require_vehicle_still(ros_interface, steps)

    _send_dvl_command_step(ros_interface, steps, "reset_dead_reckoning")
    time.sleep(0.25)
    _require_dvl_good(ros_interface, steps, "dvl_good_after_reset")
    _require_topic_alive(ros_interface, steps, "depth", "depth_fresh", timeout=5.0)

    previous_pose = ros_interface.status().get("pose", {})
    restart = process_manager.restart_localization_filter()
    _append_test_step(steps, "ekf", "ok", "restarted")
    fresh_since = time.monotonic()
    odom_refreshed = ros_interface.wait_for_odom_after(fresh_since, timeout=6.0)
    set_pose = None
    if odom_refreshed:
        set_pose = ros_interface.set_localization_origin()
        _append_test_step(steps, "origin", "ok", "set_pose sent")
    else:
        _append_test_step(steps, "origin", "warn", "odometry did not refresh")

    ros_interface.clear_path()
    _append_test_step(steps, "path", "ok", "cleared")

    warning = next((step["detail"] for step in steps if step["status"] == "warn"), "")
    return {
        "message": f"Ready with warning: {warning}" if warning else "Ready",
        "origin": {
            "method": "test_sequence_restart_then_set_pose",
            "previous_pose": previous_pose,
            "restart": restart,
            "set_pose": set_pose,
            "odom_refreshed": odom_refreshed,
        },
        "steps": steps,
    }


def _bag_record_options(
    body: dict,
    include_default_topics: bool = False,
) -> tuple[list[str], bool]:
    record_all = bool(body.get("record_all", False))
    raw_topics = body["topics"] if "topics" in body else DEFAULT_BAG_TOPICS
    if not isinstance(raw_topics, list):
        raise RuntimeError("topics must be a list")
    topics = [str(topic).strip() for topic in raw_topics if str(topic).strip()]
    if include_default_topics:
        topics = list(dict.fromkeys([*DEFAULT_BAG_TOPICS, *topics]))
    return topics, record_all


def _save_bag_selection(
    bag_selection: dict[str, object],
    topics: list[str],
    record_all: bool,
) -> None:
    bag_selection["record_all"] = record_all
    bag_selection["topics"] = list(dict.fromkeys(topics))


def _append_test_step(
    steps: list[dict[str, str]],
    name: str,
    status: str,
    detail: str = "",
) -> None:
    steps.append({"name": name, "status": status, "detail": detail})


def _wait_for_dvl_command_subscriber(
    ros_interface: RosInterface,
    timeout: float,
) -> int:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        count = ros_interface.dvl_command_subscriber_count()
        if count > 0:
            return count
        time.sleep(0.1)
    return ros_interface.dvl_command_subscriber_count()


def _wait_for_topic_publisher(
    ros_interface: RosInterface,
    topic: str,
    timeout: float,
) -> int:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        count = ros_interface.topic_publisher_count(topic)
        if count > 0:
            return count
        time.sleep(0.1)
    return ros_interface.topic_publisher_count(topic)


def _send_dvl_command_step(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
    command: str,
    parameter_name: str = "",
    parameter_value: str = "",
) -> None:
    ros_interface.publish_dvl_command(command, parameter_name, parameter_value)
    detail = parameter_name
    if parameter_value:
        detail = f"{detail}={parameter_value}" if detail else parameter_value
    _append_test_step(steps, f"dvl_{command}", "ok", detail)


def _require_dvl_good(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
    name: str,
) -> None:
    ok, snapshot = ros_interface.wait_for_dvl_good(duration_s=2.0, timeout_s=10.0)
    detail = _dvl_quality_detail(snapshot)
    if not ok:
        _append_test_step(steps, name, "bad", detail)
        raise RuntimeError(f"{name} failed: {detail}")
    _append_test_step(steps, name, "ok", detail)


def _require_attitude_level(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
) -> None:
    ok, snapshot = ros_interface.wait_for_attitude_level(
        duration_s=1.0,
        timeout_s=5.0,
        max_tilt_deg=ATTITUDE_MAX_TILT_DEG,
    )
    tilt = snapshot.get("recent_max_tilt_deg", snapshot.get("tilt_deg"))
    detail = (
        f"tilt={float(tilt):.1f} deg"
        if isinstance(tilt, (int, float))
        else str(snapshot.get("reason", "no attitude"))
    )
    if not ok:
        _append_test_step(steps, "attitude", "bad", detail)
        raise RuntimeError(f"attitude precheck failed: {detail}")
    _append_test_step(steps, "attitude", "ok", detail)


def _require_mode_ready(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
) -> None:
    ok, snapshot = ros_interface.wait_for_mode_ready(READY_MODES, timeout_s=3.0)
    detail = f"mode={snapshot.get('mode') or '--'}"
    if not ok:
        reason = str(snapshot.get("reason", detail))
        _append_test_step(steps, "mode", "bad", reason)
        raise RuntimeError(f"mode precheck failed: {reason}")
    _append_test_step(steps, "mode", "ok", detail)


def _require_vehicle_still(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
) -> None:
    ok, snapshot = ros_interface.wait_for_vehicle_still(
        duration_s=1.0,
        timeout_s=5.0,
        max_speed_mps=STILLNESS_MAX_SPEED_MPS,
    )
    speed = snapshot.get("recent_max_speed_mps")
    detail = (
        f"speed<={float(speed):.3f} m/s"
        if isinstance(speed, (int, float))
        else str(snapshot.get("reason", "no speed"))
    )
    if not ok:
        _append_test_step(steps, "stillness", "bad", detail)
        raise RuntimeError(f"stillness precheck failed: {detail}")
    _append_test_step(steps, "stillness", "ok", detail)


def _require_topic_alive(
    ros_interface: RosInterface,
    steps: list[dict[str, str]],
    topic_key: str,
    step_name: str,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    snapshot = {}
    while time.monotonic() < deadline:
        snapshot = ros_interface.status().get("topics", {}).get(topic_key, {})
        if snapshot.get("alive"):
            _append_test_step(steps, step_name, "ok", f"age={float(snapshot.get('age', 0.0)):.2f}s")
            return
        time.sleep(0.05)
    detail = str(snapshot.get("name", topic_key)) + " stale"
    _append_test_step(steps, step_name, "bad", detail)
    raise RuntimeError(f"{step_name} failed: {detail}")


def _dvl_quality_detail(snapshot: dict) -> str:
    reason = str(snapshot.get("reason", "DVL status unknown"))
    fom = snapshot.get("fom")
    altitude = snapshot.get("altitude")
    beams = snapshot.get("valid_beams")
    parts = [reason]
    if isinstance(fom, (int, float)):
        parts.append(f"fom={fom:.3f}")
    if isinstance(altitude, (int, float)):
        parts.append(f"alt={altitude:.2f}m")
    if isinstance(beams, (int, float)):
        parts.append(f"beams={int(beams)}")
    return ", ".join(parts)


def _analyze_and_write_bag(bag_path: str) -> dict:
    result = analyze_bag(bag_path)
    write_analysis_artifacts(result)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", default=8080, type=int)
    parser.add_argument("--robot-package", default="hit25_auv_ros2")
    parser.add_argument("--robot-launch", default="localization_test.launch.py")
    parser.add_argument("--start-dronecan-allocator", default="true")
    parser.add_argument("--dronecan-can-interface", default="can0")
    parser.add_argument("--dronecan-allocator-node-id", default=126, type=int)
    parser.add_argument("--dronecan-allocator-db", default="")
    parser.add_argument("--dronecan-python", default="")
    parser.add_argument("--pinger-package", default="kmu26_pinger_homing")
    parser.add_argument("--pinger-launch", default="pinger_homing_real.launch.py")
    parser.add_argument("--auto-start-phase-peak-scan", default="false")
    parser.add_argument("--phase-peak-use-audio-capture", default="false")
    parser.add_argument("--phase-peak-use-stamped-audio", default="true")
    parser.add_argument("--phase-peak-audio-device", default="")
    parser.add_argument("--phase-peak-audio-topic", default="/audio")
    parser.add_argument("--phase-peak-audio-stamped-topic", default="/audio_stamped")
    parser.add_argument("--phase-peak-audio-info-topic", default="/audio_info")
    parser.add_argument("--phase-peak-audio-sample-rate", default=96000, type=int)
    parser.add_argument("--phase-peak-min-frequency-hz", default=15000.0, type=float)
    parser.add_argument("--phase-peak-max-frequency-hz", default=25000.0, type=float)
    parser.add_argument(
        "--odom-topic",
        default=os.environ.get("KMU26_ODOM_TOPIC", "/odometry/filtered"),
    )
    parser.add_argument(
        "--depth-topic", default=os.environ.get("KMU26_DEPTH_TOPIC", "/depth/pose")
    )
    parser.add_argument(
        "--mavros-state-topic",
        default=os.environ.get("KMU26_MAVROS_STATE_TOPIC", "/mavros/state"),
    )
    parser.add_argument(
        "--pinger-homing-status-topic",
        default=os.environ.get("KMU26_PINGER_HOMING_STATUS_TOPIC", "/pinger_homing/status"),
    )
    parser.add_argument(
        "--hydrophone-direction-topic",
        default=os.environ.get("KMU26_HYDROPHONE_DIRECTION_TOPIC", "/homing/direction"),
    )
    args, _ = parser.parse_known_args()

    if not (
        15000.0
        <= args.phase_peak_min_frequency_hz
        < args.phase_peak_max_frequency_hz
        <= 25000.0
    ):
        parser.error(
            "Phase peak scan band must satisfy "
            "15000 <= min_frequency_hz < max_frequency_hz <= 25000"
        )

    topic_config = TopicConfig(
        odom=args.odom_topic,
        depth=args.depth_topic,
        mavros_state=args.mavros_state_topic,
        pinger_homing_status=args.pinger_homing_status_topic,
        hydrophone_direction=args.hydrophone_direction_topic,
    )

    app = create_app(
        robot_package=args.robot_package,
        robot_launch=args.robot_launch,
        start_dronecan_allocator=_parse_bool(args.start_dronecan_allocator),
        dronecan_can_interface=args.dronecan_can_interface,
        dronecan_allocator_node_id=args.dronecan_allocator_node_id,
        dronecan_allocator_db=args.dronecan_allocator_db,
        dronecan_python=args.dronecan_python,
        pinger_package=args.pinger_package,
        pinger_launch=args.pinger_launch,
        topic_config=topic_config,
        auto_start_phase_peak_scan=_parse_bool(args.auto_start_phase_peak_scan),
        phase_peak_scan_args={
            "use_audio_capture": (
                "true" if _parse_bool(args.phase_peak_use_audio_capture) else "false"
            ),
            "use_stamped_audio": (
                "true" if _parse_bool(args.phase_peak_use_stamped_audio) else "false"
            ),
            "audio_device": args.phase_peak_audio_device,
            "audio_topic": args.phase_peak_audio_topic,
            "audio_stamped_topic": args.phase_peak_audio_stamped_topic,
            "audio_info_topic": args.phase_peak_audio_info_topic,
            "audio_sample_rate": str(args.phase_peak_audio_sample_rate),
            "min_frequency_hz": str(args.phase_peak_min_frequency_hz),
            "max_frequency_hz": str(args.phase_peak_max_frequency_hz),
        },
    )
    uvicorn.run(app, host=args.host, port=args.port)


def _parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


if __name__ == "__main__":
    main()
