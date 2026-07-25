#!/usr/bin/env python3
"""Static contract checks for the native web GUI entry point."""

from __future__ import annotations

import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = Path(os.environ.get("UUV_WEB_GUI_WORKSPACE", ROOT.parents[1])).resolve()
ROS_WORKSPACE = WORKSPACE / "rospkg"
ROS_SOURCE = ROS_WORKSPACE / "src"
if not ROS_SOURCE.is_dir():
    ROS_SOURCE = ROS_WORKSPACE


def require_text(path: Path, needle: str, message: str) -> None:
    text = path.read_text(encoding="utf-8")
    if needle not in text:
        raise AssertionError(f"{message}: missing {needle!r} in {path}")


def reject_text(path: Path, needle: str, message: str) -> None:
    text = path.read_text(encoding="utf-8")
    if needle in text:
        raise AssertionError(f"{message}: unexpected {needle!r} in {path}")


def require_file(path: Path) -> None:
    if not path.is_file():
        raise AssertionError(f"missing required file: {path}")


def main() -> int:
    web_app = ROOT / "gui" / "web_app.py"
    web_process_manager = ROOT / "gui" / "web_process_manager.py"
    ros_package_build = ROOT / "gui" / "ros_package_build.py"
    ros_package_stack = ROOT / "gui" / "ros_package_stack.py"
    node_init_publishers = ROOT / "gui" / "node_init_publishers.py"
    node_bindings = ROOT / "gui" / "node_bindings.py"
    web_rc_replay = ROOT / "gui" / "web_rc_replay.py"
    web_tool_files = ROOT / "gui" / "web_tool_files.py"
    test_tank_layout_model = ROOT / "gui" / "test_tank_layout_model.py"
    stereo_camera_node = ROOT / "gui" / "node_stereo_camera.py"
    external_motion_subscriptions = ROOT / "gui" / "node_subscription_external_motion.py"
    yolo_detector = ROOT / "gui" / "yolo_buoy_detector.py"
    pinger_clock_barrier = ROOT / "gui" / "wait_for_mujoco_clock.py"
    pinger_rc_handoff_check = ROOT / "tools" / "check_gui_pinger_rc_handoff.py"
    # Developer checkouts keep the two control packages in a worktree group,
    # while the installer extracts each ROS package directly under src/.
    pinger_package = ROS_SOURCE / "kmu26_pinger_homing"
    grouped_pinger_package = (
        ROS_SOURCE / "kmu26_control_packages" / "kmu26_pinger_homing"
    )
    if not pinger_package.is_dir() and grouped_pinger_package.is_dir():
        pinger_package = grouped_pinger_package
    pinger_controller_cpp = (
        pinger_package / "src" / "pinger_homing" / "pinger_homing_controller.cpp"
    )
    pinger_launch = pinger_package / "launch" / "pinger_homing_real.launch.py"
    mission_package = ROS_SOURCE / "kmu26_mission_fsm"
    grouped_mission_package = (
        ROS_SOURCE / "kmu26_control_packages" / "kmu26_vision_mission_fsm"
    )
    archived_mission_package = WORKSPACE / "archive" / "kmu26_vision_mission_fsm"
    if not mission_package.is_dir():
        if grouped_mission_package.is_dir():
            mission_package = grouped_mission_package
        elif archived_mission_package.is_dir():
            mission_package = archived_mission_package
    mission_ros_cmake = mission_package / "CMakeLists.txt"
    mission_ros_package_xml = mission_package / "package.xml"
    web_entry = ROOT / "gui" / "web_control_gui.py"
    mavros_schedule = ROOT / "bridge" / "ros2_publish_schedule_mavros.py"
    index = ROOT / "gui" / "web_static" / "index.html"
    app_js = ROOT / "gui" / "web_static" / "app.js"
    style_css = ROOT / "gui" / "web_static" / "style.css"
    launcher = WORKSPACE / "run_control_gui.sh"
    process_manager_text = web_process_manager.read_text(encoding="utf-8")
    pinger_start_body = process_manager_text.split(
        "    def start_pinger_homing", 1
    )[1].split("    def stop_pinger_homing", 1)[0]

    for path in (
        web_app,
        web_process_manager,
        ros_package_build,
        ros_package_stack,
        node_init_publishers,
        node_bindings,
        web_rc_replay,
        web_tool_files,
        test_tank_layout_model,
        stereo_camera_node,
        external_motion_subscriptions,
        yolo_detector,
        pinger_clock_barrier,
        pinger_rc_handoff_check,
        pinger_controller_cpp,
        pinger_launch,
        mission_ros_cmake,
        mission_ros_package_xml,
        web_entry,
        mavros_schedule,
        index,
        app_js,
        style_css,
        launcher,
    ):
        require_file(path)

    require_text(web_app, 'parsed.path == "/api/status"', "web GUI must expose status telemetry")
    require_text(
        external_motion_subscriptions,
        '"/sim/odom"',
        "simulator GUI telemetry must subscribe to live simulation odometry",
    )
    require_text(
        external_motion_subscriptions,
        '"/odometry/filtered"',
        "real GUI telemetry must subscribe to the filtered vehicle odometry contract",
    )
    require_text(web_app, '"/api/stereo/left.jpg"', "web GUI must expose left stereo camera JPEG")
    require_text(web_app, '"/api/stereo/right.jpg"', "web GUI must expose right stereo camera JPEG")
    require_text(web_app, "self.node.stereo_camera_status()", "web GUI status must include stereo camera state")
    require_text(web_app, "self.controller.node.stereo_camera_frame(side)", "web GUI must serve cached stereo camera frames")
    require_text(web_app, 'parsed.path not in {"/api/command", "/api/rc"}', "web GUI must expose command API")
    require_text(web_app, "self.node.publish_rc_override", "web GUI must reuse existing RC override publisher")
    require_text(
        web_app,
        "self.node.suspend_rc_override_publisher()",
        "pinger start must suspend the competing web GUI RC publisher",
    )
    require_text(
        web_app,
        'self._restore_pinger_rc_publisher("process exited")',
        "pinger process exit must restore the web GUI RC publisher",
    )
    require_text(
        web_app,
        '{"pinger_homing_running": self.processes.pinger_homing_running()}',
        "web spin loop must restore RC ownership even without an HTTP status poll",
    )
    require_text(
        node_init_publishers,
        "self.publish_rc_release()",
        "RC handoff must publish release before destroying the GUI publisher",
    )
    require_text(
        node_init_publishers,
        "self.destroy_publisher(publisher)",
        "RC handoff must remove the GUI MAVROS publisher",
    )
    require_text(
        node_init_publishers,
        "def restore_rc_override_publisher",
        "GUI RC ownership must be restorable after pinger homing",
    )
    require_text(
        node_bindings,
        "cls.suspend_rc_override_publisher",
        "UuvGuiNode must bind RC handoff methods",
    )
    require_text(web_app, "self.node.publish_manual_control", "web GUI must reuse existing MANUAL_CONTROL publisher")
    require_text(web_app, "WebProcessManager", "web GUI must keep simulator process logic outside web_app")
    require_text(web_app, "WebRcReplayManager", "web GUI must keep replay logic outside web_app")
    require_text(web_process_manager, "build_sim_stack_launch_command", "web process manager must reuse GUI sim launch command")
    require_text(
        ros_package_build,
        '"env MAKEFLAGS=-j1 "',
        "GUI ROS builds must limit the inner CMake build to one compiler job",
    )
    require_text(
        ros_package_build,
        '"colcon build --executor sequential --base-paths "',
        "GUI ROS builds must not compile independent packages concurrently",
    )
    require_text(
        ros_package_stack,
        '"use_sim_time:=true"',
        "GUI MuJoCo MAVROS/EKF stack must share the simulated sensor clock",
    )
    require_text(
        web_process_manager,
        '"_ros_pkg_process", "_ros_pkg_status", "real ROS stack"',
        "web Stack Stop must stop the auto-started real ROS stack",
    )
    require_text(web_process_manager, "terminate_process_group", "web process manager must terminate child process groups")
    require_text(
        web_process_manager,
        '"auv_pinger_homing"',
        "sim pinger start must use the standalone C++ pinger package",
    )
    require_text(
        web_process_manager,
        '"pinger_homing_real.launch.py"',
        "sim pinger start must use the fixed-frequency physical launch",
    )
    reject_text(
        web_process_manager,
        "phase_controller_runner",
        "sim pinger start must not rebuild the phase process graph in the GUI",
    )
    reject_text(
        web_process_manager,
        "snr_controller_runner",
        "sim pinger start must not rebuild the SNR process graph in the GUI",
    )
    reject_text(
        web_process_manager,
        "single_hydrophone_homing_controller.py",
        "the web runtime must not execute the Python pinger controller",
    )
    require_text(
        web_process_manager,
        "forward_default = 0.48",
        "sim Phase must retain the physical forward default",
    )
    require_text(
        web_process_manager,
        'values.get("yaw_gain", 0.85)',
        "sim Phase must retain the physical yaw gain",
    )
    require_text(
        web_process_manager,
        '"use_sim_time:=false"',
        "sim pinger must use the physical estimator receive-time contract",
    )
    require_text(
        web_process_manager,
        '"audio_input_latency_s:=0.0"',
        "sim pinger must not add a simulator-only receive latency",
    )
    require_text(
        web_process_manager,
        "clock_barrier_command",
        "sim pinger homing must wait for a stable non-zero MuJoCo clock",
    )
    require_text(
        web_process_manager,
        '"dry_run:=false"',
        "sim pinger start must enable controller output",
    )
    if '"use_rc_mux:=true"' in pinger_start_body:
        raise AssertionError("sim parity must not resurrect the removed RC mux")
    if "rc_mux_stale_timeout" in pinger_start_body:
        raise AssertionError("sim parity must not carry a simulator-only RC lease")
    require_text(
        web_process_manager,
        '"direction_topic:=/homing/direction"',
        "sim pinger controller must consume the hydrophone package contract",
    )
    require_text(
        web_process_manager,
        '"amplitude_range_constant:=0.01950"',
        "sim homing must use the end-to-end PCM/IQ acoustic range calibration",
    )
    require_text(
        web_process_manager,
        "no MuJoCo pose or pinger ground truth reaches the controller",
        "sim acoustic completion must remain independent of the ground-truth pose",
    )
    reject_text(
        web_process_manager,
        '"auto_select_top:=true"',
        "fixed-frequency GUI launch must not auto-select a competing FFT peak",
    )
    reject_text(
        web_process_manager,
        '"scan_monitor_s:=10.0"',
        "fixed-frequency GUI launch must bypass the interactive FFT scan",
    )
    require_text(
        web_process_manager,
        '"reference_frequency_hz"',
        "sim pinger launch must pass the validated fixed reference frequency",
    )
    require_text(
        web_process_manager,
        '"rc_topic:=/mavros/rc/override"',
        "sim and real must share the direct MAVROS RC topic",
    )
    require_text(
        web_process_manager,
        '"mode:=ALT_HOLD"',
        "sim and real must share the ALT_HOLD controller gate",
    )
    require_text(
        ROOT / "bridge" / "ros2_hydrophone_sim.py",
        '"/pinger_homing/direction_body"',
        "viewer must subscribe to the dedicated C++ pinger body direction",
    )
    require_text(
        web_app,
        "self.processes.simulation_runtime_available()",
        "pinger start must bring up a stopped simulator",
    )
    require_text(
        web_app,
        "web pinger homing waiting for MAVROS vehicle state",
        "cold pinger start must defer mode and arm until MAVROS is connected",
    )
    require_text(
        web_app,
        'mode = "ALT_HOLD"',
        "sim GUI must request the same flight mode required by the physical controller",
    )
    reject_text(
        web_app,
        '"pinger_homing_neutral"',
        "pinger stop must not reclaim MAVROS channels with a post-release neutral override",
    )
    require_text(
        web_app,
        "pinger homing stopped; RC override released",
        "pinger stop must finish with the neutral-then-release RC handoff",
    )
    require_text(
        pinger_controller_cpp,
        '"PROBE"',
        "single-hydrophone homing must execute an observable probe manoeuvre",
    )
    require_text(
        pinger_controller_cpp,
        '"CONTACT"',
        "single-hydrophone homing must include a contact state",
    )
    require_text(web_rc_replay, "publish_rc_channels", "web replay manager must publish RC override replay samples")
    require_text(web_tool_files, "PHYSICS_PROFILE_PATH", "web tool file manager must constrain physics editor path")
    require_text(web_tool_files, "PHYSICS_PARAM_SPECS", "web physics editor must use the same param specs as the Tk GUI")
    require_text(web_tool_files, "load_buoy_layout", "web course editor must use the same buoy layout parser as the Tk GUI")
    require_text(web_tool_files, "save_buoy_layout", "web course editor must use the same buoy layout writer as the Tk GUI")
    require_text(web_app, "threading.Thread(target=httpd.shutdown", "web GUI signal shutdown must not deadlock serve_forever")
    require_text(web_app, "self.node.probe_backend()", "web GUI must refresh backend graph detection like the Tk GUI")
    require_text(
        mavros_schedule,
        'jobs.add(self.pub_mavros_state, "/mavros/state", builders["mavros_state"], on_demand=False)',
        "MAVROS state must publish continuously so arming has fresh vehicle feedback",
    )
    require_text(web_entry, "from gui.web_app import main", "direct web entry must bootstrap package imports")
    for needle in (
        "UUV Control GUI",
        "MuJoCo, ROS2, MAVROS, RViz, Ping360",
        "Vehicle Summary",
        "Attitude",
        "Depth",
        "RC Feedback",
        "Events",
        "Simulation Stack",
        "ROS2 MAVROS / RViz",
        "Pilot Control",
        "Stereo Camera",
        "Mission FSM",
        "Mission Check",
        "Sensor-driven C++ mission",
        "Left Stick",
        "Right Stick",
        "Tools",
        "RC Override Replay",
        "Sim Param Tuning",
        "Course Layout",
        "Open Physics Params",
        "Open XY Course",
        "Physics Params",
        "XY Course Layout",
        "Apply to File",
        "Test tank",
        "Save Layout",
        "Save + Restart Sim",
        "Raw JSON",
        "Raw XML",
        "Center Sticks",
        "Ping360 panel",
        "Tool editor",
        "heave +",
        "yaw -",
        "forward +",
        "lateral +",
    ):
        require_text(index, needle, f"web GUI must preserve Tk layout section {needle}")
    require_text(index, 'id="axisForward"', "web GUI must expose forward RC axis")
    require_text(index, 'id="stereoLeftImage"', "web GUI must expose one stereo image element")
    require_text(index, 'id="stereoCameraEnabled"', "web GUI must expose stereo camera receive toggle")
    require_text(index, 'id="stereoCameraVisionEnabled"', "web GUI must expose annotated vision toggle")
    require_text(index, 'id="pilotControlGroup"', "web GUI must identify the pilot control panel for camera overlay docking")
    require_text(index, 'id="stereoCameraProfile"', "web GUI must expose stereo camera launch profile select")
    require_text(index, 'id="stereoCameraApplyBtn"', "web GUI must expose stereo camera profile apply control")
    require_text(index, 'id="stereoCameraSaveBtn"', "web GUI must expose stereo camera profile save control")
    require_text(index, 'id="stereoCameraZoomBtn"', "web GUI must expose stereo camera zoom control")
    require_text(index, 'id="missionFsmPanel"', "web GUI must expose sensor mission FSM panel")
    require_text(index, 'id="missionMonitorPanel"', "web GUI must expose bottom mission inspection panel")
    require_text(index, 'id="missionMonitorState"', "web GUI must expose live mission FSM state")
    require_text(index, 'id="missionMonitorCollectorEq"', "web GUI must expose collector weld state")
    require_text(index, 'id="missionBuoyRows"', "web GUI must expose live buoy state rows")
    require_text(index, 'id="missionStartBtn"', "web GUI must expose mission FSM start control")
    require_text(index, 'id="missionStopBtn"', "web GUI must expose mission FSM stop control")
    require_text(index, 'id="missionMaxTargets"', "web GUI must expose mission target count control")
    require_text(
        index,
        'id="pingerHomingAutoArm" type="checkbox" checked',
        "sim pinger start must arm by default for one-click homing",
    )
    reject_text(index, 'id="stereoRightImage"', "web GUI must show only one stereo camera pane")
    require_text(index, 'id="leftStick"', "web GUI must expose left virtual joystick")
    require_text(index, 'id="rightStick"', "web GUI must expose right virtual joystick")
    require_text(index, 'id="modeButtons"', "web GUI must expose flight mode controls")
    require_text(index, 'id="physicsDialog"', "web GUI must expose structured physics params dialog")
    require_text(index, 'id="physicsRows"', "web GUI must expose physics params rows")
    require_text(index, 'id="courseDialog"', "web GUI must expose structured course layout dialog")
    require_text(index, 'id="courseCanvas"', "web GUI must expose top-view course canvas")
    require_text(index, 'id="courseRows"', "web GUI must expose course item rows")
    require_text(index, 'id="courseModeSelect"', "web GUI must expose course/test-tank mode selection")
    require_text(index, 'id="courseTankDimensions"', "web GUI must expose active tank dimensions")
    require_text(
        test_tank_layout_model,
        'TEST_TANK_LENGTH_M = 5.49',
        "test-tank runtime must preserve the measured tank length",
    )
    require_text(
        test_tank_layout_model,
        'TEST_TANK_PINGER_SITE_NAME = "test_tank_pinger_acoustic_site"',
        "test-tank runtime must expose a standalone hydrophone pinger site",
    )
    require_text(
        web_process_manager,
        "prepare_active_course_runtime",
        "GUI stack start must select the saved course or test-tank scene",
    )
    for command in (
        'command: "stack_start"',
        'command: "stack_reset"',
        'command: "ros_build"',
        'command: "mavros_toggle"',
        'command: "rviz_toggle"',
        'command: "rc_replay_play"',
        'command: "ping360_config"',
        'command: "stereo_camera_enabled"',
        'command: "vision_processing_enabled"',
        'command: "camera_config"',
        'command: "gt_mission_start"',
        'command: "gt_mission_stop"',
        'command: "physics_load"',
        'command: "physics_apply"',
        'command: "course_load"',
        'command: "course_save"',
        'command: "tool_read"',
        'command: "tool_save"',
    ):
        require_text(app_js, command, f"web UI must wire {command}")
    for command in (
        'if command == "stack_start"',
        'if command == "ros_build"',
        'if command == "mavros_toggle"',
        'if command == "rviz_toggle"',
        'if command == "rc_replay_play"',
        'if command == "ping360_view_start"',
        'if command == "stereo_camera_enabled"',
        'if command == "vision_processing_enabled"',
        'if command == "camera_config"',
        'if command == "gt_mission_start"',
        'if command == "gt_mission_stop"',
        'if command == "physics_load"',
        'if command == "physics_apply"',
        'if command == "course_load"',
        'if command == "course_save"',
        'if command == "tool_read"',
        'if command == "tool_save"',
    ):
        require_text(web_app, command, f"web API must handle {command}")
    require_text(
        web_app,
        'UUV_GUI_VISION_AUTO_START", "0"',
        "web GUI annotated vision preview must be opt-in at idle",
    )
    for preview_contract in (
        '"OMP_NUM_THREADS=1"',
        '"MKL_NUM_THREADS=1"',
        '"OPENBLAS_NUM_THREADS=1"',
        '"NUMEXPR_NUM_THREADS=1"',
        '"annotated_image_topic:=/vision/buoy/image_annotated/compressed"',
        '"publish_annotated_image:=true"',
        '"imgsz:=1280"',
        '"publish_per_class:=true"',
    ):
        require_text(
            web_process_manager,
            preview_contract,
            "explicit vision preview must retain the bounded CPU contract",
        )
    require_text(
        web_process_manager,
        'mission_pose_topic = "/sim/odom" if simulation_running else "/odometry/filtered"',
        "mission launch must select simulation or real odometry at runtime",
    )
    require_text(
        web_process_manager,
        'f"transport:={mission_transport}"',
        "mission launch must preserve the selected command transport",
    )
    require_text(app_js, 'fetch("/api/status"', "web UI must poll telemetry status")
    require_text(app_js, "enablePilotInputFromJoystick", "web joystick must auto-enable pilot input")
    require_text(app_js, 'document.body.classList.toggle("camera-expanded"', "expanded camera view must expose pilot controls")
    require_text(app_js, "setPilotControlDocked", "expanded camera view must move pilot controls above the camera layer")
    require_text(app_js, "document.body.appendChild(pilot)", "expanded camera pilot controls must dock at body level")
    require_text(app_js, "renderStereoCamera", "web UI must render stereo camera status")
    require_text(app_js, "yoloStatusText", "web UI must render YOLO buoy overlay status")
    require_text(app_js, "center_px", "web UI must render YOLO detection center coordinates")
    require_text(app_js, "renderCameraConfig", "web UI must render stereo camera profile status")
    require_text(app_js, "applyCameraConfig", "web UI must apply stereo camera launch profiles")
    require_text(app_js, "missionPayload", "web UI must build ground-truth mission launch payload")
    require_text(app_js, "missionFsmStatus", "web UI must render ground-truth mission status")
    require_text(app_js, "renderMissionMonitor", "web UI must render bottom mission inspection status")
    require_text(app_js, "collector_eq_active", "web mission panel must render collector weld activity")
    require_text(app_js, "capture_state", "web mission panel must render physical capture state")
    require_text(app_js, "renderMissionBuoys", "web UI must render live buoy state rows")
    require_text(app_js, "physical_detached", "web mission panel must expose physical detach status")
    require_text(app_js, "coordinate_source", "web mission panel must show live/static coordinate source")
    require_text(
        app_js,
        '$("stereoCameraApplyBtn").addEventListener("click", () => applyCameraConfig(true)',
        "web UI Apply must restart the simulator so launch-only camera profiles take effect",
    )
    require_text(app_js, 'image.src = `/api/stereo/${side}.jpg?seq=${seq}`', "web UI must refresh stereo JPEG frames")
    require_text(app_js, "toggleStereoCameraZoom", "web UI must support expanding the stereo camera pane")
    require_text(app_js, 'event.key === "Escape"', "web UI must close expanded stereo camera with Escape")
    reject_text(app_js, 'updateStereoView("right"', "web UI must refresh only one stereo camera pane")
    require_text(stereo_camera_node, 'SUBSCRIBED_SIDES = ("left",)', "web GUI ROS node must subscribe only to the visible stereo camera")
    require_text(stereo_camera_node, "destroy_subscription", "web GUI ROS node must drop camera subscription when disabled")
    require_text(stereo_camera_node, "create_yolo_buoy_detector", "web GUI camera cache must enable YOLO buoy overlay")
    require_text(stereo_camera_node, "detector.process_rgb", "web GUI camera cache must draw detection overlays before JPEG encoding")
    require_text(stereo_camera_node, '"detection"', "web GUI camera status must expose YOLO detection status")
    require_text(
        stereo_camera_node,
        '"/vision/buoy/image_annotated/compressed"',
        "web GUI must subscribe to the vision package annotated image",
    )
    require_text(
        stereo_camera_node,
        "set_stereo_camera_display_mode",
        "web GUI must switch between raw and annotated camera feeds",
    )
    require_text(yolo_detector, "from ultralytics import YOLO", "YOLO detector must support the provided .pt model")
    require_text(yolo_detector, "draw_buoy_detections", "YOLO detector must draw OpenCV bounding boxes")
    require_text(yolo_detector, '"detections"', "YOLO detector status must expose detection box coordinates")
    require_text(yolo_detector, '"xyxy"', "YOLO detector status must expose pixel box coordinates")
    require_text(yolo_detector, '"center_px"', "YOLO detector status must expose pixel center coordinates")
    require_text(yolo_detector, '"center_norm"', "YOLO detector status must expose normalized center coordinates")
    require_text(yolo_detector, "cv2.rectangle", "YOLO detector must use OpenCV rectangle drawing")
    require_text(yolo_detector, "cv2.putText", "YOLO detector must use OpenCV label drawing")
    require_text(style_css, ".pilot-group.camera-pilot-docked", "expanded camera view must keep the joystick reachable")
    require_text(style_css, "z-index: 1000", "docked pilot controls must stay above expanded camera layers")
    require_text(app_js, 'fetch("/api/rc"', "web UI must publish RC commands")
    require_text(web_app, 'self.path.startswith("/api/rc")', "web API must route /api/rc to RC command handling")
    require_text(
        web_app,
        "self._rc_client_sequences",
        "web RC backend must track monotonic sequence numbers per browser client",
    )
    require_text(
        web_app,
        "sequence <= previous[0]",
        "web RC backend must reject stale or duplicate client frames",
    )
    require_text(
        web_app,
        "def _check_rc_watchdog",
        "web RC backend must release controls after client receive loss",
    )
    require_text(
        web_app,
        '"UUV_GUI_RC_WATCHDOG_S"',
        "web RC watchdog timeout must remain configurable",
    )
    require_text(
        app_js,
        "rcRequestInFlight",
        "web RC sender must keep only one HTTP request in flight",
    )
    require_text(
        app_js,
        "state.rcPending = payload",
        "web RC sender must retain only the latest unsent frame",
    )
    require_text(
        app_js,
        "seq: nextRcSequence()",
        "web RC sender must attach a monotonic sequence to every frame",
    )
    require_text(
        app_js,
        "navigator.sendBeacon",
        "web RC sender must release control during page teardown",
    )
    require_text(
        app_js,
        'window.addEventListener("pagehide", releaseRcForInactivePage)',
        "web RC sender must bind its pagehide release",
    )
    require_text(
        app_js,
        'document.addEventListener("visibilitychange"',
        "hidden web tabs must release pilot control",
    )
    require_text(
        app_js,
        "document.hidden || state.statusPollBusy",
        "status polling must be hidden-tab guarded and single-flight",
    )
    require_text(
        app_js,
        "document.hidden || !state.stereoCameraEnabled || state.cameraPollBusy",
        "camera polling must be hidden-tab guarded and single-flight",
    )
    require_text(app_js, "bindJoystick", "web UI must implement virtual stick controls")
    require_text(app_js, "startJoystickDrag", "web virtual sticks must own pointer drag sessions")
    require_text(app_js, 'window.addEventListener("pointermove"', "web virtual sticks must keep dragging outside the pad")
    require_text(app_js, "applyJoystickDrag(session, event, { force: true })", "web virtual sticks must send the first drag frame immediately")
    require_text(app_js, "setStickAxes(session.mapping, 0, 0", "web virtual sticks must spring back to center on release")
    require_text(app_js, "const control = payload.control || {}", "web UI must read backend pilot-control state")
    require_text(app_js, "rcToggle.checked = state.rcEnabled", "web UI must sync Pilot input checkbox from backend state")
    require_text(app_js, "renderPhysicsRows", "web UI must render structured physics params rows")
    require_text(app_js, "collectPhysicsValues", "web UI must collect structured physics params values")
    require_text(app_js, "renderCourseCanvas", "web UI must render top-view XY course canvas")
    require_text(app_js, "bindCourseCanvas", "web UI must support course canvas selection and dragging")
    require_text(app_js, "courseSavePayload", "web UI must save structured XY course positions")
    require_text(app_js, "quickPhysicsOpenBtn", "web UI must expose quick physics params action")
    require_text(app_js, "quickCourseOpenBtn", "web UI must expose quick XY course action")
    require_text(style_css, ".axis-grid", "web UI must include stable axis layout styling")
    require_text(style_css, ".stick-pad", "web UI must style virtual stick controls")
    require_text(style_css, ".stereo-camera-grid", "web UI must lay out stereo camera frames")
    require_text(style_css, ".stereo-camera-group.expanded", "web UI must style expanded stereo camera mode")
    require_text(style_css, ".mission-fsm-group", "web UI must style the bottom mission FSM panel")
    require_text(style_css, ".mission-monitor-panel", "web UI must style the bottom mission inspection panel")
    require_text(style_css, ".mission-buoy-table", "web UI must style live buoy state table")
    require_text(index, "missionMonitorRobotState", "web mission monitor must show robot mission state")
    require_text(app_js, "mission.robot_state_label", "web mission monitor must render robot mission state")
    require_text(web_process_manager, "start_ground_truth_mission", "web process manager must launch ground-truth mission controller")
    require_text(pinger_launch, '("/audio_boosted", audio_topic)', "canonical launch must pass the parameterized simulator/real PCM topic to the pinned estimator")
    reject_text(web_process_manager, "/homing/direction:=/homing/direction_estimated_world", "simulator must preserve the hydrophone package output topic")
    require_text(web_process_manager, '"odometry_topic:=/odometry/filtered"', "sim Phase must retain the real filtered-odometry contract")
    reject_text(web_process_manager, '"odometry_topic:=/pinger_homing/disabled/odometry"', "real-parity simulation must not bypass localization")
    reject_text(web_process_manager, '"ROS2_UUV_HYDROPHONE_INTERFERER_COUNT": "2"', "sim Phase must not install the clean two-source acoustic shortcut")
    reject_text(web_process_manager, '"ROS2_UUV_HYDROPHONE_INTERFERER_THRUSTER_COUNT": "0"', "sim Phase must not disable propulsion-correlated noise")
    require_text(pinger_controller_cpp, "world_vector_to_body_flu", "pinger controller must transform its source estimate into body FLU")
    require_text(pinger_controller_cpp, '"/pinger_homing/direction_body"', "active estimate must drive the red direction arrow")
    require_text(pinger_controller_cpp, "publish_phase_direction_if_available(now)", "probe-time EKF estimates must drive the red direction arrow")
    require_text(pinger_controller_cpp, "probe_command", "single-hydrophone controller must actively excite independent motion axes")
    require_text(pinger_controller_cpp, "stabilized_yaw_command", "pinger yaw control must damp and slew-limit RC commands")
    require_text(pinger_controller_cpp, "limit_heave_by_vehicle_depth", "pinger depth safety must guard every RC publish path")
    require_text(web_process_manager, 'f"yaw_gain:={yaw_gain:.6f}"', "web yaw gain must reach the pinger controller")
    require_text(web_process_manager, "far_forward_limit = 0.55 if test_tank_active else 0.78", "competition far-field homing must not inherit the small-tank speed cap")
    require_text(web_process_manager, 'f"tank_max_depth_m:={tank_max_depth_m:.6f}"', "web tank depth must reach the automatic 3D controller")
    require_text(web_process_manager, 'f"success_range_m:={success_range_m:.6f}"', "web acoustic success range must reach the controller")
    require_text(web_process_manager, 'complete (acoustic range)', "web homing result must distinguish sensor-range success from physical capture")
    require_text(index, 'id="pingerHomingTankMaxDepth"', "web pinger panel must expose only the tank maximum depth")
    reject_text(index, 'id="pingerHomingExpectedDepth"', "test-tank GUI must not require a pinger-depth prior")
    reject_text(index, 'id="pingerHomingVehicleTargetDepth"', "test-tank GUI must not require a fixed vehicle Z target")
    reject_text(index, 'id="pingerHomingMaxDepth"', "test-tank GUI must derive base-link depth safety from tank depth")
    require_text(index, 'id="pingerHomingSuccessRange"', "web pinger panel must expose acoustic range completion")
    require_text(app_js, "tank_max_depth_m", "web pinger payload must include only the tank maximum depth prior")
    require_text(index, 'id="pingerHomingAlgorithm"', "web pinger panel must expose the physical Phase mode")
    require_text(
        index,
        '<option value="phase" selected>Phase / odometry (real parity)</option>',
        "web pinger panel must default to the physical Phase/odometry controller",
    )
    reject_text(index, 'value="no_odom_phase"', "real-parity simulator GUI must not advertise an odometry bypass")
    reject_text(index, 'value="snr"', "real-parity simulator GUI must not label the Phase launch as SNR")
    require_text(
        index,
        'id="pingerHomingProbePwmDelta" type="number" min="1" max="250" step="1" value="20"',
        "web pinger panel must expose the physical 20 PWM probe delta",
    )
    require_text(
        index,
        'id="pingerHomingApproachPwmDelta" type="number" min="1" max="250" step="1" value="25"',
        "web pinger panel must expose the physical 25 PWM approach delta",
    )
    reject_text(index, 'id="pingerHomingApproachDuration"', "adaptive physical re-estimation must not expose the obsolete fixed no-odom leg")
    require_text(index, 'id="pingerHomingMode" disabled', "web pinger mode must be fixed to ALT_HOLD")
    require_text(index, '<option value="ALT_HOLD" selected>ALT_HOLD</option>', "web pinger mode must match the physical launch")
    reject_text(index, '<option value="MANUAL">MANUAL</option>', "web pinger panel must not advertise unsupported MANUAL mode")
    reject_text(index, 'id="pingerHomingUseYolo"', "standalone acoustic homing must not advertise a dead YOLO-final toggle")
    reject_text(index, 'id="pingerHomingYoloRange"', "standalone acoustic homing must not advertise a dead YOLO handoff range")
    require_text(
        app_js,
        'probe_pwm_delta: numericValue("pingerHomingProbePwmDelta", 20)',
        "web pinger start payload must forward the physical probe PWM delta",
    )
    require_text(
        app_js,
        'approach_pwm_delta: numericValue("pingerHomingApproachPwmDelta", 25)',
        "web pinger start payload must forward the physical approach PWM delta",
    )
    require_text(app_js, 'mode: "ALT_HOLD"', "web pinger start must fix the vehicle mode to ALT_HOLD")
    require_text(app_js, 'use_yolo_final: false', "standalone acoustic homing must explicitly disable dead vision handoff")
    require_text(
        index,
        'id="pingerHomingForwardFast" type="number" min="0" max="1" step="0.01" value="0.48"',
        "web pinger panel must display the legacy Python Phase forward default",
    )
    require_text(
        index,
        'id="pingerHomingYawGain" type="number" min="0.1" max="2" step="0.05" value="0.85"',
        "web pinger panel must display the legacy Python Phase yaw default",
    )
    require_text(
        app_js,
        'forward_fast: numericValue("pingerHomingForwardFast", 0.48)',
        "web pinger payload must retain the legacy Python Phase forward fallback",
    )
    require_text(
        app_js,
        'yaw_gain: numericValue("pingerHomingYawGain", 0.85)',
        "web pinger payload must retain the legacy Python Phase yaw fallback",
    )
    require_text(
        app_js,
        '$("pingerHomingForwardFast").value = "0.48"',
        "web GUI must restore the physical Phase forward default",
    )
    require_text(
        app_js,
        '$("pingerHomingYawGain").value = "0.85"',
        "web GUI must restore the physical Phase yaw default",
    )
    require_text(web_process_manager, '"odometry_topic:=/odometry/filtered"', "ordinary sim pinger modes must consume the real-vehicle odometry contract")
    require_text(web_process_manager, '"navigation_mode:=odometry"', "sim pinger launch must fix the physical localization mode")
    require_text(web_process_manager, '"state_topic:=/mavros/state"', "sim pinger controller must consume the real-vehicle MAVROS state contract")
    require_text(web_process_manager, 'f"max_runtime_s:={max_runtime_s:.6f}"', "web pinger launcher must pass a scene-appropriate runtime deadline")
    require_text(
        web_process_manager,
        'values.get("max_runtime_s", 180.0)',
        "sim Phase must use the physical runtime default",
    )
    require_text(web_process_manager, "include_workspace=True", "web pinger launch must source the packaged ROS workspace")
    require_text(
        web_process_manager,
        "status_grace_deadline = time.monotonic() + 0.50",
        "sim pinger watcher must preserve the final ROS status before terminating the launch group",
    )
    require_text(web_app, "camera feed disabled during pinger homing", "pinger validation must disable expensive camera rendering")
    reject_text(web_process_manager, "pinger_homing_capture_guard.py", "standalone pinger approach must remain independent of mission collector state")
    require_text(web_process_manager, "def mission_running", "web process manager must expose mission-running ownership")
    require_text(web_process_manager, "ground_truth_buoy_fsm", "web mission launcher must prefer the C++ mission FSM when installed")
    require_text(web_process_manager, "cpp_runner", "web mission launcher must execute the resolved C++ FSM path")
    require_text(web_process_manager, "ros_bash_command(command", "web mission launcher must keep the ROS environment for the C++ FSM")
    require_text(
        web_process_manager,
        'mission_transport = "rc_override" if external_mavros_controls else "command_override"',
        "external MAVROS mission control must publish RC override instead of a disabled simulator command topic",
    )
    require_text(index, '<option value="rc_override" selected>', "web mission must default to the live MAVROS RC path")
    require_text(web_process_manager, '"--wait-armed"', "one-click web mission start must wait for automatic arm")
    require_text(web_process_manager, "--status-json", "web mission launcher must request live mission status JSON")
    require_text(mission_ros_cmake, "ground_truth_buoy_fsm", "mission ROS package must build the C++ simulator oracle")
    require_text(mission_ros_cmake, "src/ground_truth_buoy_fsm.cpp", "mission ROS package must own its simulator-oracle source")
    require_text(mission_ros_package_xml, "<depend>std_msgs</depend>", "mission ROS package manifest must declare its string-message dependency")
    require_text(web_app, "mission_monitor", "web API must expose live mission monitor payload")
    require_text(web_app, "web pilot control released for mission", "web mission start must release pilot RC ownership")
    require_text(web_app, "web mission auto arm requested", "web mission start must arm the vehicle automatically")
    require_text(web_app, "self.processes.mission_running()", "web RC publisher must not override active mission commands")
    require_text(style_css, ".stereo-view.no-signal::before", "web UI must reserve camera space without signal")
    require_text(style_css, ".stick-pad:not(.dragging) .stick-knob", "web virtual stick must animate back to center")
    require_text(style_css, ".tool-action-row", "web tools panel must keep tuning/layout actions scannable")
    require_text(style_css, ".physics-row", "web UI must style structured physics params rows")
    require_text(style_css, ".course-editor", "web UI must style the XY course layout editor")
    require_text(style_css, "#courseCanvas", "web UI must style the top-view course canvas")
    require_text(style_css, ".group legend", "web UI must preserve Tk LabelFrame-like groups")
    require_text(launcher, "--web)", "run_control_gui must accept --web frontend switch")
    require_text(launcher, "UUV_GUI_FRONTEND", "run_control_gui must accept UUV_GUI_FRONTEND")
    require_text(launcher, "web_control_gui.py", "run_control_gui must select web_control_gui.py")

    print("web_gui_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
