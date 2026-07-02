#!/usr/bin/env python3
"""Static contract checks for the native web GUI entry point."""

from __future__ import annotations

import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = Path(os.environ.get("UUV_WEB_GUI_WORKSPACE", ROOT.parents[1])).resolve()


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
    web_rc_replay = ROOT / "gui" / "web_rc_replay.py"
    web_tool_files = ROOT / "gui" / "web_tool_files.py"
    stereo_camera_node = ROOT / "gui" / "node_stereo_camera.py"
    yolo_detector = ROOT / "gui" / "yolo_buoy_detector.py"
    web_entry = ROOT / "gui" / "web_control_gui.py"
    mavros_schedule = ROOT / "bridge" / "ros2_publish_schedule_mavros.py"
    index = ROOT / "gui" / "web_static" / "index.html"
    app_js = ROOT / "gui" / "web_static" / "app.js"
    style_css = ROOT / "gui" / "web_static" / "style.css"
    launcher = WORKSPACE / "run_control_gui.sh"

    for path in (
        web_app,
        web_process_manager,
        web_rc_replay,
        web_tool_files,
        stereo_camera_node,
        yolo_detector,
        web_entry,
        mavros_schedule,
        index,
        app_js,
        style_css,
        launcher,
    ):
        require_file(path)

    require_text(web_app, 'parsed.path == "/api/status"', "web GUI must expose status telemetry")
    require_text(web_app, '"/api/stereo/left.jpg"', "web GUI must expose left stereo camera JPEG")
    require_text(web_app, '"/api/stereo/right.jpg"', "web GUI must expose right stereo camera JPEG")
    require_text(web_app, "self.node.stereo_camera_status()", "web GUI status must include stereo camera state")
    require_text(web_app, "self.controller.node.stereo_camera_frame(side)", "web GUI must serve cached stereo camera frames")
    require_text(web_app, 'parsed.path not in {"/api/command", "/api/rc"}', "web GUI must expose command API")
    require_text(web_app, "self.node.publish_rc_override", "web GUI must reuse existing RC override publisher")
    require_text(web_app, "self.node.publish_manual_control", "web GUI must reuse existing MANUAL_CONTROL publisher")
    require_text(web_app, "WebProcessManager", "web GUI must keep simulator process logic outside web_app")
    require_text(web_app, "WebRcReplayManager", "web GUI must keep replay logic outside web_app")
    require_text(web_process_manager, "build_sim_stack_launch_command", "web process manager must reuse GUI sim launch command")
    require_text(web_process_manager, "terminate_process_group", "web process manager must terminate child process groups")
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
        "Save XML",
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
    require_text(index, 'id="pilotControlGroup"', "web GUI must identify the pilot control panel for camera overlay docking")
    require_text(index, 'id="stereoCameraProfile"', "web GUI must expose stereo camera launch profile select")
    require_text(index, 'id="stereoCameraApplyBtn"', "web GUI must expose stereo camera profile apply control")
    require_text(index, 'id="stereoCameraSaveBtn"', "web GUI must expose stereo camera profile save control")
    require_text(index, 'id="stereoCameraZoomBtn"', "web GUI must expose stereo camera zoom control")
    reject_text(index, 'id="stereoRightImage"', "web GUI must show only one stereo camera pane")
    require_text(index, 'id="leftStick"', "web GUI must expose left virtual joystick")
    require_text(index, 'id="rightStick"', "web GUI must expose right virtual joystick")
    require_text(index, 'id="modeButtons"', "web GUI must expose flight mode controls")
    require_text(index, 'id="physicsDialog"', "web GUI must expose structured physics params dialog")
    require_text(index, 'id="physicsRows"', "web GUI must expose physics params rows")
    require_text(index, 'id="courseDialog"', "web GUI must expose structured course layout dialog")
    require_text(index, 'id="courseCanvas"', "web GUI must expose top-view course canvas")
    require_text(index, 'id="courseRows"', "web GUI must expose course item rows")
    for command in (
        'command: "stack_start"',
        'command: "stack_reset"',
        'command: "ros_build"',
        'command: "mavros_toggle"',
        'command: "rviz_toggle"',
        'command: "rc_replay_play"',
        'command: "ping360_config"',
        'command: "stereo_camera_enabled"',
        'command: "camera_config"',
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
        'if command == "camera_config"',
        'if command == "physics_load"',
        'if command == "physics_apply"',
        'if command == "course_load"',
        'if command == "course_save"',
        'if command == "tool_read"',
        'if command == "tool_save"',
    ):
        require_text(web_app, command, f"web API must handle {command}")
    require_text(app_js, 'fetch("/api/status"', "web UI must poll telemetry status")
    require_text(app_js, "enablePilotInputFromJoystick", "web joystick must auto-enable pilot input")
    require_text(app_js, 'document.body.classList.toggle("camera-expanded"', "expanded camera view must expose pilot controls")
    require_text(app_js, "setPilotControlDocked", "expanded camera view must move pilot controls above the camera layer")
    require_text(app_js, "document.body.appendChild(pilot)", "expanded camera pilot controls must dock at body level")
    require_text(app_js, "renderStereoCamera", "web UI must render stereo camera status")
    require_text(app_js, "yoloStatusText", "web UI must render YOLO buoy overlay status")
    require_text(app_js, "renderCameraConfig", "web UI must render stereo camera profile status")
    require_text(app_js, "applyCameraConfig", "web UI must apply stereo camera launch profiles")
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
    require_text(yolo_detector, "from ultralytics import YOLO", "YOLO detector must support the provided .pt model")
    require_text(yolo_detector, "draw_buoy_detections", "YOLO detector must draw OpenCV bounding boxes")
    require_text(yolo_detector, "cv2.rectangle", "YOLO detector must use OpenCV rectangle drawing")
    require_text(yolo_detector, "cv2.putText", "YOLO detector must use OpenCV label drawing")
    require_text(style_css, ".pilot-group.camera-pilot-docked", "expanded camera view must keep the joystick reachable")
    require_text(style_css, "z-index: 1000", "docked pilot controls must stay above expanded camera layers")
    require_text(app_js, 'fetch("/api/rc"', "web UI must publish RC commands")
    require_text(web_app, 'self.path.startswith("/api/rc")', "web API must route /api/rc to RC command handling")
    require_text(app_js, "bindJoystick", "web UI must implement virtual stick controls")
    require_text(app_js, "startJoystickDrag", "web virtual sticks must own pointer drag sessions")
    require_text(app_js, 'window.addEventListener("pointermove"', "web virtual sticks must keep dragging outside the pad")
    require_text(app_js, "applyJoystickDrag(session, event, { force: true })", "web virtual sticks must send the first drag frame immediately")
    require_text(app_js, "setStickAxes(session.mapping, 0, 0", "web virtual sticks must spring back to center on release")
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
