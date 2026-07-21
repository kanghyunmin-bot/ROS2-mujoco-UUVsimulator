#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ROS_WORKSPACE = ROOT / "rospkg"
ROS_SOURCE = ROS_WORKSPACE / "src"
if not ROS_SOURCE.is_dir():
    ROS_SOURCE = ROS_WORKSPACE
PKG = ROS_SOURCE / "kmu26_auv"


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def load_rviz_converter():
    path = ROOT / "uuv_mujoco" / "current" / "gui" / "rviz_config_ros2.py"
    spec = importlib.util.spec_from_file_location("rviz_config_ros2", path)
    require(spec is not None and spec.loader is not None, "rviz converter import spec missing")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    cmake = read(PKG / "CMakeLists.txt")
    package = read(PKG / "package.xml")
    launch = read(PKG / "launch" / "rov_start.launch.py")
    node = read(PKG / "src" / "mission_rviz_visualizer.cpp")
    rviz = read(PKG / "rviz" / "rov.rviz")
    gui_publishers = read(ROOT / "uuv_mujoco" / "current" / "gui" / "node_init_publishers.py")
    gui_camera = read(ROOT / "uuv_mujoco" / "current" / "gui" / "node_stereo_camera.py")

    require("find_package(visualization_msgs REQUIRED)" in cmake, "visualization_msgs CMake dependency missing")
    require("add_executable(mission_rviz_visualizer" in cmake, "mission_rviz_visualizer target missing")
    require("mission_rviz_visualizer" in launch, "mission visualizer launch node missing")
    require("use_mission_rviz_visualizer" in launch, "launch toggle argument missing")
    require("<depend>visualization_msgs</depend>" in package, "visualization_msgs package dependency missing")
    require("/mission/rviz_markers" in rviz, "RViz mission marker topic missing")
    require(
        "Class: rviz_default_plugins/MarkerArray" in rviz or
        "Class: rviz/MarkerArray" in rviz,
        "RViz MarkerArray display missing",
    )
    require("mission_status_json" in node and "MarkerArray" in node, "visualizer node core behavior missing")
    require("course_boundary_margin_m" in node, "course boundary visualization source missing")
    require("DETECTED" in node and "SEARCHING" in node, "search/detection labels missing")
    require("/uuv_mujoco/yolo_buoy_detections" in launch, "YOLO detection launch topic missing")
    require("parse_yolo_status" in node and "mission_yolo_bbox" in node, "YOLO RViz marker path missing")
    require("yolo_zone_gate_enabled" in launch, "YOLO zone gate launch toggle missing")
    require("yolo_detection_points_to_opponent_zone" in node, "YOLO opponent-zone gate missing")
    require("opponent zone ray" in node, "YOLO ignored-zone RViz label missing")
    require("mission_own_course" in launch and "course_boundary_x" in launch, "RViz own-zone launch settings missing")
    require("/uuv_mujoco/yolo_buoy_detections" in gui_publishers, "GUI YOLO publisher topic missing")
    require("json.dumps(payload" in gui_camera, "GUI YOLO JSON publish payload missing")

    converter = load_rviz_converter()
    converted = converter.ros2_rviz_text(PKG / "rviz" / "rov.rviz")
    require("Class: rviz_default_plugins/MarkerArray" in converted, "ROS2 MarkerArray conversion missing")
    require("/mission/rviz_markers" in converted, "converted RViz marker topic missing")

    print("rviz_mission_visualizer_contract=PASS")


if __name__ == "__main__":
    main()
