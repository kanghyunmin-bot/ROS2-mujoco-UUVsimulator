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
    path = ROOT / "sim" / "current" / "gui" / "rviz_config_ros2.py"
    spec = importlib.util.spec_from_file_location("rviz_config_ros2", path)
    require(spec is not None and spec.loader is not None, "rviz converter import spec missing")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> None:
    launch = read(PKG / "launch" / "rov_start.launch.py")
    rviz = read(PKG / "rviz" / "rov.rviz")
    vision_runner = read(ROOT / "sim" / "current" / "tools" / "run_vision_mission.py")
    web_app = read(ROOT / "sim" / "current" / "gui" / "web_app.py")

    # Upstream removed mission_rviz_visualizer.cpp.  Keep rov.rviz usable and
    # expose the live mission state/physical collector result through the web
    # monitor instead of requiring a deleted executable.
    require(
        'DeclareLaunchArgument("use_mission_rviz_visualizer", default_value="false")'
        in launch,
        "removed mission visualizer must stay disabled by default",
    )
    require('"/mission/state"' in vision_runner, "current mission-state topic missing")
    require(
        '"/mujoco/course_buoys/status"' in vision_runner,
        "physical mission result topic missing",
    )
    require("mission_monitor" in web_app, "web mission monitor missing")

    converter = load_rviz_converter()
    converted = converter.ros2_rviz_text(PKG / "rviz" / "rov.rviz")
    require("Visualization Manager:" in converted, "ROS2 RViz conversion missing")
    require(len(converted) >= len(rviz) // 2, "converted RViz config is unexpectedly truncated")

    print("rviz_mission_visualizer_contract=PASS")


if __name__ == "__main__":
    main()
