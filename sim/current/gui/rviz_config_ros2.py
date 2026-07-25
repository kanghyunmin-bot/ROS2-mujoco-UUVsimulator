"""ROS1 RViz config compatibility transforms for ROS2 RViz."""

from __future__ import annotations

from pathlib import Path


RVIZ_CLASS_REPLACEMENTS = {
    "Class: rviz/Displays": "Class: rviz_common/Displays",
    "Class: rviz/Selection": "Class: rviz_common/Selection",
    "Class: rviz/Tool Properties": "Class: rviz_common/Tool Properties",
    "Class: rviz/Views": "Class: rviz_common/Views",
    "Class: rviz/Time": "Class: rviz_common/Time",
    "Class: rviz/Grid": "Class: rviz_default_plugins/Grid",
    "Class: rviz/TF": "Class: rviz_default_plugins/TF",
    "Class: rviz/RobotModel": "Class: rviz_default_plugins/RobotModel",
    "Class: rviz/Odometry": "Class: rviz_default_plugins/Odometry",
    "Class: rviz/Marker": "Class: rviz_default_plugins/Marker",
    "Class: rviz/MarkerArray": "Class: rviz_default_plugins/MarkerArray",
    "Class: rviz/Interact": "Class: rviz_default_plugins/Interact",
    "Class: rviz/MoveCamera": "Class: rviz_default_plugins/MoveCamera",
    "Class: rviz/Select": "Class: rviz_default_plugins/Select",
    "Class: rviz/FocusCamera": "Class: rviz_default_plugins/FocusCamera",
    "Class: rviz/Measure": "Class: rviz_default_plugins/Measure",
    "Class: rviz/SetInitialPose": "Class: rviz_default_plugins/SetInitialPose",
    "Class: rviz/SetGoal": "Class: rviz_default_plugins/SetGoal",
    "Class: rviz/PublishPoint": "Class: rviz_default_plugins/PublishPoint",
    "Class: rviz/Orbit": "Class: rviz_default_plugins/Orbit",
}

SIM_ODOM_DISPLAY = """    - Alpha: 1
      Class: rviz_default_plugins/Odometry
      Color: 25; 170; 255
      Enabled: true
      Keep: 200
      Length: 1.4
      Name: Sim Odom
      Position Use Topic: true
      Topic:
        Value: /sim/odom
      Value: true
"""


def ros2_rviz_text(source_path: Path) -> str:
    text = source_path.read_text(encoding="utf-8", errors="replace")
    for old, new in RVIZ_CLASS_REPLACEMENTS.items():
        text = text.replace(old, new)
    return with_sim_odom_display(text)


def with_sim_odom_display(text: str) -> str:
    if "Value: /sim/odom" in text:
        return text
    text = text.replace("  Enabled: true\n  Global Options:", f"{SIM_ODOM_DISPLAY}  Enabled: true\n  Global Options:")
    if "- /Sim Odom1" not in text:
        text = text.replace("        - /Odometry1\n", "        - /Odometry1\n        - /Sim Odom1\n")
    return text


__all__ = ["RVIZ_CLASS_REPLACEMENTS", "SIM_ODOM_DISPLAY", "ros2_rviz_text", "with_sim_odom_display"]
