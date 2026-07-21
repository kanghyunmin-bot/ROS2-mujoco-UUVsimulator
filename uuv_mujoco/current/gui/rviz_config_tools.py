"""RViz configuration generation helpers for the GUI."""

from __future__ import annotations

from pathlib import Path

from .config import PING360_RVIZ_CONFIG, ROS2_RVIZ_COMPAT_CONFIG, ROS_PACKAGE_RVIZ_CONFIG
from .rviz_config_ping360 import ping360_rviz_text
from .rviz_config_ros2 import ros2_rviz_text


def prepare_ros2_rviz_config(source_path: Path = ROS_PACKAGE_RVIZ_CONFIG) -> Path:
    """Create a ROS2-compatible RViz copy without modifying the rospkg source."""
    text = ros2_rviz_text(source_path)
    ROS2_RVIZ_COMPAT_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    if not ROS2_RVIZ_COMPAT_CONFIG.exists() or ROS2_RVIZ_COMPAT_CONFIG.read_text(
        encoding="utf-8", errors="replace"
    ) != text:
        ROS2_RVIZ_COMPAT_CONFIG.write_text(text, encoding="utf-8")
    return ROS2_RVIZ_COMPAT_CONFIG


def prepare_ping360_rviz_config() -> Path:
    """Create a Ping360 RViz view outside the real-robot rospkg."""
    text = ping360_rviz_text()
    PING360_RVIZ_CONFIG.parent.mkdir(parents=True, exist_ok=True)
    if not PING360_RVIZ_CONFIG.exists() or PING360_RVIZ_CONFIG.read_text(
        encoding="utf-8", errors="replace"
    ) != text:
        PING360_RVIZ_CONFIG.write_text(text, encoding="utf-8")
    return PING360_RVIZ_CONFIG


__all__ = ["prepare_ping360_rviz_config", "prepare_ros2_rviz_config"]
