"""Ping360 RViz/rqt process helpers."""

from __future__ import annotations

import os
import shlex
import signal
from pathlib import Path

from .config import APP_ROOT
from .ros_tools import ros_bash_command


def build_ping360_view_shell_command(rviz_config: Path) -> str:
    return "\n".join(
        [
            "if command -v rviz2 >/dev/null 2>&1; then",
            f"  exec rviz2 -d {shlex.quote(str(rviz_config))}",
            "elif command -v ros2 >/dev/null 2>&1; then",
            "  exec ros2 run rqt_image_view rqt_image_view /ping360/scan_image",
            "else",
            "  echo 'rviz2/ros2 not found after sourcing ROS setup' >&2",
            "  exit 127",
            "fi",
        ]
    )


def build_ping360_view_ros_command(rviz_config: Path) -> list[str]:
    return ros_bash_command(
        build_ping360_view_shell_command(rviz_config),
        cwd=APP_ROOT,
        include_workspace=True,
    )


def terminate_ping360_view_process(proc) -> None:
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass


__all__ = [
    "build_ping360_view_ros_command",
    "build_ping360_view_shell_command",
    "terminate_ping360_view_process",
]
