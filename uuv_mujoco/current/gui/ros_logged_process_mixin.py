"""Compatibility facade for logged ROS process helpers."""

from __future__ import annotations

from pathlib import Path
import subprocess

from .ros_logged_process_start import start_logged_ros_process
from .ros_logged_process_watch import watch_ros_process


class RosLoggedProcessMixin:
    def _start_logged_ros_process(
        self,
        *,
        cmd: list[str],
        label: str,
        log_prefix: str,
        attr_name: str,
        status_callback,
    ) -> None:
        start_logged_ros_process(
            self,
            cmd=cmd,
            label=label,
            log_prefix=log_prefix,
            attr_name=attr_name,
            status_callback=status_callback,
        )

    def _watch_ros_process(
        self,
        proc: subprocess.Popen[str],
        log_path: Path,
        label: str,
        attr_name: str,
        status_callback,
    ) -> None:
        watch_ros_process(self, proc, log_path, label, attr_name, status_callback)


__all__ = ["RosLoggedProcessMixin"]
