"""ROS-related process state helpers for the GUI panel."""

from __future__ import annotations


def _process_running(proc) -> bool:
    return proc is not None and proc.poll() is None


def _ros_pkg_running(self) -> bool:
    return _process_running(self._ros_pkg_process)


def _ros_build_running(self) -> bool:
    return _process_running(self._ros_build_process)


def _rviz_running(self) -> bool:
    return _process_running(self._rviz_process)


def _gui_external_mavros_controls_enabled(self) -> bool:
    return self._ros_pkg_running() and self._env_flag("UUV_GUI_USE_EXTERNAL_MAVROS", True)


__all__ = [
    "_gui_external_mavros_controls_enabled",
    "_ros_build_running",
    "_ros_pkg_running",
    "_rviz_running",
]
