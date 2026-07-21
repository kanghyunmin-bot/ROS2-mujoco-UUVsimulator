"""RViz process controls for the GUI."""

from __future__ import annotations

from .config import ROS_PACKAGE_RVIZ_CONFIG
from .ros_tools import prepare_ros2_rviz_config, ros_bash_command
from .runtime import shlex


class RvizProcessMixin:
    def _toggle_rviz(self) -> None:
        if self._rviz_running():
            self._terminate_process_group(self._rviz_process)
            self._set_rviz_status("rviz: stopping")
            self._refresh_ros2_buttons()
            return
        self._start_rviz()

    def _start_rviz(self) -> None:
        if not ROS_PACKAGE_RVIZ_CONFIG.exists():
            self._set_rviz_status(f"rviz config missing: {ROS_PACKAGE_RVIZ_CONFIG}")
            return
        try:
            rviz_config = prepare_ros2_rviz_config()
        except Exception as exc:
            self._set_rviz_status(f"rviz config prepare failed: {exc}")
            return
        command = " ".join(
            shlex.quote(part)
            for part in ("rviz2", "-d", str(rviz_config))
        )
        self._start_logged_ros_process(
            cmd=ros_bash_command(command, include_workspace=False),
            label="rviz",
            log_prefix="gui_rviz",
            attr_name="_rviz_process",
            status_callback=self._set_rviz_status,
        )


__all__ = ["RvizProcessMixin"]
