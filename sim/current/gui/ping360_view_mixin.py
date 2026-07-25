"""Ping360 viewer process controls."""

from __future__ import annotations

from .ping360_view_process import build_ping360_view_ros_command, terminate_ping360_view_process
from .ping360_view_status import ping360_view_running, set_ping360_view_status
from .ros_tools import prepare_ping360_rviz_config


class Ping360ViewMixin:
    def _ping360_view_running(self) -> bool:
        return ping360_view_running(self._ping360_view_process)

    def _set_ping360_view_status(self, text: str) -> None:
        set_ping360_view_status(self, text)

    def _start_ping360_view(self) -> None:
        if self._ping360_view_running():
            self._set_ping360_view_status("ping360 view: already open")
            return

        try:
            rviz_config = prepare_ping360_rviz_config()
        except Exception as exc:
            self._set_ping360_view_status(f"ping360 rviz config failed: {exc}")
            return

        self._start_logged_ros_process(
            cmd=build_ping360_view_ros_command(rviz_config),
            label="ping360 view",
            log_prefix="ping360_view",
            attr_name="_ping360_view_process",
            status_callback=self._set_ping360_view_status,
        )

    def _stop_ping360_view(self) -> None:
        proc = self._ping360_view_process
        if proc is None or proc.poll() is not None:
            self._set_ping360_view_status("ping360 view: closed")
            self._ping360_view_process = None
            return
        terminate_ping360_view_process(proc)
        self._set_ping360_view_status("ping360 view: closing")
        self.node.push_event("ping360 view close requested")


__all__ = ["Ping360ViewMixin"]
