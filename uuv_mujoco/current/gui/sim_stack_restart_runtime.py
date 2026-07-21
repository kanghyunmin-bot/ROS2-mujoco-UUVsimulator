"""Restart sequencing for GUI-started simulator stacks."""

from __future__ import annotations


def _restart_sim_stack_after_mavros_mode_change(self, *, want_mavros_running: bool, reason: str) -> None:
    if not self._sim_stack_running():
        return
    if _restart_blocked_by_external_stack(self, reason):
        return

    self._set_sim_stack_status(f"sim: restarting for {reason}")
    self.node.push_event(f"sim restart requested for {reason}")
    self._terminate_sim_stack_process()
    _schedule_restart_when_ready(self, want_mavros_running=want_mavros_running)


def _restart_blocked_by_external_stack(self, reason: str) -> bool:
    if self._tracked_sim_stack_running() or not self._external_sim_stack_running():
        return False
    self._external_sim_stack_running_cached = True
    self._set_sim_stack_status(f"sim: external stack running; Stop/Reset before {reason}")
    self.node.push_event(f"sim restart skipped for {reason}: external stack is not GUI-owned")
    self._refresh_sim_stack_controls()
    return True


def _schedule_restart_when_ready(self, *, want_mavros_running: bool) -> None:
    def start_when_ready() -> None:
        if self._closed:
            return
        if self._sim_stack_running():
            self.root.after(500, start_when_ready)
            return
        if self._ros_pkg_running() != want_mavros_running:
            self.root.after(500, start_when_ready)
            return
        self._start_sim_stack()

    self.root.after(1500, start_when_ready)


__all__ = ["_restart_sim_stack_after_mavros_mode_change"]
