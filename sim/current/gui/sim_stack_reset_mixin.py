"""Simulator stack termination and reset helpers."""

from __future__ import annotations

from .sim_stack_reset_commands import reset_sim_stack_blocking
from .sim_stack_reset_thread import reset_thread_running, start_reset_thread
from .sim_stack_reset_worker import run_sim_stack_reset


class SimStackResetMixin:
    def _terminate_sim_stack_process(self) -> None:
        proc = self._sim_stack_process
        if proc is None or proc.poll() is not None:
            return
        self._terminate_process_group(proc, timeout_s=5.0)
        if self._sim_stack_process is proc:
            self._sim_stack_process = None
            self._sim_stack_owned_by_gui = False

    def _reset_sim_stack_blocking(self, timeout_s: float = 12.0) -> None:
        reset_sim_stack_blocking(timeout_s=timeout_s)

    def _stop_sim_stack(self) -> None:
        self._terminate_sim_stack_process()
        self._set_sim_stack_status("sim: resetting stack")
        self._refresh_sim_stack_controls()
        self.node.push_event("sim stack reset requested")
        if not reset_thread_running(self._sim_stack_reset_thread):
            start_reset_thread(self, self._run_sim_stack_reset)

    def _run_sim_stack_reset(self) -> None:
        run_sim_stack_reset(self)


__all__ = ["SimStackResetMixin"]
