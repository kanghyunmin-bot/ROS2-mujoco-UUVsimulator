"""Simulator stack status and external-process detection helpers."""

from __future__ import annotations

import tkinter as tk
from .sim_stack_process_probe import (
    external_sim_stack_commands,
    wait_for_external_sim_stack_exit,
)
from .sim_stack_status_controls import refresh_sim_stack_control_buttons, tracked_sim_stack_running
from .sim_stack_status_refresh import refresh_sim_stack_status
from .sim_stack_status_text import set_sim_stack_status_text


class SimStackStatusMixin:
    def _tracked_sim_stack_running(self) -> bool:
        return tracked_sim_stack_running(self._sim_stack_process)

    def _external_sim_stack_commands(self) -> list[str]:
        return external_sim_stack_commands(self._matching_process_commands)

    def _external_sim_stack_running(self) -> bool:
        return bool(self._external_sim_stack_commands())

    def _sim_stack_running(self) -> bool:
        return self._tracked_sim_stack_running() or self._external_sim_stack_running()

    def _wait_for_external_sim_stack_exit(self, timeout_s: float = 10.0) -> bool:
        exited, still_running = wait_for_external_sim_stack_exit(
            self._matching_process_commands,
            timeout_s=timeout_s,
        )
        self._external_sim_stack_running_cached = still_running
        return exited

    def _refresh_sim_stack_controls(self) -> None:
        refresh_sim_stack_control_buttons(self, tk)

    def _refresh_sim_stack_status(self) -> None:
        refresh_sim_stack_status(self)

    def _set_sim_stack_status(self, text: str) -> None:
        set_sim_stack_status_text(self.root, self.sim_stack_status_var, text)


__all__ = ["SimStackStatusMixin"]
