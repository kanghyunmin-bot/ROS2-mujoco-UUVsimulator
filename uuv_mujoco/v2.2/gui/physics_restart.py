"""Restart flow after GUI physics-profile edits."""

from __future__ import annotations

import threading

from .physics_restart_finish import schedule_physics_restart_finish
from .physics_restart_reset import run_physics_reset_script


def _restart_sim_stack_after_physics_apply(self) -> None:
    if self._sim_stack_reset_thread is not None and self._sim_stack_reset_thread.is_alive():
        self._set_sim_stack_status("sim: restart already in progress")
        return
    self._set_sim_stack_status("sim: resetting for physics params")
    self.node.push_event("physics params restart requested")
    self._terminate_sim_stack_process()
    self._refresh_sim_stack_controls()

    def reset_and_start() -> None:
        proc = run_physics_reset_script(self)
        if proc is None:
            return
        schedule_physics_restart_finish(self, proc)

    self._sim_stack_reset_thread = threading.Thread(target=reset_and_start, daemon=True)
    self._sim_stack_reset_thread.start()
