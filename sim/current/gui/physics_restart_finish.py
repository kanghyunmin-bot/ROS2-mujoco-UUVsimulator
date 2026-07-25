"""GUI-thread finish step for physics-profile restarts."""

from __future__ import annotations


def finish_physics_restart(self, proc) -> None:
    if self._closed:
        return
    if proc.returncode != 0:
        self._set_sim_stack_status(f"sim reset failed: rc={proc.returncode}")
        return
    # The reset helper kills the whole external sim stack, including processes
    # that may not be direct children of the current GUI. Drop any stale handle
    # so the immediate restart cannot be skipped.
    self._sim_stack_process = None
    self._set_sim_stack_status("sim: starting with physics params")
    self._refresh_sim_stack_controls()
    self._start_sim_stack(extra_args=["--no-reset"])


def schedule_physics_restart_finish(self, proc) -> None:
    try:
        self.root.after(0, lambda: finish_physics_restart(self, proc))
    except Exception:
        pass


__all__ = ["finish_physics_restart", "schedule_physics_restart_finish"]
