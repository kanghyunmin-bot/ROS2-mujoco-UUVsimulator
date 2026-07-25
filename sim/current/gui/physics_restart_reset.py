"""Reset-script execution for physics-profile restarts."""

from __future__ import annotations

from .physics_restart_reset_events import (
    handle_external_stack_still_running,
    publish_physics_reset_log_lines,
)
from .physics_restart_reset_paths import reset_script_missing_status, sim_stack_reset_paths
from .physics_restart_process import execute_physics_reset_process


def run_reset_subprocess(self, *, reset_script, sim_stack_dir):
    try:
        return execute_physics_reset_process(reset_script=reset_script, sim_stack_dir=sim_stack_dir)
    except Exception as exc:
        self._set_sim_stack_status(f"sim reset failed: {exc}")
        self.node.push_event(f"physics params sim reset failed: {exc}")
        return None


def run_physics_reset_script(self):
    RESET_SIM_STACK_SCRIPT, SIM_STACK_DIR = sim_stack_reset_paths()
    if not RESET_SIM_STACK_SCRIPT.exists():
        self._set_sim_stack_status(reset_script_missing_status(RESET_SIM_STACK_SCRIPT))
        return None
    proc = run_reset_subprocess(self, reset_script=RESET_SIM_STACK_SCRIPT, sim_stack_dir=SIM_STACK_DIR)
    if proc is None:
        return None
    publish_physics_reset_log_lines(self, proc.stdout)
    if proc.returncode == 0 and not self._wait_for_external_sim_stack_exit(timeout_s=8.0):
        handle_external_stack_still_running(self)
        return None
    return proc


__all__ = [
    "handle_external_stack_still_running",
    "publish_physics_reset_log_lines",
    "run_physics_reset_script",
    "run_reset_subprocess",
    "sim_stack_reset_paths",
]
