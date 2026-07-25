"""Blocking simulator stack reset worker."""

from __future__ import annotations

from .config import RESET_SIM_STACK_SCRIPT
from .sim_stack_reset_commands import open_reset_sim_stack_process, reset_script_exists
from .sim_stack_reset_output import iter_reset_status_lines
from .sim_stack_reset_result import apply_reset_result, refresh_sim_stack_controls_async


def run_sim_stack_reset(owner) -> None:
    if not reset_script_exists():
        owner._set_sim_stack_status(f"reset script missing: {RESET_SIM_STACK_SCRIPT}")
        return
    try:
        proc = open_reset_sim_stack_process()
        for short_line in iter_reset_status_lines(proc.stdout):
            owner.node.push_event(short_line)
            owner._set_sim_stack_status(f"sim: {short_line}")
        rc = proc.wait()
    except Exception as exc:
        owner._set_sim_stack_status(f"sim reset failed: {exc}")
        owner.node.push_event(f"sim reset failed: {exc}")
        return
    apply_reset_result(owner, rc)
    refresh_sim_stack_controls_async(owner)


__all__ = ["run_sim_stack_reset"]
