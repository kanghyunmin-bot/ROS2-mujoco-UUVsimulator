"""GUI button-state helpers for simulator-stack status."""

from __future__ import annotations

from typing import Any

from .sim_stack_status_logic import reset_thread_running, sim_stack_controls_running


def tracked_sim_stack_running(process: Any) -> bool:
    return process is not None and process.poll() is None


def refresh_sim_stack_control_buttons(owner: Any, tk_module: Any) -> None:
    running = sim_stack_controls_running(
        tracked=tracked_sim_stack_running(owner._sim_stack_process),
        external_cached=bool(getattr(owner, "_external_sim_stack_running_cached", False)),
        reset_running=reset_thread_running(owner._sim_stack_reset_thread),
    )
    start_button = getattr(owner, "sim_stack_start_button", None)
    if start_button is not None:
        start_button.config(
            text="Stack Running" if running else "Start SITL/MuJoCo",
            state=tk_module.DISABLED if running else tk_module.NORMAL,
        )
    stop_button = getattr(owner, "sim_stack_stop_button", None)
    if stop_button is not None:
        stop_button.config(state=tk_module.NORMAL)
    preset_combo = getattr(owner, "sim_launch_preset_combo", None)
    if preset_combo is not None:
        preset_combo.config(state=tk_module.DISABLED if running else "readonly")


__all__ = ["refresh_sim_stack_control_buttons", "tracked_sim_stack_running"]
