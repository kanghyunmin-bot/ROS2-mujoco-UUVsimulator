"""Simulator stack launch compatibility mixin."""

from __future__ import annotations

from .sim_stack_launch_args import (
    _append_initial_depth_args,
    _append_mavros_surface_args,
    _gui_sim_stack_env,
    _normalized_sim_extra_args,
)
from .sim_stack_launch_runtime import (
    _restart_sim_stack_after_mavros_mode_change,
    _start_sim_stack,
)
from .sim_stack_log_watcher import SimStackLogWatcherMixin


class SimStackLaunchMixin(SimStackLogWatcherMixin):
    _gui_sim_stack_env = _gui_sim_stack_env
    _append_mavros_surface_args = _append_mavros_surface_args
    _normalized_sim_extra_args = _normalized_sim_extra_args
    _append_initial_depth_args = _append_initial_depth_args
    _restart_sim_stack_after_mavros_mode_change = _restart_sim_stack_after_mavros_mode_change
    _start_sim_stack = _start_sim_stack


__all__ = ["SimStackLaunchMixin"]
