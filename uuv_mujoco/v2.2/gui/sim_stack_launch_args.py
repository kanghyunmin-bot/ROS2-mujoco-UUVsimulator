"""Argument and environment helpers for GUI-started simulator stacks."""

from __future__ import annotations

from .config import SIM_STACK_DIR
from .runtime import os, sys
from .sim_stack_env import (
    build_gui_sim_stack_env,
    build_initial_depth_args,
    normalize_sim_extra_args,
)


def _gui_sim_stack_env(self) -> dict[str, str]:
    return build_gui_sim_stack_env(
        os.environ,
        backend=self._sim_stack_backend(),
        sim_stack_dir=SIM_STACK_DIR,
    )


def _append_mavros_surface_args(self, cmd: list[str]) -> None:
    if self._gui_external_mavros_controls_enabled():
        cmd.append("--ros2-real-pkg-compat")
        self.node.push_event("MAVROS surface: external node owns arm/mode/RC")
        return
    self.node.push_event("MAVROS surface: internal sim bridge owns arm/mode/RC")


def _normalized_sim_extra_args(self, extra_args: list[str] | None) -> list[str]:
    result = normalize_sim_extra_args(
        extra_args,
        os.environ,
        platform_name=sys.platform,
    )
    for event in result.events:
        self.node.push_event(event)
    return list(result.args)


def _append_initial_depth_args(self, cmd: list[str], launch_extra_args: list[str]) -> None:
    result = build_initial_depth_args(
        os.environ,
        launch_extra_args=launch_extra_args,
    )
    cmd.extend(result.args)
    for event in result.events:
        self.node.push_event(event)


__all__ = [
    "_append_initial_depth_args",
    "_append_mavros_surface_args",
    "_gui_sim_stack_env",
    "_normalized_sim_extra_args",
]
