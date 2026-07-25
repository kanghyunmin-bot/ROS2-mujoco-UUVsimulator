"""Start guards for GUI-started simulator stacks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from .sim_stack_launch_target import resolve_sim_stack_launch_target, sim_stack_start_script_error


@dataclass
class SimStackStartGuard:
    target: Any | None
    status_message: str | None = None
    mark_external_running: bool = False
    refresh_controls: bool = False

    @property
    def blocked(self) -> bool:
        return self.status_message is not None


def evaluate_sim_stack_start_guard(owner) -> SimStackStartGuard:
    if owner._tracked_sim_stack_running():
        return SimStackStartGuard(target=None, status_message="sim: already running")
    if owner._external_sim_stack_running():
        owner._external_sim_stack_running_cached = False
        owner.node.push_event("external sim stack detected; Start will reset/reclaim it")
    target = resolve_sim_stack_launch_target(owner._sim_stack_backend())
    start_error = sim_stack_start_script_error(target.start_script)
    if start_error is not None:
        return SimStackStartGuard(target=target, status_message=start_error)
    return SimStackStartGuard(target=target)


def apply_blocked_sim_stack_start(owner, guard: SimStackStartGuard) -> None:
    if guard.mark_external_running:
        owner._external_sim_stack_running_cached = True
    if guard.status_message is not None:
        owner._set_sim_stack_status(guard.status_message)
    if guard.refresh_controls:
        owner._refresh_sim_stack_controls()


__all__ = ["SimStackStartGuard", "apply_blocked_sim_stack_start", "evaluate_sim_stack_start_guard"]
