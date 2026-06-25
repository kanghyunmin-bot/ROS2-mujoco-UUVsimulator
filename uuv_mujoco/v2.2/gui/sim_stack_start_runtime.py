"""Start runtime for GUI-started simulator stacks."""

from __future__ import annotations

from .sim_stack_start_guards import apply_blocked_sim_stack_start, evaluate_sim_stack_start_guard
from .sim_stack_start_process import start_gui_sim_stack_process
from .sim_stack_start_state import prepare_gui_sim_stack_start, record_gui_sim_stack_started


def _start_sim_stack(self, extra_args: list[str] | None = None) -> None:
    guard = evaluate_sim_stack_start_guard(self)
    if guard.blocked:
        apply_blocked_sim_stack_start(self, guard)
        return

    prepare_gui_sim_stack_start(self)
    started = start_gui_sim_stack_process(
        self,
        target=guard.target,
        extra_args=extra_args,
        env=self._gui_sim_stack_env(),
    )
    if started is None:
        return
    record_gui_sim_stack_started(self, target=guard.target, started=started)


__all__ = ["_start_sim_stack"]
