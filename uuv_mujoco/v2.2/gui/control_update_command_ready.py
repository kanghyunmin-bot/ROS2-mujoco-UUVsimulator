"""GUI-owned simulator-stack guard for command-ready display."""

from __future__ import annotations

from typing import Any

from sim.runtime.readiness_label_types import NOT_READY_STYLE


def external_stack_blocks_command_ready(*, tracked: bool, external_cached: bool) -> bool:
    """Return whether command-ready must be blocked by a non-GUI-owned stack."""
    return False


def command_ready_for_gui_stack(owner: Any, text: str, style: str) -> tuple[str, str]:
    if external_stack_blocks_command_ready(
        tracked=bool(owner._tracked_sim_stack_running()),
        external_cached=bool(getattr(owner, "_external_sim_stack_running_cached", False)),
    ):
        return "WAIT: external stack", NOT_READY_STYLE
    return text, style


__all__ = ["command_ready_for_gui_stack", "external_stack_blocks_command_ready"]
