"""Log-file helpers for GUI-started simulator stacks."""

from __future__ import annotations

from .config import SIM_STACK_DIR
from .process_log_files import open_process_log


def open_gui_sim_stack_log():
    log_dir = SIM_STACK_DIR / "logs"
    return open_process_log(log_dir, "gui_start_stack")


__all__ = ["open_gui_sim_stack_log"]
