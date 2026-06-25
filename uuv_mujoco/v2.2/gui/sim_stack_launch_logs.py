"""Log-file helpers for GUI-started simulator stacks."""

from __future__ import annotations

import datetime as _dt

from .config import SIM_STACK_DIR


def open_gui_sim_stack_log():
    log_dir = SIM_STACK_DIR / "logs"
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / f"gui_start_stack_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
    return log_path, log_path.open("w", encoding="utf-8")


__all__ = ["open_gui_sim_stack_log"]
