"""Log-file ownership for GUI-started ROS processes."""

from __future__ import annotations

import datetime as _dt
from pathlib import Path
from typing import Callable, TextIO

from .config import SIM_STACK_DIR


def open_logged_process_file(
    *,
    label: str,
    log_prefix: str,
    status_callback: Callable[[str], None],
) -> tuple[Path, TextIO] | None:
    log_dir = SIM_STACK_DIR / "logs"
    try:
        log_dir.mkdir(parents=True, exist_ok=True)
        log_path = log_dir / f"{log_prefix}_{_dt.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        log_file = log_path.open("w", encoding="utf-8")
    except Exception as exc:
        status_callback(f"{label}: log open failed: {exc}")
        return None
    return log_path, log_file


__all__ = ["open_logged_process_file"]
