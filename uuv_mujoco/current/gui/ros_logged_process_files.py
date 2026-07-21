"""Log-file ownership for GUI-started ROS processes."""

from __future__ import annotations

from pathlib import Path
from typing import Callable, TextIO

from .config import SIM_STACK_DIR
from .process_log_files import open_process_log


def open_logged_process_file(
    *,
    label: str,
    log_prefix: str,
    status_callback: Callable[[str], None],
) -> tuple[Path, TextIO] | None:
    log_dir = SIM_STACK_DIR / "logs"
    try:
        log_path, log_file = open_process_log(log_dir, log_prefix)
    except Exception as exc:
        status_callback(f"{label}: log open failed: {exc}")
        return None
    return log_path, log_file


__all__ = ["open_logged_process_file"]
