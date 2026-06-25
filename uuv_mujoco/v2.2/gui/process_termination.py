"""Process termination helpers shared by GUI process controls."""

from __future__ import annotations

import subprocess

from .process_termination_signal import kill_group_or_process, terminate_group_or_process


def terminate_process_group(proc: subprocess.Popen[str] | None, timeout_s: float = 4.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    pgid = terminate_group_or_process(proc)
    try:
        proc.wait(timeout=max(0.2, float(timeout_s)))
        return
    except Exception:
        pass
    kill_group_or_process(proc, pgid)
    try:
        proc.wait(timeout=1.0)
    except Exception:
        pass


__all__ = ["terminate_process_group"]
