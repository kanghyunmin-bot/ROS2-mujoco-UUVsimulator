"""Signal helpers for GUI process termination."""

from __future__ import annotations

import os
import signal
import subprocess


def terminate_group_or_process(proc: subprocess.Popen[str]) -> int | None:
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
        return pgid
    except Exception:
        try:
            proc.terminate()
        except Exception:
            pass
    return None


def kill_group_or_process(proc: subprocess.Popen[str], pgid: int | None) -> None:
    try:
        if pgid is not None:
            os.killpg(pgid, signal.SIGKILL)
        else:
            proc.kill()
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


__all__ = ["kill_group_or_process", "terminate_group_or_process"]
