"""Process termination helpers shared by GUI process controls."""

from __future__ import annotations

import os
import signal
import subprocess
import time

from .process_termination_signal import kill_group_or_process, terminate_group_or_process


def terminate_process_group(proc: subprocess.Popen[str] | None, timeout_s: float = 4.0) -> None:
    if proc is None or proc.poll() is not None:
        return
    pgid = terminate_group_or_process(proc)
    try:
        proc.wait(timeout=max(0.2, float(timeout_s)))
        # The launcher often exits immediately on SIGTERM while ArduSub or
        # MAVProxy ignores it and remains in the same process group.  Do not
        # treat leader exit as group exit; give residual children a short
        # grace period and then kill the exact, GUI-owned group.
        terminate_exited_process_group(proc, timeout_s=min(1.0, max(0.2, float(timeout_s))))
        return
    except Exception:
        pass
    kill_group_or_process(proc, pgid)
    try:
        proc.wait(timeout=1.0)
    except Exception:
        pass
    terminate_exited_process_group(proc, timeout_s=0.2)


def terminate_exited_process_group(proc: subprocess.Popen[str] | None, timeout_s: float = 1.0) -> None:
    """Stop children left in a GUI-started process group after its leader exits.

    GUI subprocesses are started with ``start_new_session=True``, so the
    leader PID is also the process-group ID.  ``terminate_process_group``
    intentionally returns once that leader has exited; this companion handles
    the important case where a launcher exits before ArduSub/MAVProxy children.
    """
    if proc is None or proc.poll() is None:
        return
    try:
        pgid = int(proc.pid)
        os.killpg(pgid, signal.SIGTERM)
    except (AttributeError, ProcessLookupError, PermissionError, ValueError):
        return

    deadline = time.monotonic() + max(0.0, float(timeout_s))
    while time.monotonic() < deadline:
        try:
            os.killpg(pgid, 0)
        except ProcessLookupError:
            return
        except PermissionError:
            return
        time.sleep(0.05)
    try:
        os.killpg(pgid, signal.SIGKILL)
    except (ProcessLookupError, PermissionError):
        pass


__all__ = ["terminate_exited_process_group", "terminate_process_group"]
