"""Process lifecycle helpers for roll-stability candidate launchers."""

from __future__ import annotations

import os
import signal
import subprocess
from pathlib import Path


def interrupt_process_group(proc: subprocess.Popen) -> None:
    try:
        os.killpg(proc.pid, signal.SIGINT)
    except Exception:
        pass


def kill_process_group(proc: subprocess.Popen) -> None:
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except Exception:
        pass


def wait_after_interrupt(proc: subprocess.Popen, timeout_s: float = 5.0) -> None:
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        kill_process_group(proc)


def launch_headless_candidate(*, root: Path, start_script: Path, launch_log: Path) -> subprocess.Popen:
    with launch_log.open("w") as log:
        return subprocess.Popen(
            [
                str(start_script),
                "--sitl-no-rebuild",
                "--ros2",
                "--",
                "--headless",
                "--no-qgc-video",
                "--tank-35x30x11",
            ],
            cwd=root,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )


__all__ = ["interrupt_process_group", "launch_headless_candidate", "wait_after_interrupt"]
