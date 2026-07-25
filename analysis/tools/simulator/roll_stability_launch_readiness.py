"""Readiness wait helpers for roll-stability candidate launchers."""

from __future__ import annotations

import time
from pathlib import Path


FAILURE_MARKERS = (
    "SITL exited while waiting for readiness",
    "MuJoCo exited while waiting for readiness",
    "readiness timeout",
)


def read_launch_log_if_present(launch_log: Path) -> str:
    if not launch_log.exists():
        return ""
    return launch_log.read_text(errors="replace")


def raise_if_launch_failed(last_text: str) -> None:
    for marker in FAILURE_MARKERS:
        if marker in last_text:
            raise RuntimeError(f"launcher readiness failed: {marker}")


def wait_for_readiness_marker(proc, launch_log: Path, timeout_s: float) -> None:
    deadline = time.monotonic() + float(timeout_s)
    last_text = ""
    while time.monotonic() < deadline:
        last_text = read_launch_log_if_present(launch_log) or last_text
        if "readiness OK:" in last_text:
            return
        raise_if_launch_failed(last_text)
        if proc.poll() is not None:
            raise RuntimeError(f"launcher exited before readiness, code={proc.returncode}")
        time.sleep(0.25)
    tail = "\n".join(last_text.splitlines()[-40:])
    raise RuntimeError(f"launcher readiness timeout after {timeout_s:.1f}s\n{tail}")


__all__ = ["wait_for_readiness_marker"]
