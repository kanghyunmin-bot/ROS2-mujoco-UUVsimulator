"""Probe helpers for macOS mjpython compatibility checks."""

from __future__ import annotations

from pathlib import Path

from dev_os_compat_mjpython_result import probe_summary
from dev_os_compat_python_probe import probe_python


def candidate_passes(candidate: Path, probed: list[str]) -> bool:
    status, payload, output = probe_python(candidate, require_viewer=True)
    if payload is None:
        probed.append(f"{candidate}: probe failed: {output}")
        return False
    mujoco_ok = bool(payload.get("mujoco_ok"))
    viewer_ok = bool(payload.get("viewer_ok"))
    probed.append(probe_summary(candidate, payload, mujoco_ok, viewer_ok))
    return status == 0 and mujoco_ok and viewer_ok


__all__ = ["candidate_passes"]
