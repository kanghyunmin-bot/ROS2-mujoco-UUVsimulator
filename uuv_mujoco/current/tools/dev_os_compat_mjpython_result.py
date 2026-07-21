"""Result helpers for macOS mjpython compatibility checks."""

from __future__ import annotations

from pathlib import Path

from dev_os_compat_common import CheckResult


def missing_mjpython_result(require_viewer: bool) -> CheckResult:
    return CheckResult(
        "mjpython_launcher",
        "fail" if require_viewer else "warn",
        "macOS MuJoCo viewer launch requires mjpython; set MJ311_MJPYTHON or install mujoco in MJ311_ROOT",
    )


def probe_summary(candidate: Path, payload: dict[str, object], mujoco_ok: bool, viewer_ok: bool) -> str:
    mujoco_detail = payload.get("mujoco_version") if mujoco_ok else payload.get("mujoco_error")
    viewer_detail = "ok" if viewer_ok else payload.get("viewer_error", "failed")
    return f"{candidate}: mujoco={mujoco_detail} viewer={viewer_detail}"


def failed_mjpython_result(require_viewer: bool, runtime_viewer_ok: bool, probed: list[str]) -> CheckResult:
    return CheckResult(
        "mjpython_launcher",
        "warn" if require_viewer and runtime_viewer_ok else ("fail" if require_viewer else "warn"),
        "no mjpython candidate passed; " + " | ".join(probed),
    )


__all__ = ["failed_mjpython_result", "missing_mjpython_result", "probe_summary"]
