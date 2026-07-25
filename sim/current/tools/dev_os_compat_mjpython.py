"""macOS mjpython launcher checks for MuJoCo viewer runs."""

from __future__ import annotations

import platform

from dev_os_compat_common import CheckResult
from dev_os_compat_mjpython_probe import candidate_passes
from dev_os_compat_mjpython_result import failed_mjpython_result, missing_mjpython_result
from dev_os_compat_python_probe import runtime_mjpython_candidates


def check_mjpython(
    results: list[CheckResult],
    *,
    headless: bool,
    require_viewer: bool,
    explicit_mjpython: str | None,
    runtime_viewer_ok: bool,
) -> None:
    if headless:
        results.append(CheckResult("mjpython_launcher", "pass", "headless mode uses the selected Python launcher"))
        return
    if platform.system() != "Darwin":
        results.append(
            CheckResult(
                "mjpython_launcher",
                "pass",
                "non-macOS viewer path can use the selected Python launcher when display is available",
            )
        )
        return

    candidates = runtime_mjpython_candidates(explicit_mjpython)
    if not candidates:
        results.append(missing_mjpython_result(require_viewer))
        return

    probed: list[str] = []
    for candidate in candidates:
        if candidate_passes(candidate, probed):
            results.append(CheckResult("mjpython_launcher", "pass", str(candidate)))
            return

    results.append(failed_mjpython_result(require_viewer, runtime_viewer_ok, probed))


__all__ = ["check_mjpython"]
