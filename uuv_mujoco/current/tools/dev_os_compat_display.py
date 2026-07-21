"""Display environment checks for MuJoCo viewer runs."""

from __future__ import annotations

import os
import platform

from dev_os_compat_common import CheckResult


def check_display(results: list[CheckResult], *, headless: bool, target_system: str) -> None:
    system = platform.system()
    display = os.environ.get("DISPLAY", "")
    wayland = os.environ.get("WAYLAND_DISPLAY", "")
    if headless:
        results.append(CheckResult("viewer_display", "pass", "headless mode requested"))
        return
    if target_system == "Linux" and system != "Linux":
        results.append(
            CheckResult(
                "viewer_display_target",
                "pass",
                "Ubuntu viewer contract deferred to target host; require DISPLAY or WAYLAND_DISPLAY there",
            )
        )
        return
    if system == "Darwin":
        results.append(CheckResult("viewer_display", "pass", "macOS viewer path uses local window server"))
        return
    if system == "Linux":
        results.append(_linux_display_result(display, wayland))
        return
    results.append(CheckResult("viewer_display", "warn", f"untested host OS: {system}"))


def _linux_display_result(display: str, wayland: str) -> CheckResult:
    if display or wayland:
        return CheckResult("viewer_display", "pass", f"DISPLAY={display!r} WAYLAND_DISPLAY={wayland!r}")
    return CheckResult(
        "viewer_display",
        "warn",
        "no DISPLAY/WAYLAND_DISPLAY; use --headless or configure X11/Wayland forwarding",
    )


__all__ = ["check_display"]
