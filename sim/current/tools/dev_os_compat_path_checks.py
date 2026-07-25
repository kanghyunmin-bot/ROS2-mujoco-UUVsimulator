"""Reusable path-group checks for development OS compatibility."""

from __future__ import annotations

import os
from pathlib import Path

from dev_os_compat_common import ROOT, CheckResult


def relative_missing(paths: list[Path]) -> list[str]:
    return [str(path.relative_to(ROOT)) for path in paths if not path.exists()]


def relative_not_executable(paths: list[Path]) -> list[str]:
    return [str(path.relative_to(ROOT)) for path in paths if path.exists() and not os.access(path, os.X_OK)]


def append_required_files_check(
    results: list[CheckResult],
    *,
    name: str,
    paths: list[Path],
    pass_detail: str,
) -> None:
    missing = relative_missing(paths)
    if missing:
        results.append(CheckResult(name, "fail", "missing: " + ", ".join(missing)))
        return
    results.append(CheckResult(name, "pass", pass_detail))


def append_executable_scripts_check(
    results: list[CheckResult],
    *,
    name: str,
    paths: list[Path],
    pass_detail: str,
) -> None:
    missing = relative_missing(paths)
    if missing:
        results.append(CheckResult(name, "fail", "missing: " + ", ".join(missing)))
        return
    not_executable = relative_not_executable(paths)
    if not_executable:
        results.append(CheckResult(name, "warn", "not executable: " + ", ".join(not_executable)))
        return
    results.append(CheckResult(name, "pass", pass_detail))


__all__ = [
    "append_executable_scripts_check",
    "append_required_files_check",
    "relative_missing",
    "relative_not_executable",
]
