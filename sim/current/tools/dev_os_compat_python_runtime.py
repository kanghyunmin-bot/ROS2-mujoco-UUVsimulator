"""Runtime Python and MuJoCo import checks."""

from __future__ import annotations

from dev_os_compat_common import (
    CheckResult,
)
from dev_os_compat_python_checks import check_mujoco, check_python
from dev_os_compat_python_probe import runtime_python_candidates
from dev_os_compat_python_select import select_first_passing_runtime_python


def select_runtime_python(
    results: list[CheckResult],
    *,
    explicit_python: str | None,
    require_viewer: bool,
) -> dict[str, object] | None:
    candidates = runtime_python_candidates(explicit_python)
    if not candidates:
        results.append(CheckResult("runtime_python", "fail", "no executable Python candidate found"))
        return None

    selection = select_first_passing_runtime_python(candidates, require_viewer=require_viewer)
    if selection.passed:
        results.append(CheckResult("runtime_python", "pass", str(selection.passed_candidate)))
        return selection.payload

    results.append(CheckResult("runtime_python", "fail", selection.failure_detail()))
    return selection.fallback_payload


__all__ = ["check_mujoco", "check_python", "select_runtime_python"]
