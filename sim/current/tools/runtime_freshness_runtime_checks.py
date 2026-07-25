"""Runtime alias and runner checks for active-runtime freshness."""

from __future__ import annotations

from typing import Any

from runtime_freshness_issue import issue


def append_runtime_issues(issues: list[dict[str, str]], inputs: dict[str, Any]) -> None:
    if not inputs["active_alias_exists"]:
        issues.append(issue("fail", "missing_current_alias", "sim/current is missing"))
    _append_runtime_dir_issues(issues, inputs)
    if not inputs["runner_exists"]:
        issues.append(
            issue(
                "fail",
                "missing_runtime_runner",
                f"missing {inputs['resolved_runtime']}/run_uuv_mujoco.py",
            )
        )


def _append_runtime_dir_issues(issues: list[dict[str, str]], inputs: dict[str, Any]) -> None:
    if inputs.get("runtime_dir_uses_active_alias"):
        return
    resolved_runtime = str(inputs.get("resolved_runtime") or "")
    issues.append(
        issue(
            "warn",
            "explicit_runtime_dir_bypasses_current_alias",
            f"runtime dir is {resolved_runtime}, not {inputs.get('active_alias_path')}",
        )
    )


__all__ = ["append_runtime_issues"]
