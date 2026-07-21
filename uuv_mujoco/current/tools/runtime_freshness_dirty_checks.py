"""Dirty working-tree checks for active-runtime freshness."""

from __future__ import annotations

from typing import Any

from runtime_freshness_issue import issue


def append_dirty_worktree_issues(issues: list[dict[str, str]], inputs: dict[str, Any]) -> None:
    dirty_count = int(inputs.get("working_tree_dirty_count") or 0)
    active_dirty_count = int(inputs.get("active_runtime_dirty_count") or 0)
    if dirty_count:
        issues.append(
            issue(
                "warn",
                "working_tree_dirty",
                f"working tree has {dirty_count} changed/untracked paths; RUNTIME_VERSION records this dirty runtime state",
            )
        )
    if active_dirty_count:
        issues.append(
            issue(
                "warn",
                "active_runtime_dirty",
                f"active runtime has {active_dirty_count} changed/untracked paths under uuv_mujoco/current backing files",
            )
        )
    submodule_status = str(inputs.get("ardupilot_submodule_status") or "").strip()
    if submodule_status.startswith(("-", "+", "U")):
        issues.append(
            issue(
                "warn",
                "ardupilot_submodule_not_recorded_pointer",
                f"ArduPilot submodule status is {submodule_status}; do not treat firmware provenance as clean",
            )
        )


__all__ = ["append_dirty_worktree_issues"]
