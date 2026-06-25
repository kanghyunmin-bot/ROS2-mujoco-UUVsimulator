"""Evaluate active-runtime freshness evidence."""

from __future__ import annotations

from typing import Any

from runtime_freshness_dirty_checks import append_dirty_worktree_issues
from runtime_freshness_runtime_checks import append_runtime_issues
from runtime_freshness_source_checks import append_source_issues
from runtime_freshness_version_checks import append_runtime_version_issues


def evaluate_freshness(inputs: dict[str, Any]) -> dict[str, Any]:
    issues: list[dict[str, str]] = []
    runtime_version = inputs["runtime_version"]
    head = inputs["head"]
    remote_head = inputs["remote_head"]
    remote_ref = inputs["remote_ref"]
    fetch_status = inputs["fetch"]

    append_runtime_issues(issues, inputs)
    append_source_issues(issues, inputs)
    append_dirty_worktree_issues(issues, inputs)
    append_runtime_version_issues(issues, runtime_version, head, remote_head, remote_ref)
    status = "fail" if any(item["level"] == "fail" for item in issues) else "warn" if issues else "pass"
    return {
        "status": status,
        "workspace": inputs["workspace"],
        "active_runtime": inputs["resolved_runtime"],
        "active_alias": f"uuv_mujoco/current -> {inputs['active_alias_text']}",
        "runtime_dir_uses_active_alias": inputs.get("runtime_dir_uses_active_alias", False),
        "branch": inputs["branch"],
        "head": head,
        "remote_ref": remote_ref,
        "remote_head": remote_head,
        "runtime_version": inputs["runtime_version_path"],
        "working_tree_dirty_count": inputs.get("working_tree_dirty_count", 0),
        "active_runtime_dirty_count": inputs.get("active_runtime_dirty_count", 0),
        "working_tree_dirty_sample": inputs.get("working_tree_dirty_sample", []),
        "active_runtime_dirty_sample": inputs.get("active_runtime_dirty_sample", []),
        "ardupilot_submodule_status": inputs.get("ardupilot_submodule_status", ""),
        "fetch": fetch_status,
        "issues": issues,
    }
