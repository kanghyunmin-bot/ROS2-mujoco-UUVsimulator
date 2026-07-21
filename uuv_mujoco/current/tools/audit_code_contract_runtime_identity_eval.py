"""Evaluation helpers for active-runtime identity checks."""

from __future__ import annotations

import os

from audit_code_contract_runtime_identity_inputs import RuntimeIdentityInputs


def evaluate_runtime_identity(inputs: RuntimeIdentityInputs) -> dict[str, bool | str]:
    launchers_ready = all(path.exists() and os.access(path, os.X_OK) for path in inputs.root_launchers)
    launchers_call_freshness = all(
        path.exists() and "check_runtime_freshness.py" in path.read_text(encoding="utf-8", errors="replace")
        for path in inputs.root_launchers
    )
    checks = {
        "version_mentions_current": inputs.runtime_version.get("active_runtime") == "uuv_mujoco/current",
        "version_mentions_backing": inputs.runtime_version.get("backing_directory") == "uuv_mujoco/v2.2",
        "version_mentions_branch": inputs.runtime_version.get("source_branch") == "uuv_sim",
        "launchers_ready": launchers_ready,
        "launchers_call_freshness": launchers_call_freshness,
        "freshness_ready": inputs.freshness_script.exists(),
        "branch_current": inputs.repo_branch == "uuv_sim",
        "remote_current": not inputs.origin_uuv_sim or inputs.repo_head == inputs.origin_uuv_sim,
    }
    status = inputs.alias_status
    if status == "PASS" and not all(bool(value) for value in checks.values()):
        status = "WARN"
    return {**checks, "status": status}
