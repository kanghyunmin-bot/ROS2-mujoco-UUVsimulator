"""Build active-runtime version metadata payloads."""

from __future__ import annotations

from datetime import date
from typing import Any

from runtime_freshness_version_constants import (
    COMPATIBILITY_RUNNER,
    DEFAULT_FRESHNESS_POLICY,
    PRIMARY_RUNNER,
    RUNTIME_VERSION_NOTE,
    SOURCE_AUDIT_CHECK,
)
from runtime_freshness_version_dirty import runtime_dirty_state, runtime_status_from_dirty_state
from runtime_freshness_version_paths import backing_directory, root_launchers


def build_runtime_version(
    inputs: dict[str, Any],
    existing: dict[str, Any],
    *,
    freshness_status: str = "unknown",
) -> dict[str, Any]:
    branch = inputs.get("branch") or "unknown"
    today = date.today().isoformat()
    dirty_state = runtime_dirty_state(inputs)
    return {
        "schema": int(existing.get("schema") or 1),
        "updated_at": today,
        "active_runtime": "uuv_mujoco/current",
        "backing_directory": backing_directory(inputs.get("active_alias_text", ""), existing),
        "status": runtime_status_from_dirty_state(dirty_state),
        "freshness_status": freshness_status,
        "active_runtime_label": f"current-{today}-{branch}",
        "source_branch": branch,
        "source_remote": inputs.get("source_remote") or existing.get("source_remote") or "",
        "source_head": inputs.get("head") or "",
        "origin_uuv_sim_head": inputs.get("remote_head") or "",
        "dirty_state": dirty_state,
        "freshness_policy": existing.get("freshness_policy") or DEFAULT_FRESHNESS_POLICY,
        "primary_runner": PRIMARY_RUNNER,
        "compatibility_runner": COMPATIBILITY_RUNNER,
        "root_launchers": root_launchers(),
        "source_audit_check": SOURCE_AUDIT_CHECK,
        "note": RUNTIME_VERSION_NOTE,
    }


__all__ = ["build_runtime_version"]
