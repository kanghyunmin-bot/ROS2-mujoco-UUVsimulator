"""RUNTIME_VERSION freshness checks."""

from __future__ import annotations

from typing import Any

from runtime_freshness_issue import issue


def append_runtime_version_issues(
    issues: list[dict[str, str]],
    runtime_version: dict[str, Any],
    head: str,
    remote_head: str,
    remote_ref: str,
) -> None:
    if runtime_version.get("active_runtime") != "uuv_mujoco/current":
        issues.append(
            issue(
                "warn",
                "runtime_version_not_current_alias",
                "RUNTIME_VERSION.json does not mark uuv_mujoco/current as active",
            )
        )
    if runtime_version.get("source_head") and head and runtime_version.get("source_head") != head:
        issues.append(
            issue(
                "warn",
                "runtime_version_source_head_stale",
                f"RUNTIME_VERSION source_head {runtime_version.get('source_head')} != local HEAD {head}",
            )
        )
    recorded_remote = runtime_version.get("origin_uuv_sim_head")
    if recorded_remote and remote_head and recorded_remote != remote_head:
        issues.append(
            issue(
                "warn",
                "runtime_version_remote_head_stale",
                f"RUNTIME_VERSION origin_uuv_sim_head {recorded_remote} != {remote_ref} {remote_head}",
            )
        )


__all__ = ["append_runtime_version_issues"]
