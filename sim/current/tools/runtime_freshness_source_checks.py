"""Git source freshness checks for the active runtime."""

from __future__ import annotations

from typing import Any

from runtime_freshness_issue import issue


def append_source_issues(issues: list[dict[str, str]], inputs: dict[str, Any]) -> None:
    branch = inputs["branch"]
    head = inputs["head"]
    remote_head = inputs["remote_head"]
    remote_ref = inputs["remote_ref"]
    fetch_status = inputs["fetch"]

    if branch != "uuv_sim":
        issues.append(issue("warn", "unexpected_branch", f"active branch is {branch or '<unknown>'}"))
    if fetch_status["attempted"] and fetch_status["code"] not in (0, None):
        issues.append(issue("warn", "fetch_failed", fetch_status["output"] or "git fetch failed"))
    if remote_head and head and head != remote_head:
        issues.append(
            issue(
                "fail",
                "local_head_not_remote_head",
                f"local HEAD {head} != {remote_ref} {remote_head}",
            )
        )


__all__ = ["append_source_issues"]
