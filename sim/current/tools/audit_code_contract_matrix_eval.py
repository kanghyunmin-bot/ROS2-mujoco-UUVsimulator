"""Evaluation helpers for the top-level source-contract matrix."""

from __future__ import annotations

from typing import Iterable

from audit_code_contract_matrix_issue import matrix_issue
from audit_code_contract_types import Check


def check_statuses(checks: list[Check]) -> dict[str, str]:
    return {check.check_id: check.status for check in checks}


def domain_status(
    statuses: dict[str, str],
    required: Iterable[str],
    warn_ok: Iterable[str],
) -> tuple[str, list[str]]:
    warn_ok_set = set(warn_ok)
    issues = [
        issue
        for check_id in required
        if (issue := matrix_issue(check_id, statuses.get(check_id), warn_ok_set)) is not None
    ]
    if any(item.startswith(("missing:", "fail:")) for item in issues):
        return "FAIL", issues
    if any(item.startswith("warn:") for item in issues):
        return "WARN", issues
    return "PASS", issues


def aggregate_matrix_status(domain_statuses: Iterable[str]) -> str:
    statuses = tuple(domain_statuses)
    if "FAIL" in statuses:
        return "FAIL"
    if "WARN" in statuses:
        return "WARN"
    return "PASS"


__all__ = ["aggregate_matrix_status", "check_statuses", "domain_status"]
