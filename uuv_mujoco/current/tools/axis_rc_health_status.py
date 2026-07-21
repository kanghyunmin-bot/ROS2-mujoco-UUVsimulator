"""Overall status aggregation for axis RC health checks."""

from __future__ import annotations

from typing import Any


def overall_health(checks: list[dict[str, Any]]) -> str:
    if any(check["severity"] == "fail" for check in checks):
        return "fail"
    if any(check["severity"] == "warn" for check in checks):
        return "warn"
    return "pass"


__all__ = ["overall_health"]
