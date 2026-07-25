"""Shared issue helpers for active-runtime freshness checks."""

from __future__ import annotations


def issue(level: str, issue_id: str, detail: str) -> dict[str, str]:
    return {"level": level, "id": issue_id, "detail": detail}


__all__ = ["issue"]
