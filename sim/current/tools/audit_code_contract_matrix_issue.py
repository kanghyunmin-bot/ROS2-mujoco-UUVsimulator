"""Single-check issue classification for the contract matrix gate."""

from __future__ import annotations


def matrix_issue(check_id: str, status: str | None, warn_ok: set[str]) -> str | None:
    if status is None:
        return f"missing:{check_id}"
    if status == "PASS":
        return None
    if status == "WARN" and check_id in warn_ok:
        return f"warn:{check_id}"
    return f"{status.lower()}:{check_id}"


__all__ = ["matrix_issue"]
