"""Line-level parsers for closed-loop parameter contract audits."""

from __future__ import annotations


def parse_param_assignment(raw_line: str) -> tuple[str, str] | None:
    line = raw_line.strip()
    if not line or line.startswith("#"):
        return None
    parts = [part for part in line.replace(",", " ").split() if part]
    if len(parts) < 2:
        return None
    return parts[0], parts[1]


def is_start_sitl_enforced_header(raw_line: str) -> bool:
    return "[start-sitl] enforcing params via" in raw_line


def is_start_sitl_enforced_row(raw_line: str) -> bool:
    return raw_line.startswith("  ")


__all__ = [
    "is_start_sitl_enforced_header",
    "is_start_sitl_enforced_row",
    "parse_param_assignment",
]
