"""Severity flag helpers for axis RC health checks."""

from __future__ import annotations


def apply_failure_flag(flags: list[str], severity: str, flag: str) -> str:
    flags.append(flag)
    return "fail"


def apply_warning_flag(flags: list[str], severity: str, flag: str) -> str:
    flags.append(flag)
    return "warn" if severity == "ok" else severity


__all__ = ["apply_failure_flag", "apply_warning_flag"]
