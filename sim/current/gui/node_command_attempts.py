"""Shared retry-attempt helpers for GUI command requests."""

from __future__ import annotations


def should_log_attempt(attempt: int) -> bool:
    return attempt == 1 or attempt % 4 == 0


__all__ = ["should_log_attempt"]
