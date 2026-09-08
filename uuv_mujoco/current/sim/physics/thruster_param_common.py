"""Shared helpers for thruster parameter loading."""

from __future__ import annotations

from typing import Callable, Optional


def is_number(value) -> bool:
    return not isinstance(value, bool) and isinstance(value, (int, float))


def log_optional(log: Optional[Callable[[str], None]], message: str) -> None:
    if log is not None:
        log(message)


__all__ = ["is_number", "log_optional"]
