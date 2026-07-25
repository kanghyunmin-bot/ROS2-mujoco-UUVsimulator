"""Scalar parsers for simulation profile fields."""

from __future__ import annotations

from typing import Any


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, float(value)))


def bool_from_value(value: Any, default: bool = False) -> bool:
    if value is None:
        return bool(default)
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on", "enable", "enabled"}


__all__ = ["bool_from_value", "clamp"]
