"""Small numeric helpers for RC contracts."""

from __future__ import annotations


def clamp_float(value: float, lower: float, upper: float) -> float:
    return max(float(lower), min(float(upper), float(value)))


__all__ = ["clamp_float"]
