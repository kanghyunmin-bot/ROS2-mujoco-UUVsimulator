"""Wall-clock age helpers for SITL status payloads."""

from __future__ import annotations


def wall_age_s(now_wall: float, last_wall: float) -> float:
    return float(max(0.0, now_wall - last_wall)) if last_wall > 0.0 else float("inf")


__all__ = ["wall_age_s"]
