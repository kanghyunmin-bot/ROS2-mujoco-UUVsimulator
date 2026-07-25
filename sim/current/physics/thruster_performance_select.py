"""Candidate selection helpers for offline thruster performance curves."""

from __future__ import annotations

from typing import Any

from .thruster_performance_parse import parse_thruster_performance_curve


def parse_thruster_performance_candidates(curves_raw: list[Any]) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for curve in curves_raw:
        candidate = parse_thruster_performance_curve(curve)
        if candidate is not None:
            candidates.append(candidate)
    return candidates


def select_nearest_thruster_performance_candidate(
    candidates: list[dict[str, Any]],
    requested: float,
) -> dict[str, Any]:
    return min(candidates, key=lambda item: abs(float(item["voltage"]) - requested))


__all__ = [
    "parse_thruster_performance_candidates",
    "select_nearest_thruster_performance_candidate",
]
