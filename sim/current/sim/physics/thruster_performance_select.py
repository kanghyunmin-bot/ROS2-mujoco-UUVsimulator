"""Runtime selection helpers for thruster performance curves."""

from __future__ import annotations

from typing import Any

from sim.physics.thruster_performance_parse import parse_performance_curve


def parse_performance_candidates(payload: dict[str, Any]) -> list[dict[str, Any]]:
    candidates: list[dict[str, Any]] = []
    for curve in payload["curves"]:
        parsed = parse_performance_curve(curve)
        if parsed is not None:
            candidates.append(parsed)
    return candidates


def select_nearest_performance_candidate(candidates: list[dict[str, Any]], requested: float) -> dict[str, Any]:
    return min(candidates, key=lambda item: abs(item["voltage"] - requested))


def selected_curve_config(selected: dict[str, Any], requested: float) -> dict[str, Any]:
    return {
        "active": True,
        "requested_voltage": requested,
        "selected_voltage": float(selected["voltage"]),
        "pwm": selected["pwm"],
        "force": selected["force"],
    }


__all__ = [
    "parse_performance_candidates",
    "select_nearest_performance_candidate",
    "selected_curve_config",
]
