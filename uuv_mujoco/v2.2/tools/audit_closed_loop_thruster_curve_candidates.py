"""Thruster-performance curve candidate extraction."""

from __future__ import annotations

from typing import Any

from audit_closed_loop_thruster_curve_values import (
    curve_voltage_from_mapping_key,
    curve_voltage_from_row,
)


def curve_candidates(curves: Any) -> list[tuple[float, dict[str, Any]]]:
    if isinstance(curves, dict):
        return curve_candidates_from_mapping(curves)
    if isinstance(curves, list):
        return curve_candidates_from_list(curves)
    return []


def curve_candidates_from_mapping(curves: dict[str, Any]) -> list[tuple[float, dict[str, Any]]]:
    candidates: list[tuple[float, dict[str, Any]]] = []
    for key, curve in curves.items():
        voltage = curve_voltage_from_mapping_key(key, curve)
        if voltage is not None and isinstance(curve, dict):
            candidates.append((voltage, curve))
    return candidates


def curve_candidates_from_list(curves: list[Any]) -> list[tuple[float, dict[str, Any]]]:
    candidates: list[tuple[float, dict[str, Any]]] = []
    for curve in curves:
        voltage = curve_voltage_from_row(curve)
        if voltage is not None and isinstance(curve, dict):
            candidates.append((voltage, curve))
    return candidates


__all__ = [
    "curve_candidates",
    "curve_candidates_from_list",
    "curve_candidates_from_mapping",
]
