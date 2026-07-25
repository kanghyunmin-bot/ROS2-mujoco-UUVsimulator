"""Force-value extraction for thruster-performance curves."""

from __future__ import annotations

from typing import Any

from audit_closed_loop_thruster_curve_values import append_force_value


def force_values_from_curve(curve: dict[str, Any]) -> list[float]:
    force_values: list[float] = []
    if isinstance(curve.get("force_n"), list):
        for value in curve.get("force_n", []):
            append_force_value(force_values, value)
    for row in curve.get("samples", []):
        if isinstance(row, dict) and "force_n" in row:
            append_force_value(force_values, row["force_n"])
        elif isinstance(row, (list, tuple)) and len(row) >= 2:
            append_force_value(force_values, row[-1])
    return force_values


__all__ = ["force_values_from_curve"]
