"""Thruster-performance curve selection for closed-loop contract audits."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from audit_closed_loop_thruster_curve_parse import curve_candidates, force_values_from_curve


def selected_thruster_curve(path: Path, requested_voltage: float) -> dict[str, Any]:
    if not path.exists():
        return {"active": False, "reason": f"missing file: {path}"}
    payload = json.loads(path.read_text(encoding="utf-8"))
    candidates = curve_candidates(payload.get("curves", {}))
    if not candidates:
        return {"active": False, "reason": "no usable curves"}
    voltage, curve = min(candidates, key=lambda item: abs(item[0] - requested_voltage))
    force_values = force_values_from_curve(curve)
    return {
        "active": bool(force_values),
        "requested_voltage": requested_voltage,
        "selected_voltage": voltage,
        "force_min_n": min(force_values) if force_values else None,
        "force_max_n": max(force_values) if force_values else None,
    }


__all__ = [
    "curve_candidates",
    "force_values_from_curve",
    "selected_thruster_curve",
]
