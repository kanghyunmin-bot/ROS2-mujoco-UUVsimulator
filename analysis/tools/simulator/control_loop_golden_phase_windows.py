"""Phase window helpers for golden control-loop summaries."""

from __future__ import annotations

from typing import Any

from control_loop_golden_math import finite_float


def phase_windows(payload: dict[str, Any]) -> dict[str, tuple[float, float]]:
    metadata = payload.get("metadata", {})
    node_start = finite_float(metadata.get("node_start_wall_mono_s"))
    if node_start is None:
        return {}
    windows: dict[str, tuple[float, float]] = {}
    for phase in payload.get("phases", []):
        start = finite_float(phase.get("start"))
        end = finite_float(phase.get("end"))
        name = str(phase.get("name", ""))
        if not name or start is None or end is None:
            continue
        windows[name] = (node_start + start, node_start + end)
    return windows


__all__ = ["phase_windows"]
