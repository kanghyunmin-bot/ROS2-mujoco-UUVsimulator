"""Phase metric construction for control-loop golden fingerprints."""

from __future__ import annotations

from typing import Any

from control_loop_golden_phase_metrics import base_phase_metrics, response_phase_metrics
from control_loop_golden_math import finite_float


def build_phase_metrics(
    row: dict[str, Any],
    *,
    phase_meta: dict[str, dict[str, Any]],
    thruster_by_phase: dict[str, dict[str, Any]],
) -> tuple[str, dict[str, Any]] | None:
    name = str(row.get("phase", ""))
    if not name:
        return None
    metrics = base_phase_metrics(row)
    metrics.update(response_phase_metrics(row))
    if name in phase_meta:
        _add_phase_duration(metrics, phase_meta[name])
    metrics.update(thruster_by_phase.get(name, {}))
    return name, metrics


def _add_phase_duration(metrics: dict[str, Any], meta: dict[str, Any]) -> None:
    start = finite_float(meta.get("start"))
    end = finite_float(meta.get("end"))
    if start is not None and end is not None:
        metrics["duration_s"] = end - start


def build_phase_fingerprint(payload: dict[str, Any], thruster_by_phase: dict[str, dict[str, Any]]) -> dict[str, dict[str, Any]]:
    phases: dict[str, dict[str, Any]] = {}
    phase_meta = {str(item.get("name")): item for item in payload.get("phases", [])}
    for row in payload.get("summary", []):
        phase = build_phase_metrics(row, phase_meta=phase_meta, thruster_by_phase=thruster_by_phase)
        if phase is not None:
            name, metrics = phase
            phases[name] = metrics
    return phases


__all__ = ["build_phase_fingerprint", "build_phase_metrics"]
