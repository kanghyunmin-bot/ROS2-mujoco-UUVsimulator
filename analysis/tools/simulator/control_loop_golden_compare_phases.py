"""Phase comparison helpers for golden control-loop comparison."""

from __future__ import annotations

import argparse
from typing import Any

from control_loop_golden_compare_metrics import comparable_metric_names
from control_loop_golden_compare_records import record_metric_comparison


def record_candidate_health(
    candidate: dict[str, Any],
    failures: list[dict[str, Any]],
    warnings: list[dict[str, Any]],
) -> None:
    cand_health = str(candidate.get("health", {}).get("overall", "unknown"))
    if cand_health == "fail":
        failures.append({"kind": "candidate_health", "candidate": cand_health})
    elif cand_health == "warn":
        warnings.append({"kind": "candidate_health", "candidate": cand_health})


def compare_phase_metrics(
    *,
    phase_name: str,
    base_phase: dict[str, Any],
    cand_phase: dict[str, Any],
    args: argparse.Namespace,
    failures: list[dict[str, Any]],
    warnings: list[dict[str, Any]],
    comparisons: list[dict[str, Any]],
) -> None:
    for metric in comparable_metric_names(base_phase, cand_phase):
        record_metric_comparison(
            failures,
            warnings,
            comparisons,
            phase_name=phase_name,
            metric=metric,
            candidate_value=float(cand_phase[metric]),
            baseline_value=float(base_phase[metric]),
            args=args,
        )


def compare_phase_set(
    candidate: dict[str, Any],
    baseline: dict[str, Any],
    args: argparse.Namespace,
    failures: list[dict[str, Any]],
    warnings: list[dict[str, Any]],
    comparisons: list[dict[str, Any]],
) -> None:
    cand_phases = candidate.get("phases", {})
    base_phases = baseline.get("phases", {})
    for phase_name, base_phase in base_phases.items():
        cand_phase = cand_phases.get(phase_name)
        if cand_phase is None:
            failures.append({"kind": "missing_phase", "phase": phase_name})
            continue
        compare_phase_metrics(
            phase_name=phase_name,
            base_phase=base_phase,
            cand_phase=cand_phase,
            args=args,
            failures=failures,
            warnings=warnings,
            comparisons=comparisons,
        )


__all__ = ["compare_phase_metrics", "compare_phase_set", "record_candidate_health"]
