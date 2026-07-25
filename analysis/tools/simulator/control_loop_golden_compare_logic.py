"""Comparison policy for control-loop golden fingerprints."""

from __future__ import annotations

import argparse
from typing import Any

from control_loop_golden_compare_metrics import comparable_metric_names, metric_tolerance
from control_loop_golden_compare_phases import compare_phase_set, record_candidate_health
from control_loop_golden_compare_records import record_metric_comparison


def compare_fingerprints(
    candidate: dict[str, Any],
    baseline: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    failures: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    comparisons: list[dict[str, Any]] = []

    record_candidate_health(candidate, failures, warnings)
    compare_phase_set(candidate, baseline, args, failures, warnings, comparisons)

    overall = "fail" if failures else ("warn" if warnings else "pass")
    return {
        "overall": overall,
        "failures": failures,
    "warnings": warnings,
        "comparison_count": len(comparisons),
    }


__all__ = [
    "comparable_metric_names",
    "compare_fingerprints",
    "metric_tolerance",
    "record_metric_comparison",
]
