"""Comparison record helpers for golden control-loop comparison."""

from __future__ import annotations

import argparse
from typing import Any

from control_loop_golden_compare_metrics import metric_is_failure, metric_tolerance


def metric_comparison_item(
    *,
    phase_name: str,
    metric: str,
    candidate_value: float,
    baseline_value: float,
    args: argparse.Namespace,
) -> dict[str, Any]:
    abs_diff = abs(candidate_value - baseline_value)
    abs_tol, rel_tol = metric_tolerance(metric, candidate_value, baseline_value, args)
    allowed = max(abs_tol, rel_tol * max(abs(candidate_value), abs(baseline_value), 1.0))
    return {
        "phase": phase_name,
        "metric": metric,
        "candidate": candidate_value,
        "baseline": baseline_value,
        "abs_diff": abs_diff,
        "allowed": allowed,
    }


def record_metric_comparison(
    failures: list[dict[str, Any]],
    warnings: list[dict[str, Any]],
    comparisons: list[dict[str, Any]],
    *,
    phase_name: str,
    metric: str,
    candidate_value: float,
    baseline_value: float,
    args: argparse.Namespace,
) -> None:
    item = metric_comparison_item(
        phase_name=phase_name,
        metric=metric,
        candidate_value=candidate_value,
        baseline_value=baseline_value,
        args=args,
    )
    if item["abs_diff"] > item["allowed"]:
        if metric_is_failure(metric):
            failures.append(item)
        else:
            warnings.append(item)
    comparisons.append(item)


__all__ = ["metric_comparison_item", "record_metric_comparison"]
