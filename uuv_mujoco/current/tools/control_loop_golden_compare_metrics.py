"""Metric selection and tolerance helpers for golden control-loop comparison."""

from __future__ import annotations

import argparse
from typing import Any


def metric_tolerance(metric: str, value_a: float, value_b: float, args: argparse.Namespace) -> tuple[float, float]:
    scale = max(abs(value_a), abs(value_b), 1.0)
    if "pwm" in metric or metric.startswith("rcin_") or metric.startswith("rcout_"):
        return args.pwm_tol, 0.0
    if metric.endswith("_samples") or metric == "samples":
        return max(2.0, 0.1 * scale), 0.0
    if metric.startswith("thr_force_body_"):
        return args.force_abs_tol, args.force_rel_tol
    if metric.startswith("thr_torque_body_"):
        return args.torque_abs_tol, args.torque_rel_tol
    return args.abs_tol, args.relative_tol


def comparable_metric_names(base_phase: dict[str, Any], cand_phase: dict[str, Any]) -> list[str]:
    return sorted(
        key
        for key in set(base_phase) & set(cand_phase)
        if key not in {"axis"} and isinstance(base_phase.get(key), (int, float)) and isinstance(cand_phase.get(key), (int, float))
    )


def metric_is_failure(metric: str) -> bool:
    return metric.startswith("expected_metric") or metric.startswith("rcout_") or metric.startswith("thr_")


__all__ = ["comparable_metric_names", "metric_is_failure", "metric_tolerance"]
