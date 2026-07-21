"""Metric facade for roll stability sweeps."""

from __future__ import annotations

from typing import Any

from roll_stability_math import quat_to_rpy_deg, rms, stddev, unwrap_rad
from roll_stability_pose_metrics import compute_pose_metrics
from roll_stability_rc_metrics import compute_rc_metrics
from roll_stability_score import compute_stability_score


def compute_metrics(
    samples: list[dict[str, Any]],
    depth_samples: list[tuple[float, float]],
    rc_samples: list[tuple[float, list[int]]],
    measure_s: float,
    pulse_s: float,
) -> dict[str, Any]:
    if len(samples) < max(8, measure_s * 2):
        raise RuntimeError(f"too few pose samples collected: {len(samples)}")

    metrics: dict[str, Any] = {}
    metrics.update(compute_pose_metrics(samples, depth_samples, pulse_s))
    metrics.update(compute_rc_metrics(rc_samples))
    metrics["score"] = compute_stability_score(metrics)
    return metrics


__all__ = ["compute_metrics", "quat_to_rpy_deg", "rms", "stddev", "unwrap_rad"]
