"""Roll-stability candidate scoring contract."""

from __future__ import annotations

import math
from typing import Any


def compute_stability_score(metrics: dict[str, Any]) -> float:
    depth_drift = metrics["depth_drift_m"] if math.isfinite(metrics["depth_drift_m"]) else 0.0
    return float(
        metrics["roll_rms_deg"]
        + 0.35 * metrics["pitch_rms_deg"]
        + 8.0 * metrics["gyro_x_rms_rad_s"]
        + 15.0 * metrics["depth_std_m"]
        + 10.0 * abs(depth_drift)
    )


__all__ = ["compute_stability_score"]
