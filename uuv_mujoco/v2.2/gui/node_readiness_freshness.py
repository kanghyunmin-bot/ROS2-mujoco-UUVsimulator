"""Freshness predicates for GUI command-readiness checks."""

from __future__ import annotations

import math

from .models import TelemetrySnapshot


def state_fresh(snap: TelemetrySnapshot) -> bool:
    return bool(snap.connected) and math.isfinite(snap.state_age_s) and snap.state_age_s < 3.0


def depth_fresh(snap: TelemetrySnapshot) -> bool:
    return math.isfinite(snap.depth_m) and math.isfinite(snap.depth_age_s) and snap.depth_age_s < 3.0


def imu_fresh(snap: TelemetrySnapshot) -> bool:
    return math.isfinite(snap.imu_age_s) and snap.imu_age_s < 3.0


def rcout_fresh(snap: TelemetrySnapshot) -> bool:
    return math.isfinite(snap.rc_out_age_s) and snap.rc_out_age_s < 3.0


def real_start_fresh(snap: TelemetrySnapshot) -> bool:
    return (
        not bool(snap.real_start_required)
        or (math.isfinite(snap.real_start_age_s) and snap.real_start_age_s < 3.0)
    )


__all__ = ["depth_fresh", "imu_fresh", "rcout_fresh", "real_start_fresh", "state_fresh"]
