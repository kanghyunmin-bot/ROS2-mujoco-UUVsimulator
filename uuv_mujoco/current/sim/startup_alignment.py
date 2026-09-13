"""Simulator startup readiness from ArduSub's reported estimator health."""

from __future__ import annotations

import math
import os

ALIGNMENT_WAIT = "자세 정렬 중 · 정지 유지 / ARM 대기"


def alignment_required() -> bool:
    return os.environ.get("UUV_STARTUP_ALIGNMENT_HOLD", "0").lower() in {"1", "true", "yes", "on"}


def estimator_aligned(status: dict) -> bool:
    """Require a fresh EKF_ATTITUDE flag, not elapsed wall-clock time.

    ArduSub 4.1.2 sets this flag only when its EKF is healthy and tilt alignment
    is complete. No GPS/horizontal-position flag is required underwater.
    """
    try:
        age = float(status.get("ekf_age_s", math.inf))
        flags = int(status.get("ekf_flags", 0))
    except (ValueError, TypeError, OverflowError):
        return False
    return bool(flags & 1) and math.isfinite(age) and 0 <= age < 5.0


def transport_aligned(transport) -> bool:
    getter = getattr(transport, "mavlink_telemetry_status", None)
    return callable(getter) and estimator_aligned(getter())


def bridge_aligned(bridge) -> bool:
    return transport_aligned(getattr(bridge, "_sitl_transport", None))


def startup_wait_reason(status: dict) -> str:
    if status.get("startup_alignment_required", False) and not estimator_aligned(status):
        return ALIGNMENT_WAIT
    return ""
