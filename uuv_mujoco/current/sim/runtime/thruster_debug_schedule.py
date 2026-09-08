"""Sampling schedule helpers for MuJoCo thruster debug CSV output."""

from __future__ import annotations


# Match the normal 100 Hz SITL actuator cadence.  The previous 20 Hz stream
# could not preserve the JSON-servo reversals that occur between MAVLink
# SERVO_OUTPUT_RAW telemetry packets.
THRUSTER_DEBUG_SAMPLE_PERIOD_S = 0.01
THRUSTER_DEBUG_TIME_EPSILON_S = 1e-9


def should_emit_thruster_debug_sample(
    *,
    sim_t: float,
    next_sample_t: float,
    epsilon_s: float = THRUSTER_DEBUG_TIME_EPSILON_S,
) -> bool:
    """Return whether the current sim time has reached the next sample slot."""
    return float(sim_t) + float(epsilon_s) >= float(next_sample_t)


def advance_thruster_debug_sample_time(
    *,
    sim_t: float,
    next_sample_t: float,
    period_s: float = THRUSTER_DEBUG_SAMPLE_PERIOD_S,
    epsilon_s: float = THRUSTER_DEBUG_TIME_EPSILON_S,
) -> float:
    """Advance the sample cursor past ``sim_t`` using the historical hold policy."""
    cursor = float(next_sample_t)
    while float(sim_t) + float(epsilon_s) >= cursor:
        cursor += float(period_s)
    return cursor


__all__ = [
    "THRUSTER_DEBUG_SAMPLE_PERIOD_S",
    "THRUSTER_DEBUG_TIME_EPSILON_S",
    "advance_thruster_debug_sample_time",
    "should_emit_thruster_debug_sample",
]
