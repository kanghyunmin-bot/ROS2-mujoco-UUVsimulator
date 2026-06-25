"""Metric helpers for golden control-loop thruster summaries."""

from __future__ import annotations

from control_loop_golden_math import finite_float, max_abs, mean, rms


def summarize_numeric_columns(
    rows: list[dict[str, str]],
    columns: list[str],
) -> dict[str, float]:
    metrics: dict[str, float] = {}
    for col in columns:
        vals = [finite_float(row.get(col)) for row in rows]
        finite = [value for value in vals if value is not None]
        if not finite:
            continue
        metrics[f"{col}_mean"] = float(mean(finite) or 0.0)
        metrics[f"{col}_rms"] = float(rms(finite) or 0.0)
        metrics[f"{col}_peak_abs"] = float(max_abs(finite) or 0.0)
    return metrics


def summarize_pwm_deltas(
    rows: list[dict[str, str]],
    pwm_columns: list[str],
    *,
    trim_pwm: float = 1500.0,
) -> dict[str, float]:
    pwm_deltas: list[float] = []
    for col in pwm_columns:
        pwm_deltas.extend(
            abs(value - float(trim_pwm))
            for value in (finite_float(row.get(col)) for row in rows)
            if value is not None
        )
    if not pwm_deltas:
        return {}
    return {
        "thruster_pwm_max_delta": float(max(pwm_deltas)),
        "thruster_pwm_mean_abs_delta": float(mean(pwm_deltas) or 0.0),
    }


__all__ = ["summarize_numeric_columns", "summarize_pwm_deltas"]
