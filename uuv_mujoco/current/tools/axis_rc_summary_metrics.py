"""Phase summary metrics for axis RC override checks."""

from __future__ import annotations

import math
from typing import Any

from axis_rc_contract import AXIS_TO_CHANNEL, EXPECTED_AXIS_METRIC, MIN_EXPECTED_PEAK, Phase, RC_NEUTRAL
from axis_rc_metric_math import finite_values, mean, rms


ODOM_VELOCITY_FALLBACK = {
    "dvl_vx": "odom_vx",
    "dvl_vy": "odom_vy",
    "dvl_vz": "odom_vz",
}


def _tail_values(values: list[float]) -> list[float]:
    finite = finite_values(values)
    if not finite:
        return []
    tail_count = max(1, len(finite) // 4)
    return finite[-tail_count:]


def _channel_delta_stats(samples: list[dict[str, Any]], prefix: str, channels: range) -> tuple[float, float, float]:
    rows: list[list[float]] = []
    for sample in samples:
        row: list[float] = []
        for idx in channels:
            value = float(sample.get(f"{prefix}{idx}", float("nan")))
            if not math.isfinite(value):
                break
            row.append(value - RC_NEUTRAL)
        else:
            rows.append(row)
    if not rows:
        return float("nan"), float("nan"), float("nan")
    common_values = [mean(row) for row in rows]
    diff_values = [max(abs(value - common) for value in row) for row, common in zip(rows, common_values)]
    mean_abs_values = [mean([abs(value) for value in row]) for row in rows]
    return mean(common_values), max(diff_values, default=float("nan")), mean(mean_abs_values)


def _signed_vertical_output_stats(samples: list[dict[str, Any]]) -> tuple[float, float, float]:
    """Project RCOUT5..8 through the configured 6DOF motor directions.

    A pure heave command is (+,-,-,+) in final PWM delta for the current
    ArduSub motor-direction contract.  A raw channel mean therefore cancels
    the very signal this metric is intended to observe.
    """
    rows: list[list[float]] = []
    signs = (1.0, -1.0, -1.0, 1.0)
    for sample in samples:
        projected: list[float] = []
        for idx, sign in zip(range(5, 9), signs):
            value = float(sample.get(f"rcout{idx}", float("nan")))
            if not math.isfinite(value):
                break
            projected.append(sign * (value - RC_NEUTRAL))
        else:
            rows.append(projected)
    if not rows:
        return float("nan"), float("nan"), float("nan")
    common_values = [mean(row) for row in rows]
    diff_values = [max(abs(value - common) for value in row) for row, common in zip(rows, common_values)]
    mean_abs_values = [mean([abs(value) for value in row]) for row in rows]
    return mean(common_values), max(diff_values, default=float("nan")), mean(mean_abs_values)


def _first_onset_delay(
    samples: list[dict[str, Any]],
    *,
    phase_start_s: float,
    key: str,
    threshold: float,
    center: float = 0.0,
) -> float:
    for sample in samples:
        value = float(sample.get(key, float("nan")))
        sample_t = float(sample.get("t", float("nan")))
        if not math.isfinite(value) or not math.isfinite(sample_t):
            continue
        if abs(value - center) >= threshold:
            return max(0.0, sample_t - phase_start_s)
    return float("nan")


def _max_peak(samples: list[dict[str, Any]], key: str) -> float:
    values = [float(sample.get(key, float("nan"))) for sample in samples]
    return max((abs(value) for value in values if math.isfinite(value)), default=float("nan"))


def _select_expected_metric(axis: str, row: dict[str, Any]) -> str | None:
    primary = EXPECTED_AXIS_METRIC.get(axis)
    if not primary:
        return None
    primary_peak = float(row.get(f"{primary}_peak_abs", float("nan")))
    if math.isfinite(primary_peak):
        return primary
    # The strict Bar30/IMU ALT_HOLD profile intentionally has no DVL topic.
    # Odometry remains a valid plant-response oracle for the axis acceptance
    # test, so do not turn missing optional DVL telemetry into a false failure.
    return ODOM_VELOCITY_FALLBACK.get(primary, primary)


def _add_onset_latency_metrics(row: dict[str, Any], phase: Phase, samples: list[dict[str, Any]]) -> None:
    axis = str(phase.axis)
    channel_idx = AXIS_TO_CHANNEL.get(axis)
    if channel_idx is not None:
        row["rcin_onset_delay_s"] = _first_onset_delay(
            samples,
            phase_start_s=float(phase.start),
            key=f"rcin{channel_idx + 1}",
            threshold=5.0,
            center=RC_NEUTRAL,
        )
    else:
        row["rcin_onset_delay_s"] = float("nan")

    expected_metric = row.get("expected_metric")
    if not expected_metric:
        row["expected_metric_onset_delay_s"] = float("nan")
        row["response_after_rcin_delay_s"] = float("nan")
        return

    peak = _max_peak(samples, expected_metric)
    metric_threshold = max(float(MIN_EXPECTED_PEAK.get(axis, 0.0)), 0.25 * peak) if math.isfinite(peak) else float("nan")
    row["expected_metric_onset_threshold"] = metric_threshold
    response_delay = (
        _first_onset_delay(
            samples,
            phase_start_s=float(phase.start),
            key=expected_metric,
            threshold=metric_threshold,
        )
        if math.isfinite(metric_threshold)
        else float("nan")
    )
    row["expected_metric_onset_delay_s"] = response_delay
    rcin_delay = float(row.get("rcin_onset_delay_s", float("nan")))
    if math.isfinite(response_delay) and math.isfinite(rcin_delay):
        row["response_after_rcin_delay_s"] = max(0.0, response_delay - rcin_delay)
    else:
        row["response_after_rcin_delay_s"] = float("nan")


def summarize(samples: list[dict[str, Any]], phases: list[Phase]) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for phase in phases:
        part = [s for s in samples if s.get("phase") == phase.name]
        row: dict[str, Any] = {
            "phase": phase.name,
            "axis": phase.axis,
            "command": phase.command,
            "samples": len(part),
        }
        for key in ("gyro_x", "gyro_y", "gyro_z", "dvl_vx", "dvl_vy", "dvl_vz", "odom_vx", "odom_vy", "odom_vz"):
            values = [float(s.get(key, float("nan"))) for s in part]
            tail = _tail_values(values)
            row[f"{key}_mean"] = mean(values)
            row[f"{key}_rms"] = rms(values)
            row[f"{key}_peak_abs"] = max((abs(v) for v in values if math.isfinite(v)), default=float("nan"))
            row[f"{key}_tail_mean"] = mean(tail)
            row[f"{key}_tail_rms"] = rms(tail)
            row[f"{key}_tail_peak_abs"] = max((abs(v) for v in tail), default=float("nan"))
        for key in ("roll_rad", "pitch_rad", "yaw_rad", "depth_m"):
            values = [float(s.get(key, float("nan"))) for s in part]
            finite = finite_values(values)
            if key == "depth_m" and not finite:
                # Strict real-package parity exposes Bar30 depth to ArduSub but
                # does not publish the optional /depth helper topic.  MAVROS
                # local odometry is up-positive, hence depth is -odom_z.
                values = [-float(s.get("odom_z", float("nan"))) for s in part]
                finite = finite_values(values)
            row[f"{key}_mean"] = mean(values)
            row[f"{key}_span"] = (max(finite) - min(finite)) if finite else float("nan")
        for prefix in ("rcin", "rcout"):
            deltas = []
            tail_deltas = []
            for idx in range(1, 9):
                values = [float(s.get(f"{prefix}{idx}", float("nan"))) for s in part]
                finite = finite_values(values)
                tail = _tail_values(values)
                deltas.extend(abs(v - RC_NEUTRAL) for v in finite)
                tail_deltas.extend(abs(v - RC_NEUTRAL) for v in tail)
            row[f"{prefix}_max_delta"] = max(deltas, default=float("nan"))
            row[f"{prefix}_mean_abs_delta"] = mean(deltas)
            row[f"{prefix}_tail_max_delta"] = max(tail_deltas, default=float("nan"))
            row[f"{prefix}_tail_mean_abs_delta"] = mean(tail_deltas)
            tail_samples = part[-max(1, len(part) // 4) :] if part else []
            h_common, h_diff, h_mean_abs = _channel_delta_stats(tail_samples, prefix, range(1, 5))
            if prefix == "rcout":
                v_common, v_diff, v_mean_abs = _signed_vertical_output_stats(tail_samples)
            else:
                v_common, v_diff, v_mean_abs = _channel_delta_stats(tail_samples, prefix, range(5, 9))
            row[f"{prefix}_tail_horizontal_common_delta"] = h_common
            row[f"{prefix}_tail_horizontal_diff_max_abs"] = h_diff
            row[f"{prefix}_tail_horizontal_mean_abs_delta"] = h_mean_abs
            row[f"{prefix}_tail_vertical_common_delta"] = v_common
            row[f"{prefix}_tail_vertical_diff_max_abs"] = v_diff
            row[f"{prefix}_tail_vertical_mean_abs_delta"] = v_mean_abs
        expected_metric = _select_expected_metric(str(phase.axis), row)
        if expected_metric:
            row["expected_metric"] = expected_metric
            row["expected_metric_mean"] = row.get(f"{expected_metric}_mean", float("nan"))
            row["expected_metric_rms"] = row.get(f"{expected_metric}_rms", float("nan"))
            row["expected_metric_peak_abs"] = row.get(f"{expected_metric}_peak_abs", float("nan"))
            row["expected_metric_tail_mean"] = row.get(f"{expected_metric}_tail_mean", float("nan"))
            row["expected_metric_tail_rms"] = row.get(f"{expected_metric}_tail_rms", float("nan"))
            row["expected_metric_tail_peak_abs"] = row.get(f"{expected_metric}_tail_peak_abs", float("nan"))
        _add_onset_latency_metrics(row, phase, part)
        row["armed_fraction"] = mean([1.0 if s.get("armed") else 0.0 for s in part])
        rows.append(row)
    return rows


__all__ = ["summarize"]
