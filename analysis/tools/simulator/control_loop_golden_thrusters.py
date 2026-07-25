"""Thruster/body-wrench phase summaries for golden control-loop checks."""

from __future__ import annotations

from typing import Any

from control_loop_golden_phase_windows import phase_windows
from control_loop_golden_thruster_columns import interesting_thruster_columns, sitl_pwm_columns
from control_loop_golden_thruster_io import load_thruster_csv
from control_loop_golden_thruster_metrics import summarize_numeric_columns, summarize_pwm_deltas
from control_loop_golden_thruster_rows import rows_in_window, rows_with_wall_time


def summarize_thrusters(
    payload: dict[str, Any],
    thruster_rows: list[dict[str, str]],
    fieldnames: list[str],
) -> dict[str, dict[str, float]]:
    if not thruster_rows:
        return {}
    windows = phase_windows(payload)
    if not windows:
        return {}

    pwm_columns = sitl_pwm_columns(fieldnames)
    interesting = interesting_thruster_columns(fieldnames)
    row_wall = rows_with_wall_time(thruster_rows)

    by_phase: dict[str, dict[str, float]] = {}
    for phase_name, (start, end) in windows.items():
        part = rows_in_window(row_wall, start=start, end=end)
        if not part:
            continue
        metrics: dict[str, float] = {"thruster_samples": float(len(part))}
        metrics.update(summarize_numeric_columns(part, interesting))
        metrics.update(summarize_pwm_deltas(part, pwm_columns))
        by_phase[phase_name] = metrics
    return by_phase
