#!/usr/bin/env python3
"""Build and compare OS-independent closed-loop control fingerprints.

The input is the axis_rc_override_check output.  When MuJoCo thruster debug CSV
is supplied, this also aligns thruster/body wrench rows to the same phase
windows using monotonic wall time.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


RESPONSE_METRICS = (
    "samples",
    "armed_fraction",
    "rcin_max_delta",
    "rcout_max_delta",
    "rcin_mean_abs_delta",
    "rcout_mean_abs_delta",
    "expected_metric_mean",
    "expected_metric_rms",
    "expected_metric_peak_abs",
    "gyro_x_mean",
    "gyro_y_mean",
    "gyro_z_mean",
    "gyro_x_rms",
    "gyro_y_rms",
    "gyro_z_rms",
    "gyro_x_peak_abs",
    "gyro_y_peak_abs",
    "gyro_z_peak_abs",
    "dvl_vx_mean",
    "dvl_vy_mean",
    "dvl_vz_mean",
    "dvl_vx_rms",
    "dvl_vy_rms",
    "dvl_vz_rms",
    "dvl_vx_peak_abs",
    "dvl_vy_peak_abs",
    "dvl_vz_peak_abs",
    "odom_vx_mean",
    "odom_vy_mean",
    "odom_vz_mean",
    "roll_rad_span",
    "pitch_rad_span",
    "yaw_rad_span",
    "depth_m_mean",
    "depth_m_span",
)

THRUSTER_BODY_COLUMNS = (
    "thr_force_body_x",
    "thr_force_body_y",
    "thr_force_body_z",
    "thr_torque_body_x",
    "thr_torque_body_y",
    "thr_torque_body_z",
)


def finite_float(value: Any) -> float | None:
    try:
        out = float(value)
    except (TypeError, ValueError):
        return None
    return out if math.isfinite(out) else None


def mean(values: list[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    return sum(finite) / len(finite)


def rms(values: list[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    return math.sqrt(sum(v * v for v in finite) / len(finite))


def max_abs(values: list[float]) -> float | None:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return None
    return max(abs(v) for v in finite)


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text())


def load_thruster_csv(path: Path | None) -> tuple[list[dict[str, str]], list[str]]:
    if path is None or not path.exists():
        return [], []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        return rows, list(reader.fieldnames or [])


def phase_windows(payload: dict[str, Any]) -> dict[str, tuple[float, float]]:
    metadata = payload.get("metadata", {})
    node_start = finite_float(metadata.get("node_start_wall_mono_s"))
    if node_start is None:
        return {}
    windows: dict[str, tuple[float, float]] = {}
    for phase in payload.get("phases", []):
        start = finite_float(phase.get("start"))
        end = finite_float(phase.get("end"))
        name = str(phase.get("name", ""))
        if not name or start is None or end is None:
            continue
        windows[name] = (node_start + start, node_start + end)
    return windows


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

    pwm_columns = [f"sitl_ch{i}_pwm" for i in range(1, 9) if f"sitl_ch{i}_pwm" in fieldnames]
    servo_norm_columns = [name for name in fieldnames if name.endswith("_servo_norm")]
    thruster_force_columns = [
        name
        for name in fieldnames
        if name.endswith("_force") and not name.startswith("thr_force_body_")
    ]
    interesting = list(THRUSTER_BODY_COLUMNS) + pwm_columns + servo_norm_columns + thruster_force_columns
    interesting = [name for name in interesting if name in fieldnames]

    by_phase: dict[str, dict[str, float]] = {}
    row_wall: list[tuple[float, dict[str, str]]] = []
    for row in thruster_rows:
        wall = finite_float(row.get("wall_mono_s"))
        if wall is not None:
            row_wall.append((wall, row))

    for phase_name, (start, end) in windows.items():
        part = [row for wall, row in row_wall if start <= wall <= end]
        if not part:
            continue
        metrics: dict[str, float] = {"thruster_samples": float(len(part))}
        for col in interesting:
            vals = [finite_float(row.get(col)) for row in part]
            finite = [v for v in vals if v is not None]
            if not finite:
                continue
            metrics[f"{col}_mean"] = float(mean(finite) or 0.0)
            metrics[f"{col}_rms"] = float(rms(finite) or 0.0)
            metrics[f"{col}_peak_abs"] = float(max_abs(finite) or 0.0)
        pwm_deltas: list[float] = []
        for col in pwm_columns:
            pwm_deltas.extend(abs(v - 1500.0) for v in (finite_float(row.get(col)) for row in part) if v is not None)
        if pwm_deltas:
            metrics["thruster_pwm_max_delta"] = float(max(pwm_deltas))
            metrics["thruster_pwm_mean_abs_delta"] = float(mean(pwm_deltas) or 0.0)
        by_phase[phase_name] = metrics
    return by_phase


def build_fingerprint(payload: dict[str, Any], thruster_csv: Path | None = None) -> dict[str, Any]:
    thruster_rows, fieldnames = load_thruster_csv(thruster_csv)
    thruster_by_phase = summarize_thrusters(payload, thruster_rows, fieldnames)

    phases: dict[str, dict[str, Any]] = {}
    phase_meta = {str(item.get("name")): item for item in payload.get("phases", [])}
    for row in payload.get("summary", []):
        name = str(row.get("phase", ""))
        if not name:
            continue
        metrics: dict[str, Any] = {
            "axis": str(row.get("axis", "")),
            "command": finite_float(row.get("command")),
        }
        for key in RESPONSE_METRICS:
            value = finite_float(row.get(key))
            if value is not None:
                metrics[key] = value
        if name in phase_meta:
            start = finite_float(phase_meta[name].get("start"))
            end = finite_float(phase_meta[name].get("end"))
            if start is not None and end is not None:
                metrics["duration_s"] = end - start
        metrics.update(thruster_by_phase.get(name, {}))
        phases[name] = metrics

    metadata = payload.get("metadata", {})
    meta_keys = (
        "mode",
        "input_mode",
        "command",
        "axis_s",
        "neutral_s",
        "baseline_s",
        "sample_hz",
        "axes",
        "release_initial_depth_hold",
        "post_release_neutral_s",
        "rc_neutral",
        "rc_span",
    )
    return {
        "metadata": {key: metadata.get(key) for key in meta_keys if key in metadata},
        "sample_count": payload.get("sample_count"),
        "health": payload.get("health", {}),
        "phases": phases,
        "has_thruster_debug": bool(thruster_by_phase),
    }


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


def compare_fingerprints(
    candidate: dict[str, Any],
    baseline: dict[str, Any],
    args: argparse.Namespace,
) -> dict[str, Any]:
    failures: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    comparisons: list[dict[str, Any]] = []

    cand_health = str(candidate.get("health", {}).get("overall", "unknown"))
    if cand_health == "fail":
        failures.append({"kind": "candidate_health", "candidate": cand_health})
    elif cand_health == "warn":
        warnings.append({"kind": "candidate_health", "candidate": cand_health})

    cand_phases = candidate.get("phases", {})
    base_phases = baseline.get("phases", {})
    for phase_name, base_phase in base_phases.items():
        cand_phase = cand_phases.get(phase_name)
        if cand_phase is None:
            failures.append({"kind": "missing_phase", "phase": phase_name})
            continue
        metric_names = sorted(
            key
            for key in set(base_phase) & set(cand_phase)
            if key not in {"axis"} and isinstance(base_phase.get(key), (int, float)) and isinstance(cand_phase.get(key), (int, float))
        )
        for metric in metric_names:
            a = float(cand_phase[metric])
            b = float(base_phase[metric])
            abs_diff = abs(a - b)
            abs_tol, rel_tol = metric_tolerance(metric, a, b, args)
            allowed = max(abs_tol, rel_tol * max(abs(a), abs(b), 1.0))
            item = {
                "phase": phase_name,
                "metric": metric,
                "candidate": a,
                "baseline": b,
                "abs_diff": abs_diff,
                "allowed": allowed,
            }
            if abs_diff > allowed:
                if metric.startswith("expected_metric") or metric.startswith("rcout_") or metric.startswith("thr_"):
                    failures.append(item)
                else:
                    warnings.append(item)
            comparisons.append(item)

    overall = "fail" if failures else ("warn" if warnings else "pass")
    return {
        "overall": overall,
        "failures": failures,
        "warnings": warnings,
        "comparison_count": len(comparisons),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--thruster-csv", type=Path)
    parser.add_argument("--baseline-thruster-csv", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--abs-tol", type=float, default=0.02)
    parser.add_argument("--relative-tol", type=float, default=0.15)
    parser.add_argument("--pwm-tol", type=float, default=8.0)
    parser.add_argument("--force-abs-tol", type=float, default=2.0)
    parser.add_argument("--force-rel-tol", type=float, default=0.20)
    parser.add_argument("--torque-abs-tol", type=float, default=0.25)
    parser.add_argument("--torque-rel-tol", type=float, default=0.20)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    candidate_payload = load_json(args.candidate)
    candidate = build_fingerprint(candidate_payload, args.thruster_csv)

    result: dict[str, Any] = {
        "candidate": str(args.candidate),
        "thruster_csv": str(args.thruster_csv) if args.thruster_csv else None,
        "fingerprint": candidate,
    }

    if args.baseline:
        baseline_payload = load_json(args.baseline)
        baseline = build_fingerprint(baseline_payload, args.baseline_thruster_csv)
        result["baseline"] = str(args.baseline)
        result["comparison"] = compare_fingerprints(candidate, baseline, args)
        overall = str(result["comparison"]["overall"])
    else:
        health = str(candidate.get("health", {}).get("overall", "unknown"))
        overall = "fail" if health == "fail" else ("warn" if health == "warn" else "pass")
        result["comparison"] = {
            "overall": overall,
            "failures": [] if overall != "fail" else [{"kind": "candidate_health", "candidate": health}],
            "warnings": [] if overall != "warn" else [{"kind": "candidate_health", "candidate": health}],
            "comparison_count": 0,
        }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")

    phase_count = len(candidate.get("phases", {}))
    has_thr = "yes" if candidate.get("has_thruster_debug") else "no"
    print(f"[control-loop-compare] out={args.out}")
    print(f"[control-loop-compare] phases={phase_count} thruster_debug={has_thr} overall={overall}")
    failures = result["comparison"].get("failures", [])
    for item in failures[:12]:
        print(f"[control-loop-compare] FAIL {item}")
    warnings = result["comparison"].get("warnings", [])
    for item in warnings[:8]:
        print(f"[control-loop-compare] WARN {item}")
    return 1 if overall == "fail" else 0


if __name__ == "__main__":
    raise SystemExit(main())
