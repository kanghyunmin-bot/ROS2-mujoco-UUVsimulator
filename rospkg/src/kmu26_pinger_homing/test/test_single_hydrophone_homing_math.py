#!/usr/bin/env python3
"""Deterministic regression for single-hydrophone range-difference fitting."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "pinger_homing"))

from single_hydrophone_homing_math import (  # noqa: E402
    derive_vehicle_depth_limit,
    estimate_source_from_range_differences,
    filter_unit_vector,
    limit_heave_by_vehicle_depth,
    select_auto_probe_heave,
    stabilized_yaw_command,
    update_range_success_timer,
)


def main() -> int:
    _check_yaw_stabilization()
    _check_depth_safety()
    _check_tank_depth_only_auto_z()
    _check_competition_tank_bounded_auto_z()
    _check_range_success_timer()
    source = np.array([9.0, -4.0, -7.5], dtype=np.float64)
    path = []
    for value in np.linspace(0.0, 1.8, 45):
        path.append([value, 0.0, 0.0])
    for value in np.linspace(0.0, 1.4, 40):
        path.append([1.8, value, 0.0])
    for value in np.linspace(0.0, -1.2, 40):
        path.append([1.8, 1.4, value])
    positions = np.asarray(path, dtype=np.float64)
    times = np.arange(len(positions), dtype=np.float64) * 0.05
    ranges = np.linalg.norm(source[None, :] - positions, axis=1)
    bias = 0.012
    changes = ranges - ranges[0] + bias * times
    rng = np.random.default_rng(2607)
    changes += rng.normal(0.0, 0.0015, size=len(changes))
    changes[67] += 0.035
    absolute_ranges = ranges + rng.normal(0.0, 0.12, size=len(ranges))
    absolute_ranges[42] += 1.5
    estimate = estimate_source_from_range_differences(
        positions,
        changes,
        times,
        absolute_ranges_m=absolute_ranges,
    )
    if estimate is None:
        raise AssertionError("source fit returned no estimate")
    position_error = float(np.linalg.norm(estimate.source_world - source))
    if position_error > 0.70:
        raise AssertionError(f"source position error too large: {position_error:.3f}m")
    if abs(estimate.bias_range_rate_mps - bias) > 0.02:
        raise AssertionError(
            f"range-rate bias error too large: {estimate.bias_range_rate_mps - bias:+.4f}m/s"
        )
    print(
        "single_hydrophone_homing_math=PASS "
        f"position_error={position_error:.3f}m rms={estimate.rms_residual_m:.4f}m "
        f"bias={estimate.bias_range_rate_mps:+.4f}m/s"
    )
    return 0


def _check_depth_safety() -> None:
    disabled = limit_heave_by_vehicle_depth(
        -0.18,
        vehicle_z_world=-5.0,
        max_vehicle_depth_m=0.0,
        soft_margin_m=0.15,
        recovery_heave=0.12,
    )
    if disabled.command_heave != -0.18 or disabled.limit_active:
        raise AssertionError(f"disabled depth limit changed command: {disabled}")

    clear = limit_heave_by_vehicle_depth(
        -0.18,
        vehicle_z_world=-0.80,
        max_vehicle_depth_m=1.0,
        soft_margin_m=0.15,
        recovery_heave=0.12,
    )
    if clear.command_heave != -0.18 or clear.limit_active:
        raise AssertionError(f"depth limit activated outside soft margin: {clear}")

    soft = limit_heave_by_vehicle_depth(
        -0.18,
        vehicle_z_world=-0.925,
        max_vehicle_depth_m=1.0,
        soft_margin_m=0.15,
        recovery_heave=0.12,
    )
    if not math.isclose(soft.command_heave, -0.09, abs_tol=1.0e-12) or not soft.limit_active:
        raise AssertionError(f"soft depth attenuation failed: {soft}")

    hard = limit_heave_by_vehicle_depth(
        -0.18,
        vehicle_z_world=-1.0,
        max_vehicle_depth_m=1.0,
        soft_margin_m=0.15,
        recovery_heave=0.12,
    )
    if hard.command_heave != 0.12 or not hard.recovery_active:
        raise AssertionError(f"hard depth recovery failed: {hard}")

    missing_pose = limit_heave_by_vehicle_depth(
        -0.18,
        vehicle_z_world=None,
        max_vehicle_depth_m=1.0,
        soft_margin_m=0.15,
        recovery_heave=0.12,
    )
    if missing_pose.command_heave != 0.0 or not missing_pose.limit_active:
        raise AssertionError(f"missing-pose depth safety is not fail-closed: {missing_pose}")


def _check_tank_depth_only_auto_z() -> None:
    limit = derive_vehicle_depth_limit(1.32)
    if not math.isclose(limit, 1.05, abs_tol=1.0e-12):
        raise AssertionError(f"tank-only derived vehicle depth limit mismatch: {limit}")
    if select_auto_probe_heave(0.654, max_vehicle_depth_m=limit) != -0.10:
        raise AssertionError("shallow automatic Z probe must select downward excitation")
    if select_auto_probe_heave(0.90, max_vehicle_depth_m=limit) != 0.10:
        raise AssertionError("deep automatic Z probe must select upward excitation")

    source = np.array([1.38, -0.616, -1.08], dtype=np.float64)
    path = []
    for value in np.linspace(0.0, 1.0, 45):
        path.append([-1.80 + 0.80 * value, 0.0, -0.654])
    for value in np.linspace(0.0, 1.0, 40):
        path.append([-1.00, -0.55 * value, -0.654])
    for value in np.linspace(0.0, 1.0, 40):
        path.append([-1.00, -0.55, -0.654 - 0.22 * value])
    positions = np.asarray(path, dtype=np.float64)
    times = np.arange(len(positions), dtype=np.float64) * 0.05
    ranges = np.linalg.norm(source[None, :] - positions, axis=1)
    estimate = estimate_source_from_range_differences(
        positions,
        ranges - ranges[0],
        times,
        absolute_ranges_m=ranges,
        min_source_z_world=-1.32,
        max_source_z_world=-0.05,
    )
    if estimate is None:
        raise AssertionError("tank-only automatic 3D source fit returned no estimate")
    if abs(float(estimate.source_world[2] - source[2])) > 0.05:
        raise AssertionError(f"automatic pinger Z error is too large: {estimate.source_world}")


def _check_competition_tank_bounded_auto_z() -> None:
    source = np.array([-0.007, 8.929, -8.645], dtype=np.float64)
    start = np.array([-16.475, -13.673, -0.654], dtype=np.float64)
    path = []
    for value in np.linspace(0.0, 1.0, 80):
        path.append(start + np.array([0.50 * value, 0.0, 0.0]))
    for value in np.linspace(0.0, 1.0, 80):
        path.append(start + np.array([0.50, 0.30 * value, 0.0]))
    for value in np.linspace(0.0, 1.0, 80):
        path.append(start + np.array([0.50, 0.30, -0.30 * value]))
    positions = np.asarray(path, dtype=np.float64)
    times = np.arange(len(positions), dtype=np.float64) * 0.05
    ranges = np.linalg.norm(source[None, :] - positions, axis=1)
    wrong_phase_seed = start + ranges[0] * np.array([0.80, 0.14, -0.58])
    estimate = estimate_source_from_range_differences(
        positions,
        ranges - ranges[0],
        times,
        initial_source_world=wrong_phase_seed,
        absolute_ranges_m=ranges,
        min_source_z_world=-11.0,
        max_source_z_world=-0.05,
    )
    if estimate is None:
        raise AssertionError("competition-pool bounded source fit returned no estimate")
    if not -11.0 <= float(estimate.source_world[2]) <= -0.05:
        raise AssertionError(f"competition-pool source escaped Z bounds: {estimate.source_world}")


def _check_range_success_timer() -> None:
    started, complete = update_range_success_timer(
        1.00, threshold_m=1.05, hold_s=0.8, now_s=10.0, started_s=None
    )
    if started != 10.0 or complete:
        raise AssertionError("range success timer did not start")
    started, complete = update_range_success_timer(
        1.04, threshold_m=1.05, hold_s=0.8, now_s=10.79, started_s=started
    )
    if complete:
        raise AssertionError("range success timer completed early")
    started, complete = update_range_success_timer(
        1.02, threshold_m=1.05, hold_s=0.8, now_s=10.80, started_s=started
    )
    if not complete:
        raise AssertionError("range success timer did not complete after continuous hold")
    started, complete = update_range_success_timer(
        1.10, threshold_m=1.05, hold_s=0.8, now_s=10.81, started_s=started
    )
    if started is not None or complete:
        raise AssertionError("range success timer did not reset outside threshold")


def _check_yaw_stabilization() -> None:
    filtered = filter_unit_vector(
        np.array([1.0, 0.0, 0.0]),
        np.array([0.8, 0.6, 0.0]),
        alpha=0.2,
    )
    if not 0.0 < math.atan2(float(filtered[1]), float(filtered[0])) < math.atan2(0.6, 0.8):
        raise AssertionError("direction filter did not attenuate bearing step")
    command = stabilized_yaw_command(
        math.radians(35.0),
        0.0,
        0.0,
        dt_s=1.0 / 30.0,
        gain=0.85,
        rate_damping=0.18,
        deadband_rad=math.radians(2.5),
        command_limit=0.42,
        slew_rate_per_s=0.90,
    )
    if not math.isclose(command, 0.03, abs_tol=1.0e-9):
        raise AssertionError(f"yaw slew limit failed: {command}")
    damped = stabilized_yaw_command(
        math.radians(15.0),
        0.8,
        0.2,
        dt_s=1.0,
        gain=0.85,
        rate_damping=0.18,
        deadband_rad=math.radians(2.5),
        command_limit=0.42,
        slew_rate_per_s=1.0,
    )
    if not 0.0 < damped < 0.2:
        raise AssertionError(f"yaw-rate damping failed: {damped}")


if __name__ == "__main__":
    raise SystemExit(main())
