#!/usr/bin/env python3
"""Smoke checks for axis RC onset-latency metrics."""

from __future__ import annotations

import math

from axis_rc_contract import Phase
from axis_rc_summary_metrics import summarize


def _sample(t: float, *, rcin4: int, gyro_z: float) -> dict[str, object]:
    sample: dict[str, object] = {
        "t": t,
        "phase": "yaw_pos",
        "mode": "MANUAL",
        "armed": True,
        "manual_input": True,
        "gyro_z": gyro_z,
        "gyro_x": 0.0,
        "gyro_y": 0.0,
        "dvl_vx": 0.0,
        "dvl_vy": 0.0,
        "dvl_vz": 0.0,
        "odom_vx": 0.0,
        "odom_vy": 0.0,
        "odom_vz": 0.0,
        "roll_rad": 0.0,
        "pitch_rad": 0.0,
        "yaw_rad": 0.0,
        "depth_m": 0.0,
    }
    for idx in range(1, 9):
        sample[f"rcin{idx}"] = 1500
        sample[f"rcout{idx}"] = 1500
    sample["rcin4"] = int(rcin4)
    return sample


def _assert_close(actual: float, expected: float, label: str) -> None:
    if abs(float(actual) - float(expected)) > 1e-9:
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def main() -> int:
    rows = summarize(
        [
            _sample(10.00, rcin4=1500, gyro_z=0.00),
            _sample(10.05, rcin4=1650, gyro_z=0.01),
            _sample(10.15, rcin4=1650, gyro_z=0.20),
        ],
        [Phase("yaw_pos", "yaw", 0.5, 10.0, 11.0)],
    )
    row = rows[0]
    _assert_close(float(row["rcin_onset_delay_s"]), 0.05, "RCIN onset delay")
    _assert_close(float(row["expected_metric_onset_delay_s"]), 0.15, "sensor onset delay")
    _assert_close(float(row["response_after_rcin_delay_s"]), 0.10, "response-after-RCIN delay")

    heave_samples = []
    for t, rcin3, odom_vz in ((20.00, 1500, 0.0), (20.05, 1650, 0.0), (20.20, 1650, 0.03)):
        sample = _sample(t, rcin4=1500, gyro_z=0.0)
        sample["phase"] = "heave_pos"
        sample["rcin3"] = rcin3
        sample["dvl_vz"] = float("nan")
        sample["odom_vz"] = odom_vz
        if t == 20.20:
            sample["rcout5"] = 1540
            sample["rcout6"] = 1460
            sample["rcout7"] = 1460
            sample["rcout8"] = 1540
        heave_samples.append(sample)
    heave_row = summarize(
        heave_samples,
        [Phase("heave_pos", "heave", 0.5, 20.0, 21.0)],
    )[0]
    assert heave_row["expected_metric"] == "odom_vz", heave_row
    assert math.isclose(float(heave_row["expected_metric_peak_abs"]), 0.03), heave_row
    _assert_close(float(heave_row["expected_metric_onset_delay_s"]), 0.20, "odom fallback onset")
    _assert_close(float(heave_row["rcout_tail_vertical_common_delta"]), 40.0, "signed heave RCOUT")
    _assert_close(float(heave_row["rcout_tail_vertical_diff_max_abs"]), 0.0, "pure heave differential")
    print("axis_rc_latency_metrics=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
