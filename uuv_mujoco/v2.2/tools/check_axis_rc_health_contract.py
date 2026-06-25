#!/usr/bin/env python3
"""Regression checks for GUI axis RC health-gate semantics."""

from __future__ import annotations

from axis_rc_health_metrics import build_health


def _roll_row(
    *,
    rcin_delta: float,
    rcout_delta: float,
    gyro_peak: float,
    response_after_rcin_delay_s: float = 0.1,
) -> dict[str, object]:
    return {
        "phase": "roll_pos",
        "axis": "roll",
        "samples": 40,
        "armed_fraction": 1.0,
        "rcin_max_delta": rcin_delta,
        "rcout_max_delta": rcout_delta,
        "expected_metric_peak_abs": gyro_peak,
        "response_after_rcin_delay_s": response_after_rcin_delay_s,
        "gyro_z_tail_peak_abs": 0.0,
        "dvl_vz_tail_mean": 0.0,
    }


def _health_for(row: dict[str, object]) -> dict[str, object]:
    return build_health(
        [row],
        input_mode="rc-override",
        sample_hz=25.0,
        axis_s=2.0,
        neutral_s=3.0,
        baseline_s=2.0,
    )


def _single_check(health: dict[str, object]) -> dict[str, object]:
    checks = list(health["checks"])
    assert len(checks) == 1, checks
    return checks[0]


def _neutral_row(
    *,
    phase: str,
    rcin_tail_delta: float,
    rcout_tail_delta: float,
    rcout_tail_mean_delta: float,
    rcout_vertical_common_delta: float = 0.0,
    rcout_vertical_diff_delta: float = 0.0,
    rcout_horizontal_diff_delta: float = 0.0,
) -> dict[str, object]:
    return {
        "phase": phase,
        "axis": "neutral",
        "samples": 40,
        "armed_fraction": 1.0,
        "rcin_max_delta": rcin_tail_delta,
        "rcin_tail_max_delta": rcin_tail_delta,
        "rcout_max_delta": rcout_tail_delta,
        "rcout_tail_max_delta": rcout_tail_delta,
        "rcout_tail_mean_abs_delta": rcout_tail_mean_delta,
        "rcout_tail_vertical_common_delta": rcout_vertical_common_delta,
        "rcout_tail_vertical_diff_max_abs": rcout_vertical_diff_delta,
        "rcout_tail_horizontal_diff_max_abs": rcout_horizontal_diff_delta,
        "gyro_z_tail_peak_abs": 0.0,
        "dvl_vz_tail_mean": 0.0,
    }


def _health_for_rows(rows: list[dict[str, object]], *, input_mode: str = "rc-override") -> dict[str, object]:
    return build_health(
        rows,
        input_mode=input_mode,
        sample_hz=25.0,
        axis_s=2.0,
        neutral_s=3.0,
        baseline_s=2.0,
    )


def main() -> int:
    delayed_rcout = _single_check(_health_for(_roll_row(rcin_delta=75.0, rcout_delta=0.0, gyro_peak=0.05)))
    assert delayed_rcout["severity"] == "warn", delayed_rcout
    assert delayed_rcout["flags"] == ["rcout_telemetry_not_observed_in_phase"], delayed_rcout

    missing_response = _single_check(_health_for(_roll_row(rcin_delta=75.0, rcout_delta=0.0, gyro_peak=0.0)))
    assert missing_response["severity"] == "fail", missing_response
    assert "rcout_not_moving" in missing_response["flags"], missing_response

    moving_rcout = _single_check(_health_for(_roll_row(rcin_delta=75.0, rcout_delta=75.0, gyro_peak=0.05)))
    assert moving_rcout["severity"] == "ok", moving_rcout
    assert moving_rcout["flags"] == [], moving_rcout

    slow_response = _single_check(
        _health_for(_roll_row(rcin_delta=75.0, rcout_delta=75.0, gyro_peak=0.05, response_after_rcin_delay_s=5.0))
    )
    assert slow_response["severity"] == "warn", slow_response
    assert slow_response["flags"] == ["slow_primary_axis_response"], slow_response

    neutral_ok = _health_for_rows(
        [
            _neutral_row(
                phase="neutral_hold",
                rcin_tail_delta=0.0,
                rcout_tail_delta=3.0,
                rcout_tail_mean_delta=1.0,
            )
        ]
    )
    assert neutral_ok["overall"] == "pass", neutral_ok

    neutral_saturated = _health_for_rows(
        [
            _neutral_row(
                phase="baseline_neutral",
                rcin_tail_delta=0.0,
                rcout_tail_delta=400.0,
                rcout_tail_mean_delta=117.0,
            ),
            _neutral_row(
                phase="neutral_hold",
                rcin_tail_delta=0.0,
                rcout_tail_delta=400.0,
                rcout_tail_mean_delta=164.0,
            ),
        ]
    )
    assert neutral_saturated["overall"] == "fail", neutral_saturated
    aggregate = neutral_saturated["checks"][-1]
    assert aggregate["phase"] == "neutral_only_rcout_contract", aggregate
    assert aggregate["flags"] == ["neutral_only_rcout_saturated"], aggregate

    neutral_heave_common = _health_for_rows(
        [
            _neutral_row(
                phase="neutral_hold",
                rcin_tail_delta=0.0,
                rcout_tail_delta=60.0,
                rcout_tail_mean_delta=20.0,
                rcout_vertical_common_delta=60.0,
            )
        ]
    )
    assert neutral_heave_common["overall"] == "warn", neutral_heave_common
    aggregate = neutral_heave_common["checks"][-1]
    assert aggregate["phase"] == "neutral_only_rcout_contract", aggregate
    assert aggregate["flags"] == ["neutral_only_heave_common_active"], aggregate

    neutral_attitude_differential = _health_for_rows(
        [
            _neutral_row(
                phase="neutral_hold",
                rcin_tail_delta=0.0,
                rcout_tail_delta=60.0,
                rcout_tail_mean_delta=20.0,
                rcout_vertical_common_delta=0.0,
                rcout_vertical_diff_delta=60.0,
            )
        ]
    )
    assert neutral_attitude_differential["overall"] == "warn", neutral_attitude_differential
    aggregate = neutral_attitude_differential["checks"][-1]
    assert aggregate["phase"] == "neutral_only_rcout_contract", aggregate
    assert aggregate["flags"] == ["neutral_only_rcout_attitude_differential"], aggregate

    manual_control_stale_rcin = _health_for_rows(
        [
            _neutral_row(
                phase="neutral_hold",
                rcin_tail_delta=75.0,
                rcout_tail_delta=3.0,
                rcout_tail_mean_delta=1.0,
            )
        ],
        input_mode="manual-control",
    )
    assert manual_control_stale_rcin["overall"] == "pass", manual_control_stale_rcin

    print("axis_rc_health_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
