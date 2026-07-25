"""Metadata and console reporting for axis RC override checks."""

from __future__ import annotations

import argparse
from pathlib import Path

from axis_rc_contract import RC_NEUTRAL, RC_SPAN


def build_metadata(node, args: argparse.Namespace, invert_heave_rc: bool) -> dict[str, object]:
    return {
        "mode": args.mode,
        "input_mode": args.input_mode,
        "command": args.command,
        "axis_s": args.axis_s,
        "neutral_s": args.neutral_s,
        "baseline_s": args.baseline_s,
        "pulse_on_s": args.pulse_on_s,
        "pulse_off_s": args.pulse_off_s,
        "pre_arm_settle_s": args.pre_arm_settle_s,
        "post_arm_settle_s": args.post_arm_settle_s,
        "post_mode_settle_s": args.post_mode_settle_s,
        "pre_dive_s": args.pre_dive_s,
        "pre_dive_command": args.pre_dive_command,
        "pre_settle_s": args.pre_settle_s,
        "release_initial_depth_hold": bool(args.release_initial_depth_hold),
        "post_release_neutral_s": args.post_release_neutral_s,
        "neutral_only": bool(args.neutral_only),
        "sample_hz": args.sample_hz,
        "publish_hz": args.publish_hz,
        "axes": args.axes,
        "rc_neutral": RC_NEUTRAL,
        "rc_span": RC_SPAN,
        "invert_heave_rc": bool(invert_heave_rc),
        "node_start_wall_mono_s": node.start_wall,
    }


def print_summary(out_dir: Path, node, summary: list[dict[str, object]], health: dict[str, object]) -> None:
    print(f"[axis-check] out={out_dir}")
    print(f"[axis-check] samples={len(node.samples)} phases={len(node.phases)}")
    print(f"[axis-check] health={health['overall']}")
    for check in health.get("checks", []):
        if check.get("severity") == "ok":
            continue
        flags = ",".join(str(flag) for flag in check.get("flags", [])) or "none"
        detail = (
            f"[axis-check] health_{check['severity']} "
            f"{check.get('phase', 'unknown')}/{check.get('axis', 'unknown')} "
            f"flags={flags}"
        )
        if "max_rcout_tail_delta" in check:
            detail += f" max_rcout_tail_delta={float(check['max_rcout_tail_delta']):.1f}"
        if "max_rcout_tail_mean_abs_delta" in check:
            detail += (
                " max_rcout_tail_mean_abs_delta="
                f"{float(check['max_rcout_tail_mean_abs_delta']):.1f}"
            )
        print(detail)
    for row in summary:
        if row.get("axis") == "neutral":
            continue
        response_metric = str(row.get("expected_metric", "response"))
        response_mean = float(row.get("expected_metric_mean", float("nan")))
        response_peak = float(row.get("expected_metric_peak_abs", float("nan")))
        print(
            "[axis-check] "
            f"{row['phase']}: gyro_peak=({row['gyro_x_peak_abs']:.3f},"
            f"{row['gyro_y_peak_abs']:.3f},{row['gyro_z_peak_abs']:.3f}) "
            f"{response_metric}_mean={response_mean:+.3f} peak={response_peak:.3f} "
            f"depth_span={row['depth_m_span']:.3f} "
            f"rcin_delay={float(row.get('rcin_onset_delay_s', float('nan'))):.3f}s "
            f"resp_after_rcin={float(row.get('response_after_rcin_delay_s', float('nan'))):.3f}s"
        )


__all__ = ["build_metadata", "print_summary"]
