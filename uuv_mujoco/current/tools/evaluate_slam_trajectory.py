#!/usr/bin/env python3
"""Evaluate an estimated TUM trajectory against evaluation-only ground truth."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from sim.evaluation.trajectory_metrics import (  # noqa: E402
    evaluate_trajectory,
    load_tum_trajectory,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--estimate", type=Path, required=True)
    parser.add_argument("--ground-truth", type=Path, required=True)
    parser.add_argument("--alignment", choices=("none", "se3", "sim3"), default="se3")
    parser.add_argument("--estimate-time-offset-s", type=float, default=0.0)
    parser.add_argument("--max-interpolation-gap-s", type=float, default=0.1)
    parser.add_argument("--rpe-delta-s", type=float, default=1.0)
    parser.add_argument("--rpe-tolerance-s", type=float, default=0.05)
    parser.add_argument("--json-out", type=Path)
    parser.add_argument("--markdown-out", type=Path)
    return parser.parse_args()


def render_markdown(metrics: dict[str, object]) -> str:
    """Render a compact, durable trajectory-evaluation report."""

    ate_translation = metrics["ate_translation_m"]
    ate_rotation = metrics["ate_rotation_deg"]
    rpe_translation = metrics["rpe_translation_m"]
    rpe_rotation = metrics["rpe_rotation_deg"]
    rows = (
        ("ATE translation", "m", ate_translation),
        ("ATE rotation", "deg", ate_rotation),
        ("RPE translation", "m", rpe_translation),
        ("RPE rotation", "deg", rpe_rotation),
    )
    lines = [
        "# SLAM Trajectory Evaluation",
        "",
        f"- alignment: `{metrics['alignment']}`",
        f"- alignment scale: `{metrics['alignment_scale']:.9g}`",
        f"- estimate clock correction: `{metrics['estimate_time_offset_s']:.9g} s`",
        f"- matched samples: `{metrics['matched_sample_count']}` "
        f"(`{100.0 * metrics['match_ratio']:.2f}%`)",
        f"- RPE pair separation: `{metrics['rpe_delta_requested_s']:.9g} s`",
        "",
        "| Metric | Unit | Count | RMSE | Median | P95 | Max |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: |",
    ]
    for label, unit, summary in rows:
        lines.append(
            f"| {label} | {unit} | {summary['count']} | "
            f"{_format_metric(summary['rmse'])} | "
            f"{_format_metric(summary['median'])} | "
            f"{_format_metric(summary['p95'])} | "
            f"{_format_metric(summary['max'])} |"
        )
    lines.extend(
        (
            "",
            "Ground truth is evaluation-only and must not be connected to the SLAM, "
            "state-estimation, MAVROS, or controller graph.",
        )
    )
    return "\n".join(lines) + "\n"


def _format_metric(value: float | None) -> str:
    return "N/A" if value is None else f"{value:.9g}"


def main() -> int:
    args = parse_args()
    metrics = evaluate_trajectory(
        load_tum_trajectory(args.estimate),
        load_tum_trajectory(args.ground_truth),
        alignment=args.alignment,
        estimate_time_offset_s=args.estimate_time_offset_s,
        max_interpolation_gap_s=args.max_interpolation_gap_s,
        rpe_delta_s=args.rpe_delta_s,
        rpe_tolerance_s=args.rpe_tolerance_s,
    )
    encoded = json.dumps(metrics, indent=2, sort_keys=True) + "\n"
    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        args.json_out.write_text(encoded, encoding="utf-8")
    if args.markdown_out is not None:
        args.markdown_out.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_out.write_text(render_markdown(metrics), encoding="utf-8")
    print(encoded, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
