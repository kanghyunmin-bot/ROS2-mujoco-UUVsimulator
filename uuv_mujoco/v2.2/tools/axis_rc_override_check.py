#!/usr/bin/env python3
"""Axis-by-axis RC override validation for the MuJoCo + ArduSub SITL stack."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from axis_rc_cli_args import LOG_ROOT, parse_args
from axis_rc_metrics import build_health, summarize
from axis_rc_report import build_metadata, print_summary
from axis_rc_sequence import (
    althold_heave_is_inverted,
    append_neutral_phase,
    prepare_vehicle,
    run_axis_sequence,
)


def main() -> int:
    args = parse_args()

    import rclpy

    from axis_rc_node import AxisRcOverrideCheck
    from axis_rc_plotting import write_outputs

    out_dir = args.out_dir or LOG_ROOT / f"axis_rc_override_check_{time.strftime('%Y%m%d_%H%M%S')}"
    invert_heave_rc = althold_heave_is_inverted(args.mode)
    rclpy.init()
    node = AxisRcOverrideCheck(
        sample_hz=args.sample_hz,
        input_mode=args.input_mode,
        invert_heave_rc=invert_heave_rc,
    )
    try:
        prepare_vehicle(node, args)
        node.recording = True
        append_neutral_phase(node, args, "baseline_neutral", args.baseline_s)
        run_axis_sequence(node, args)
        node.recording = False
        node.spin_with_rc(1.0, hz=args.publish_hz, input_mode=args.input_mode)
        if args.disarm_at_end:
            node.call_arm(False, timeout=args.wait_timeout)
        node.release_rc()

        summary = summarize(node.samples, node.phases)
        metadata = build_metadata(node, args, invert_heave_rc)
        write_outputs(out_dir, node.samples, node.phases, summary, metadata)
        health = build_health(
            summary,
            input_mode=args.input_mode,
            sample_hz=args.sample_hz,
            axis_s=args.axis_s,
            neutral_s=args.neutral_s,
            baseline_s=args.baseline_s,
        )
        print_summary(out_dir, node, summary, health)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
