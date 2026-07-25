"""Command-line arguments for axis RC override checks."""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from axis_rc_contract import AXIS_ORDER


WORKSPACE = Path(__file__).resolve().parents[1]
LOG_ROOT = WORKSPACE / "logs"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--command", type=float, default=0.25)
    parser.add_argument("--axis-s", type=float, default=5.0)
    parser.add_argument("--neutral-s", type=float, default=3.0)
    parser.add_argument("--baseline-s", type=float, default=5.0)
    parser.add_argument(
        "--pulse-on-s",
        type=float,
        default=0.0,
        help=(
            "When positive, axis phases publish command pulses for this many seconds "
            "instead of holding one command for the entire phase."
        ),
    )
    parser.add_argument(
        "--pulse-off-s",
        type=float,
        default=0.0,
        help="Neutral interval between pulsed command bursts during long latency soak tests.",
    )
    parser.add_argument("--sample-hz", type=float, default=400.0)
    parser.add_argument(
        "--publish-hz",
        type=float,
        default=400.0,
        help="RC/manual command publication rate. Use 400 Hz for sub-10 ms latency checks.",
    )
    parser.add_argument("--mode", default="ALT_HOLD")
    parser.add_argument("--input-mode", choices=("rc-override", "manual-control", "both"), default="rc-override")
    parser.add_argument("--wait-timeout", type=float, default=90.0)
    parser.add_argument("--axes", nargs="+", choices=AXIS_ORDER, default=list(AXIS_ORDER))
    parser.add_argument("--pre-dive-s", type=float, default=0.0)
    parser.add_argument("--pre-dive-command", type=float, default=-0.35)
    parser.add_argument("--pre-settle-s", type=float, default=0.5)
    parser.add_argument(
        "--pre-arm-settle-s",
        type=float,
        default=0.0,
        help="Neutral RC settle time before arming; useful for EKF3 Bar30 depth convergence.",
    )
    parser.add_argument(
        "--post-arm-settle-s",
        type=float,
        default=float(os.environ.get("UUV_POST_ARM_SETTLE_S", "2.0")),
        help="Neutral RC settle time after arming and before mode change.",
    )
    parser.add_argument(
        "--post-mode-settle-s",
        type=float,
        default=float(os.environ.get("UUV_POST_MODE_SETTLE_S", "2.0")),
        help=(
            "Neutral RC settle time after the requested mode is confirmed and "
            "before releasing an initial-depth hold."
        ),
    )
    parser.add_argument("--switch-initial-depth-before-arm", action="store_true")
    parser.add_argument("--release-initial-depth-hold", action="store_true")
    parser.add_argument("--post-release-neutral-s", type=float, default=0.5)
    parser.add_argument(
        "--neutral-only",
        action="store_true",
        help="Record only the baseline neutral hold phase; useful for AltHold drift checks.",
    )
    parser.add_argument("--disarm-at-end", action="store_true")
    return parser.parse_args()


__all__ = ["LOG_ROOT", "WORKSPACE", "parse_args"]
