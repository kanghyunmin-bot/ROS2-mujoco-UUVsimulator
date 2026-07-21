#!/usr/bin/env python3
"""Record ALT_HOLD closed-loop signals on one timeline.

This tool is intentionally read-only. It does not publish commands, change
parameters, or touch ArduPilot. It records the signals needed to separate:

* GUI MANUAL_CONTROL authority
* RC override authority
* Bar30/depth and vertical velocity sign
* ArduSub servo output
* MuJoCo plant drift
"""

from __future__ import annotations

import argparse
from pathlib import Path

from althold_diagnostics_node import AltHoldDiagnosticsLogger


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=20.0, help="Recording duration in seconds.")
    parser.add_argument("--sample-hz", type=float, default=20.0, help="CSV sampling rate.")
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parents[1] / "logs" / "diagnostics"),
        help="Directory for CSV/plot outputs.",
    )
    parser.add_argument("--no-plot", dest="plot", action="store_false", help="Skip PNG plot generation.")
    parser.set_defaults(plot=True)
    return parser.parse_args()


def main() -> None:
    logger = AltHoldDiagnosticsLogger(parse_args())
    try:
        while logger.rclpy.ok() and not logger.done:
            logger.rclpy.spin_once(logger.node, timeout_sec=0.1)
    except KeyboardInterrupt:
        logger._finish()
    finally:
        if not logger.done:
            logger._finish()
        try:
            logger.node.destroy_node()
        except Exception:
            pass
        try:
            logger.rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
