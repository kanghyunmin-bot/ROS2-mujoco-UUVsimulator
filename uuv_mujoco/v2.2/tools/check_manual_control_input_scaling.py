#!/usr/bin/env python3
"""Contract check for ROS ManualControl normalized/raw axis scaling."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_manual_control_input import (  # noqa: E402
    _manual_control_axis_to_norm,
    _manual_control_heave_to_norm,
)


def _assert_close(actual: float, expected: float, label: str) -> None:
    if abs(float(actual) - float(expected)) > 1.0e-9:
        raise AssertionError(f"{label}: expected {expected:.6f}, got {actual:.6f}")


def main() -> int:
    _assert_close(_manual_control_axis_to_norm(0.0), 0.0, "normalized x neutral")
    _assert_close(_manual_control_axis_to_norm(0.3), 0.3, "normalized x positive")
    _assert_close(_manual_control_axis_to_norm(-0.3), -0.3, "normalized x negative")
    _assert_close(_manual_control_axis_to_norm(1000.0), 1.0, "raw x positive")
    _assert_close(_manual_control_axis_to_norm(-1000.0), -1.0, "raw x negative")

    _assert_close(_manual_control_heave_to_norm(0.0), 0.0, "normalized z neutral")
    _assert_close(_manual_control_heave_to_norm(0.3), 0.3, "normalized z positive")
    _assert_close(_manual_control_heave_to_norm(-0.3), -0.3, "normalized z negative")
    _assert_close(_manual_control_heave_to_norm(500.0), 0.0, "raw z neutral")
    _assert_close(_manual_control_heave_to_norm(650.0), 0.3, "raw z positive")
    _assert_close(_manual_control_heave_to_norm(350.0), -0.3, "raw z negative")

    print("manual_control_input_scaling=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
