#!/usr/bin/env python3
"""Regression checks for MuJoCo ENU/FLU to ArduPilot NED/FRD frames."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_bridge_config_frames import configure_dvl_and_frame_transforms  # noqa: E402
from bridge.ros2_state_setpoint_math import current_yaw_ned  # noqa: E402


class _Bridge:
    @staticmethod
    def _env_to_clamped_float(_name: str, default: float, _lower: float, _upper: float) -> float:
        return default


def main() -> int:
    bridge = _Bridge()
    configure_dvl_and_frame_transforms(bridge)

    transform = bridge._enu_to_ned
    east_enu = np.array([1.0, 0.0, 0.0])
    north_enu = np.array([0.0, 1.0, 0.0])
    up_enu = np.array([0.0, 0.0, 1.0])
    assert np.allclose(transform @ east_enu, [0.0, 1.0, 0.0])
    assert np.allclose(transform @ north_enu, [1.0, 0.0, 0.0])
    assert np.allclose(transform @ up_enu, [0.0, 0.0, -1.0])
    assert np.allclose(transform @ transform.T, np.eye(3))
    assert np.isclose(np.linalg.det(transform), 1.0)

    # A FLU vehicle pointing along ENU +x points east, which is +90 degrees
    # in NED yaw.  The former NWU matrix incorrectly reported zero here.
    yaw_ned = current_yaw_ned(bridge, np.eye(3, dtype=np.float64))
    assert math.isclose(yaw_ned, math.pi / 2.0, abs_tol=1.0e-12), yaw_ned

    # The full world/body conversion must round-trip exactly.
    yaw_enu = math.radians(37.0)
    rot_enu_flu = np.array(
        [
            [math.cos(yaw_enu), -math.sin(yaw_enu), 0.0],
            [math.sin(yaw_enu), math.cos(yaw_enu), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    rot_ned_frd = transform @ rot_enu_flu @ bridge._bmj_to_frd.T
    recovered_enu_flu = transform.T @ rot_ned_frd @ bridge._bmj_to_frd
    assert np.allclose(recovered_enu_flu, rot_enu_flu, atol=1.0e-12)

    print("sitl_frame_contract=PASS yaw_ned_for_east_deg=90.000")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
