#!/usr/bin/env python3
"""Regression checks against held-out April 2 pressure observations."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
sys.path.insert(0, str(CURRENT / "tools"))
from fit_bag_pressure_profile import model_scatter
from bridge.ros2_imu_bar30_sensor_runtime import (
    configure_imu_bar30_sensor_runtime,
    advance_imu_bar30_sensor_runtime,
)
from test_imu_bar30_sensor_runtime import (
    FakeBridge,
    base_state,
    imu_state,
    vertical_state,
)

TUNED = CURRENT / "config/sensor_models/imu_bar30_bag_20260402.json"
BASE = CURRENT / "config/sensor_models/imu_bar30_uncalibrated_prior.json"


class BagPressureProfileTest(unittest.TestCase):
    def test_observation_scatter_matches_held_out_window(self) -> None:
        # Override permits an explicit before-fix run without changing source files.
        path = Path(os.environ.get("BAG0402_TEST_PROFILE", TUNED))
        data = json.loads(path.read_text())
        data["bar30"]["model"]["nominal_rate_hz"] = 2.0
        simulated = float(np.median(model_scatter(data, 50)))
        observed = 24.255045104847902  # held-out 115-140 s, 50 pressure messages
        self.assertLess(abs(simulated - observed) / observed, 0.20)

    def test_runtime_uses_two_hz_pressure_and_preserves_fcu_capture(self) -> None:
        with patch.dict(
            os.environ,
            {"ROS2_UUV_IMU_BAR30_SENSOR_CONFIG_PATH": str(TUNED)},
            clear=True,
        ):
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
        self.assertEqual(bridge._imu_sensor_model_config.nominal_rate_hz, 400.0)
        self.assertEqual(bridge._bar30_sensor_model_config.nominal_rate_hz, 2.0)
        stamps = []
        for tick in range(2001):
            result = advance_imu_bar30_sensor_runtime(
                bridge, base_state(tick * 0.0025), imu_state(), vertical_state()
            )
            stamps.extend(x.sample.sample_time_s for x in result[3])
        self.assertGreaterEqual(len(stamps), 9)
        np.testing.assert_allclose(np.diff(stamps), 0.5, atol=1e-10)

    def test_only_supported_model_parameters_change(self) -> None:
        baseline = json.loads(BASE.read_text())
        tuned = json.loads(TUNED.read_text())
        self.assertEqual(baseline["imu"], tuned["imu"])
        for key, value in baseline["bar30"]["model"].items():
            if key not in ["white_noise_std_pa", "nominal_rate_hz", "basis"]:
                self.assertEqual(value, tuned["bar30"]["model"][key], key)
        self.assertEqual(tuned["calibration_status"], "unvalidated_prior")


if __name__ == "__main__":
    unittest.main()
