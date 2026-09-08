#!/usr/bin/env python3
"""Regression tests for flight-controller paths that must not bypass sensors."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
START_SCRIPT = ROOT / "start_ardusub_sitl_mj311.sh"
DOCKER_START_SCRIPT = ROOT / "start_docker_sitl_mujoco_mj311.sh"
DOCKER_COMPOSE = ROOT.parents[1] / "docker" / "ardusub" / "docker-compose.yml"
REAL_PARAMS = ROOT / "config" / "ardusub_realrobot_contract.param"


def _real_parameter(name: str) -> str:
    match = re.search(
        rf"(?m)^{re.escape(name)}\s+([^\s#]+)\s*$",
        REAL_PARAMS.read_text(encoding="utf-8"),
    )
    if match is None:
        raise AssertionError(f"missing real-vehicle parameter: {name}")
    return match.group(1)


class SitlNoBypassContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.start_text = START_SCRIPT.read_text(encoding="utf-8")
        cls.docker_start_text = DOCKER_START_SCRIPT.read_text(encoding="utf-8")
        cls.docker_compose_text = DOCKER_COMPOSE.read_text(encoding="utf-8")

    def test_arming_checks_match_real_vehicle_contract(self) -> None:
        real_value = _real_parameter("ARMING_CHECK")
        self.assertEqual(real_value, "194")
        self.assertIn(
            'append_param_if_not_overridden "ARMING_CHECK" '
            '"${SITL_ARMING_CHECK:-194}"',
            self.start_text,
        )
        self.assertNotIn(
            'append_param_if_not_overridden "ARMING_CHECK" '
            '"${SITL_ARMING_CHECK:-0}"',
            self.start_text,
        )

    def test_all_normal_estimator_paths_default_to_ekf3(self) -> None:
        self.assertEqual(_real_parameter("AHRS_EKF_TYPE"), "3")
        self.assertGreaterEqual(
            self.start_text.count(
                'append_param_if_not_overridden "AHRS_EKF_TYPE" '
                '"${SITL_AHRS_EKF_TYPE:-3}"'
            ),
            2,
        )
        self.assertNotIn("${SITL_AHRS_EKF_TYPE:-10}", self.start_text)
        self.assertIn(
            'export SITL_AHRS_EKF_TYPE="${SITL_AHRS_EKF_TYPE:-3}"',
            self.docker_start_text,
        )
        self.assertNotIn(
            "${SITL_AHRS_EKF_TYPE:-10}",
            self.docker_start_text,
        )
        self.assertIn(
            'export ROS2_UUV_SITL_JSON_TIMING_MODE="'
            '${ROS2_UUV_SITL_JSON_TIMING_MODE:-lockstep}"',
            self.docker_start_text,
        )
        self.assertIn(
            "SITL_AHRS_EKF_TYPE: ${SITL_AHRS_EKF_TYPE:-3}",
            self.docker_compose_text,
        )
        self.assertNotIn(
            "${SITL_AHRS_EKF_TYPE:-10}",
            self.docker_compose_text,
        )

    def test_hardware_watchdog_exception_is_narrow_and_documented(self) -> None:
        # BRD_OPTIONS bit 0 controls a physical/POSIX watchdog, not a modeled
        # vehicle sensor. Disabling that bit is a legitimate SITL adaptation;
        # it must not be coupled to disabling flight/sensor arming checks.
        self.assertEqual(_real_parameter("BRD_OPTIONS"), "1")
        self.assertIn(
            'append_param_if_not_overridden "BRD_OPTIONS" '
            '"${SITL_BRD_OPTIONS:-0}"',
            self.start_text,
        )
        self.assertIn("SIGALRM watchdog", self.start_text)

    def test_control_link_failsafes_match_real_vehicle_contract(self) -> None:
        expected = {
            "FS_GCS_ENABLE": "2",
            "FS_PILOT_INPUT": "2",
            "FS_PILOT_TIMEOUT": "3.000000",
        }
        for name, value in expected.items():
            self.assertEqual(_real_parameter(name), value)
        self.assertIn(
            'append_param_if_not_overridden "FS_GCS_ENABLE" '
            '"${SITL_FS_GCS_ENABLE:-2}"',
            self.start_text,
        )
        self.assertIn(
            'append_param_if_not_overridden "FS_PILOT_INPUT" '
            '"${SITL_FS_PILOT_INPUT:-2}"',
            self.start_text,
        )
        self.assertIn(
            'append_param_if_not_overridden "FS_PILOT_TIMEOUT" '
            '"${SITL_FS_PILOT_TIMEOUT:-3.0}"',
            self.start_text,
        )
        self.assertNotIn("${SITL_FS_GCS_ENABLE:-0}", self.start_text)
        self.assertNotIn("${SITL_FS_PILOT_INPUT:-0}", self.start_text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
