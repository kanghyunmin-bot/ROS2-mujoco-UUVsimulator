#!/usr/bin/env python3
"""Unit tests for the strict live-parity standalone harness."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT / "tools") not in sys.path:
    sys.path.insert(0, str(ROOT / "tools"))

from check_strict_live_parity import (  # noqa: E402
    RosLiveProbe,
    _cleanup_arm,
    _collect_parameter_evidence,
    build_arg_parser,
)
from strict_live_parity_contract import (  # noqa: E402
    PARAMETER_NAMES,
    assess_arm_event_sequence,
    assess_clock,
    assess_firmware,
    assess_neutral_heave,
    assess_parameters,
    assess_publisher_ownership,
    assess_topic_counts,
    decode_ros_numeric_parameters,
    parse_parameter_text,
    parse_sitl_log,
)


class StrictLiveParityContractTest(unittest.TestCase):
    def test_arm_cleanup_always_releases_then_disarms(self) -> None:
        class FakeApi:
            def __init__(self) -> None:
                self.calls: list[tuple[str, dict[str, object] | None]] = []

            def post_rc(self, payload: dict[str, object]) -> dict[str, object]:
                self.calls.append(("rc", payload))
                return {"ok": True}

            def post_command(
                self,
                payload: dict[str, object],
            ) -> dict[str, object]:
                self.calls.append(("command", payload))
                return {"ok": True}

            def get_status(self) -> dict[str, object]:
                return {"telemetry": {"armed": False}}

        api = FakeApi()
        result = _cleanup_arm(api, arm_report=None)
        self.assertTrue(result["passed"])
        self.assertEqual([kind for kind, _payload in api.calls], ["rc", "command"])
        self.assertEqual(api.calls[1][1], {"command": "arm", "value": False})

    def test_sitl_log_parser_prefers_actual_ap_version_and_last_params(self) -> None:
        parsed = parse_sitl_log(
            """
[start-sitl] parameter compatibility filter enabled for ArduSub V4.8.0-dev
  AHRS_EKF_TYPE 10
  RC3_TRIM 1500
  RC_OPTIONS 0
  ARMING_CHECK 0
  FS_GCS_ENABLE 0
  FS_PILOT_INPUT 0
  FS_PILOT_TIMEOUT 10
RiTW: Starting ArduSub : /workspace/ardupilot_sub_stable/build/sitl/bin/ardusub -w
AP: ArduSub V4.1.2 (abc123)
Saved 1234 parameters to mav.parm
  AHRS_EKF_TYPE 3
  RC3_TRIM 1100
  RC_OPTIONS 32
  ARMING_CHECK 194
  FS_GCS_ENABLE 2
  FS_PILOT_INPUT 2
  FS_PILOT_TIMEOUT 3
"""
        )
        self.assertEqual(parsed["firmware"], "ArduSub V4.1.2")
        self.assertEqual(
            parsed["ardusub_binary"],
            "/workspace/ardupilot_sub_stable/build/sitl/bin/ardusub",
        )
        self.assertEqual(
            parsed["parameters"],
            {
                "AHRS_EKF_TYPE": 3.0,
                "RC3_TRIM": 1100.0,
                "RC_OPTIONS": 32.0,
                "ARMING_CHECK": 194.0,
                "FS_GCS_ENABLE": 2.0,
                "FS_PILOT_INPUT": 2.0,
                "FS_PILOT_TIMEOUT": 3.0,
            },
        )
        self.assertTrue(parsed["parameter_snapshot_saved"])
        self.assertEqual(parsed["parameter_snapshot_count"], 1234)
        self.assertTrue(assess_firmware(parsed["firmware"])["passed"])

    def test_parameter_parser_and_bit5_verdict(self) -> None:
        values = parse_parameter_text(
            "AHRS_EKF_TYPE    3.000000\n"
            "RC3_TRIM         1100.000000\n"
            "RC_OPTIONS       160.000000\n"
            "ARMING_CHECK     194.000000\n"
            "FS_GCS_ENABLE    2.000000\n"
            "FS_PILOT_INPUT   2.000000\n"
            "FS_PILOT_TIMEOUT 3.000000\n"
        )
        result = assess_parameters(values, source="mock", source_fresh=True)
        self.assertTrue(result["passed"])
        self.assertTrue(result["checks"]["RC_OPTIONS_bit5"]["bit5_set"])
        self.assertTrue(result["checks"]["ARMING_CHECK"]["passed"])

        values["RC_OPTIONS"] = 128.0
        result = assess_parameters(values, source="mock", source_fresh=True)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["RC_OPTIONS_bit5"]["passed"])

        values["RC_OPTIONS"] = 32.0
        values["ARMING_CHECK"] = 0.0
        result = assess_parameters(values, source="mock", source_fresh=True)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["ARMING_CHECK"]["passed"])

        values["ARMING_CHECK"] = 194.0
        values["FS_PILOT_INPUT"] = 0.0
        result = assess_parameters(values, source="mock", source_fresh=True)
        self.assertFalse(result["passed"])
        self.assertFalse(result["checks"]["FS_PILOT_INPUT"]["passed"])

    def test_ros_parameter_decoder_preserves_integer_zero_and_reports_gaps(self) -> None:
        decoded, errors = decode_ros_numeric_parameters(
            ["ZERO_INT", "DOUBLE", "MISSING", "OMITTED"],
            [
                SimpleNamespace(type=2, integer_value=0),
                SimpleNamespace(type=3, double_value=3.25),
                SimpleNamespace(type=0),
            ],
        )
        self.assertEqual(decoded, {"ZERO_INT": 0.0, "DOUBLE": 3.25})
        self.assertIn("live MAVROS cache", errors["MISSING"])
        self.assertIn("omitted", errors["OMITTED"])

    def test_live_parameter_probe_uses_only_ros2_batch_get_api(self) -> None:
        class FakeGetParameters:
            class Request:
                def __init__(self) -> None:
                    self.names: list[str] = []

        class FakeFuture:
            def __init__(self, response: object) -> None:
                self._response = response

            def done(self) -> bool:
                return True

            def result(self) -> object:
                return self._response

        class FakeClient:
            def __init__(self, response: object) -> None:
                self.response = response
                self.requests: list[object] = []

            def wait_for_service(self, *, timeout_sec: float) -> bool:
                return timeout_sec >= 0.0

            def call_async(self, request: object) -> FakeFuture:
                self.requests.append(request)
                return FakeFuture(self.response)

        get_client = FakeClient(
            SimpleNamespace(
                values=[
                    SimpleNamespace(type=2, integer_value=3),
                    SimpleNamespace(type=2, integer_value=1100),
                    SimpleNamespace(type=2, integer_value=32),
                    SimpleNamespace(type=2, integer_value=194),
                    SimpleNamespace(type=2, integer_value=2),
                    SimpleNamespace(type=2, integer_value=2),
                    SimpleNamespace(type=3, double_value=3.0),
                ]
            )
        )

        class FakeNode:
            def __init__(self) -> None:
                self.created_services: list[str] = []
                self.destroyed_clients: list[FakeClient] = []

            def create_client(self, _service_type: object, name: str) -> FakeClient:
                self.created_services.append(name)
                if name == "/mavros/param/get_parameters":
                    return get_client
                raise AssertionError(f"legacy or unexpected parameter service: {name}")

            def destroy_client(self, client: FakeClient) -> None:
                self.destroyed_clients.append(client)

        interfaces_module = ModuleType("rcl_interfaces")
        interfaces_srv_module = ModuleType("rcl_interfaces.srv")
        interfaces_srv_module.GetParameters = FakeGetParameters
        interfaces_module.srv = interfaces_srv_module

        probe = RosLiveProbe.__new__(RosLiveProbe)
        probe.node = FakeNode()
        probe.spin_once = lambda _timeout_s: None
        with patch.dict(
            sys.modules,
            {
                "rcl_interfaces": interfaces_module,
                "rcl_interfaces.srv": interfaces_srv_module,
            },
        ):
            values, errors = probe.get_mavros_parameters(
                PARAMETER_NAMES,
                service_timeout_s=0.1,
            )

        self.assertFalse(errors)
        self.assertEqual(
            values,
            {
                "AHRS_EKF_TYPE": 3.0,
                "RC3_TRIM": 1100.0,
                "RC_OPTIONS": 32.0,
                "ARMING_CHECK": 194.0,
                "FS_GCS_ENABLE": 2.0,
                "FS_PILOT_INPUT": 2.0,
                "FS_PILOT_TIMEOUT": 3.0,
            },
        )
        self.assertEqual(
            probe.node.created_services,
            ["/mavros/param/get_parameters"],
        )
        self.assertEqual(
            get_client.requests[0].names,
            list(PARAMETER_NAMES),
        )

    def test_stale_initial_log_is_refreshed_before_snapshot_fallback(self) -> None:
        class FakeProbe:
            @staticmethod
            def get_mavros_parameters(
                names: list[str] | tuple[str, ...],
                *,
                service_timeout_s: float,
            ) -> tuple[dict[str, float], dict[str, str]]:
                del service_timeout_s
                return {}, {str(name): "ROS service timed out" for name in names}

        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            firmware_root = temporary_root / "ardupilot_sub_stable"
            binary = firmware_root / "build" / "sitl" / "bin" / "ardusub"
            artifact = firmware_root / "mav.parm"
            artifact.parent.mkdir(parents=True)
            artifact.write_text(
                "AHRS_EKF_TYPE 3\n"
                "RC3_TRIM 1100\n"
                "RC_OPTIONS 32\n"
                "ARMING_CHECK 194\n"
                "FS_GCS_ENABLE 2\n"
                "FS_PILOT_INPUT 2\n"
                "FS_PILOT_TIMEOUT 3\n",
                encoding="utf-8",
            )
            freshness_boundary = 10_000.0
            os.utime(artifact, (freshness_boundary + 1.0, freshness_boundary + 1.0))

            log_path = temporary_root / "sitl_20260901_120000.log"
            initial_parsed_log = parse_sitl_log("AP: ArduSub V4.1.2 (old-read)\n")
            self.assertFalse(initial_parsed_log["parameter_snapshot_saved"])
            log_path.write_text(
                f"RiTW: Starting ArduSub : {binary} -w\n"
                "AP: ArduSub V4.1.2 (same-run)\n"
                "Saved 1257 parameters to mav.parm\n",
                encoding="utf-8",
            )

            result = _collect_parameter_evidence(
                FakeProbe(),
                parsed_log=initial_parsed_log,
                log_path=log_path,
                service_timeout_s=0.1,
                not_before_wall_s=freshness_boundary,
                runtime_active=True,
            )

            self.assertTrue(result["passed"])
            self.assertTrue(result["evidence_live"])
            self.assertEqual(result["evidence_method"], "mavproxy_parameter_snapshot")
            self.assertEqual(result["source"], str(artifact))
            self.assertTrue(result["source_fresh"])
            self.assertTrue(result["artifact"]["snapshot_marker_observed"])
            self.assertEqual(result["artifact"]["snapshot_parameter_count"], 1257)

    def test_stale_mavproxy_artifact_is_diagnostic_only(self) -> None:
        class FakeProbe:
            @staticmethod
            def get_mavros_parameters(
                names: list[str] | tuple[str, ...],
                *,
                service_timeout_s: float,
            ) -> tuple[dict[str, float], dict[str, str]]:
                del service_timeout_s
                return {}, {str(name): "ROS service timed out" for name in names}

        with tempfile.TemporaryDirectory() as temporary_directory:
            temporary_root = Path(temporary_directory)
            firmware_root = temporary_root / "ardupilot_sub_stable"
            binary = firmware_root / "build" / "sitl" / "bin" / "ardusub"
            artifact = firmware_root / "mav.parm"
            artifact.parent.mkdir(parents=True)
            artifact.write_text(
                "AHRS_EKF_TYPE 3\n"
                "RC3_TRIM 1100\n"
                "RC_OPTIONS 32\n"
                "ARMING_CHECK 194\n"
                "FS_GCS_ENABLE 2\n"
                "FS_PILOT_INPUT 2\n"
                "FS_PILOT_TIMEOUT 3\n",
                encoding="utf-8",
            )
            freshness_boundary = 20_000.0
            os.utime(artifact, (freshness_boundary - 1.0, freshness_boundary - 1.0))
            log_path = temporary_root / "sitl_20260901_120000.log"
            log_path.write_text(
                f"RiTW: Starting ArduSub : {binary} -w\n"
                "AP: ArduSub V4.1.2 (same-run)\n"
                "Saved 1257 parameters to mav.parm\n",
                encoding="utf-8",
            )

            result = _collect_parameter_evidence(
                FakeProbe(),
                parsed_log={},
                log_path=log_path,
                service_timeout_s=0.1,
                not_before_wall_s=freshness_boundary,
                runtime_active=True,
            )

            self.assertFalse(result["passed"])
            self.assertFalse(result["evidence_live"])
            self.assertFalse(result["source_fresh"])
            self.assertEqual(result["evidence_method"], "mavproxy_parameter_snapshot")

    def test_clock_allows_duplicates_but_rejects_regression(self) -> None:
        good = assess_clock([1_000, 1_000, 2_000, 3_000])
        self.assertTrue(good["passed"])
        self.assertEqual(good["duplicate_count"], 1)

        bad = assess_clock([1_000, 2_000, 1_500])
        self.assertFalse(bad["passed"])
        self.assertEqual(len(bad["regressions"]), 1)

    def test_topic_counts_require_every_surface(self) -> None:
        counts = {
            "/clock": 10,
            "/mavros/state": 2,
            "/mavros/imu/data": 10,
            "/odometry/filtered": 10,
            "/dvl/data": 4,
            "/dvl/position": 4,
            "/dvl/twist": 4,
        }
        self.assertTrue(assess_topic_counts(counts)["passed"])
        counts["/dvl/position"] = 0
        self.assertFalse(assess_topic_counts(counts)["passed"])

    def test_ownership_rejects_truth_or_duplicate_publishers(self) -> None:
        owners = {
            "/dvl/data": ["/dvl_a50_node"],
            "/dvl/position": ["/dvl_a50_node"],
            "/dvl/twist": ["/dvl_to_twist_bridge"],
            "/odometry/filtered": ["/ekf_filter_node"],
            "/mavros/imu/data": ["/mavros/imu"],
        }
        self.assertTrue(assess_publisher_ownership(owners)["passed"])

        owners["/mavros/imu/data"] = ["/rogue/imu"]
        result = assess_publisher_ownership(owners)
        self.assertFalse(result["checks"]["/mavros/imu/data"]["passed"])
        self.assertEqual(
            result["checks"]["/mavros/imu/data"]["unexpected"],
            ["/rogue/imu"],
        )

        owners["/mavros/imu/data"] = ["/mavros/imu", "/rogue/imu"]
        result = assess_publisher_ownership(owners)
        self.assertFalse(result["checks"]["/mavros/imu/data"]["passed"])
        self.assertEqual(
            result["checks"]["/mavros/imu/data"]["duplicate_owner_count"],
            1,
        )

        owners["/mavros/imu/data"] = ["/mavros/imu"]

        owners["/odometry/filtered"].append("/uuv_mujoco_bridge")
        result = assess_publisher_ownership(owners)
        self.assertFalse(result["passed"])
        self.assertEqual(
            result["checks"]["/odometry/filtered"]["unexpected"],
            ["/uuv_mujoco_bridge"],
        )

        owners["/odometry/filtered"] = [
            "/primary/ekf_filter_node",
            "/duplicate/ekf_filter_node",
        ]
        result = assess_publisher_ownership(owners)
        self.assertFalse(result["checks"]["/odometry/filtered"]["passed"])
        self.assertEqual(
            result["checks"]["/odometry/filtered"]["duplicate_owner_count"],
            1,
        )

    def test_arm_sequence_requires_low_then_armed_then_neutral(self) -> None:
        events = [
            "arm sequence: arming-low RC3=1100",
            "state armed=True",
            "neutral RC3=1500 commanded",
        ]
        self.assertTrue(assess_arm_event_sequence(events)["passed"])
        self.assertFalse(
            assess_arm_event_sequence(list(reversed(events)))["passed"]
        )

    def test_neutral_heave_thresholds(self) -> None:
        calm = [
            {"position_z_m": 0.0, "vertical_speed_mps": 0.02},
            {"position_z_m": 0.03, "vertical_speed_mps": 0.04},
            {"position_z_m": 0.05, "vertical_speed_mps": 0.01},
        ]
        self.assertTrue(
            assess_neutral_heave(
                calm,
                max_abs_vertical_speed_mps=0.20,
                max_vertical_drift_m=0.20,
            )["passed"]
        )
        runaway = [
            {"position_z_m": 0.0, "vertical_speed_mps": 0.05},
            {"position_z_m": 0.5, "vertical_speed_mps": 0.8},
        ]
        self.assertFalse(
            assess_neutral_heave(
                runaway,
                max_abs_vertical_speed_mps=0.20,
                max_vertical_drift_m=0.20,
            )["passed"]
        )

    def test_lifecycle_and_arm_are_default_off(self) -> None:
        args = build_arg_parser().parse_args([])
        self.assertFalse(args.start_stack)
        self.assertFalse(args.exercise_arm)


if __name__ == "__main__":
    unittest.main(verbosity=2)
