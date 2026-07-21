#!/usr/bin/env python3
"""Process-free checks for simulator GUI Pinger tuning launch arguments."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch


CURRENT_DIR = Path(__file__).resolve().parents[1]
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from gui import web_process_manager  # noqa: E402


class _Node:
    def __init__(self) -> None:
        self.events: list[str] = []

    def push_event(self, text: str) -> None:
        self.events.append(str(text))


class _RunningProcess:
    def poll(self):
        return None


class PingerHomingLaunchParametersTest(unittest.TestCase):
    def _launch(self, values: dict[str, object]) -> tuple[str, dict[str, object], _Node]:
        node = _Node()
        manager = web_process_manager.WebProcessManager(node)
        captured: dict[str, object] = {}

        def capture_start(**kwargs: object) -> dict[str, object]:
            captured.update(kwargs)
            manager._pinger_homing_process = _RunningProcess()  # type: ignore[assignment]
            return {"status": "captured"}

        manager._start_plain_process = capture_start  # type: ignore[method-assign]
        with patch.object(
            web_process_manager,
            "load_course_layout_config",
            return_value={"active_mode": "competition"},
        ), patch.object(web_process_manager, "RC_PWM_SPAN", 400.0):
            result = manager.start_pinger_homing(values)

        command = "\n".join(captured["cmd"])  # type: ignore[arg-type]
        self.assertIn("mode:=STABILIZE", command)
        self.assertIn("auto_mode:=true", command)
        self.assertNotIn("mode:=MANUAL", command)
        return command, result, node

    def test_operator_values_are_forwarded_to_cpp_launch(self) -> None:
        command, result, node = self._launch(
            {
                "algorithm": "no_odom_phase",
                "navigation_mode": "no_odom_phase",
                "reference_frequency_hz": 22750.25,
                "probe_pwm_delta": 64,
                "approach_pwm_delta": 104,
                "approach_duration_s": 6.25,
            }
        )

        self.assertIn("reference_frequency_hz:=22750.250000", command)
        self.assertIn("rc_pwm_span:=400.000000", command)
        self.assertIn("no_odom_probe_pwm_delta:=64", command)
        self.assertIn("no_odom_approach_pwm_delta:=104", command)
        self.assertIn("no_odom_forward_duration_s:=6.250000", command)
        self.assertIn("no_odom_probe_leg_s:=0.600000", command)
        self.assertIn("no_odom_probe_neutral_s:=0.200000", command)
        self.assertIn("no_odom_probe_settle_s:=0.200000", command)
        self.assertIn("no_odom_initial_confirmation_probes:=2", command)
        self.assertIn("no_odom_initial_confirmation_dot_min:=0.750000", command)
        self.assertEqual(
            result["tuning"],
            {
                "reference_frequency_hz": 22750.25,
                "rc_pwm_span": 400.0,
                "probe_pwm_delta": 64,
                "approach_pwm_delta": 104,
                "forward_duration_s": 6.25,
                "probe_leg_s": 0.6,
                "probe_neutral_s": 0.2,
                "probe_settle_s": 0.2,
                "initial_confirmation_probes": 2,
            },
        )
        self.assertTrue(any("reference=22750.250Hz" in event for event in node.events))

    def test_canonical_names_win_over_short_gui_aliases(self) -> None:
        command, result, _node = self._launch(
            {
                "no_odom_probe_pwm_delta": 72,
                "probe_pwm_delta": 99,
                "no_odom_approach_pwm_delta": 112,
                "approach_pwm_delta": 155,
                "no_odom_forward_duration_s": 3.5,
                "approach_duration_s": 9.0,
            }
        )

        self.assertIn("no_odom_probe_pwm_delta:=72", command)
        self.assertIn("no_odom_approach_pwm_delta:=112", command)
        self.assertIn("no_odom_forward_duration_s:=3.500000", command)
        self.assertEqual(result["tuning"]["probe_pwm_delta"], 72)  # type: ignore[index]

    def test_values_are_bounded_to_controller_safe_limits(self) -> None:
        command, result, _node = self._launch(
            {
                "reference_frequency_hz": 999999,
                "probe_pwm_delta": 1,
                "approach_pwm_delta": 9999,
                "forward_duration_s": 0.01,
            }
        )

        self.assertIn("reference_frequency_hz:=47000.000000", command)
        self.assertIn("no_odom_probe_pwm_delta:=32", command)
        self.assertIn("no_odom_approach_pwm_delta:=200", command)
        self.assertIn("no_odom_forward_duration_s:=0.500000", command)
        self.assertEqual(result["tuning"]["reference_frequency_hz"], 47000.0)  # type: ignore[index]

    def test_malformed_values_use_validated_safe_defaults(self) -> None:
        command, result, _node = self._launch(
            {
                "reference_frequency_hz": "21164; touch /tmp/unsafe",
                "probe_pwm_delta": "not-a-number",
                "approach_pwm_delta": float("inf"),
                "approach_duration_s": None,
            }
        )

        self.assertNotIn("not-a-number", command)
        self.assertNotIn("touch /tmp/unsafe", command)
        self.assertIn("reference_frequency_hz:=21164.000000", command)
        self.assertIn("no_odom_probe_pwm_delta:=0", command)
        self.assertIn("no_odom_approach_pwm_delta:=0", command)
        self.assertIn("no_odom_forward_duration_s:=40.000000", command)
        self.assertEqual(result["tuning"]["reference_frequency_hz"], 21164.0)  # type: ignore[index]


if __name__ == "__main__":
    unittest.main()
