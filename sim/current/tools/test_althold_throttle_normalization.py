#!/usr/bin/env python3
"""Regression checks for ArduSub 4.1 ALT_HOLD throttle normalization."""

from __future__ import annotations

import unittest
from pathlib import Path


CURRENT = Path(__file__).resolve().parents[1]
WORKSPACE = CURRENT.parents[1]


def normalized_vehicle_throttle(
    throttle_vehicle_frame_z: float,
    raw_throttle_factor: float,
    pilot_throttle_normalized: float,
) -> float:
    value = (
        0.5
        + 0.5 * throttle_vehicle_frame_z
        + raw_throttle_factor * (pilot_throttle_normalized - 0.5)
    )
    return max(0.0, min(1.0, value))


def pilot_climb_rate_from_earth_input(
    earth_vertical_input: float,
    *,
    throttle_deadzone_control: float,
    pilot_gain: float,
    speed_up_cm_s: float,
    speed_down_cm_s: float,
) -> float:
    earth_vertical_input = max(-1.0, min(1.0, earth_vertical_input))
    deadzone = max(0.0, min(0.8, 2.0 * throttle_deadzone_control * pilot_gain / 1000.0))
    if earth_vertical_input > deadzone:
        scaled = (earth_vertical_input - deadzone) / (1.0 - deadzone)
        return scaled * speed_up_cm_s
    if earth_vertical_input < -deadzone:
        scaled = (earth_vertical_input + deadzone) / (1.0 - deadzone)
        return scaled * speed_down_cm_s
    return 0.0


class AltHoldThrottleNormalizationTest(unittest.TestCase):
    def test_level_vehicle_preserves_position_controller_output(self) -> None:
        for controller_output in (0.0, 0.308, 0.5, 0.667, 1.0):
            bidirectional = 2.0 * (controller_output - 0.5)
            actual = normalized_vehicle_throttle(bidirectional, 0.0, 0.5)
            self.assertAlmostEqual(actual, controller_output)

    def test_level_correction_does_not_reverse_sign(self) -> None:
        controller_output = 0.667
        bidirectional = 2.0 * (controller_output - 0.5)
        corrected = normalized_vehicle_throttle(bidirectional, 0.0, 0.5)
        corrected_bidirectional = 2.0 * (corrected - 0.5)
        legacy_bidirectional = 2.0 * (bidirectional - 0.5)
        self.assertGreater(corrected_bidirectional, 0.0)
        self.assertLess(legacy_bidirectional, 0.0)

    def test_vertical_body_axis_pilot_input_keeps_neutral_and_endpoints(self) -> None:
        self.assertAlmostEqual(normalized_vehicle_throttle(0.0, 1.0, 0.5), 0.5)
        self.assertAlmostEqual(normalized_vehicle_throttle(0.0, 1.0, 1.0), 1.0)
        self.assertAlmostEqual(normalized_vehicle_throttle(0.0, 1.0, 0.0), 0.0)

    def test_stable_firmware_contains_explicit_range_conversion(self) -> None:
        source = (
            WORKSPACE / "sim/ardupilot" / "ArduSub" / "control_althold.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("0.5f + 0.5f * throttle_vehicle_frame.z", source)
        self.assertIn("pilot_throttle_input - 0.5f", source)
        self.assertIn("motors.set_throttle(constrain_float(throttle_normalized", source)

    def test_althold_handoff_seeds_from_current_motor_throttle(self) -> None:
        source = (
            WORKSPACE / "sim/ardupilot" / "ArduSub" / "control_althold.cpp"
        ).read_text(encoding="utf-8")
        init_source = source.split("bool Sub::althold_init()", 1)[1].split(
            "float Sub::stopping_distance()", 1
        )[0]
        self.assertIn(
            "attitude_control.set_throttle_out(motors.get_throttle(), true, 100.0);",
            init_source,
        )
        self.assertNotIn("set_throttle_out(0.75", init_source)

    def test_sitl_attitude_gains_are_stable_without_mutating_hardware_contract(self) -> None:
        launcher = (CURRENT / "start_ardusub_sitl_mj311.sh").read_text(encoding="utf-8")
        expected_sim_gains = {
            "ATC_ANG_RLL_P": ("SITL_ATC_ANG_RLL_P", "4.5"),
            "ATC_ANG_PIT_P": ("SITL_ATC_ANG_PIT_P", "4.5"),
            "ATC_ANG_YAW_P": ("SITL_ATC_ANG_YAW_P", "2.0"),
            "ATC_RAT_RLL_P": ("SITL_ATC_RAT_RLL_P", "0.08"),
            "ATC_RAT_RLL_I": ("SITL_ATC_RAT_RLL_I", "0.08"),
            "ATC_RAT_RLL_D": ("SITL_ATC_RAT_RLL_D", "0.0015"),
            "ATC_RAT_PIT_P": ("SITL_ATC_RAT_PIT_P", "0.08"),
            "ATC_RAT_PIT_I": ("SITL_ATC_RAT_PIT_I", "0.03"),
            "ATC_RAT_PIT_D": ("SITL_ATC_RAT_PIT_D", "0.0015"),
            "ATC_RAT_YAW_P": ("SITL_ATC_RAT_YAW_P", "0.12"),
            "ATC_RAT_YAW_I": ("SITL_ATC_RAT_YAW_I", "0.0"),
            "ATC_RAT_YAW_D": ("SITL_ATC_RAT_YAW_D", "0.0036"),
        }
        for param, (environment_name, default) in expected_sim_gains.items():
            with self.subTest(param=param):
                self.assertIn(
                    f'append_param_if_not_overridden "{param}" '
                    f'"${{{environment_name}:-{default}}}"',
                    launcher,
                )
        forced_block = launcher.split("is_sim_forced_param()", 1)[1].split(
            "FILTERED_REAL_PARAM_FILE", 1
        )[0]
        self.assertRegex(
            forced_block,
            r"(?m)^\s*ATC_ANG_RLL_P\|ATC_ANG_PIT_P\|ATC_ANG_YAW_P\|"
            r"ATC_RAT_RLL_P\|ATC_RAT_RLL_I\|ATC_RAT_RLL_D\|"
            r"ATC_RAT_PIT_P\|ATC_RAT_PIT_I\|ATC_RAT_PIT_D\|"
            r"ATC_RAT_YAW_P\|ATC_RAT_YAW_I\|ATC_RAT_YAW_D\)$",
        )
        self.assertRegex(forced_block, r"(?m)^\s*PSC_ACCZ_P\)$")
        self.assertIn(
            'append_param_if_not_overridden "PSC_ACCZ_P" "${SITL_PSC_ACCZ_P:-2.50}"',
            launcher,
        )
        real_params = (CURRENT / "config" / "ardusub_realrobot_contract.param").read_text(
            encoding="utf-8"
        )
        self.assertRegex(real_params, r"(?m)^ATC_RAT_RLL_D\s+0\.020000$")
        self.assertRegex(real_params, r"(?m)^ATC_RAT_PIT_D\s+0\.020000$")
        self.assertRegex(real_params, r"(?m)^ATC_RAT_YAW_D\s+0\.020000$")
        self.assertRegex(real_params, r"(?m)^PSC_ACCZ_P\s+0\.500000$")

    def test_vectored_climb_rate_uses_normalized_deadzone_and_speed_units(self) -> None:
        kwargs = {
            "throttle_deadzone_control": 100.0,
            "pilot_gain": 0.5,
            "speed_up_cm_s": 100.0,
            "speed_down_cm_s": 100.0,
        }
        self.assertEqual(pilot_climb_rate_from_earth_input(0.10, **kwargs), 0.0)
        self.assertEqual(pilot_climb_rate_from_earth_input(-0.10, **kwargs), 0.0)
        self.assertGreater(pilot_climb_rate_from_earth_input(0.15, **kwargs), 0.0)
        self.assertLess(pilot_climb_rate_from_earth_input(-0.15, **kwargs), 0.0)
        self.assertAlmostEqual(pilot_climb_rate_from_earth_input(1.0, **kwargs), 100.0)
        self.assertAlmostEqual(pilot_climb_rate_from_earth_input(-1.0, **kwargs), -100.0)

    def test_stable_firmware_does_not_mix_climb_rate_and_rc_units(self) -> None:
        source = (
            WORKSPACE / "sim/ardupilot" / "ArduSub" / "control_althold.cpp"
        ).read_text(encoding="utf-8")
        depth_source = source.split("void Sub::control_depth()", 1)[1]
        self.assertNotIn("get_pilot_desired_climb_rate(500 +", depth_source)
        self.assertIn("deadzone_normalized", depth_source)
        self.assertIn("scaled_vertical_input * g.pilot_speed_up", depth_source)

    def test_vectored_depth_input_preserves_pilot_failsafe(self) -> None:
        source = (
            WORKSPACE / "sim/ardupilot" / "ArduSub" / "control_althold.cpp"
        ).read_text(encoding="utf-8")
        depth_source = source.split("void Sub::control_depth()", 1)[1]
        self.assertIn(
            "const float pilot_forward_input = failsafe.pilot_input ? 0.0f",
            depth_source,
        )
        self.assertIn(
            "const float pilot_lateral_input = failsafe.pilot_input ? 0.0f",
            depth_source,
        )
        self.assertIn(
            "const float pilot_throttle_input = failsafe.pilot_input ? 0.5f",
            depth_source,
        )
        self.assertIn("pilot_throttle_input - 0.5f", depth_source)
        self.assertNotIn(
            "raw_throttle_factor * (channel_throttle->norm_input() - 0.5f)",
            depth_source,
        )

    def test_standard_stabilize_forwards_live_roll_pitch_targets(self) -> None:
        source = (
            WORKSPACE / "sim/ardupilot" / "ArduSub" / "control_stabilize.cpp"
        ).read_text(encoding="utf-8")
        standard_frame = source.split("default:", 1)[1]
        self.assertIn("last_roll = desired_roll_rate;", standard_frame)
        self.assertIn("last_pitch = desired_pitch_rate;", standard_frame)
        self.assertIn(
            "input_euler_angle_roll_pitch_yaw(desired_roll_rate, desired_pitch_rate, last_pilot_heading",
            standard_frame,
        )
        self.assertNotIn(
            "input_euler_angle_roll_pitch_yaw(last_roll, last_pitch, last_pilot_heading",
            standard_frame,
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
