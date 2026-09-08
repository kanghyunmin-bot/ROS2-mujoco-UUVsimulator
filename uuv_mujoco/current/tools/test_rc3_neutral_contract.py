#!/usr/bin/env python3
"""Regression checks for ArduSub bidirectional RC3 neutral semantics."""

from __future__ import annotations

import re
import subprocess
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = ROOT.parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.config_backend import PILOT_CONTROL_RC_OVERRIDE  # noqa: E402
from gui.control_update_format import pilot_heave_axis_summary  # noqa: E402


RC_MIN = 1100
RC_MAX = 1900
RC_DEADZONE = 30
LEGACY_412_TRIM = 1100
NEWER_TRIM = 1500


def _ardusub_norm_input(pwm: float, *, trim: int) -> float:
    """Mirror ``RC_Channel::norm_input()`` for non-reversed RC3."""
    if pwm < trim:
        value = 0.0 if RC_MIN >= trim else (pwm - trim) / (trim - RC_MIN)
    else:
        value = 0.0 if RC_MAX <= trim else (pwm - trim) / (RC_MAX - trim)
    return max(-1.0, min(1.0, value))


def _ardusub_range_control(pwm: float) -> float:
    """Mirror ``RC_Channel::pwm_to_range_dz()`` for a 0..1000 channel."""
    radio_in = max(float(RC_MIN), min(float(RC_MAX), float(pwm)))
    trim_low = RC_MIN + RC_DEADZONE
    if radio_in <= trim_low:
        return 0.0
    return 1000.0 * (radio_in - trim_low) / (RC_MAX - trim_low)


def _ardusub_range_control_mid() -> float:
    """Mirror ``RC_Channel::get_control_mid()`` for a 0..1000 channel."""
    radio_mid = (RC_MIN + RC_MAX) // 2
    trim_low = RC_MIN + RC_DEADZONE
    return 1000.0 * (radio_mid - trim_low) / (RC_MAX - trim_low)


def _firmware_version(ardupilot_dir: Path) -> str:
    version_text = (ardupilot_dir / "ArduSub" / "version.h").read_text(encoding="utf-8")
    match = re.search(r'^#define\s+THISFIRMWARE\s+"([^"]+)"', version_text, re.MULTILINE)
    if match is None:
        raise AssertionError(f"THISFIRMWARE missing from {ardupilot_dir}")
    return match.group(1)


def _launcher_rc3_trim_default(start_text: str, firmware_version: str) -> int:
    """Execute the launcher's pure firmware-to-trim resolver."""
    match = re.search(
        r"(?ms)^resolve_default_rc3_trim\(\) \{\n.*?^\}\n",
        start_text,
    )
    if match is None:
        raise AssertionError("launcher RC3 trim resolver is missing")
    completed = subprocess.run(
        [
            "bash",
            "-c",
            f'{match.group(0)}\nresolve_default_rc3_trim "$1"',
            "rc3-contract",
            firmware_version,
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return int(completed.stdout.strip())


class Rc3NeutralContractTest(unittest.TestCase):
    def test_legacy_412_formula_maps_1500_to_zero_thrust(self) -> None:
        normalized = _ardusub_norm_input(1500, trim=LEGACY_412_TRIM)
        motor_throttle = normalized
        bidirectional_thrust = 2.0 * (motor_throttle - 0.5)
        self.assertAlmostEqual(normalized, 0.5)
        self.assertAlmostEqual(motor_throttle, 0.5)
        self.assertAlmostEqual(bidirectional_thrust, 0.0)

        source = (
            REPOSITORY_ROOT / "ardupilot_sub_stable" / "ArduSub" / "control_manual.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("motors.set_throttle(channel_throttle->norm_input());", source)

    def test_newer_formula_maps_1500_to_zero_thrust(self) -> None:
        normalized = _ardusub_norm_input(1500, trim=NEWER_TRIM)
        motor_throttle = (normalized + 1.0) / 2.0
        bidirectional_thrust = 2.0 * (motor_throttle - 0.5)
        self.assertAlmostEqual(normalized, 0.0)
        self.assertAlmostEqual(motor_throttle, 0.5)
        self.assertAlmostEqual(bidirectional_thrust, 0.0)

        source = (REPOSITORY_ROOT / "ardupilot" / "ArduSub" / "mode_manual.cpp").read_text(
            encoding="utf-8"
        )
        self.assertIn(
            "set_throttle((channel_throttle->norm_input() + 1.0f) / 2.0f);",
            source,
        )

    def test_althold_neutral_is_zero_climb_rate(self) -> None:
        # ALT_HOLD uses RC3 as a 0..1000 range channel. Its midpoint is based
        # on RC3_MIN/RC3_MAX and RC3_DZ, independently of RC3_TRIM.
        throttle_control = _ardusub_range_control(1500)
        self.assertAlmostEqual(throttle_control, _ardusub_range_control_mid())

    def test_gui_neutral_summary_matches_firmware_contract(self) -> None:
        rc3_pwm, target_vz = pilot_heave_axis_summary(0.0, mode=PILOT_CONTROL_RC_OVERRIDE)
        self.assertEqual(rc3_pwm, 1500)
        self.assertAlmostEqual(target_vz, 0.0)

    def test_launcher_selected_stable_412_defaults_to_trim_1100(self) -> None:
        start_text = (ROOT / "start_ardusub_sitl_mj311.sh").read_text(encoding="utf-8")
        real_params = (ROOT / "config" / "ardusub_realrobot_contract.param").read_text(encoding="utf-8")
        selected_ardupilot = REPOSITORY_ROOT / "ardupilot_sub_stable"
        self.assertTrue(selected_ardupilot.is_dir())
        version = _firmware_version(selected_ardupilot)
        self.assertEqual(version, "ArduSub V4.1.2")
        self.assertEqual(_launcher_rc3_trim_default(start_text, version), LEGACY_412_TRIM)
        self.assertIn(
            'append_param_if_not_overridden "RC3_TRIM" '
            '"${SITL_RC3_TRIM:-$SITL_DEFAULT_RC3_TRIM}"',
            start_text,
        )
        self.assertRegex(real_params, r"(?m)^RC3_TRIM\s+1100\s*$")

    def test_launcher_newer_firmware_defaults_to_trim_1500(self) -> None:
        start_text = (ROOT / "start_ardusub_sitl_mj311.sh").read_text(encoding="utf-8")
        version = _firmware_version(REPOSITORY_ROOT / "ardupilot")
        self.assertNotEqual(version, "ArduSub V4.1.2")
        self.assertEqual(_launcher_rc3_trim_default(start_text, version), NEWER_TRIM)

    def test_version_specific_arming_inputs_keep_throttle_check_enabled(self) -> None:
        start_text = (ROOT / "start_ardusub_sitl_mj311.sh").read_text(encoding="utf-8")
        self.assertIn(
            'append_param_if_not_overridden "RC_OPTIONS" "${SITL_RC_OPTIONS:-32}"',
            start_text,
        )

        # ArduSub 4.1.2's base arming check requires range control input zero:
        # arm at RC3_MIN, then move to the separate zero-thrust PWM of 1500.
        self.assertAlmostEqual(_ardusub_range_control(RC_MIN), 0.0)
        stable_arming = (
            REPOSITORY_ROOT
            / "ardupilot_sub_stable"
            / "libraries"
            / "AP_Arming"
            / "AP_Arming.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("if (c->get_control_in() != 0)", stable_arming)

        # Newer ArduSub checks the throttle against trim, so centered 1500
        # passes when the newer firmware-specific trim is selected.
        self.assertLessEqual(abs(1500 - NEWER_TRIM), RC_DEADZONE)
        newer_arming = (
            REPOSITORY_ROOT / "ardupilot" / "ArduSub" / "AP_Arming_Sub.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("sub.channel_throttle->in_trim_dz()", newer_arming)


if __name__ == "__main__":
    unittest.main(verbosity=2)
