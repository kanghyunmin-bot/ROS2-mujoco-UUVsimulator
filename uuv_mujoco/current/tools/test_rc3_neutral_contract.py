#!/usr/bin/env python3
"""Regression checks for ArduSub bidirectional RC3 neutral semantics."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.config_backend import PILOT_CONTROL_RC_OVERRIDE  # noqa: E402
from gui.control_update_format import pilot_heave_axis_summary  # noqa: E402
from sim.contracts import althold_climb_rate_from_rc3_pwm  # noqa: E402


RC_MIN = 1100
RC_MAX = 1900
RC_TRIM = RC_MIN


def _ardusub_norm_input(pwm: float) -> float:
    """Mirror RC_Channel::norm_input() for the non-reversed RC3 range."""
    if pwm < RC_TRIM:
        value = 0.0 if RC_MIN >= RC_TRIM else (pwm - RC_TRIM) / (RC_TRIM - RC_MIN)
    else:
        value = 0.0 if RC_MAX <= RC_TRIM else (pwm - RC_TRIM) / (RC_MAX - RC_TRIM)
    return max(-1.0, min(1.0, value))


class Rc3NeutralContractTest(unittest.TestCase):
    def test_stabilize_neutral_is_zero_bidirectional_thrust(self) -> None:
        normalized = _ardusub_norm_input(1500)
        bidirectional_thrust = 2.0 * (normalized - 0.5)
        self.assertAlmostEqual(normalized, 0.5)
        self.assertAlmostEqual(bidirectional_thrust, 0.0)

    def test_althold_neutral_is_zero_climb_rate(self) -> None:
        target = althold_climb_rate_from_rc3_pwm(
            1500,
            rc_min=RC_MIN,
            rc_max=RC_MAX,
            rc_trim=RC_TRIM,
            rc_deadzone=30,
            pilot_speed_up=100.0,
            pilot_speed_dn=0.0,
            gain=0.5,
        )
        self.assertAlmostEqual(target, 0.0)

    def test_gui_neutral_summary_matches_firmware_contract(self) -> None:
        rc3_pwm, target_vz = pilot_heave_axis_summary(0.0, mode=PILOT_CONTROL_RC_OVERRIDE)
        self.assertEqual(rc3_pwm, 1500)
        self.assertAlmostEqual(target_vz, 0.0)

    def test_startup_default_matches_real_vehicle_range_trim(self) -> None:
        start_text = (ROOT / "start_ardusub_sitl_mj311.sh").read_text(encoding="utf-8")
        real_params = (ROOT / "config" / "ardusub_realrobot_contract.param").read_text(encoding="utf-8")
        self.assertIn('append_param_if_not_overridden "RC3_TRIM" "${SITL_RC3_TRIM:-1100}"', start_text)
        self.assertRegex(real_params, r"(?m)^RC3_TRIM\s+1100\s*$")


if __name__ == "__main__":
    unittest.main(verbosity=2)
