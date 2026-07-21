"""Manual-control and RC callbacks for ALT_HOLD diagnostics."""

from __future__ import annotations

import math

from althold_diagnostics_contract import (
    RC_NEUTRAL,
    manual_heave_to_expected_rc3,
    rc3_to_expected_althold_climb,
)


def on_manual(owner, msg) -> None:
    owner.state.manual_x = float(getattr(msg, "x", math.nan))
    owner.state.manual_y = float(getattr(msg, "y", math.nan))
    owner.state.manual_z = float(getattr(msg, "z", math.nan))
    owner.state.manual_r = float(getattr(msg, "r", math.nan))
    if math.isfinite(owner.state.manual_z):
        rc3 = manual_heave_to_expected_rc3(owner.state.manual_z)
        owner.state.manual_expected_rc3 = rc3
        owner.state.manual_expected_althold_climb_cm_s = rc3_to_expected_althold_climb(rc3)


def on_rc_in(owner, msg) -> None:
    channels = list(getattr(msg, "channels", []))
    if len(channels) >= 3:
        owner.state.rc_in_ch3 = float(channels[2])
        owner.state.rc_in_source = "rc_override_mirror"


def on_rc_out(owner, msg) -> None:
    channels = list(getattr(msg, "channels", []))
    if len(channels) >= 8:
        vertical = [float(channels[idx]) for idx in (4, 5, 6, 7)]
        owner.state.rc_out_ch5 = vertical[0]
        owner.state.rc_out_ch6 = vertical[1]
        owner.state.rc_out_ch7 = vertical[2]
        owner.state.rc_out_ch8 = vertical[3]
        owner.state.rc_out_vertical_mean = sum(vertical) / 4.0
        owner.state.rc_out_vertical_span = max(vertical) - min(vertical)
        # Final ArduSub SERVO5..8 PWM -> MuJoCo vertical actuator signs.
        signs = (-1.0, 1.0, 1.0, -1.0)
        owner.state.rc_out_vertical_plant_cmd_norm = sum(
            ((pwm - RC_NEUTRAL) / 400.0) * sign
            for pwm, sign in zip(vertical, signs)
        ) / 4.0


__all__ = ["on_manual", "on_rc_in", "on_rc_out"]
