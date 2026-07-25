"""Compatibility facade for RC override and joystick contracts."""

from __future__ import annotations

from .rc_althold import althold_climb_rate_from_rc3_pwm
from .rc_axis import RC_AXIS_BY_CHANNEL, RC_AXIS_BY_NAME, RC_AXIS_CONTRACTS, RcAxisContract
from .rc_constants import (
    NEUTRAL_DEFAULT_CHANNEL_COUNT,
    PRIMARY_RC_CHANNEL_COUNT,
    PWM_CENTER,
    RC_EXTENSION_NO_CHANGE_VALUE,
    RC_IGNORE_VALUE,
    RC_OVERRIDE_CHANNEL_COUNT,
    RC_RELEASE_VALUE,
    RC_VALID_MAX_PWM,
    RC_VALID_MIN_PWM,
)
from .rc_frames import neutral_rc_override_frame, normalize_ardusub_rc_override, sanitize_primary_rc
from .rc_joystick import effective_joystick_gain, manual_heave_to_rc3_pwm


__all__ = [
    "PWM_CENTER",
    "RC_VALID_MIN_PWM",
    "RC_VALID_MAX_PWM",
    "RC_OVERRIDE_CHANNEL_COUNT",
    "PRIMARY_RC_CHANNEL_COUNT",
    "NEUTRAL_DEFAULT_CHANNEL_COUNT",
    "RC_RELEASE_VALUE",
    "RC_IGNORE_VALUE",
    "RC_EXTENSION_NO_CHANGE_VALUE",
    "RcAxisContract",
    "RC_AXIS_CONTRACTS",
    "RC_AXIS_BY_NAME",
    "RC_AXIS_BY_CHANNEL",
    "sanitize_primary_rc",
    "neutral_rc_override_frame",
    "normalize_ardusub_rc_override",
    "effective_joystick_gain",
    "manual_heave_to_rc3_pwm",
    "althold_climb_rate_from_rc3_pwm",
]
