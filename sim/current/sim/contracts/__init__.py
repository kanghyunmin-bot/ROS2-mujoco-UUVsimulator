"""Shared simulation contracts.

The first phase re-exports the existing live contract source so replay tools and
future runtime code can converge without duplicating pressure/rate formulas.
"""

from .baro import (
    AP_BARO_FRONTEND_PA_PER_M,
    AP_BARO_SITL_SSL_AIR_PRESSURE_PA,
    AP_BARO_SITL_WATER_DENSITY_KG_M3,
    STANDARD_GRAVITY_M_S2,
    BaroPressureLaw,
    surface_pressure_for_depth_sample,
)
from .observability import CONTROLLER_PARITY_OBSERVATION, PLANT_INPUT_OBSERVATION, ObservationPoint
from .rates import REAL_ROBOT_SENSOR_RATES_HZ
from .rc import (
    NEUTRAL_DEFAULT_CHANNEL_COUNT,
    PRIMARY_RC_CHANNEL_COUNT,
    PWM_CENTER,
    RC_AXIS_BY_CHANNEL,
    RC_AXIS_BY_NAME,
    RC_AXIS_CONTRACTS,
    RC_EXTENSION_NO_CHANGE_VALUE,
    RC_IGNORE_VALUE,
    RC_OVERRIDE_CHANNEL_COUNT,
    RC_RELEASE_VALUE,
    RC_VALID_MAX_PWM,
    RC_VALID_MIN_PWM,
    RcAxisContract,
    althold_climb_rate_from_rc3_pwm,
    effective_joystick_gain,
    manual_heave_to_rc3_pwm,
    neutral_rc_override_frame,
    normalize_ardusub_rc_override,
    sanitize_primary_rc,
)

__all__ = [
    "AP_BARO_FRONTEND_PA_PER_M",
    "AP_BARO_SITL_SSL_AIR_PRESSURE_PA",
    "AP_BARO_SITL_WATER_DENSITY_KG_M3",
    "REAL_ROBOT_SENSOR_RATES_HZ",
    "STANDARD_GRAVITY_M_S2",
    "BaroPressureLaw",
    "surface_pressure_for_depth_sample",
    "CONTROLLER_PARITY_OBSERVATION",
    "PLANT_INPUT_OBSERVATION",
    "ObservationPoint",
    "NEUTRAL_DEFAULT_CHANNEL_COUNT",
    "PRIMARY_RC_CHANNEL_COUNT",
    "PWM_CENTER",
    "RC_AXIS_BY_CHANNEL",
    "RC_AXIS_BY_NAME",
    "RC_AXIS_CONTRACTS",
    "RC_EXTENSION_NO_CHANGE_VALUE",
    "RC_IGNORE_VALUE",
    "RC_OVERRIDE_CHANNEL_COUNT",
    "RC_RELEASE_VALUE",
    "RC_VALID_MAX_PWM",
    "RC_VALID_MIN_PWM",
    "RcAxisContract",
    "althold_climb_rate_from_rc3_pwm",
    "effective_joystick_gain",
    "manual_heave_to_rc3_pwm",
    "neutral_rc_override_frame",
    "normalize_ardusub_rc_override",
    "sanitize_primary_rc",
]
