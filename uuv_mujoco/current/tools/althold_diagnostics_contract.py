"""ALT_HOLD diagnostic data contract and ArduSub RC math mirrors."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.contracts import (  # noqa: E402
    althold_climb_rate_from_rc3_pwm as contract_althold_climb_rate_from_rc3_pwm,
    effective_joystick_gain as contract_effective_joystick_gain,
    manual_heave_to_rc3_pwm as contract_manual_heave_to_rc3_pwm,
)


RC_MIN = 1100
RC_MAX = 1900
RC_NEUTRAL = 1500
RC3_TRIM = 1100
JS_GAIN_DEFAULT = 0.5
JS_GAIN_MIN = 0.25
JS_GAIN_MAX = 1.0
JS_GAIN_STEPS = 1
JS_THR_GAIN = 1.0
PILOT_SPEED_UP_CM_S = 100.0
PILOT_SPEED_DN_CM_S = 0.0
RC3_DZ = 30


@dataclass
class Snapshot:
    t_s: float

    manual_x: float = math.nan
    manual_y: float = math.nan
    manual_z: float = math.nan
    manual_r: float = math.nan
    manual_expected_rc3: float = math.nan
    manual_expected_althold_climb_cm_s: float = math.nan

    rc_in_ch3: float = math.nan
    rc_in_source: str = "none"

    depth_bar30_m: float = math.nan
    pressure_bar30_pa: float = math.nan
    mavros_local_depth_m: float = math.nan
    mavros_velz_down_mps: float = math.nan
    dvl_velz_down_mps: float = math.nan
    mavros_roll_rad: float = math.nan
    mavros_pitch_rad: float = math.nan
    mavros_yaw_rad: float = math.nan

    rc_out_ch5: float = math.nan
    rc_out_ch6: float = math.nan
    rc_out_ch7: float = math.nan
    rc_out_ch8: float = math.nan
    rc_out_vertical_mean: float = math.nan
    rc_out_vertical_span: float = math.nan
    rc_out_vertical_plant_cmd_norm: float = math.nan

    mujoco_depth_m: float = math.nan
    mujoco_depth_rate_down_mps: float = math.nan
    mujoco_roll_rad: float = math.nan
    mujoco_pitch_rad: float = math.nan
    mujoco_yaw_rad: float = math.nan
    sim_odom_depth_m: float = math.nan
    sim_odom_depth_rate_down_mps: float = math.nan
    sim_odom_roll_rad: float = math.nan
    sim_odom_pitch_rad: float = math.nan
    sim_odom_yaw_rad: float = math.nan

    sitl_att_age_s: float = math.nan
    sitl_att_roll_rad: float = math.nan
    sitl_att_pitch_rad: float = math.nan
    sitl_att_yaw_rad: float = math.nan
    sitl_att_rollspeed_rad_s: float = math.nan
    sitl_att_pitchspeed_rad_s: float = math.nan
    sitl_att_yawspeed_rad_s: float = math.nan
    sitl_raw_imu_age_s: float = math.nan
    sitl_raw_imu_xgyro_rad_s: float = math.nan
    sitl_raw_imu_ygyro_rad_s: float = math.nan
    sitl_raw_imu_zgyro_rad_s: float = math.nan
    sitl_scaled_imu_age_s: float = math.nan
    sitl_scaled_imu_xgyro_rad_s: float = math.nan
    sitl_scaled_imu_ygyro_rad_s: float = math.nan
    sitl_scaled_imu_zgyro_rad_s: float = math.nan

    live_json_timestamp_s: float = math.nan
    live_json_gyro_x_rad_s: float = math.nan
    live_json_gyro_y_rad_s: float = math.nan
    live_json_gyro_z_rad_s: float = math.nan
    live_json_accel_x_mps2: float = math.nan
    live_json_accel_y_mps2: float = math.nan
    live_json_accel_z_mps2: float = math.nan
    live_json_roll_rad: float = math.nan
    live_json_pitch_rad: float = math.nan
    live_json_yaw_rad: float = math.nan


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def effective_js_gain() -> float:
    """Mirror ArduSub joystick.cpp init_joystick() gain selection."""
    return contract_effective_joystick_gain(
        gain_default=JS_GAIN_DEFAULT,
        gain_min=JS_GAIN_MIN,
        gain_max=JS_GAIN_MAX,
        gain_steps=JS_GAIN_STEPS,
    )


def manual_heave_to_expected_rc3(heave: float) -> float:
    """Mirror ArduSub joystick.cpp MANUAL_CONTROL.z -> RC3 override."""
    return float(
        contract_manual_heave_to_rc3_pwm(
            heave,
            pwm_neutral=RC_NEUTRAL,
            rc_min=RC_MIN,
            rc_max=RC_MAX,
            gain=effective_js_gain(),
            throttle_gain=JS_THR_GAIN,
        )
    )


def rc3_to_expected_althold_climb(rc3_pwm: float) -> float:
    """Approximate level-vehicle ALT_HOLD target climb rate from RC3."""
    return contract_althold_climb_rate_from_rc3_pwm(
        rc3_pwm,
        rc_min=RC_MIN,
        rc_max=RC_MAX,
        rc_trim=RC3_TRIM,
        rc_deadzone=RC3_DZ,
        pilot_speed_up=PILOT_SPEED_UP_CM_S,
        pilot_speed_dn=PILOT_SPEED_DN_CM_S,
        gain=effective_js_gain(),
    )
