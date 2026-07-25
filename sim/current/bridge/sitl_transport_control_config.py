"""RC/control-mode state initialization for SitlTransport."""

from __future__ import annotations

import os

import numpy as np

from bridge.sitl_env import env_flag, env_to_pwm


def initialize_sitl_control_state(transport: object) -> None:
    transport._sitl_external_servo_override_until_wall = -1.0
    transport._sitl_external_servo_override_log_wall = -1.0
    transport._sitl_last_rc_override_warn_wall = -1.0
    transport._sitl_last_rc_override_log_wall = -1.0
    transport._sitl_last_external_rc_override_wall = -1.0
    transport._sitl_last_rc_override_values = [1500] * 8 + [0] * 10
    transport._sitl_last_rc_override_values_wall = -1.0
    transport._sitl_neutral_rc_keepalive = env_flag("ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE", False)
    transport._sitl_neutral_rc_keepalive_interval_s = float(
        np.clip(float(os.getenv("ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE_INTERVAL_S", "0.10")), 0.02, 1.0)
    )
    transport._sitl_neutral_rc_keepalive_holdoff_s = float(
        np.clip(float(os.getenv("ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE_HOLDOFF_S", "0.35")), 0.0, 5.0)
    )
    transport._sitl_last_neutral_rc_keepalive_wall = -1.0
    transport._sitl_rc_neutral_pwm = env_to_pwm("ROS2_UUV_SITL_RC_NEUTRAL_PWM", 1500)
    transport._sitl_last_manual_control_log_wall = -1.0
    transport._sitl_manual_control_primed = False
    transport._uuv_run_mode = os.getenv("UUV_RUN_MODE", "closed_loop").strip().lower()
    if transport._uuv_run_mode not in {"closed_loop", "plant_replay"}:
        transport._uuv_run_mode = "closed_loop"
    transport._plant_replay_mode = transport._uuv_run_mode == "plant_replay"
    transport._sitl_json_servo_fallback = env_flag("ROS2_UUV_SITL_JSON_SERVO_FALLBACK", True)
    transport._sitl_json_servo_ignored_warn_wall = -1.0
    transport._allow_rcout_plant_override = env_flag("ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE", False)
    print(
        "[sitl_transport] run mode: "
        f"{transport._uuv_run_mode}, json_servo_active={int(transport._sitl_json_servo_fallback)}, "
        f"rcout_plant_override={int(transport._allow_rcout_plant_override)}",
        flush=True,
    )
