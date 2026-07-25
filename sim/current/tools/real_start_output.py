"""Output formatting for real-start state extraction."""

from __future__ import annotations

import shlex
from typing import Any


def print_shell(state: dict[str, Any]) -> None:
    mapping = {
        "UUV_REAL_START_SOURCE_T_S": state["source_t_s"],
        "UUV_REAL_START_DEPTH_M": state["depth_m"],
        "UUV_REAL_START_BASE_DEPTH_M": state["base_depth_m"],
        "UUV_REAL_START_BASE_X_M": state["base_x_m"],
        "UUV_REAL_START_BASE_Y_M": state["base_y_m"],
        "UUV_REAL_START_ROLL_RAD": state["roll_rad"],
        "UUV_REAL_START_PITCH_RAD": state["pitch_rad"],
        "UUV_REAL_START_YAW_RAD": state["yaw_rad"],
        "UUV_REAL_START_BODY_VX_MPS": state["body_vx_mps"],
        "UUV_REAL_START_BODY_VY_MPS": state["body_vy_mps"],
        "UUV_REAL_START_BODY_VZ_MPS": state["body_vz_mps"],
        "UUV_REAL_START_BODY_WX_RADPS": state["body_wx_radps"],
        "UUV_REAL_START_BODY_WY_RADPS": state["body_wy_radps"],
        "UUV_REAL_START_BODY_WZ_RADPS": state["body_wz_radps"],
        "UUV_REAL_START_MODE": state["mode"],
        "UUV_REAL_START_ARMED": "1" if state["armed"] else "0",
        "UUV_REAL_START_DEPTH_SOURCE": state["depth_source"],
        "UUV_REAL_START_BASE_DEPTH_SOURCE": state["base_depth_source"],
        "UUV_REAL_START_BASE_XY_SOURCE": state["base_xy_source"],
        "UUV_REAL_START_ATTITUDE_SOURCE": state["attitude_source"],
        "UUV_REAL_START_VELOCITY_SOURCE": state["velocity_source"],
        "UUV_REAL_START_ANGULAR_VELOCITY_SOURCE": state["angular_velocity_source"],
        "UUV_REAL_START_STATIC_PRESSURE_PA": state["static_pressure_pa"],
        "UUV_REAL_START_BAR30_SURFACE_PRESSURE_PA": state["bar30_surface_pressure_pa"],
        "UUV_REAL_START_BARO_REAL_GND_PRESSURE_PA": state["baro_real_ground_pressure_pa"],
        "UUV_REAL_START_BARO_REAL_GND_SOURCE": state["baro_real_ground_source"],
        "UUV_REAL_START_BARO_SITL_GND_PRESSURE_PA": state["baro_sitl_ground_pressure_pa"],
        "UUV_REAL_START_BARO_JSON_DEPTH_M": state["baro_json_depth_m"],
        "UUV_REAL_START_BARO_FRONTEND_DEPTH_M": state["baro_frontend_depth_m"],
    }
    for key, value in mapping.items():
        print(f"{key}={shlex.quote(str(value))}")
