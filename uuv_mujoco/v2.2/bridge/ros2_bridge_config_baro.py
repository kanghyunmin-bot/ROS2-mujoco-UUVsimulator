"""Bar30/AP_Baro pressure contract configuration for Ros2Bridge."""

from __future__ import annotations

import os

import numpy as np

from bridge.sitl_env import env_flag, env_to_float
from sim.contracts import (
    AP_BARO_FRONTEND_PA_PER_M,
    AP_BARO_SITL_SSL_AIR_PRESSURE_PA,
    AP_BARO_SITL_WATER_DENSITY_KG_M3,
    BaroPressureLaw,
)


def configure_baro_pressure_contract(bridge: object) -> None:
    bridge._bar30_surface_pressure_pa = bridge._env_to_clamped_float(
        "ROS2_UUV_BAR30_SURFACE_PRESSURE_PA",
        101640.0,
        80000.0,
        120000.0,
    )
    bridge._bar30_water_density = bridge._env_to_clamped_float(
        "ROS2_UUV_BAR30_WATER_DENSITY",
        float(bridge.model.opt.density),
        900.0,
        1200.0,
    )
    bridge._bar30_gravity = bridge._env_to_clamped_float(
        "ROS2_UUV_BAR30_GRAVITY",
        9.80665,
        9.5,
        10.0,
    )
    bridge._sitl_baro_depth_contract = str(
        os.environ.get("ROS2_UUV_SITL_BARO_DEPTH_CONTRACT", "frontend_match")
    ).strip().lower()
    if bridge._sitl_baro_depth_contract not in {"frontend_match", "raw_depth"}:
        bridge._sitl_baro_depth_contract = "frontend_match"
    bridge._baro_frontend_pa_per_m = bridge._env_to_clamped_float(
        "ROS2_UUV_BARO_FRONTEND_PA_PER_M",
        AP_BARO_FRONTEND_PA_PER_M,
        1000.0,
        20000.0,
    )
    bridge._baro_sitl_ground_pressure_pa = bridge._env_to_clamped_float(
        "ROS2_UUV_BARO_SITL_GND_PRESSURE_PA",
        101473.796875,
        80000.0,
        120000.0,
    )
    bridge._baro_real_ground_pressure_pa = bridge._env_to_clamped_float(
        "ROS2_UUV_BARO_REAL_GND_PRESSURE_PA",
        bridge._bar30_surface_pressure_pa,
        80000.0,
        120000.0,
    )
    bridge._baro_pressure_law = BaroPressureLaw(
        real_ground_pressure_pa=bridge._baro_real_ground_pressure_pa,
        sitl_ground_pressure_pa=bridge._baro_sitl_ground_pressure_pa,
        specific_gravity=bridge._env_to_clamped_float("ROS2_UUV_BARO_SPEC_GRAV", 1.0, 0.5, 1.5),
        frontend_pa_per_m=bridge._baro_frontend_pa_per_m,
        sitl_ssl_air_pressure_pa=bridge._env_to_clamped_float(
            "ROS2_UUV_BARO_SITL_SSL_AIR_PRESSURE_PA",
            AP_BARO_SITL_SSL_AIR_PRESSURE_PA,
            80000.0,
            120000.0,
        ),
        sitl_water_density_kg_m3=bridge._env_to_clamped_float(
            "ROS2_UUV_BARO_SITL_WATER_DENSITY",
            AP_BARO_SITL_WATER_DENSITY_KG_M3,
            900.0,
            1200.0,
        ),
        gravity_m_s2=bridge._bar30_gravity,
    )
    bridge._water_surface_z = float(env_to_float("UUV_WATER_SURFACE_Z", 0.0))
    bridge._sitl_home_alt_m = float(env_to_float("ROS2_UUV_HOME_ALT_M", 0.0))
    bridge._ekf_contract = str(os.environ.get("UUV_EKF_CONTRACT", "althold_baro")).strip().lower()
    bridge._sitl_dvl_rangefinder_enabled = env_flag(
        "ROS2_UUV_SITL_DVL_RANGEFINDER_ENABLE",
        bridge._ekf_contract
        in {"real_param_parity", "poshold_extnav", "real-ekf", "real_ekf", "extnav"},
    )
    bridge._gravity_enu = np.array([0.0, 0.0, -bridge._bar30_gravity], dtype=np.float64)


__all__ = ["configure_baro_pressure_contract"]
