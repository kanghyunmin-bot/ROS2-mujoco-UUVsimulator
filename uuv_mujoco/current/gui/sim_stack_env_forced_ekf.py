"""Forced GUI-start EKF and sensor environment contract."""

from __future__ import annotations

from typing import Mapping


def ekf_sensor_contract(env: Mapping[str, str], *, ekf_contract: str) -> dict[str, str]:
    poshold_extnav = ekf_contract == "poshold_extnav"
    return {
        # GUI live-control defaults to Bar30+IMU ALT_HOLD. The real 4.1.2
        # ExternalNav/DVL parity contract stays opt-in through UUV_EKF_CONTRACT.
        "SITL_USE_REAL_PARAM_FILE": "1",
        "SITL_EKF3_EXTNAV": "1" if poshold_extnav else "0",
        "SITL_EKF3_EXTNAV_POSZ": "1",
        "SITL_EKF3_EXTNAV_VELZ": "6" if poshold_extnav else "0",
        # Strict GUI runs must exercise the same EKF3 estimator class used by
        # the vehicle.  AHRS type 10 reads perfect SITL pose/velocity and would
        # leak MuJoCo truth into flight control and /mavros/imu/data.  Any
        # asynchronous JSON stability issue must be fixed at the timing/
        # transport boundary instead of selecting the truth AHRS backend.
        "SITL_AHRS_EKF_TYPE": "3",
        # Keep ArduPilot's clock on the MuJoCo sensor timestamp and block its
        # scheduler until the next sensor payload. Explicit GUI env input may
        # still select "async" for transport diagnostics.
        "ROS2_UUV_SITL_JSON_TIMING_MODE": "lockstep",
        "ROS2_UUV_SITL_EXTNAV_ENABLE": "1" if poshold_extnav else "0",
        "ROS2_UUV_SITL_EXTNAV_HZ": env.get("ROS2_UUV_SITL_EXTNAV_HZ", "15"),
        "ROS2_UUV_REQUIRE_EXTNAV_TX": "1" if poshold_extnav else "0",
        # The bridge fixes SITL vertical feedback to Bar30 depth. Keep rangefinder
        # disabled until MAVLink DISTANCE_SENSOR or SITL rangefinder is selected.
        "SITL_RNGFND1_TYPE": "0",
        "SITL_SURFACE_DEPTH": "-10.0",
        "SITL_SURFACE_MAX_THR": "0.1",
        "ROS2_UUV_EXTNAV_MAX_STALE_S": "2.0",
        "ROS2_UUV_BAR30_SURFACE_PRESSURE_PA": env.get(
            "ROS2_UUV_BAR30_SURFACE_PRESSURE_PA",
            "101640.0",
        ),
    }


__all__ = ["ekf_sensor_contract"]
