"""SITL transport coordinator for MuJoCo <-> ArduSub exchange.

This module owns:
1) Base-link truth state extraction used by the standard SITL control path.
2) Binding runtime helpers for JSON servo, JSON sensor, MAVLink, and replay paths.
"""

from __future__ import annotations

import os

import mujoco
import numpy as np

from bridge.sitl_env import env_to_float
from bridge.sitl_initialization import initialize_sensor_replay_state
from bridge.sitl_transport_config import (
    initialize_extnav_state,
    initialize_json_servo_transport,
    initialize_mavlink_transport,
    initialize_sitl_control_state,
    log_extnav_startup_state,
)
from bridge.sitl_transport_bindings import SitlTransportBindings

os.environ.setdefault("MAVLINK20", "1")


class SitlTransport(SitlTransportBindings):
    """Standard SITL transport path used by the MuJoCo runtime."""

    def __init__(
        self,
        model: mujoco.MjModel,
        sitl_ip: str,
        sitl_port: int,
        sitl_send_port: int,
        sitl_mavlink_endpoint: str,
        sitl_mavlink_servo_hz: float,
        sitl_mavlink_target_sysid: int,
        sitl_mavlink_target_compid: int,
        sitl_mavlink_source_sysid: int,
        sitl_mavlink_source_compid: int,
        enu_to_ned: np.ndarray,
        surface_pressure_pa: float,
        water_density: float,
        gravity: float,
        home_alt_m: float,
        rangefinder_max_m: float,
        command_debug: bool,
        truth_extnav_allowed: bool = True,
    ) -> None:
        self.model = model
        self._base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._dvl_altitude_sensor_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SENSOR,
            "dvl_altitude",
        )

        self._enu_to_ned = np.asarray(enu_to_ned, dtype=np.float64)
        self._bar30_surface_pressure_pa = float(surface_pressure_pa)
        self._bar30_water_density = float(water_density)
        self._bar30_gravity = float(gravity)
        self._sitl_home_alt_m = float(home_alt_m)
        self._water_surface_z = env_to_float("UUV_WATER_SURFACE_Z", 0.0)
        self._sitl_rangefinder_max_m = float(rangefinder_max_m)
        self._sitl_cmd_debug = bool(command_debug)
        # Strict real-package compatibility must exercise the physical
        # DVL -> driver -> MAVROS path.  Synthetic ExternalNav is derived from
        # MuJoCo truth and is therefore an explicit, non-parity diagnostic.
        self._sitl_truth_extnav_allowed = bool(truth_extnav_allowed)
        initialize_sensor_replay_state(
            self,
            surface_pressure_pa=self._bar30_surface_pressure_pa,
            water_density=self._bar30_water_density,
            gravity=self._bar30_gravity,
            home_alt_m=self._sitl_home_alt_m,
        )

        initialize_json_servo_transport(
            self,
            sitl_ip=sitl_ip,
            sitl_port=int(sitl_port),
            sitl_send_port=int(sitl_send_port),
        )
        initialize_sitl_control_state(self)
        initialize_mavlink_transport(
            self,
            sitl_mavlink_endpoint=sitl_mavlink_endpoint,
            sitl_mavlink_servo_hz=float(sitl_mavlink_servo_hz),
            sitl_mavlink_target_sysid=int(sitl_mavlink_target_sysid),
            sitl_mavlink_target_compid=int(sitl_mavlink_target_compid),
            sitl_mavlink_source_sysid=int(sitl_mavlink_source_sysid),
            sitl_mavlink_source_compid=int(sitl_mavlink_source_compid),
        )
        initialize_extnav_state(self)

        self._connect_sitl()
        self._connect_sitl_mavlink()
        log_extnav_startup_state(self)


__all__ = ["SitlTransport"]
