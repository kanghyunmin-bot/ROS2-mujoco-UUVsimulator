"""SITL transport helpers for MuJoCo <-> ArduSub JSON exchange.

This module owns:
1) Base-link truth state extraction used by the standard SITL control path.
2) ArduPilot JSON UDP socket + MAVLink SERVO_OUTPUT_RAW input handling.
3) JSON payload construction for the standard MuJoCo -> ArduSub path.
"""

from __future__ import annotations

import json
import os
import socket
import struct
import time
from dataclasses import dataclass
from typing import Callable, Optional

import mujoco
import numpy as np


@dataclass(frozen=True)
class VerticalEstimate:
    """Single-source vertical state exported to ArduSub SITL."""

    depth_m: float
    pressure_pa: float | None
    pos_ned: np.ndarray
    vel_ned: np.ndarray
    alt_m: float


class SitlTransport:
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
        self._water_surface_z = self._env_to_float("UUV_WATER_SURFACE_Z", 0.0)
        self._sitl_rangefinder_max_m = float(rangefinder_max_m)
        self._sitl_cmd_debug = bool(command_debug)

        self.sitl_sock = None
        self.sitl_addr = (sitl_ip, int(sitl_port))
        self.sitl_send_addr = (sitl_ip, int(sitl_send_port))
        self.sitl_listen_addr = ("0.0.0.0", int(sitl_port))
        self._sitl_client_addr = None
        self._sitl_send_target = None
        self._sitl_client_last_wall = -1.0
        self._sitl_client_logged = False
        self._sitl_no_client_warn_interval_s = 3.0
        self._sitl_last_client_missing_wall = -1.0
        self._sitl_last_command_stale_wall = -1.0
        self._sitl_last_send_wall = -1.0
        self._sitl_last_send_err_wall = -1.0
        self._sitl_last_no_client_wall = -1.0
        self._sitl_last_sensor_log_wall = -1.0
        self._sitl_send_counter = 0
        self._sitl_first_servo_wall = -1.0
        self._sitl_last_nonneutral_servo_wall = -1.0
        self._sitl_last_neutral_warn_wall = -1.0
        self._sitl_nonfinite_warned = False
        self._sitl_prev_sim_t = None
        self._sitl_prev_pos_enu = None
        self._sitl_last_cmd_log = -1.0
        self._sitl_last_servo_pkt = None
        self._sitl_servo_callback = None
        self._sitl_external_servo_override_until_wall = -1.0
        self._sitl_external_servo_override_log_wall = -1.0
        self._sitl_last_rc_override_warn_wall = -1.0
        self._sitl_last_rc_override_log_wall = -1.0
        self._sitl_last_external_rc_override_wall = -1.0
        self._sitl_last_rc_override_values = [1500] * 8
        self._sitl_last_rc_override_values_wall = -1.0
        self._sitl_neutral_rc_keepalive = self._env_flag("ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE", False)
        self._sitl_neutral_rc_keepalive_interval_s = float(
            np.clip(float(os.getenv("ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE_INTERVAL_S", "0.10")), 0.02, 1.0)
        )
        self._sitl_neutral_rc_keepalive_holdoff_s = float(
            np.clip(float(os.getenv("ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE_HOLDOFF_S", "0.35")), 0.0, 5.0)
        )
        self._sitl_last_neutral_rc_keepalive_wall = -1.0
        self._sitl_rc_neutral_pwm = self._env_to_pwm("ROS2_UUV_SITL_RC_NEUTRAL_PWM", 1500)
        self._sitl_last_manual_control_log_wall = -1.0
        self._sitl_manual_control_primed = False
        self._sitl_json_servo_fallback = self._env_flag("ROS2_UUV_SITL_JSON_SERVO_FALLBACK", True)
        self._sitl_json_servo_ignored_warn_wall = -1.0
        self._allow_rcout_plant_override = self._env_flag("ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE", False)

        self._sitl_mavlink_endpoint = str(sitl_mavlink_endpoint or "").strip()
        requested_servo_hz = float(sitl_mavlink_servo_hz)
        self._sitl_mavlink_servo_hz = float(np.clip(requested_servo_hz, 1.0, 20.0))
        if requested_servo_hz > self._sitl_mavlink_servo_hz + 1e-6:
            print(
                "[sitl_transport] sitl_mavlink_servo_hz clamped to "
                f"{self._sitl_mavlink_servo_hz:.1f}Hz (requested {requested_servo_hz:.1f}Hz) "
                "to avoid ArduSub message-rate overrun.",
                flush=True,
            )
        self._sitl_mavlink_target_sysid = max(0, min(255, int(sitl_mavlink_target_sysid)))
        self._sitl_mavlink_target_compid = max(0, min(255, int(sitl_mavlink_target_compid)))
        self._sitl_mavlink_source_system = max(1, min(255, int(sitl_mavlink_source_sysid or 255)))
        self._sitl_mavlink_source_component = max(1, min(255, int(sitl_mavlink_source_compid or 190)))
        self._sitl_mav = None
        self._sitl_mavutil = None
        self._sitl_mav_hb = None
        self._sitl_cmd_mavlink_endpoint = os.getenv("ROS2_UUV_SITL_COMMAND_MAVLINK_ENDPOINT", "").strip()
        self._sitl_cmd_mav = None
        self._sitl_cmd_mav_hb = None
        self._sitl_cmd_mav_last_connect_attempt_wall = -1.0
        self._sitl_vehicle_armed = False
        self._sitl_last_vehicle_armed = None
        self._sitl_vehicle_mode = ""
        self._sitl_last_vehicle_mode = None
        self._sitl_pending_arm_target: bool | None = None
        self._sitl_pending_arm_start_wall = -1.0
        self._sitl_pending_arm_last_send_wall = -1.0
        self._sitl_last_disarmed_servo_warn_wall = -1.0
        self._sitl_last_all_min_servo_warn_wall = -1.0
        self._sitl_mav_last_hb_wall = -1.0
        self._sitl_mav_last_msg_wall = -1.0
        self._sitl_mav_last_req_wall = -1.0
        self._sitl_mav_last_heartbeat_send_wall = -1.0
        self._sitl_cmd_mav_last_heartbeat_send_wall = -1.0
        self._sitl_mav_last_wait_warn_wall = -1.0
        self._sitl_mav_target_mismatch_warn_wall = -1.0
        self._sitl_mav_wait_warn_interval_s = 3.0
        mavlink_poll_default_hz = 50.0 if self._sitl_json_servo_fallback else max(50.0, 3.0 * self._sitl_mavlink_servo_hz)
        self._sitl_mavlink_poll_hz = float(
            np.clip(
                self._env_to_float("ROS2_UUV_SITL_MAVLINK_POLL_HZ", mavlink_poll_default_hz),
                5.0,
                200.0,
            )
        )
        self._sitl_command_poll_hz = float(
            np.clip(
                self._env_to_float("ROS2_UUV_SITL_COMMAND_POLL_HZ", 25.0),
                2.0,
                100.0,
            )
        )
        self._sitl_mavlink_poll_budget = int(
            np.clip(
                round(self._env_to_float("ROS2_UUV_SITL_MAVLINK_POLL_BUDGET", 48.0)),
                1,
                256,
            )
        )
        self._sitl_command_poll_budget = int(
            np.clip(
                round(self._env_to_float("ROS2_UUV_SITL_COMMAND_POLL_BUDGET", 24.0)),
                1,
                128,
            )
        )
        self._sitl_json_poll_budget = int(
            np.clip(
                round(self._env_to_float("ROS2_UUV_SITL_JSON_POLL_BUDGET", 96.0)),
                1,
                512,
            )
        )
        self._sitl_next_mavlink_poll_wall = 0.0
        self._sitl_next_command_poll_wall = 0.0
        self._sitl_extnav_enabled = self._env_flag(
            "ROS2_UUV_SITL_EXTNAV_ENABLE",
            self._env_flag("SITL_EKF3_EXTNAV", False),
        )
        self._sitl_extnav_rate_hz = float(
            np.clip(
                self._env_to_float("ROS2_UUV_SITL_EXTNAV_HZ", 30.0),
                5.0,
                50.0,
            )
        )
        self._sitl_extnav_origin_lat_e7 = int(
            round(self._env_to_float("ROS2_UUV_SITL_EXTNAV_ORIGIN_LAT", 47.607584) * 1.0e7)
        )
        self._sitl_extnav_origin_lon_e7 = int(
            round(self._env_to_float("ROS2_UUV_SITL_EXTNAV_ORIGIN_LON", -122.343911) * 1.0e7)
        )
        self._sitl_extnav_origin_alt_mm = int(
            round(self._env_to_float("ROS2_UUV_SITL_EXTNAV_ORIGIN_ALT_M", 0.0) * 1000.0)
        )
        ignored_extnav_options = [
            name
            for name in ("ROS2_UUV_SITL_EXTNAV_Z_UP", "ROS2_UUV_SITL_EXTNAV_VELZ_SCALE")
            if os.getenv(name) not in (None, "")
        ]
        if ignored_extnav_options:
            print(
                "[sitl_transport] ignoring ExternalNav z override envs "
                f"{', '.join(ignored_extnav_options)}; ExternalNav uses canonical NED down-positive state.",
                flush=True,
            )
        self._sitl_extnav_last_send_sim_t = -1.0
        self._sitl_extnav_last_bootstrap_wall = -1.0
        self._sitl_extnav_bootstrap_count = 0
        self._sitl_extnav_last_log_wall = -1.0
        self._sitl_extnav_send_failed_wall = -1.0
        self._sitl_extnav_required = self._env_flag("ROS2_UUV_REQUIRE_EXTNAV_TX", self._sitl_extnav_enabled)
        self._sitl_extnav_min_tx_hz = float(
            np.clip(self._env_to_float("ROS2_UUV_EXTNAV_MIN_TX_HZ", 10.0), 1.0, 50.0)
        )
        self._sitl_extnav_grace_s = float(
            np.clip(self._env_to_float("ROS2_UUV_EXTNAV_TX_GRACE_S", 6.0), 0.5, 30.0)
        )
        self._sitl_extnav_max_stale_s = float(
            np.clip(self._env_to_float("ROS2_UUV_EXTNAV_MAX_STALE_S", 0.5), 0.1, 5.0)
        )
        self._sitl_extnav_start_wall = time.monotonic()
        self._sitl_extnav_last_send_wall = -1.0
        self._sitl_extnav_tx_window_start_wall = self._sitl_extnav_start_wall
        self._sitl_extnav_tx_window_count = 0
        self._sitl_extnav_fault = ""

        self._connect_sitl()
        self._connect_sitl_mavlink()
        if self._sitl_extnav_enabled:
            print(
                "[sitl_transport] EKF3 ExternalNav output enabled "
                f"({self._sitl_extnav_rate_hz:.1f}Hz VISION_POSITION_ESTIMATE/VISION_SPEED_ESTIMATE, "
                "NED down-positive z/vel-z)",
                flush=True,
            )

    def _sensor_slice(self, sensor_id: int, data: mujoco.MjData) -> np.ndarray | None:
        if sensor_id < 0:
            return None
        adr = int(self.model.sensor_adr[sensor_id])
        dim = int(self.model.sensor_dim[sensor_id])
        return data.sensordata[adr : adr + dim]

    @staticmethod
    def _env_to_int(env_name: str, default: int) -> int:
        value = os.getenv(env_name)
        if not value:
            return default
        try:
            parsed = int(value)
        except ValueError:
            print(f"[sitl_transport] invalid {env_name}={value!r}, using {default}", flush=True)
            return default
        return max(0, parsed)

    @staticmethod
    def _env_to_pwm(env_name: str, default: int) -> int:
        value = os.getenv(env_name)
        if value:
            try:
                default = int(value)
            except ValueError:
                print(f"[sitl_transport] invalid {env_name}={value!r}, using {default}", flush=True)
        return int(np.clip(int(default), 1100, 1900))

    @staticmethod
    def _env_to_float(env_name: str, default: float) -> float:
        value = os.getenv(env_name)
        if not value:
            return float(default)
        try:
            parsed = float(value)
        except ValueError:
            print(f"[sitl_transport] invalid {env_name}={value!r}, using {default}", flush=True)
            return float(default)
        return float(parsed)

    @staticmethod
    def _env_flag(env_name: str, default: bool) -> bool:
        value = os.getenv(env_name)
        if value is None or value == "":
            return bool(default)
        return value.strip().lower() in {"1", "true", "yes", "on", "enable", "enabled"}

    @staticmethod
    def _finite_or_zero(value: float) -> float:
        return float(value) if np.isfinite(value) else 0.0

    @staticmethod
    def _pressure_abs_from_depth_m(depth_m: float, surface_pressure_pa: float, rho: float, gravity: float) -> float:
        depth = float(max(0.0, depth_m))
        return float(surface_pressure_pa + rho * gravity * depth)

    def pressure_abs_from_depth_m(self, depth_m: float) -> float:
        return self._pressure_abs_from_depth_m(
            depth_m,
            self._bar30_surface_pressure_pa,
            self._bar30_water_density,
            self._bar30_gravity,
        )

    def base_pos_world(self, data: mujoco.MjData) -> np.ndarray | None:
        if self._base_id < 0:
            return None
        pos_world = np.array(data.xpos[self._base_id], dtype=np.float64)
        if not np.all(np.isfinite(pos_world)):
            return None
        return pos_world

    def estimate_base_velocity_enu(self, sim_t: float, base_pos_enu: np.ndarray) -> np.ndarray:
        """Return simple finite-difference base velocity in ENU."""
        vel_enu = np.zeros(3, dtype=np.float64)
        prev_t = self._sitl_prev_sim_t
        prev_pos = self._sitl_prev_pos_enu
        if prev_t is not None and prev_pos is not None:
            dt = sim_t - float(prev_t)
            if 1.0e-4 <= dt <= 0.2:
                vel_fd = (base_pos_enu - prev_pos) / dt
                if np.all(np.isfinite(vel_fd)):
                    vel_enu = np.clip(vel_fd, -8.0, 8.0)
        self._sitl_prev_sim_t = sim_t
        self._sitl_prev_pos_enu = base_pos_enu.copy()
        return vel_enu

    def sitl_rangefinder_from_model(self, data: mujoco.MjData) -> float | None:
        """Return downward-looking DVL altitude as a JSON rangefinder distance."""
        dvl_alt = self._sensor_slice(self._dvl_altitude_sensor_id, data)
        if dvl_alt is None or len(dvl_alt) < 1:
            return None
        distance_m = float(dvl_alt[0])
        if not np.isfinite(distance_m):
            return None
        if distance_m < 0.0 or distance_m > self._sitl_rangefinder_max_m:
            return None
        return distance_m

    def estimate_vertical_state(
        self,
        base_pos_enu: np.ndarray,
        base_vel_enu: np.ndarray,
    ) -> VerticalEstimate | None:
        """Legacy base-link vertical fallback.

        The active ArduSub SITL path should provide a Bar30-site
        VerticalEstimate from Ros2Bridge. This fallback is kept only for
        older call paths and uses the same waterline depth convention.
        """
        if base_pos_enu is None or base_vel_enu is None:
            return None
        if not np.all(np.isfinite(base_pos_enu)) or not np.all(np.isfinite(base_vel_enu)):
            return None

        base_depth_m = float(max(0.0, self._water_surface_z - float(base_pos_enu[2])))
        pos_ned = self._enu_to_ned @ np.asarray(base_pos_enu, dtype=np.float64)
        vel_ned = self._enu_to_ned @ np.asarray(base_vel_enu, dtype=np.float64)
        pos_ned[2] = base_depth_m
        vel_ned[2] = float(-float(base_vel_enu[2]))

        pressure_pa = self.pressure_abs_from_depth_m(base_depth_m)
        alt_m = float(self._sitl_home_alt_m - base_depth_m)
        return VerticalEstimate(
            depth_m=base_depth_m,
            pressure_pa=pressure_pa,
            pos_ned=pos_ned,
            vel_ned=vel_ned,
            alt_m=alt_m,
        )

    def set_servo_handler(self, callback: Optional[Callable[[list[int]], None]]) -> None:
        self._sitl_servo_callback = callback
        print("[sitl_transport] SITL control mode set: servo-direct", flush=True)

    def inject_servo_pwm_values(
        self,
        pwm_values: list[int],
        *,
        hold_s: float = 1.0,
        source: str = "replay_rcout",
    ) -> None:
        """Inject recorded SERVO_OUTPUT_RAW values as the active plant command.

        This is for RCOUT-based plant identification: ArduSub still runs and
        receives sensors, but its generated servo packets are ignored while a
        fresh recorded RCOUT sample is being held.
        """
        now_wall = time.monotonic()
        if not self._allow_rcout_plant_override:
            if now_wall - self._sitl_last_rc_override_warn_wall > 3.0:
                print(
                    "[sitl_transport] RCOUT plant override rejected by "
                    "ROS2_UUV_ALLOW_RCOUT_PLANT_OVERRIDE=0",
                    flush=True,
                )
                self._sitl_last_rc_override_warn_wall = now_wall
            return
        hold_s = float(np.clip(float(hold_s), 0.05, 5.0))
        self._sitl_external_servo_override_until_wall = now_wall + hold_s
        if now_wall - self._sitl_external_servo_override_log_wall > 3.0:
            print(
                f"[sitl_transport] external SERVO_OUTPUT_RAW override active "
                f"source={source} hold_s={hold_s:.2f}",
                flush=True,
            )
            self._sitl_external_servo_override_log_wall = now_wall
        self._handle_pwm_values([int(v) for v in pwm_values], now_wall, source=source)

    @property
    def vehicle_armed(self) -> bool:
        return bool(self._sitl_vehicle_armed)

    @property
    def vehicle_mode(self) -> str:
        return str(self._sitl_vehicle_mode or "")

    @property
    def last_rc_override_values(self) -> list[int]:
        return list(self._sitl_last_rc_override_values[:8])

    @property
    def last_rc_override_age_s(self) -> float:
        if self._sitl_last_rc_override_values_wall <= 0.0:
            return float("inf")
        return float(max(0.0, time.monotonic() - self._sitl_last_rc_override_values_wall))

    def _neutral_rc_values(self) -> list[int]:
        return [int(self._sitl_rc_neutral_pwm)] * 8

    def _normalize_rc_override_values(
        self,
        pwm_values: list[int],
    ) -> list[int]:
        values = list(pwm_values[:8])
        if len(values) < 8:
            values.extend([65535] * (8 - len(values)))
        # ArduPilot 4.1 handles channel 1..8 RC override as:
        #   0      -> clear override for that channel
        #   65535  -> leave channel unchanged
        # MAVROS also uses 65534 as CHAN_RELEASE, but this ArduSub version does
        # not special-case it for channels 1..8, so translate it to 0.
        normalized_values: list[int] = []
        for raw in values[:8]:
            value = int(raw)
            if value <= 0 or value == 65534:
                normalized_values.append(0)
            elif value == 65535:
                normalized_values.append(65535)
            elif value < 800 or value > 2200:
                normalized_values.append(0)
            else:
                normalized_values.append(value)
        return normalized_values

    @property
    def mavlink_connected(self) -> bool:
        now = time.monotonic()
        if self._sitl_cmd_mav_hb is not None or self._sitl_mav_hb is not None:
            return True
        if self._sitl_mav_last_msg_wall > 0.0 and now - self._sitl_mav_last_msg_wall < 3.0:
            return True
        for mav in (self._sitl_cmd_mav, self._sitl_mav):
            if mav is None:
                continue
            try:
                if getattr(mav, "clients", None):
                    return True
            except Exception:
                continue
        return False

    def _mav_for_commands(self):
        return self._sitl_cmd_mav if self._sitl_cmd_mav is not None else self._sitl_mav

    def _mav_for_external_nav(self):
        return self._sitl_cmd_mav if self._sitl_cmd_mav is not None else self._sitl_mav

    def _resolve_mav_target(self, mav=None) -> tuple[int, int] | None:
        mav = mav if mav is not None else self._mav_for_commands()
        if mav is None:
            return None
        target_sys = int(self._sitl_mavlink_target_sysid)
        target_comp = int(self._sitl_mavlink_target_compid)
        if target_sys <= 0 or target_comp <= 0:
            hb = self._sitl_cmd_mav_hb if mav is self._sitl_cmd_mav else self._sitl_mav_hb
            if hb is None:
                hb = self._sitl_mav_hb
            if hb is None:
                return None
            if target_sys <= 0:
                target_sys = int(hb.get_srcSystem())
            if target_comp <= 0:
                target_comp = int(hb.get_srcComponent())
        if target_sys <= 0 or target_comp <= 0:
            return None
        try:
            mav.target_system = target_sys
            mav.target_component = target_comp
        except Exception:
            pass
        return target_sys, target_comp

    def _send_arm_disarm_mavlink(
        self,
        mav,
        target_sys: int,
        target_comp: int,
        arm_value: bool,
        *,
        force: bool = False,
    ) -> None:
        if mav is None or self._sitl_mavutil is None:
            return
        force_magic = 2989.0 if arm_value else 21196.0
        mav.mav.command_long_send(
            int(target_sys),
            int(target_comp),
            int(self._sitl_mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM),
            0,
            1.0 if arm_value else 0.0,
            force_magic if force else 0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def _service_pending_arm_command(self, now_wall: float) -> None:
        target_arm = self._sitl_pending_arm_target
        if target_arm is None:
            return
        if bool(self._sitl_vehicle_armed) == bool(target_arm):
            print(f"[sitl_transport] pending arm target reached: armed={bool(target_arm)}", flush=True)
            self._sitl_pending_arm_target = None
            return
        age_s = now_wall - self._sitl_pending_arm_start_wall
        if age_s > 90.0:
            print(f"[sitl_transport] pending arm target timed out: armed={bool(target_arm)}", flush=True)
            self._sitl_pending_arm_target = None
            return
        if now_wall - self._sitl_pending_arm_last_send_wall < 1.0:
            return
        mav = self._mav_for_commands()
        target = self._resolve_mav_target(mav)
        if mav is None or target is None:
            return
        target_sys, target_comp = target
        self._send_gcs_heartbeat(force=True, mav=mav)
        self._send_arm_disarm_mavlink(mav, target_sys, target_comp, bool(target_arm), force=True)
        if target_arm:
            try:
                neutral_values = self._neutral_rc_values()
                mav.mav.rc_channels_override_send(
                    int(target_sys),
                    int(target_comp),
                    *(int(v) for v in neutral_values),
                )
            except Exception:
                pass
        self._sitl_pending_arm_last_send_wall = now_wall
        if self._sitl_cmd_debug:
            print(f"[sitl_transport] pending arm command resent: armed={bool(target_arm)}", flush=True)

    def _heartbeat_is_vehicle(self, msg) -> bool:
        if msg is None or self._sitl_mavutil is None:
            return False
        try:
            if msg.get_type() != "HEARTBEAT":
                return False
            src_sys = int(msg.get_srcSystem())
            src_comp = int(msg.get_srcComponent())
            target_sys = int(self._sitl_mavlink_target_sysid)
            target_comp = int(self._sitl_mavlink_target_compid)
            if target_sys > 0 and src_sys != target_sys:
                return False
            if target_comp > 0 and src_comp != target_comp:
                return False
            if target_sys > 0 or target_comp > 0:
                return True
            autopilot_mega = int(self._sitl_mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
            return int(getattr(msg, "autopilot", -1)) == autopilot_mega
        except Exception:
            return False

    def _update_vehicle_heartbeat(self, msg, *, command_link: bool = False) -> None:
        if not self._heartbeat_is_vehicle(msg):
            return
        if command_link:
            self._sitl_cmd_mav_hb = msg
        else:
            self._sitl_mav_hb = msg
            self._sitl_mav_last_hb_wall = time.monotonic()
        try:
            armed_flag = int(self._sitl_mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            armed = (int(getattr(msg, "base_mode", 0)) & armed_flag) != 0
            try:
                mode = str(self._sitl_mavutil.mode_string_v10(msg))
            except Exception:
                mode = ""
            self._sitl_vehicle_armed = armed
            if mode:
                self._sitl_vehicle_mode = mode
            mode_changed = mode and self._sitl_last_vehicle_mode != mode
            armed_changed = self._sitl_last_vehicle_armed is None or self._sitl_last_vehicle_armed != armed
            if armed_changed or mode_changed:
                src = "command" if command_link else "servo"
                print(
                    f"[sitl_transport] vehicle heartbeat state: armed={armed} mode={self._sitl_vehicle_mode or 'UNKNOWN'} via {src} link",
                    flush=True,
                )
                self._sitl_last_vehicle_armed = armed
                if mode:
                    self._sitl_last_vehicle_mode = mode
        except Exception:
            pass

    def _ensure_mavlink_peer(self, mav, *, timeout_s: float = 1.5) -> bool:
        """For udpin links, wait until pymavlink has seen ArduSub's UDP peer."""
        if mav is None:
            return False
        clients = getattr(mav, "clients", None)
        if clients:
            return True
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            try:
                msg = mav.recv_match(type=["HEARTBEAT"], blocking=False)
            except Exception:
                msg = None
            if msg is not None and self._heartbeat_is_vehicle(msg):
                self._update_vehicle_heartbeat(msg, command_link=(mav is self._sitl_cmd_mav))
                return True
            clients = getattr(mav, "clients", None)
            if clients:
                return True
            time.sleep(0.02)
        clients = getattr(mav, "clients", None)
        return bool(clients)

    def send_rc_override(self, pwm_values: list[int]) -> bool:
        """Forward MAVROS-style RC override to ArduSub SITL over MAVLink."""
        mav = self._mav_for_commands()
        if mav is None:
            self._warn_rc_override_not_forwarded("MAVLink endpoint is disabled or unavailable")
            return False
        target = self._resolve_mav_target(mav)
        if target is None:
            self._warn_rc_override_not_forwarded("target system/component not resolved yet; waiting for ArduSub heartbeat")
            return False
        target_sys, target_comp = target
        values = self._normalize_rc_override_values(pwm_values)
        try:
            if not self._ensure_mavlink_peer(mav, timeout_s=0.2):
                self._warn_rc_override_not_forwarded("MAVLink UDP peer not discovered yet; waiting for ArduSub heartbeat")
                return False
            self._send_gcs_heartbeat(force=True, mav=mav)
            mav.mav.rc_channels_override_send(
                int(target_sys),
                int(target_comp),
                *(int(v) for v in values[:8]),
            )
            self._sitl_last_external_rc_override_wall = time.monotonic()
            self._sitl_last_rc_override_values = [int(v) for v in values[:8]]
            self._sitl_last_rc_override_values_wall = self._sitl_last_external_rc_override_wall
            if self._sitl_cmd_debug:
                now = time.monotonic()
                if now - self._sitl_last_rc_override_log_wall > 0.2:
                    print(
                        f"[sitl_transport] RC override forwarded to ArduSub "
                        f"target={target_sys}:{target_comp} pwm[1..8]={tuple(int(v) for v in values[:8])}",
                        flush=True,
                    )
                    self._sitl_last_rc_override_log_wall = now
            return True
        except Exception as exc:
            print(f"[sitl_transport] RC override send failed: {exc}", flush=True)
            return False

    def _send_neutral_rc_keepalive(self, now_wall: float) -> None:
        if not self._sitl_neutral_rc_keepalive or not self._sitl_vehicle_armed:
            return
        if now_wall - self._sitl_last_neutral_rc_keepalive_wall < self._sitl_neutral_rc_keepalive_interval_s:
            return
        if (
            self._sitl_last_external_rc_override_wall > 0.0
            and now_wall - self._sitl_last_external_rc_override_wall < self._sitl_neutral_rc_keepalive_holdoff_s
        ):
            return
        mav = self._mav_for_commands()
        target = self._resolve_mav_target(mav)
        if mav is None or target is None:
            return
        target_sys, target_comp = target
        try:
            self._send_gcs_heartbeat(force=True, mav=mav)
            values = self._neutral_rc_values()
            mav.mav.rc_channels_override_send(
                int(target_sys),
                int(target_comp),
                *(int(v) for v in values),
            )
            self._sitl_last_neutral_rc_keepalive_wall = now_wall
        except Exception:
            return

    def send_manual_control(
        self,
        *,
        x: float,
        y: float,
        z: float,
        r: float,
        buttons: int = 0,
    ) -> bool:
        """Forward MAVROS-style normalized MANUAL_CONTROL to ArduSub SITL."""
        mav = self._mav_for_commands()
        if mav is None:
            return False
        target = self._resolve_mav_target(mav)
        if target is None:
            return False
        target_sys, _target_comp = target

        def axis_to_int(value: float) -> int:
            value = float(value)
            if -1.0 <= value <= 1.0:
                value *= 1000.0
            return int(np.clip(round(value), -1000, 1000))

        def thrust_to_int(value: float) -> int:
            value = float(value)
            if -1.0 <= value <= 1.0:
                value = 500.0 + value * 500.0
            return int(np.clip(round(value), 0, 1000))

        try:
            if not self._ensure_mavlink_peer(mav, timeout_s=0.2):
                return False
            self._send_gcs_heartbeat(force=True, mav=mav)
            x_i = axis_to_int(x)
            y_i = axis_to_int(y)
            z_i = thrust_to_int(z)
            r_i = axis_to_int(r)
            if not self._sitl_manual_control_primed:
                if abs(x_i) <= 50 and abs(y_i) <= 50 and abs(z_i - 500) <= 50 and abs(r_i) <= 50:
                    self._sitl_manual_control_primed = True
                else:
                    # ArduSub's joystick path ignores held non-neutral input
                    # until it has first seen all axes near neutral.
                    mav.mav.manual_control_send(
                        int(target_sys),
                        0,
                        0,
                        500,
                        0,
                        int(buttons) & 0xFFFF,
                    )
                    time.sleep(0.02)
                    self._sitl_manual_control_primed = True
            mav.mav.manual_control_send(
                int(target_sys),
                x_i,
                y_i,
                z_i,
                r_i,
                int(buttons) & 0xFFFF,
            )
            # MANUAL_CONTROL is also pilot input. Treat it as recent external
            # control so neutral RC keepalive cannot interleave with joystick
            # commands when keepalive is enabled for non-GUI runs.
            self._sitl_last_external_rc_override_wall = time.monotonic()
            if self._sitl_cmd_debug:
                now = time.monotonic()
                if now - self._sitl_last_manual_control_log_wall > 0.2:
                    print(
                        "[sitl_transport] MANUAL_CONTROL forwarded to ArduSub "
                        f"target={target_sys} axes=(x={x_i}, y={y_i}, "
                        f"z={z_i}, r={r_i}) buttons={int(buttons) & 0xFFFF}",
                        flush=True,
                    )
                    self._sitl_last_manual_control_log_wall = now
            return True
        except Exception as exc:
            print(f"[sitl_transport] MANUAL_CONTROL send failed: {exc}", flush=True)
            return False

    def _warn_rc_override_not_forwarded(self, reason: str) -> None:
        now = time.monotonic()
        if now - self._sitl_last_rc_override_warn_wall < 2.0:
            return
        print(f"[sitl_transport] RC override not forwarded to ArduSub: {reason}", flush=True)
        self._sitl_last_rc_override_warn_wall = now

    def send_arm_command(self, arm: bool) -> bool:
        """Forward arm/disarm request to ArduSub SITL over MAVLink."""
        mav = self._mav_for_commands()
        if mav is None or self._sitl_mavutil is None:
            return False
        target = self._resolve_mav_target(mav)
        if target is None:
            return False
        target_sys, target_comp = target
        last_neutral_override_wall = 0.0

        def send_neutral_rc_override() -> None:
            nonlocal last_neutral_override_wall
            if not arm:
                return
            now = time.monotonic()
            if now - last_neutral_override_wall < 0.08:
                return
            try:
                self._send_gcs_heartbeat(force=True, mav=mav)
                neutral_values = self._neutral_rc_values()
                mav.mav.rc_channels_override_send(
                    int(target_sys),
                    int(target_comp),
                    *(int(v) for v in neutral_values),
                )
                last_neutral_override_wall = now
            except Exception:
                pass

        def send_arm_disarm_command(arm_value: bool, force: bool = False) -> None:
            self._send_arm_disarm_mavlink(mav, target_sys, target_comp, arm_value, force=force)

        def send_once(force: bool = False) -> None:
            send_arm_disarm_command(arm, force)

        def heartbeat_matches(expected_arm: bool = arm) -> bool:
            try:
                return bool(mav.motors_armed()) == bool(expected_arm)
            except Exception:
                return False

        def wait_arm_result(
            timeout_s: float,
            expected_arm: bool = arm,
            resend: Callable[[], None] | None = None,
            resend_interval_s: float = 1.0,
        ) -> bool:
            deadline = time.monotonic() + float(timeout_s)
            arm_command = int(self._sitl_mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            accepted = int(self._sitl_mavutil.mavlink.MAV_RESULT_ACCEPTED)
            last_resend_wall = 0.0
            while time.monotonic() < deadline:
                send_neutral_rc_override()
                if resend is not None:
                    now = time.monotonic()
                    if now - last_resend_wall >= float(resend_interval_s):
                        try:
                            self._send_gcs_heartbeat(force=True, mav=mav)
                            resend()
                        except Exception:
                            pass
                        last_resend_wall = now
                if self._sitl_vehicle_armed == bool(expected_arm):
                    return True
                mavs = [mav]
                if self._sitl_mav is not None and self._sitl_mav is not mav:
                    mavs.append(self._sitl_mav)
                for rx_mav in mavs:
                    try:
                        msg = rx_mav.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=False)
                    except Exception:
                        msg = None
                    if msg is None:
                        continue
                    msg_type = str(msg.get_type())
                    if msg_type == "HEARTBEAT":
                        self._update_vehicle_heartbeat(msg, command_link=(rx_mav is self._sitl_cmd_mav))
                        if heartbeat_matches(expected_arm) or self._sitl_vehicle_armed == bool(expected_arm):
                            return True
                    elif msg_type == "COMMAND_ACK":
                        command = int(getattr(msg, "command", -1))
                        result = int(getattr(msg, "result", -1))
                        if self._sitl_cmd_debug:
                            link = "command" if rx_mav is self._sitl_cmd_mav else "servo"
                            print(
                                f"[sitl_transport] COMMAND_ACK link={link} command={command} result={result}",
                                flush=True,
                            )
                        if command == arm_command and result == accepted and self._sitl_cmd_debug:
                            print(
                                "[sitl_transport] arm/disarm command accepted; waiting for armed heartbeat",
                                flush=True,
                            )
                time.sleep(0.02)
            return heartbeat_matches(expected_arm) or self._sitl_vehicle_armed == bool(expected_arm)

        try:
            if not self._ensure_mavlink_peer(mav, timeout_s=2.0):
                print("[sitl_transport] arm/disarm not sent: MAVLink UDP peer was not discovered", flush=True)
                return False
            self._send_gcs_heartbeat(force=True, mav=mav)
            if arm:
                warmup_deadline = time.monotonic() + 0.35
                while time.monotonic() < warmup_deadline:
                    send_neutral_rc_override()
                    time.sleep(0.04)
            send_once(force=False)
            if not self._env_flag("ROS2_UUV_SITL_ARM_CONFIRM_WAIT", False):
                # Do not block the MuJoCo/ROS service callback waiting for a later
                # HEARTBEAT. Blocking here stalls JSON sensor updates, which in turn
                # delays ArduSub's state transition and makes arm/disarm look failed.
                for _ in range(3):
                    self._send_gcs_heartbeat(force=True, mav=mav)
                    send_once(force=True)
                    send_neutral_rc_override()
                    time.sleep(0.08)
                self._sitl_pending_arm_target = bool(arm)
                self._sitl_pending_arm_start_wall = time.monotonic()
                self._sitl_pending_arm_last_send_wall = 0.0
                print(f"[sitl_transport] arm/disarm command queued: armed={bool(arm)}", flush=True)
                return True
            if wait_arm_result(8.0, resend=lambda: send_once(force=False)):
                print(f"[sitl_transport] arm state confirmed: armed={bool(arm)}", flush=True)
                return True
            if arm:
                print("[sitl_transport] normal arm not confirmed; retrying force arm", flush=True)
                send_once(force=True)
                if wait_arm_result(75.0, resend=lambda: send_once(force=True)):
                    print("[sitl_transport] force arm confirmed", flush=True)
                    return True
                print(
                    "[sitl_transport] force arm command sent but heartbeat confirmation was not observed",
                    flush=True,
                )
                # A delayed accepted arm can still arrive after the ROS service has
                # timed out. Cancel it immediately so the simulator never enters an
                # armed/no-valid-input state behind the GUI's back.
                send_arm_disarm_command(False, force=True)
                wait_arm_result(
                    15.0,
                    expected_arm=False,
                    resend=lambda: send_arm_disarm_command(False, force=True),
                    resend_interval_s=0.5,
                )
                return False
            print("[sitl_transport] normal disarm not confirmed; retrying force disarm", flush=True)
            send_once(force=True)
            if wait_arm_result(45.0, resend=lambda: send_once(force=True)):
                print("[sitl_transport] force disarm confirmed", flush=True)
                return True
            print(
                "[sitl_transport] force disarm command sent but heartbeat confirmation was not observed",
                flush=True,
            )
            return False
        except Exception as exc:
            print(f"[sitl_transport] arm/disarm send failed: {exc}", flush=True)
            return False

    def send_set_mode(self, mode: str) -> bool:
        """Forward custom mode request to ArduSub SITL over MAVLink."""
        mav = self._mav_for_commands()
        if mav is None:
            return False
        target = self._resolve_mav_target(mav)
        if target is None:
            return False
        target_sys, target_comp = target
        try:
            if not self._ensure_mavlink_peer(mav, timeout_s=1.0):
                print(f"[sitl_transport] set_mode {mode!r} not sent: MAVLink UDP peer was not discovered", flush=True)
                return False
            self._send_gcs_heartbeat(force=True, mav=mav)
            mode_text = str(mode).strip()
            mode_map = {}
            try:
                mavlink_defs = self._sitl_mavutil.mavlink
                mode_map = self._sitl_mavutil.mode_mapping_byname(mavlink_defs.MAV_TYPE_SUBMARINE) or {}
            except Exception:
                mavlink_defs = self._sitl_mavutil.mavlink
            fallback_modes = {
                "STABILIZE": 0,
                "ACRO": 1,
                "ALT_HOLD": 2,
                "AUTO": 3,
                "GUIDED": 4,
                "CIRCLE": 7,
                "SURFACE": 9,
                "POSHOLD": 16,
                "MANUAL": 19,
            }
            if mode_text.upper() in mode_map:
                mode_id = int(mode_map[mode_text.upper()])
            elif mode_text.upper() in fallback_modes:
                mode_id = int(fallback_modes[mode_text.upper()])
            else:
                mode_id = int(mode_text)
            mav.mav.command_long_send(
                int(target_sys),
                int(target_comp),
                int(mavlink_defs.MAV_CMD_DO_SET_MODE),
                0,
                int(mavlink_defs.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
                float(mode_id),
                0,
                0,
                0,
                0,
                0,
            )
            if self._sitl_cmd_debug:
                print(
                    f"[sitl_transport] set_mode forwarded to ArduSub target={target_sys}:{target_comp} "
                    f"mode={mode_text!r} custom_mode={mode_id}",
                    flush=True,
                )
            return True
        except Exception as exc:
            print(f"[sitl_transport] set_mode send failed for {mode!r}: {exc}", flush=True)
            return False

    def send_body_velocity_setpoint(
        self,
        *,
        forward_mps: float,
        left_mps: float,
        up_mps: float,
        yaw_rate_rad_s: float,
    ) -> bool:
        """Forward ROS body FLU velocity command to ArduSub GUIDED velocity control."""
        mav = self._mav_for_commands()
        if mav is None or self._sitl_mavutil is None:
            return False
        target = self._resolve_mav_target(mav)
        if target is None:
            return False
        target_sys, target_comp = target
        mavlink_defs = self._sitl_mavutil.mavlink
        type_mask = int(
            mavlink_defs.POSITION_TARGET_TYPEMASK_X_IGNORE
            | mavlink_defs.POSITION_TARGET_TYPEMASK_Y_IGNORE
            | mavlink_defs.POSITION_TARGET_TYPEMASK_Z_IGNORE
            | mavlink_defs.POSITION_TARGET_TYPEMASK_AX_IGNORE
            | mavlink_defs.POSITION_TARGET_TYPEMASK_AY_IGNORE
            | mavlink_defs.POSITION_TARGET_TYPEMASK_AZ_IGNORE
            | mavlink_defs.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        )
        try:
            mav.mav.set_position_target_local_ned_send(
                0,
                int(target_sys),
                int(target_comp),
                int(mavlink_defs.MAV_FRAME_BODY_NED),
                type_mask,
                0.0,
                0.0,
                0.0,
                float(forward_mps),
                float(-left_mps),
                float(-up_mps),
                0.0,
                0.0,
                0.0,
                0.0,
                float(-yaw_rate_rad_s),
            )
            return True
        except Exception as exc:
            print(f"[sitl_transport] body velocity setpoint send failed: {exc}", flush=True)
            return False

    def send_position_target_local_ned(
        self,
        *,
        coordinate_frame: int,
        type_mask: int,
        x: float,
        y: float,
        z: float,
        vx: float,
        vy: float,
        vz: float,
        afx: float,
        afy: float,
        afz: float,
        yaw: float,
        yaw_rate: float,
    ) -> bool:
        """Forward MAVROS raw local setpoint to ArduSub SITL unchanged."""
        mav = self._mav_for_commands()
        if mav is None:
            return False
        target = self._resolve_mav_target(mav)
        if target is None:
            return False
        target_sys, target_comp = target
        try:
            mav.mav.set_position_target_local_ned_send(
                0,
                int(target_sys),
                int(target_comp),
                int(coordinate_frame),
                int(type_mask),
                float(x),
                float(y),
                float(z),
                float(vx),
                float(vy),
                float(vz),
                float(afx),
                float(afy),
                float(afz),
                float(yaw),
                float(yaw_rate),
            )
            return True
        except Exception as exc:
            print(f"[sitl_transport] raw local setpoint send failed: {exc}", flush=True)
            return False

    def _connect_sitl(self) -> None:
        """Initialize UDP socket for ArduPilot SITL JSON interface."""
        try:
            self.sitl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sitl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sitl_rcvbuf = 1024 * 1024
            sitl_sndbuf = 1024 * 1024
            try:
                env_sitl_rcvbuf = int(os.getenv("ROS2_UUV_SITL_RCVBUF", str(sitl_rcvbuf)))
                if env_sitl_rcvbuf > 0:
                    sitl_rcvbuf = env_sitl_rcvbuf
            except ValueError:
                pass
            try:
                env_sitl_sndbuf = int(os.getenv("ROS2_UUV_SITL_SNDBUF", str(sitl_sndbuf)))
                if env_sitl_sndbuf > 0:
                    sitl_sndbuf = env_sitl_sndbuf
            except ValueError:
                pass
            self.sitl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, sitl_rcvbuf)
            self.sitl_sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, sitl_sndbuf)
            self.sitl_sock.bind(self.sitl_listen_addr)
            self.sitl_sock.setblocking(False)
            sock_addr = self.sitl_sock.getsockname()
            print(
                f"[sitl_transport] SITL socket initialized (listen {sock_addr}, "
                f"servo_target={self.sitl_addr}, sensor_target={self.sitl_send_addr})",
                flush=True,
            )
        except Exception as exc:
            print(f"[sitl_transport] Failed to init SITL socket: {exc}", flush=True)

    def _connect_sitl_mavlink(self) -> None:
        """Initialize MAVLink input channel for SITL servo outputs."""
        endpoint = self._sitl_mavlink_endpoint
        if endpoint and endpoint.strip().lower() in {"none", "off", "disabled", "disable"}:
            self._sitl_mav = None
            self._sitl_mavutil = None
            print(
                "[sitl_transport] SITL MAVLink servo input disabled; using JSON UDP servo packets only.",
                flush=True,
            )
            return
        try:
            from pymavlink import mavutil
        except Exception as exc:
            raise RuntimeError("pymavlink is required for SITL MAVLink servo input") from exc

        if not endpoint:
            endpoint = f"udpin:0.0.0.0:{int(os.getenv('ROS2_UUV_SITL_MAV_PORT', '14660'))}"
            self._sitl_mavlink_endpoint = endpoint
        self._sitl_mav = mavutil.mavlink_connection(
            endpoint,
            source_system=self._sitl_mavlink_source_system,
            source_component=self._sitl_mavlink_source_component,
            autoreconnect=True,
        )
        self._sitl_mavutil = mavutil
        print(
            f"[sitl_transport] SITL MAVLink servo input enabled: endpoint={endpoint}",
            flush=True,
        )
        if self._sitl_json_servo_fallback:
            print(
                "[sitl_transport] SITL JSON servo packets are the active thruster source; "
                "MAVLink SERVO_OUTPUT_RAW is used for vehicle state/telemetry only.",
                flush=True,
            )
        self._connect_sitl_command_mavlink(mavutil)

    def _command_mavlink_disabled(self) -> bool:
        endpoint = self._sitl_cmd_mavlink_endpoint
        return (not endpoint) or endpoint.lower() in {"servo", "same", "none", "disabled", "off"}

    def _connect_sitl_command_mavlink(self, mavutil) -> None:
        endpoint = self._sitl_cmd_mavlink_endpoint
        if self._command_mavlink_disabled():
            if endpoint:
                print(
                    "[sitl_transport] SITL MAVLink command output using servo link",
                    flush=True,
                )
            return
        try:
            self._sitl_cmd_mav = mavutil.mavlink_connection(
                endpoint,
                source_system=self._sitl_mavlink_source_system,
                source_component=self._sitl_mavlink_source_component,
                autoreconnect=True,
            )
            print(
                f"[sitl_transport] SITL MAVLink command output enabled: endpoint={endpoint}",
                flush=True,
            )
        except Exception as exc:
            self._sitl_cmd_mav = None
            print(f"[sitl_transport] SITL MAVLink command output unavailable: {exc}", flush=True)

    def _ensure_command_mavlink_connected(self) -> None:
        if self._sitl_cmd_mav is not None or self._sitl_mavutil is None or self._command_mavlink_disabled():
            return
        now_wall = time.monotonic()
        if now_wall - self._sitl_cmd_mav_last_connect_attempt_wall < 2.0:
            return
        self._sitl_cmd_mav_last_connect_attempt_wall = now_wall
        self._connect_sitl_command_mavlink(self._sitl_mavutil)

    def _send_gcs_heartbeat(self, *, force: bool = False, mav=None) -> None:
        mav = mav if mav is not None else self._mav_for_commands()
        if mav is None or self._sitl_mavutil is None:
            return
        now = time.monotonic()
        is_command_link = mav is self._sitl_cmd_mav
        last_wall = (
            self._sitl_cmd_mav_last_heartbeat_send_wall
            if is_command_link
            else self._sitl_mav_last_heartbeat_send_wall
        )
        if not force and now - last_wall < 1.0:
            return
        try:
            mavlink_defs = self._sitl_mavutil.mavlink
            mav.mav.heartbeat_send(
                int(mavlink_defs.MAV_TYPE_GCS),
                int(mavlink_defs.MAV_AUTOPILOT_INVALID),
                0,
                0,
                int(mavlink_defs.MAV_STATE_ACTIVE),
            )
            if is_command_link:
                self._sitl_cmd_mav_last_heartbeat_send_wall = now
            else:
                self._sitl_mav_last_heartbeat_send_wall = now
        except Exception:
            pass

    def _request_sitl_mavlink_servo_stream(self) -> None:
        if self._sitl_mav is None:
            return
        if self._sitl_mav_hb is None and (
            self._sitl_mavlink_target_sysid <= 0 or self._sitl_mavlink_target_compid <= 0
        ):
            return
        if self._sitl_mavutil is None:
            return
        now_wall = time.monotonic()
        stream_fresh = (
            self._sitl_mav_last_msg_wall > 0.0
            and (now_wall - self._sitl_mav_last_msg_wall)
            <= max(2.0, 4.0 / max(self._sitl_mavlink_servo_hz, 1.0))
        )
        req_period_s = 12.0 if stream_fresh else 2.0
        if now_wall - self._sitl_mav_last_req_wall < req_period_s:
            return
        self._sitl_mav_last_req_wall = now_wall
        if self._sitl_mavlink_target_sysid > 0 and self._sitl_mavlink_target_compid > 0:
            target_sys = int(self._sitl_mavlink_target_sysid)
            target_comp = int(self._sitl_mavlink_target_compid)
        elif self._sitl_mav_hb is not None:
            target_sys = int(self._sitl_mav_hb.get_srcSystem())
            target_comp = int(self._sitl_mav_hb.get_srcComponent())
        else:
            return
        interval_us = float(max(1.0, 1.0e6 / self._sitl_mavlink_servo_hz))
        try:
            mavlink_defs = self._sitl_mavutil.mavlink
            self._sitl_mav.mav.command_long_send(
                target_sys,
                target_comp,
                mavlink_defs.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mavlink_defs.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW),
                interval_us,
                0,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception:
            pass

    def _handle_pwm_values(self, pwm_values: list[int], now_wall: float, source: str) -> None:
        external_source = str(source).startswith("replay_rcout")
        if (
            not external_source
            and self._sitl_external_servo_override_until_wall > 0.0
            and now_wall < self._sitl_external_servo_override_until_wall
        ):
            return
        if not self._sitl_vehicle_armed:
            active = [int(v) for v in pwm_values[:8] if int(v) > 0 and int(v) != 65535]
            nonneutral_disarmed = any(abs(v - 1500) > 12 for v in active)
            if nonneutral_disarmed and now_wall - self._sitl_last_disarmed_servo_warn_wall > 3.0:
                print(
                    f"[sitl_transport] SITL({source}) servo output ignored while disarmed "
                    f"pwm[1..8]={tuple(int(v) for v in pwm_values[:8])}",
                    flush=True,
                )
                self._sitl_last_disarmed_servo_warn_wall = now_wall
            pwm_values = [1500] * max(8, len(pwm_values))
        else:
            active = [int(v) for v in pwm_values[:8] if int(v) > 0 and int(v) != 65535]
            all_min = len(active) >= 8 and all(v <= 1120 for v in active[:8])
            if all_min:
                if now_wall - self._sitl_last_all_min_servo_warn_wall > 3.0:
                    print(
                        f"[sitl_transport] SITL({source}) all-min motor frame treated as neutral "
                        f"pwm[1..8]={tuple(int(v) for v in pwm_values[:8])}",
                        flush=True,
                    )
                    self._sitl_last_all_min_servo_warn_wall = now_wall
                pwm_values = [1500] * max(8, len(pwm_values))
        pwm_head = pwm_values[:8]
        nonneutral = False
        valid_pwm = []
        for value in pwm_head:
            iv = int(value)
            if iv <= 0 or iv == 65535:
                continue
            valid_pwm.append(iv)
            if abs(iv - 1500) > 12:
                nonneutral = True
                break
        if nonneutral:
            self._sitl_last_nonneutral_servo_wall = now_wall
        else:
            since_nonneutral = (
                now_wall - self._sitl_last_nonneutral_servo_wall
                if self._sitl_last_nonneutral_servo_wall > 0.0
                else now_wall - self._sitl_first_servo_wall
            )
            if since_nonneutral > 3.0 and now_wall - self._sitl_last_neutral_warn_wall > 3.0:
                if not valid_pwm:
                    print(
                        f"[sitl_transport] SITL({source}) servo stream has no active outputs (all 0/65535). "
                        "Check: vehicle ARM state and JSON sensor stream health.",
                        flush=True,
                    )
                else:
                    print(
                        f"[sitl_transport] SITL({source}) servo stream is neutral (all near 1500). "
                        "Check: vehicle ARM state, QGC joystick enabled, MANUAL mode.",
                        flush=True,
                    )
                self._sitl_last_neutral_warn_wall = now_wall

        if self._sitl_servo_callback is not None:
            try:
                self._sitl_servo_callback(pwm_values)
            except Exception as exc:
                print(f"[sitl_transport] SITL servo callback failed: {exc}", flush=True)
        if self._sitl_cmd_debug:
            pkt8 = tuple(int(v) for v in pwm_values[:8])
            if (self._sitl_last_servo_pkt != pkt8) and (now_wall - self._sitl_last_cmd_log > 0.15):
                print(f"[sitl_transport] SITL({source}) servo pwm[1..8]={pkt8}", flush=True)
                self._sitl_last_cmd_log = now_wall
                self._sitl_last_servo_pkt = pkt8

    def _poll_servo_mavlink(self) -> None:
        if self._sitl_mav is None:
            return
        now_wall = time.monotonic()
        self._service_pending_arm_command(now_wall)
        got_any = False
        latest_pwm_values: list[int] | None = None
        for _ in range(self._sitl_mavlink_poll_budget):
            try:
                msg = self._sitl_mav.recv_match(
                    type=["HEARTBEAT", "SERVO_OUTPUT_RAW"],
                    blocking=False,
                )
            except Exception:
                break
            if msg is None:
                break
            mtype = msg.get_type()
            if mtype == "HEARTBEAT":
                src_sys = int(msg.get_srcSystem())
                src_comp = int(msg.get_srcComponent())
                try:
                    ap = int(getattr(msg, "autopilot", -1))
                except Exception:
                    ap = -1
                autopilot_mega = -1
                if self._sitl_mavutil is not None:
                    try:
                        autopilot_mega = int(self._sitl_mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA)
                    except Exception:
                        autopilot_mega = -1
                target_sys = self._sitl_mavlink_target_sysid
                target_comp = self._sitl_mavlink_target_compid
                target_must_match = target_sys > 0 or target_comp > 0
                if target_sys > 0 and src_sys != target_sys:
                    if now_wall - self._sitl_mav_target_mismatch_warn_wall >= 2.0:
                        print(
                            f"[sitl_transport] Ignoring HEARTBEAT from src-system={src_sys}, src-comp={src_comp}; "
                            f"expecting sys={target_sys}, comp={target_comp}.",
                            flush=True,
                        )
                        self._sitl_mav_target_mismatch_warn_wall = now_wall
                    continue
                if target_comp > 0 and src_comp != target_comp:
                    if now_wall - self._sitl_mav_target_mismatch_warn_wall >= 2.0:
                        print(
                            f"[sitl_transport] Ignoring HEARTBEAT from src-system={src_sys}, src-comp={src_comp}; "
                            f"expecting sys={target_sys}, comp={target_comp}.",
                            flush=True,
                        )
                        self._sitl_mav_target_mismatch_warn_wall = now_wall
                    continue
                if (not target_must_match) and (ap != autopilot_mega):
                    continue
                if target_must_match or ap == autopilot_mega:
                    self._update_vehicle_heartbeat(msg, command_link=False)
                    self._request_sitl_mavlink_servo_stream()
                continue
            if mtype != "SERVO_OUTPUT_RAW":
                continue
            got_any = True
            self._sitl_client_last_wall = now_wall
            self._sitl_mav_last_msg_wall = now_wall
            if self._sitl_first_servo_wall <= 0.0:
                self._sitl_first_servo_wall = now_wall
            self._sitl_last_command_stale_wall = -1.0
            try:
                d = msg.to_dict()
                pwm_values = [int(d.get(f"servo{i}_raw", 0)) for i in range(1, 9)]
            except Exception:
                continue
            if self._sitl_json_servo_fallback:
                continue
            latest_pwm_values = pwm_values

        if latest_pwm_values is not None:
            self._handle_pwm_values(latest_pwm_values, now_wall, source="mavlink")

        self._request_sitl_mavlink_servo_stream()
        if not got_any:
            no_msg_age = (
                now_wall - self._sitl_mav_last_msg_wall
                if self._sitl_mav_last_msg_wall > 0.0
                else float("inf")
            )
            if (
                no_msg_age >= self._sitl_mav_wait_warn_interval_s
                and now_wall - self._sitl_mav_last_wait_warn_wall >= self._sitl_mav_wait_warn_interval_s
            ):
                print(
                    "[sitl_transport] Waiting for SITL MAVLink SERVO_OUTPUT_RAW "
                    f"on {self._sitl_mavlink_endpoint}",
                    flush=True,
                )
                self._sitl_mav_last_wait_warn_wall = now_wall

    def _poll_command_mavlink(self) -> None:
        self._ensure_command_mavlink_connected()
        if self._sitl_cmd_mav is None:
            return
        self._send_gcs_heartbeat(mav=self._sitl_cmd_mav)
        for _ in range(self._sitl_command_poll_budget):
            try:
                msg = self._sitl_cmd_mav.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=False)
            except Exception:
                break
            if msg is None:
                break
            msg_type = str(msg.get_type())
            if msg_type == "HEARTBEAT":
                self._update_vehicle_heartbeat(msg, command_link=True)
            elif msg_type == "COMMAND_ACK" and self._sitl_cmd_debug:
                print(
                    f"[sitl_transport] COMMAND_ACK link=command command={int(getattr(msg, 'command', -1))} "
                    f"result={int(getattr(msg, 'result', -1))}",
                    flush=True,
                )

    def _send_external_nav_bootstrap(self, sim_t: float, now_wall: float, mav) -> None:
        if mav is None:
            return
        if self._sitl_extnav_bootstrap_count >= 5:
            return
        if now_wall - self._sitl_extnav_last_bootstrap_wall < 1.0:
            return
        target = self._resolve_mav_target()
        if target is None:
            return
        target_sys, _target_comp = target
        try:
            mav.mav.system_time_send(
                int(time.time() * 1.0e6),
                int(max(0.0, sim_t) * 1000.0) & 0xFFFFFFFF,
            )
            mav.mav.set_gps_global_origin_send(
                int(target_sys),
                int(self._sitl_extnav_origin_lat_e7),
                int(self._sitl_extnav_origin_lon_e7),
                int(self._sitl_extnav_origin_alt_mm),
            )
            self._sitl_extnav_bootstrap_count += 1
            self._sitl_extnav_last_bootstrap_wall = now_wall
        except Exception as exc:
            if now_wall - self._sitl_extnav_send_failed_wall > 2.0:
                print(f"[sitl_transport] ExternalNav bootstrap send failed: {exc}", flush=True)
                self._sitl_extnav_send_failed_wall = now_wall

    def _send_external_nav(
        self,
        sim_t: float,
        vertical_est: VerticalEstimate,
        quat_ned_bfrd: np.ndarray,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> None:
        if not self._sitl_extnav_enabled:
            return
        self._ensure_command_mavlink_connected()
        mav = self._mav_for_external_nav()
        if mav is None:
            return
        now_wall = time.monotonic()
        self._send_external_nav_bootstrap(sim_t, now_wall, mav)
        interval_s = 1.0 / max(self._sitl_extnav_rate_hz, 1.0)
        if (
            self._sitl_extnav_last_send_sim_t >= 0.0
            and sim_t - self._sitl_extnav_last_send_sim_t < interval_s
        ):
            return
        pos_ned = np.asarray(vertical_est.pos_ned, dtype=np.float64)
        vel_ned = np.asarray(vertical_est.vel_ned, dtype=np.float64)
        quat = np.asarray(quat_ned_bfrd, dtype=np.float64)
        if (
            pos_ned.shape[0] < 3
            or vel_ned.shape[0] < 3
            or quat.shape[0] < 4
            or not np.all(np.isfinite(pos_ned[:3]))
            or not np.all(np.isfinite(vel_ned[:3]))
            or not np.all(np.isfinite(quat[:4]))
        ):
            return
        try:
            usec = int(max(0.0, float(sim_t)) * 1.0e6)
            pos_tx = pos_ned.copy()
            vel_tx = vel_ned.copy()
            mav.mav.vision_position_estimate_send(
                usec,
                float(pos_tx[0]),
                float(pos_tx[1]),
                float(pos_tx[2]),
                float(roll),
                float(pitch),
                float(yaw),
            )
            mav.mav.vision_speed_estimate_send(
                usec,
                float(vel_tx[0]),
                float(vel_tx[1]),
                float(vel_tx[2]),
            )
            self._sitl_extnav_last_send_sim_t = float(sim_t)
            self._sitl_extnav_last_send_wall = now_wall
            self._sitl_extnav_tx_window_count += 1
            window_s = now_wall - self._sitl_extnav_tx_window_start_wall
            if window_s >= 2.0:
                rate_hz = self._sitl_extnav_tx_window_count / max(window_s, 1.0e-6)
                if (
                    self._sitl_extnav_required
                    and now_wall - self._sitl_extnav_start_wall > self._sitl_extnav_grace_s
                    and rate_hz < self._sitl_extnav_min_tx_hz
                ):
                    self._sitl_extnav_fault = (
                        "ExternalNav TX rate below contract: "
                        f"{rate_hz:.2f}Hz < {self._sitl_extnav_min_tx_hz:.2f}Hz"
                    )
                self._sitl_extnav_tx_window_start_wall = now_wall
                self._sitl_extnav_tx_window_count = 0
            if self._sitl_cmd_debug and now_wall - self._sitl_extnav_last_log_wall >= 2.0:
                print(
                    "[sitl_transport] ExternalNav tx sample "
                    f"pos_tx={pos_tx[:3].tolist()} vel_tx={vel_tx[:3].tolist()} "
                    f"truth_pos_ned={pos_ned[:3].tolist()} truth_vel_ned={vel_ned[:3].tolist()} "
                    f"rpy={[float(roll), float(pitch), float(yaw)]}",
                    flush=True,
                )
                self._sitl_extnav_last_log_wall = now_wall
        except Exception as exc:
            if now_wall - self._sitl_extnav_send_failed_wall > 2.0:
                print(f"[sitl_transport] ExternalNav send failed: {exc}", flush=True)
                self._sitl_extnav_send_failed_wall = now_wall

    def _enforce_extnav_contract(self) -> None:
        if not self._sitl_extnav_required:
            return
        now_wall = time.monotonic()
        if not self._sitl_extnav_enabled:
            raise RuntimeError("ExternalNav contract required but ExternalNav output is disabled")
        if self._sitl_extnav_fault:
            raise RuntimeError(self._sitl_extnav_fault)
        if now_wall - self._sitl_extnav_start_wall < self._sitl_extnav_grace_s:
            return
        if self._sitl_extnav_last_send_wall <= 0.0:
            raise RuntimeError(
                "ExternalNav contract required but no VISION_POSITION_ESTIMATE/"
                "VISION_SPEED_ESTIMATE has been sent"
            )
        stale_s = now_wall - self._sitl_extnav_last_send_wall
        if stale_s > self._sitl_extnav_max_stale_s:
            raise RuntimeError(
                f"ExternalNav TX stale: {stale_s:.3f}s > {self._sitl_extnav_max_stale_s:.3f}s"
            )

    def _poll_servo_endpoint(self) -> None:
        if not self.sitl_sock:
            return

        now_wall = time.monotonic()
        latest_pwm_values: list[int] | None = None
        for _ in range(self._sitl_json_poll_budget):
            try:
                pkt, addr = self.sitl_sock.recvfrom(2048)
            except BlockingIOError:
                break
            except Exception:
                break

            if len(pkt) < 8:
                continue
            magic = int.from_bytes(pkt[0:2], byteorder="little", signed=False)
            if magic not in (18458, 29569):
                continue

            if self._sitl_client_addr != addr:
                if self._sitl_client_addr is None:
                    print(f"[sitl_transport] SITL servo endpoint discovered: {addr}", flush=True)
                else:
                    print(
                        f"[sitl_transport] SITL servo endpoint changed: "
                        f"{self._sitl_client_addr} -> {addr}",
                        flush=True,
                    )
                self._sitl_client_addr = addr
                self._sitl_client_logged = True

            self._sitl_client_last_wall = now_wall
            if self._sitl_first_servo_wall <= 0.0:
                self._sitl_first_servo_wall = now_wall
            self._sitl_last_command_stale_wall = -1.0

            frame_size = 2 + 2 + 4 + (16 * 2 if magic == 18458 else 32 * 2)
            if len(pkt) < frame_size:
                continue
            try:
                pwm_values = list(struct.unpack_from("<16H" if magic == 18458 else "<32H", pkt, 8))
            except struct.error:
                continue
            if self._sitl_mav is not None and not self._sitl_json_servo_fallback:
                if now_wall - self._sitl_json_servo_ignored_warn_wall > 3.0:
                    print(
                        "[sitl_transport] SITL(json) servo packet ignored because "
                        "MAVLink SERVO_OUTPUT_RAW is the active thruster source.",
                        flush=True,
                    )
                    self._sitl_json_servo_ignored_warn_wall = now_wall
                continue
            latest_pwm_values = pwm_values

        if latest_pwm_values is not None:
            self._handle_pwm_values(latest_pwm_values, now_wall, source="json")

        if self._sitl_client_addr is None:
            if now_wall - self._sitl_last_client_missing_wall >= self._sitl_no_client_warn_interval_s:
                print(
                    f"[sitl_transport] Waiting for SITL servo packets on {self.sitl_addr} "
                    "(QGC joystick enabled, vehicle armed, and mode MANUAL/STABILIZE/DEPTH_HOLD).",
                    flush=True,
                )
                self._sitl_last_client_missing_wall = now_wall
        elif (
            self._sitl_client_last_wall > 0.0
            and now_wall - self._sitl_client_last_wall >= self._sitl_no_client_warn_interval_s
        ):
            if now_wall - self._sitl_last_command_stale_wall >= self._sitl_no_client_warn_interval_s:
                print(
                    f"[sitl_transport] No new SITL servo packets for "
                    f"{now_wall - self._sitl_client_last_wall:.1f}s from {self._sitl_client_addr}",
                    flush=True,
                )
                self._sitl_last_command_stale_wall = now_wall

    def poll_servo(self) -> None:
        now_wall = time.monotonic()
        if now_wall >= self._sitl_next_command_poll_wall:
            self._sitl_next_command_poll_wall = now_wall + 1.0 / max(self._sitl_command_poll_hz, 1.0)
            self._poll_command_mavlink()
        if now_wall >= self._sitl_next_mavlink_poll_wall:
            self._sitl_next_mavlink_poll_wall = now_wall + 1.0 / max(self._sitl_mavlink_poll_hz, 1.0)
            self._send_gcs_heartbeat(mav=self._sitl_mav)
            self._poll_servo_mavlink()
        self._poll_servo_endpoint()
        self._send_neutral_rc_keepalive(now_wall)

    @staticmethod
    def quat_to_rpy(quat: np.ndarray) -> tuple[float, float, float]:
        w, x, y, z = quat
        norm = float(np.linalg.norm(quat))
        if norm <= 1e-12:
            return 0.0, 0.0, 0.0
        q0, q1, q2, q3 = w / norm, x / norm, y / norm, z / norm
        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            pitch = np.copysign(np.pi / 2.0, sinp)
        else:
            pitch = np.arcsin(sinp)
        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return float(roll), float(pitch), float(yaw)

    def send_state(
        self,
        sim_t: float,
        gyro: np.ndarray,
        acc: np.ndarray,
        vertical_est: VerticalEstimate,
        quat: np.ndarray,
        rangefinder_distance_m: float | None = None,
        pressure_pa: float | None = None,
    ) -> None:
        """Send JSON packet to ArduPilot SITL."""
        if not self.sitl_sock:
            return

        now_wall = time.monotonic()
        sitl_t = float(max(0.0, sim_t))

        roll, pitch, yaw = self.quat_to_rpy(quat)
        self._send_external_nav(sitl_t, vertical_est, quat, roll, pitch, yaw)
        self._enforce_extnav_contract()
        json_position = np.asarray(vertical_est.pos_ned, dtype=np.float64).copy()
        json_velocity = np.asarray(vertical_est.vel_ned, dtype=np.float64).copy()
        # ArduSub's JSON backend stores position.z as NED down. AP_Baro_SITL
        # then converts location altitude back into underwater pressure, so
        # Bar30 depth must stay positive-down here.
        json_position[2] = float(vertical_est.depth_m)
        json_velocity[2] = float(vertical_est.vel_ned[2])
        payload = {
            "timestamp": float(sitl_t),
            "altitude": float(vertical_est.alt_m),
            "imu": {
                "gyro": [float(x) for x in gyro],
                "accel_body": [float(x) for x in acc],
            },
            "position": [float(x) for x in json_position],
            "velocity": [float(x) for x in json_velocity],
            "attitude": [float(roll), float(pitch), float(yaw)],
            "quaternion": [float(x) for x in quat],
            "no_time_sync": True,
            "no_lockstep": True,
        }
        if rangefinder_distance_m is not None and np.isfinite(float(rangefinder_distance_m)):
            payload["rng_1"] = float(rangefinder_distance_m)

        arrs = (
            np.array(payload["imu"]["gyro"], dtype=np.float64),
            np.array(payload["imu"]["accel_body"], dtype=np.float64),
            np.array(payload["velocity"], dtype=np.float64),
            np.array(payload["position"], dtype=np.float64),
            np.array(payload["quaternion"], dtype=np.float64),
            np.array(payload["attitude"], dtype=np.float64),
        )
        if not np.isfinite(float(payload["timestamp"])) or any(not np.all(np.isfinite(a)) for a in arrs):
            if not self._sitl_nonfinite_warned:
                self._sitl_nonfinite_warned = True
                print("[sitl_transport] skip SITL packet: non-finite sensor value", flush=True)
            return

        if self._sitl_cmd_debug and now_wall - self._sitl_last_sensor_log_wall >= 2.0:
            self._sitl_last_sensor_log_wall = now_wall
            try:
                print(
                    "[sitl_transport] SITL tx sample "
                    f"t={payload['timestamp']:.3f} "
                    f"pos={payload['position']} vel={payload['velocity']} "
                    f"acc={payload['imu']['accel_body']} "
                    f"depth={vertical_est.depth_m:.3f} "
                    f"abs_alt={payload.get('altitude', float('nan')):.3f} "
                    f"dvl_alt={payload.get('rng_1', float('nan')):.3f} "
                    f"bar30_abs_pa={float(pressure_pa) if pressure_pa is not None else float('nan'):.1f}",
                    flush=True,
                )
            except Exception:
                pass

        target = self._sitl_client_addr if self._sitl_client_addr is not None else self.sitl_send_addr
        try:
            prev_target = self._sitl_send_target
            # ArduSub-4.1.2's lightweight JSON parser expects vector values to
            # start immediately after the colon, e.g. "gyro":[...].
            msg = (json.dumps(payload, separators=(",", ":")) + "\n").encode("utf-8")
            sent = self.sitl_sock.sendto(msg, target)
            self._sitl_send_counter += 1
            self._sitl_send_target = target
            if now_wall - self._sitl_last_send_wall > 5.0:
                if self._sitl_client_addr is None:
                    print(
                        f"[sitl_transport] SITL send target still default (endpoint not discovered yet): {target} "
                        f"packets_sent={self._sitl_send_counter}",
                        flush=True,
                    )
                    self._sitl_last_no_client_wall = now_wall
                elif self._sitl_cmd_debug:
                    print(
                        f"[sitl_transport] SITL send ok (sensor_target={target}, bytes={sent}, "
                        f"packets_sent={self._sitl_send_counter})",
                        flush=True,
                    )
                self._sitl_last_send_wall = now_wall
            if self._sitl_cmd_debug and prev_target is not None and prev_target != target:
                print(f"[sitl_transport] SITL send target changed to servo source: {target}", flush=True)
            if now_wall - self._sitl_last_send_err_wall > 20.0 and sent <= 0:
                print(f"[sitl_transport] SITL send warning: sent {sent} bytes to {target}", flush=True)
                self._sitl_last_send_err_wall = now_wall
        except Exception as exc:
            if now_wall - self._sitl_last_send_err_wall > 2.0:
                print(f"[sitl_transport] SITL send failed to {target}: {exc}", flush=True)
                self._sitl_last_send_err_wall = now_wall
            self._sitl_last_send_wall = now_wall

    def shutdown(self) -> None:
        if self.sitl_sock is not None:
            try:
                self.sitl_sock.close()
            except Exception:
                pass
            self.sitl_sock = None
        if self._sitl_mav is not None:
            try:
                close_fn = getattr(self._sitl_mav, "close", None)
                if callable(close_fn):
                    close_fn()
            except Exception:
                pass
            self._sitl_mav = None
        if self._sitl_cmd_mav is not None:
            try:
                close_fn = getattr(self._sitl_cmd_mav, "close", None)
                if callable(close_fn):
                    close_fn()
            except Exception:
                pass
            self._sitl_cmd_mav = None
