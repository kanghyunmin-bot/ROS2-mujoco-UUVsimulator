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
        self._sitl_mav_last_hb_wall = -1.0
        self._sitl_mav_last_msg_wall = -1.0
        self._sitl_mav_last_req_wall = -1.0
        self._sitl_mav_last_wait_warn_wall = -1.0
        self._sitl_mav_target_mismatch_warn_wall = -1.0
        self._sitl_mav_wait_warn_interval_s = 3.0

        self._connect_sitl()
        self._connect_sitl_mavlink()

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
        """Simple vertical truth model for SITL standard path."""
        if base_pos_enu is None or base_vel_enu is None:
            return None
        if not np.all(np.isfinite(base_pos_enu)) or not np.all(np.isfinite(base_vel_enu)):
            return None

        base_depth_m = float(max(0.0, -float(base_pos_enu[2])))
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

    def _resolve_mav_target(self) -> tuple[int, int] | None:
        if self._sitl_mav is None:
            return None
        target_sys = int(self._sitl_mavlink_target_sysid)
        target_comp = int(self._sitl_mavlink_target_compid)
        if target_sys <= 0 or target_comp <= 0:
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
            self._sitl_mav.target_system = target_sys
            self._sitl_mav.target_component = target_comp
        except Exception:
            pass
        return target_sys, target_comp

    def send_rc_override(self, pwm_values: list[int]) -> bool:
        """Forward MAVROS-style RC override to ArduSub SITL over MAVLink."""
        if self._sitl_mav is None:
            return False
        target = self._resolve_mav_target()
        if target is None:
            return False
        target_sys, target_comp = target
        values = list(pwm_values[:8])
        if len(values) < 8:
            values.extend([65535] * (8 - len(values)))
        try:
            self._sitl_mav.mav.rc_channels_override_send(
                int(target_sys),
                int(target_comp),
                *(int(v) for v in values[:8]),
            )
            return True
        except Exception as exc:
            print(f"[sitl_transport] RC override send failed: {exc}", flush=True)
            return False

    def send_arm_command(self, arm: bool) -> bool:
        """Forward arm/disarm request to ArduSub SITL over MAVLink."""
        if self._sitl_mav is None or self._sitl_mavutil is None:
            return False
        target = self._resolve_mav_target()
        if target is None:
            return False
        target_sys, target_comp = target
        def send_once(force: bool = False) -> None:
            self._sitl_mav.mav.command_long_send(
                int(target_sys),
                int(target_comp),
                int(self._sitl_mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM),
                0,
                1.0 if arm else 0.0,
                21196.0 if (arm and force) else 0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

        def heartbeat_matches() -> bool:
            try:
                return bool(self._sitl_mav.motors_armed()) == bool(arm)
            except Exception:
                return False

        def wait_arm_result(timeout_s: float) -> bool:
            deadline = time.monotonic() + float(timeout_s)
            arm_command = int(self._sitl_mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM)
            accepted = int(self._sitl_mavutil.mavlink.MAV_RESULT_ACCEPTED)
            while time.monotonic() < deadline:
                try:
                    msg = self._sitl_mav.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=False)
                except Exception:
                    msg = None
                if msg is not None:
                    msg_type = str(msg.get_type())
                    if msg_type == "HEARTBEAT":
                        self._sitl_mav_hb = msg
                        if heartbeat_matches():
                            return True
                    elif msg_type == "COMMAND_ACK":
                        command = int(getattr(msg, "command", -1))
                        result = int(getattr(msg, "result", -1))
                        if command == arm_command and result == accepted:
                            return True
                time.sleep(0.02)
            return heartbeat_matches()

        try:
            send_once(force=False)
            if wait_arm_result(1.0):
                print(f"[sitl_transport] arm state confirmed: armed={bool(arm)}", flush=True)
                return True
            if arm:
                print("[sitl_transport] normal arm not confirmed; retrying force arm", flush=True)
                send_once(force=True)
                if wait_arm_result(2.0):
                    print("[sitl_transport] force arm confirmed", flush=True)
                    return True
            print(
                f"[sitl_transport] arm command sent but local confirmation was not observed: "
                f"requested armed={bool(arm)}",
                flush=True,
            )
            return True
        except Exception as exc:
            print(f"[sitl_transport] arm/disarm send failed: {exc}", flush=True)
            return False

    def send_set_mode(self, mode: str) -> bool:
        """Forward custom mode request to ArduSub SITL over MAVLink."""
        if self._sitl_mav is None:
            return False
        target = self._resolve_mav_target()
        if target is None:
            return False
        try:
            self._sitl_mav.set_mode_apm(str(mode))
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
        if self._sitl_mav is None or self._sitl_mavutil is None:
            return False
        target = self._resolve_mav_target()
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
            self._sitl_mav.mav.set_position_target_local_ned_send(
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
        if self._sitl_mav is None:
            return False
        target = self._resolve_mav_target()
        if target is None:
            return False
        target_sys, target_comp = target
        try:
            self._sitl_mav.mav.set_position_target_local_ned_send(
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
        got_any = False
        while True:
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
                    self._sitl_mav_hb = msg
                    self._sitl_mav_last_hb_wall = now_wall
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
            self._handle_pwm_values(pwm_values, now_wall, source="mavlink")

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

    def _poll_servo_endpoint(self) -> None:
        if not self.sitl_sock:
            return

        now_wall = time.monotonic()
        while True:
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
            self._handle_pwm_values(pwm_values, now_wall, source="json")

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
        self._poll_servo_mavlink()
        self._poll_servo_endpoint()

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
        payload = {
            "timestamp": float(sitl_t),
            "altitude": float(vertical_est.alt_m),
            "imu": {
                "gyro": [float(x) for x in gyro],
                "accel_body": [float(x) for x in acc],
            },
            "position": [float(x) for x in vertical_est.pos_ned],
            "velocity": [float(x) for x in vertical_est.vel_ned],
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
            msg = (json.dumps(payload) + "\n").encode("utf-8")
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
