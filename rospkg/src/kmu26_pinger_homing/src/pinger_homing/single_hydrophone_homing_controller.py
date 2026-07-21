#!/usr/bin/env python3
"""Active homing controller using one PCM channel and vehicle odometry."""

from __future__ import annotations

import json
import math
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String

from single_hydrophone_homing_math import (
    SourceEstimate,
    derive_vehicle_depth_limit,
    estimate_source_from_range_differences,
    estimate_source_from_range_gradient,
    estimate_source_xy_from_absolute_ranges,
    filter_unit_vector,
    limit_heave_by_vehicle_depth,
    select_auto_probe_heave,
    stabilized_yaw_command,
    update_range_success_timer,
    world_vector_to_body_flu,
)


@dataclass(frozen=True)
class RangeSample:
    wall_s: float
    position_world: np.ndarray
    cumulative_change_m: float
    amplitude_range_m: float


class SingleHydrophoneHomingController(Node):
    RC_NEUTRAL = 1500
    RC_SPAN = 400

    def __init__(self) -> None:
        super().__init__("single_hydrophone_homing_controller")
        self.declare_parameter("odometry_topic", "/odometry/filtered")
        self.declare_parameter("vehicle_state_topic", "/mavros/state")
        self.declare_parameter("delta_range_topic", "/audio_phase_estimator/delta_range_m")
        self.declare_parameter("iq_magnitude_topic", "/audio_phase_estimator/iq_magnitude")
        self.declare_parameter("direction_input_topic", "/homing/direction")
        self.declare_parameter("direction_output_topic", "/pinger_homing/direction_body")
        self.declare_parameter("status_topic", "/pinger_homing/status")
        self.declare_parameter("rc_output_topic", "/control/pinger/rc_override")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("forward_max", 0.48)
        self.declare_parameter("max_runtime_s", 180.0)
        self.declare_parameter("yaw_gain", 0.85)
        self.declare_parameter("yaw_rate_damping", 0.18)
        self.declare_parameter("yaw_command_limit", 0.42)
        self.declare_parameter("yaw_deadband_deg", 2.5)
        self.declare_parameter("yaw_slew_rate", 0.90)
        self.declare_parameter("direction_filter_alpha", 0.22)
        self.declare_parameter("align_enter_deg", 28.0)
        self.declare_parameter("align_exit_deg", 10.0)
        self.declare_parameter("probe_scale", 0.28)
        self.declare_parameter("odometry_timeout_s", 2.0)
        self.declare_parameter("audio_timeout_s", 3.0)
        self.declare_parameter("vehicle_disconnect_grace_s", 0.0)
        self.declare_parameter("max_source_z_world", -0.5)
        self.declare_parameter("tank_max_depth_m", 11.0)
        # New mission-prior names. Negative values retain the legacy deep-pool
        # parameters below for existing launch files.
        self.declare_parameter("pinger_expected_depth_m", -1.0)
        self.declare_parameter("vehicle_target_depth_m", -1.0)
        self.declare_parameter("max_vehicle_depth_m", 0.0)
        self.declare_parameter("depth_soft_margin_m", 0.15)
        self.declare_parameter("depth_recovery_heave", 0.12)
        self.declare_parameter("probe_heave", -0.18)
        self.declare_parameter("success_range_m", 0.0)
        self.declare_parameter("success_hold_s", 0.8)
        self.declare_parameter("arrival_radius_m", 1.5)
        self.declare_parameter("arrival_hold_s", 1.0)
        self.declare_parameter("amplitude_range_constant", 0.0)
        self.declare_parameter("pinger_depth_m", 8.85)
        self.declare_parameter("pinger_contact_depth_m", 8.50)
        self.declare_parameter("source_min_x_m", -16.5)
        self.declare_parameter("source_max_x_m", 16.5)
        self.declare_parameter("source_min_y_m", -14.0)
        self.declare_parameter("source_max_y_m", 14.0)

        odometry_topic = str(self.get_parameter("odometry_topic").value)
        vehicle_state_topic = str(self.get_parameter("vehicle_state_topic").value)
        delta_range_topic = str(self.get_parameter("delta_range_topic").value)
        iq_magnitude_topic = str(self.get_parameter("iq_magnitude_topic").value)
        direction_input_topic = str(self.get_parameter("direction_input_topic").value)
        direction_output_topic = str(self.get_parameter("direction_output_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        rc_output_topic = str(self.get_parameter("rc_output_topic").value)
        self._dry_run = bool(self.get_parameter("dry_run").value)
        rate_hz = float(self.get_parameter("rate_hz").value)
        self._control_period_s = 1.0 / max(rate_hz, 1.0)
        self._forward_max = float(np.clip(self.get_parameter("forward_max").value, 0.1, 0.8))
        self._max_runtime_s = float(max(0.0, self.get_parameter("max_runtime_s").value))
        self._yaw_gain = float(np.clip(self.get_parameter("yaw_gain").value, 0.1, 2.0))
        self._yaw_rate_damping = float(
            np.clip(self.get_parameter("yaw_rate_damping").value, 0.0, 1.0)
        )
        self._yaw_command_limit = float(
            np.clip(self.get_parameter("yaw_command_limit").value, 0.10, 0.70)
        )
        self._yaw_deadband_rad = math.radians(
            float(np.clip(self.get_parameter("yaw_deadband_deg").value, 0.0, 10.0))
        )
        self._yaw_slew_rate = float(
            np.clip(self.get_parameter("yaw_slew_rate").value, 0.1, 4.0)
        )
        self._direction_filter_alpha = float(
            np.clip(self.get_parameter("direction_filter_alpha").value, 0.05, 1.0)
        )
        self._align_enter_rad = math.radians(
            float(np.clip(self.get_parameter("align_enter_deg").value, 12.0, 60.0))
        )
        self._align_exit_rad = math.radians(
            float(np.clip(self.get_parameter("align_exit_deg").value, 2.0, 20.0))
        )
        self._align_exit_rad = min(self._align_exit_rad, self._align_enter_rad - math.radians(2.0))
        self._probe_scale = float(np.clip(self.get_parameter("probe_scale").value, 0.08, 0.3))
        self._odometry_timeout_s = float(max(0.2, self.get_parameter("odometry_timeout_s").value))
        self._audio_timeout_s = float(max(0.5, self.get_parameter("audio_timeout_s").value))
        self._vehicle_disconnect_grace_s = float(
            max(0.0, self.get_parameter("vehicle_disconnect_grace_s").value)
        )
        self._max_source_z_world = float(self.get_parameter("max_source_z_world").value)
        self._tank_max_depth_m = float(max(0.0, self.get_parameter("tank_max_depth_m").value))
        self._auto_source_depth = self._tank_max_depth_m > 0.0
        # A far source in a deep competition pool needs more non-coplanar
        # motion than the compact test tank.  Derive that excitation from the
        # only depth input instead of asking for another Z/probe parameter.
        self._probe_duration_scale = (
            1.0 if self._tank_max_depth_m <= 2.0 else 1.5
        )
        self._minimum_probe_legs = 1 if self._tank_max_depth_m <= 2.0 else 2
        self._auto_probe_heave_magnitude = float(
            np.interp(self._tank_max_depth_m, [2.0, 11.0], [0.10, 0.20])
            if self._auto_source_depth
            else 0.10
        )
        legacy_pinger_depth_m = float(max(0.0, self.get_parameter("pinger_depth_m").value))
        legacy_target_depth_m = float(
            max(0.0, self.get_parameter("pinger_contact_depth_m").value)
        )
        expected_depth_m = float(self.get_parameter("pinger_expected_depth_m").value)
        target_depth_m = float(self.get_parameter("vehicle_target_depth_m").value)
        self._pinger_expected_depth_m = 0.0 if self._auto_source_depth else (
            expected_depth_m if expected_depth_m > 0.0 else legacy_pinger_depth_m
        )
        self._vehicle_target_depth_m = 0.0 if self._auto_source_depth else (
            target_depth_m if target_depth_m > 0.0 else legacy_target_depth_m
        )
        explicit_max_vehicle_depth_m = float(
            max(0.0, self.get_parameter("max_vehicle_depth_m").value)
        )
        self._max_vehicle_depth_m = (
            derive_vehicle_depth_limit(self._tank_max_depth_m)
            if self._auto_source_depth
            else explicit_max_vehicle_depth_m
        )
        if self._auto_source_depth:
            self._max_source_z_world = -0.05
        self._depth_soft_margin_m = float(
            max(0.0, self.get_parameter("depth_soft_margin_m").value)
        )
        self._depth_recovery_heave = float(
            np.clip(self.get_parameter("depth_recovery_heave").value, 0.0, 0.4)
        )
        self._probe_heave = float(np.clip(self.get_parameter("probe_heave").value, -0.4, 0.4))
        self._probe_heave_for_state = 0.0
        self._success_range_m = float(max(0.0, self.get_parameter("success_range_m").value))
        self._success_hold_s = float(max(0.1, self.get_parameter("success_hold_s").value))
        self._arrival_radius_m = float(
            max(0.0, self.get_parameter("arrival_radius_m").value)
        )
        self._arrival_hold_s = float(
            max(0.1, self.get_parameter("arrival_hold_s").value)
        )
        self._amplitude_range_constant = float(
            max(0.0, self.get_parameter("amplitude_range_constant").value)
        )
        if (
            not self._auto_source_depth
            and self._max_vehicle_depth_m > 0.0
            and self._vehicle_target_depth_m > self._max_vehicle_depth_m
        ):
            self.get_logger().warning(
                "vehicle target depth exceeds hard limit; clamping target: "
                f"{self._vehicle_target_depth_m:.3f} -> {self._max_vehicle_depth_m:.3f} m"
            )
            self._vehicle_target_depth_m = self._max_vehicle_depth_m
        self._source_min_x_m = float(self.get_parameter("source_min_x_m").value)
        self._source_max_x_m = float(self.get_parameter("source_max_x_m").value)
        self._source_min_y_m = float(self.get_parameter("source_min_y_m").value)
        self._source_max_y_m = float(self.get_parameter("source_max_y_m").value)

        self._position: np.ndarray | None = None
        self._quaternion: np.ndarray | None = None
        self._yaw_rate_rad_s = 0.0
        self._filtered_direction_body: np.ndarray | None = None
        self._last_bearing_rad = 0.0
        self._connected = False
        self._armed = False
        self._last_connected_wall = float("-inf")
        self._last_armed_wall = float("-inf")
        self._last_odom_wall = float("-inf")
        self._last_audio_wall = float("-inf")
        self._raw_delta_history: deque[float] = deque(maxlen=3)
        self._amplitude_range_history: deque[float] = deque(maxlen=9)
        self._cumulative_range_change_m = 0.0
        self._samples: deque[RangeSample] = deque(maxlen=5000)
        self._source_estimate: SourceEstimate | None = None
        self._source_smoothed: np.ndarray | None = None
        self._source_locked: np.ndarray | None = None
        self._original_direction_world: np.ndarray | None = None
        self._last_direction_wall = float("-inf")
        self._last_fit_wall = float("-inf")
        self._state = "WAIT_VEHICLE"
        self._state_started_wall = time.monotonic()
        self._active_started_wall: float | None = None
        self._probe_attempt = 0
        self._range_complete = False
        self._arrival_complete = False
        self._completion_reason = ""
        self._range_success_started_wall: float | None = None
        self._arrival_success_started_wall: float | None = None
        self._last_requested_command = (0.0, 0.0, 0.0, 0.0)
        self._last_command = (0.0, 0.0, 0.0, 0.0)
        self._depth_limit_active = False
        self._depth_recovery_active = False
        self._vehicle_depth_m: float | None = None
        self._best_amplitude_distance = float("inf")
        self._last_range_progress_wall = self._state_started_wall
        self._near_reprobe_count = 0

        self._rc_pub = self.create_publisher(OverrideRCIn, rc_output_topic, 10)
        self._direction_pub = self.create_publisher(
            Vector3Stamped, direction_output_topic, 10
        )
        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(Odometry, odometry_topic, self._on_odometry, qos_profile_sensor_data)
        self.create_subscription(State, vehicle_state_topic, self._on_vehicle_state, 10)
        self.create_subscription(
            Float64, delta_range_topic, self._on_delta_range, 50
        )
        self.create_subscription(
            Float64, iq_magnitude_topic, self._on_iq_magnitude, 50
        )
        self.create_subscription(
            Vector3Stamped,
            direction_input_topic,
            self._on_original_direction,
            10,
        )
        self._timer = self.create_timer(1.0 / max(rate_hz, 1.0), self._control_tick)
        self._status_timer = self.create_timer(0.2, self._publish_status)
        self.get_logger().info(
            "single-hydrophone active homing ready: "
            f"odom={odometry_topic} rc_output={rc_output_topic} "
            f"status={status_topic} dry_run={self._dry_run} "
            f"rate={rate_hz:.1f}Hz forward_max={self._forward_max:.2f} "
            f"yaw_gain={self._yaw_gain:.2f} "
            f"yaw_limit={self._yaw_command_limit:.2f} "
            f"tank_max_depth={self._tank_max_depth_m:.3f}m "
            f"auto_source_depth={self._auto_source_depth} "
            f"pinger_depth={'auto' if self._auto_source_depth else f'{self._pinger_expected_depth_m:.3f}m'} "
            f"vehicle_target_depth={'auto' if self._auto_source_depth else f'{self._vehicle_target_depth_m:.3f}m'} "
            f"max_vehicle_depth={self._max_vehicle_depth_m:.3f}m "
            f"depth_soft_margin={self._depth_soft_margin_m:.3f}m "
            f"probe_heave={'auto' if self._auto_source_depth else f'{self._probe_heave:+.3f}'} "
            f"arrival={self._arrival_radius_m:.3f}m/{self._arrival_hold_s:.2f}s "
            f"calibrated_success_range={self._success_range_m:.3f}m/"
            f"{self._success_hold_s:.2f}s max_runtime={self._max_runtime_s:.1f}s"
        )

    def _on_odometry(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._position = np.array([p.x, p.y, p.z], dtype=np.float64)
        self._quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
        yaw_rate = float(msg.twist.twist.angular.z)
        self._yaw_rate_rad_s = yaw_rate if math.isfinite(yaw_rate) else 0.0
        self._last_odom_wall = time.monotonic()

    def _on_vehicle_state(self, msg: State) -> None:
        now = time.monotonic()
        self._connected = bool(msg.connected)
        self._armed = bool(msg.armed)
        if self._connected:
            self._last_connected_wall = now
        if self._armed:
            self._last_armed_wall = now

    def _on_original_direction(self, msg: Vector3Stamped) -> None:
        direction = np.array([msg.vector.x, msg.vector.y, msg.vector.z], dtype=np.float64)
        norm = float(np.linalg.norm(direction))
        if np.all(np.isfinite(direction)) and norm > 1.0e-6:
            direction /= norm
            if self._original_direction_world is None:
                self._original_direction_world = direction
            else:
                filtered = 0.88 * self._original_direction_world + 0.12 * direction
                filtered_norm = float(np.linalg.norm(filtered))
                if filtered_norm > 1.0e-6:
                    self._original_direction_world = filtered / filtered_norm
            self._last_direction_wall = time.monotonic()

    def _on_delta_range(self, msg: Float64) -> None:
        if self._position is None:
            return
        delta = float(msg.data)
        if not math.isfinite(delta):
            return
        self._raw_delta_history.append(float(np.clip(delta, -0.06, 0.06)))
        filtered = float(np.median(np.asarray(self._raw_delta_history, dtype=np.float64)))
        self._cumulative_range_change_m += filtered
        now = time.monotonic()
        self._last_audio_wall = now
        amplitude_range = (
            float(np.median(np.asarray(self._amplitude_range_history, dtype=np.float64)))
            if self._amplitude_range_history
            else float("nan")
        )
        self._samples.append(
            RangeSample(
                now,
                self._position.copy(),
                self._cumulative_range_change_m,
                amplitude_range,
            )
        )

    def _on_iq_magnitude(self, msg: Float64) -> None:
        magnitude = float(msg.data)
        if (
            self._amplitude_range_constant <= 0.0
            or not math.isfinite(magnitude)
            or magnitude <= 1.0e-6
        ):
            return
        # The constant must be calibrated on the physical hydrophone. The
        # simulator value is 0.325 because coherent I/Q is A/2.
        range_m = (self._amplitude_range_constant / magnitude) ** 2
        if 0.1 < range_m < 80.0:
            self._amplitude_range_history.append(float(range_m))

    def _control_tick(self) -> None:
        now = time.monotonic()
        if self._state in {"COMPLETE", "FAILED_TIMEOUT", "FAILED_ESTIMATE"}:
            self._publish_release()
            return
        if (
            self._max_runtime_s > 0.0
            and self._active_started_wall is not None
            and now - self._active_started_wall > self._max_runtime_s
        ):
            self._transition("FAILED_TIMEOUT")
            self._publish_release()
            return
        if not self._ready(now):
            self._transition("WAIT_VEHICLE")
            self._publish_release()
            return
        if self._state == "WAIT_VEHICLE":
            if self._active_started_wall is None:
                self._active_started_wall = now
            self._transition("PROBE")

        self._maybe_fit_source(now)
        if self._maybe_complete(now):
            self._publish_release()
            return
        if self._state == "PROBE":
            self._publish_phase_direction_if_available(now)
            command = self._probe_command(now)
            self._publish_rc(*command)
            return
        if self._state == "REPROBE":
            self._publish_phase_direction_if_available(now)
            command = self._probe_command(now, mirrored=True)
            self._publish_rc(*command)
            return
        self._approach_command(now)

    def _maybe_complete(self, now: float) -> bool:
        if self._state not in {"ALIGN", "APPROACH", "CONTACT"}:
            self._range_success_started_wall = None
            self._arrival_success_started_wall = None
            return False

        if self._success_range_m > 0.0:
            amplitude_distance = self._current_amplitude_range()
            self._range_success_started_wall, calibrated_complete = update_range_success_timer(
                amplitude_distance,
                threshold_m=self._success_range_m,
                hold_s=self._success_hold_s,
                now_s=now,
                started_s=self._range_success_started_wall,
            )
            if calibrated_complete:
                self._range_complete = True
                self._completion_reason = "calibrated_range"
                self.get_logger().info(
                    "pinger homing calibrated-range success: "
                    f"range={amplitude_distance:.3f}m "
                    f"threshold={self._success_range_m:.3f}m"
                )
                self._transition("COMPLETE")
                return True

        estimated_distance = self._current_estimated_distance()
        estimate_ready = bool(
            self._arrival_radius_m > 0.0
            and self._source_locked is not None
            and self._estimate_usable()
        )
        self._arrival_success_started_wall, arrival_complete = update_range_success_timer(
            estimated_distance if estimate_ready else None,
            threshold_m=self._arrival_radius_m,
            hold_s=self._arrival_hold_s,
            now_s=now,
            started_s=self._arrival_success_started_wall,
        )
        if not arrival_complete:
            return False
        self._arrival_complete = True
        self._completion_reason = "estimated_arrival"
        self.get_logger().info(
            "pinger homing arrival success: "
            f"estimated_range={estimated_distance:.3f}m "
            f"radius={self._arrival_radius_m:.3f}m"
        )
        self._transition("COMPLETE")
        return True

    def _current_estimated_distance(self) -> float | None:
        if self._position is None:
            return None
        control_source = (
            self._source_locked if self._source_locked is not None else self._source_smoothed
        )
        if control_source is None:
            return None
        distance = float(np.linalg.norm(control_source - self._position))
        return distance if math.isfinite(distance) and distance >= 0.0 else None

    def _ready(self, now: float) -> bool:
        return bool(
            (self._dry_run or self._vehicle_armed_effective(now))
            and self._position is not None
            and self._quaternion is not None
            and now - self._last_odom_wall < self._odometry_timeout_s
            and now - self._last_audio_wall < self._audio_timeout_s
        )

    def _connection_grace_active(self, now: float) -> bool:
        return bool(
            self._vehicle_disconnect_grace_s > 0.0
            and not self._connected
            and now - self._last_connected_wall <= self._vehicle_disconnect_grace_s
        )

    def _vehicle_armed_effective(self, now: float) -> bool:
        if self._connected:
            return self._armed
        return bool(
            self._connection_grace_active(now)
            and now - self._last_armed_wall <= self._vehicle_disconnect_grace_s
        )

    def _probe_command(self, now: float, *, mirrored: bool = False) -> tuple[float, float, float, float]:
        elapsed = (now - self._state_started_wall) / self._probe_duration_scale
        sign = -1.0 if mirrored else 1.0
        scale = self._probe_scale
        sequence = (
            (1.2, (0.0, 0.0, 0.0, 0.0)),
            (4.2, (scale, 0.0, 0.0, 0.0)),
            (4.8, (0.0, 0.0, 0.0, 0.0)),
            (7.8, (0.0, sign * scale, 0.0, 0.0)),
            (8.4, (0.0, 0.0, 0.0, 0.0)),
            (10.9, (0.0, 0.0, self._probe_heave_for_state, 0.0)),
            (11.9, (0.0, 0.0, 0.0, 0.0)),
        )
        for end_s, command in sequence:
            if elapsed < end_s:
                return command
        self._maybe_fit_source(now, force=True)
        if self._estimate_usable():
            if self._probe_attempt + 1 < self._minimum_probe_legs:
                self._probe_attempt += 1
                self._transition("REPROBE" if self._state == "PROBE" else "PROBE")
                return (0.0, 0.0, 0.0, 0.0)
            if self._source_locked is None and self._estimate_usable():
                self._source_locked = self._source_smoothed.copy()
                self.get_logger().info(
                    "locked stationary pinger source at "
                    f"({self._source_locked[0]:.3f}, {self._source_locked[1]:.3f}, "
                    f"{self._source_locked[2]:.3f})"
                )
            self._transition("ALIGN")
        elif self._probe_attempt < 3:
            self._probe_attempt += 1
            self._transition("REPROBE" if self._state == "PROBE" else "PROBE")
        else:
            self._transition("FAILED_ESTIMATE")
        return (0.0, 0.0, 0.0, 0.0)

    def _maybe_fit_source(self, now: float, *, force: bool = False) -> None:
        if not force and now - self._last_fit_wall < 0.45:
            return
        self._last_fit_wall = now
        if len(self._samples) < 30:
            return
        selected = list(self._samples)
        if len(selected) > 360:
            indices = np.linspace(0, len(selected) - 1, 360, dtype=np.int64)
            selected = [selected[int(index)] for index in indices]
        positions = np.asarray([sample.position_world for sample in selected], dtype=np.float64)
        changes = np.asarray([sample.cumulative_change_m for sample in selected], dtype=np.float64)
        times = np.asarray([sample.wall_s for sample in selected], dtype=np.float64)
        absolute_ranges = np.asarray([sample.amplitude_range_m for sample in selected], dtype=np.float64)
        initial_source = self._source_smoothed
        if initial_source is None and not self._auto_source_depth:
            seed_direction = (
                self._original_direction_world.copy()
                if self._original_direction_world is not None
                else np.array([1.0, 0.0, -0.5], dtype=np.float64)
            )
            # A moving single sensor has a mirror ambiguity around a nearly
            # horizontal trajectory. The competition pinger is known to be
            # submerged, so choose the underwater branch without using its XY.
            seed_direction[2] = -max(abs(float(seed_direction[2])), 0.45)
            seed_direction /= max(float(np.linalg.norm(seed_direction)), 1.0e-9)
            valid_ranges = absolute_ranges[
                np.isfinite(absolute_ranges) & (absolute_ranges > 0.1) & (absolute_ranges < 80.0)
            ]
            seed_range = float(np.median(valid_ranges)) if len(valid_ranges) else 15.0
            initial_source = positions[0] + seed_range * seed_direction
        estimate = None
        gradient_seed = None
        if self._pinger_expected_depth_m > 0.0:
            gradient_seed = estimate_source_from_range_gradient(
                positions,
                absolute_ranges,
                source_z_world=-self._pinger_expected_depth_m,
            )
        if self._pinger_expected_depth_m > 0.0:
            estimate = estimate_source_xy_from_absolute_ranges(
                positions,
                absolute_ranges,
                source_z_world=-self._pinger_expected_depth_m,
                initial_source_world=(
                    gradient_seed.source_world
                    if gradient_seed is not None
                    else (initial_source if self._source_locked is None else None)
                ),
            )
        if estimate is None:
            estimate = estimate_source_from_range_differences(
                positions,
                changes,
                times,
                initial_source_world=initial_source,
                absolute_ranges_m=absolute_ranges,
                min_source_z_world=(
                    -self._tank_max_depth_m if self._auto_source_depth else None
                ),
                max_source_z_world=self._max_source_z_world,
                fixed_source_z_world=(
                    -self._pinger_expected_depth_m
                    if self._pinger_expected_depth_m > 0.0
                    else None
                ),
            )
        if estimate is None:
            return
        if not self._source_inside_operating_bounds(estimate.source_world):
            if force:
                self.get_logger().warning(
                    "rejected pinger source outside pool safety bounds: "
                    f"({estimate.source_world[0]:.3f}, {estimate.source_world[1]:.3f}, "
                    f"{estimate.source_world[2]:.3f})"
                )
            return
        if self._source_locked is None and self._original_direction_world is not None:
            candidate_horizontal = estimate.source_world[:2] - positions[0, :2]
            phase_horizontal = self._original_direction_world[:2]
            denominator = float(np.linalg.norm(candidate_horizontal) * np.linalg.norm(phase_horizontal))
            if denominator > 1.0e-6:
                alignment = float(np.dot(candidate_horizontal, phase_horizontal) / denominator)
                if alignment < 0.20:
                    return
        if not self._estimate_quality(estimate):
            return
        self._source_estimate = estimate
        if self._source_smoothed is None:
            self._source_smoothed = estimate.source_world.copy()
        else:
            self._source_smoothed = 0.82 * self._source_smoothed + 0.18 * estimate.source_world
        amplitude_distance = self._current_amplitude_range()
        if self._source_locked is not None and amplitude_distance is not None and amplitude_distance < 6.0:
            self._source_locked = 0.65 * self._source_locked + 0.35 * estimate.source_world

    @staticmethod
    def _estimate_quality(estimate: SourceEstimate) -> bool:
        return bool(
            estimate.sample_count >= 30
            and estimate.rms_residual_m < 0.30
            and estimate.condition_number < 2.0e5
            and 0.1 < estimate.initial_range_m < 80.0
        )

    def _estimate_usable(self) -> bool:
        return bool(
            self._source_estimate is not None
            and self._source_smoothed is not None
            and self._estimate_quality(self._source_estimate)
            and self._source_inside_operating_bounds(self._source_smoothed)
        )

    def _source_inside_operating_bounds(self, source_world: np.ndarray) -> bool:
        source = np.asarray(source_world, dtype=np.float64).reshape(3)
        source_z_valid = True
        if self._auto_source_depth:
            source_z_valid = -self._tank_max_depth_m <= float(source[2]) <= -0.05
        return bool(
            np.all(np.isfinite(source))
            and self._source_min_x_m <= float(source[0]) <= self._source_max_x_m
            and self._source_min_y_m <= float(source[1]) <= self._source_max_y_m
            and source_z_valid
        )

    def _direction_usable(self, now: float) -> bool:
        return bool(
            self._original_direction_world is not None
            and now - self._last_direction_wall < 1.0
            and np.linalg.norm(self._original_direction_world[:2]) > 0.05
        )

    def _approach_command(self, now: float) -> None:
        if self._position is None or self._quaternion is None:
            self._publish_release()
            return
        amplitude_distance = self._current_amplitude_range()
        if amplitude_distance is not None:
            if amplitude_distance < self._best_amplitude_distance - 0.12:
                self._best_amplitude_distance = amplitude_distance
                self._last_range_progress_wall = now
            elif (
                self._source_locked is not None
                and self._state == "APPROACH"
                and now - self._state_started_wall > 6.0
                and now - self._last_range_progress_wall > 8.0
                and self._near_reprobe_count < 2
            ):
                self._near_reprobe_count += 1
                self._reset_localization_for_near_probe(now)
                self._publish_release()
                return
        if self._source_locked is not None:
            vector_world = self._source_locked - self._position
            if not self._auto_source_depth:
                vector_world[2] = -self._vehicle_target_depth_m - float(self._position[2])
        elif self._estimate_usable():
            vector_world = self._source_smoothed - self._position
            if not self._auto_source_depth:
                vector_world[2] = -self._vehicle_target_depth_m - float(self._position[2])
        elif self._direction_usable(now):
            if self._auto_source_depth:
                direction_world = self._original_direction_world.copy()
                direction_world /= max(float(np.linalg.norm(direction_world)), 1.0e-9)
                direction_range = amplitude_distance if amplitude_distance is not None else 1.0
                vector_world = direction_world * max(float(direction_range), 1.0)
            else:
                horizontal_direction = self._original_direction_world[:2].copy()
                horizontal_direction /= max(float(np.linalg.norm(horizontal_direction)), 1.0e-9)
                dz = -self._vehicle_target_depth_m - float(self._position[2])
                amplitude_distance = self._current_amplitude_range()
                if amplitude_distance is not None and amplitude_distance > abs(dz):
                    horizontal_range = math.sqrt(max(amplitude_distance**2 - dz**2, 0.25))
                else:
                    horizontal_range = max(abs(dz), 1.0)
                vector_world = np.array(
                    [
                        horizontal_direction[0] * horizontal_range,
                        horizontal_direction[1] * horizontal_range,
                        dz,
                    ],
                    dtype=np.float64,
                )
        else:
            if now - self._state_started_wall > 2.0:
                self._transition("REPROBE")
            self._publish_release()
            return
        estimated_distance = float(np.linalg.norm(vector_world))
        distance = amplitude_distance if amplitude_distance is not None else estimated_distance
        if not math.isfinite(distance) or distance > 80.0:
            self._transition("REPROBE")
            self._publish_release()
            return
        vector_body = world_vector_to_body_flu(vector_world, self._quaternion)
        norm = max(float(np.linalg.norm(vector_body)), 1.0e-9)
        direction_body = filter_unit_vector(
            self._filtered_direction_body,
            vector_body / norm,
            alpha=self._direction_filter_alpha,
        )
        self._filtered_direction_body = direction_body
        self._publish_direction(direction_body)
        bearing = math.atan2(float(direction_body[1]), float(direction_body[0]))
        self._last_bearing_rad = bearing
        yaw = stabilized_yaw_command(
            bearing,
            self._yaw_rate_rad_s,
            self._last_command[3],
            dt_s=self._control_period_s,
            gain=self._yaw_gain,
            rate_damping=self._yaw_rate_damping,
            deadband_rad=self._yaw_deadband_rad,
            command_limit=self._yaw_command_limit,
            slew_rate_per_s=self._yaw_slew_rate,
        )
        heave = float(np.clip(0.75 * direction_body[2], -0.38, 0.38))

        align_threshold = self._align_exit_rad if self._state == "ALIGN" else self._align_enter_rad
        if abs(bearing) > align_threshold:
            self._transition("ALIGN")
            forward = 0.0
        else:
            self._transition("APPROACH")
            if distance > 6.0:
                forward = self._forward_max
            elif distance > 2.0:
                forward = min(self._forward_max, 0.34)
            elif distance > 0.75:
                forward = 0.20
            else:
                self._transition("CONTACT")
                forward = 0.13
                yaw = float(np.clip(yaw, -0.25, 0.25))
                heave = float(np.clip(heave, -0.20, 0.20))
        self._publish_rc(forward, 0.0, heave, yaw)

    def _reset_localization_for_near_probe(self, now: float) -> None:
        self.get_logger().warning(
            "range progress stalled; discarding source lock and probing again"
        )
        self._source_estimate = None
        self._source_smoothed = None
        self._source_locked = None
        self._filtered_direction_body = None
        self._samples.clear()
        self._raw_delta_history.clear()
        self._cumulative_range_change_m = 0.0
        self._best_amplitude_distance = float("inf")
        self._last_range_progress_wall = now
        self._probe_attempt = 0
        self._transition("REPROBE")

    def _current_amplitude_range(self) -> float | None:
        if not self._amplitude_range_history:
            return None
        value = float(np.median(np.asarray(self._amplitude_range_history, dtype=np.float64)))
        return value if math.isfinite(value) and 0.1 < value < 80.0 else None

    def _publish_direction(self, direction_body: np.ndarray) -> None:
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.vector.x, msg.vector.y, msg.vector.z = map(float, direction_body)
        self._direction_pub.publish(msg)

    def _publish_phase_direction_if_available(self, now: float) -> None:
        """Expose the live hydrophone EKF direction while probing."""
        if self._quaternion is None or not self._direction_usable(now):
            return
        direction_world = self._original_direction_world.copy()
        world_norm = float(np.linalg.norm(direction_world))
        if not math.isfinite(world_norm) or world_norm <= 1.0e-9:
            return
        direction_body = world_vector_to_body_flu(
            direction_world / world_norm,
            self._quaternion,
        )
        body_norm = float(np.linalg.norm(direction_body))
        if not math.isfinite(body_norm) or body_norm <= 1.0e-9:
            return
        self._publish_direction(direction_body / body_norm)

    def _publish_rc(
        self,
        forward: float = 0.0,
        lateral: float = 0.0,
        heave: float = 0.0,
        yaw: float = 0.0,
    ) -> None:
        self._last_requested_command = (forward, lateral, heave, yaw)
        if self._dry_run or not self._vehicle_armed_effective(time.monotonic()):
            self._last_command = (0.0, 0.0, 0.0, 0.0)
            msg = OverrideRCIn()
            msg.channels.fill(OverrideRCIn.CHAN_RELEASE)
            self._rc_pub.publish(msg)
            return
        position_z = None
        if (
            self._position is not None
            and time.monotonic() - self._last_odom_wall < self._odometry_timeout_s
        ):
            position_z = float(self._position[2])
        depth_safety = limit_heave_by_vehicle_depth(
            heave,
            vehicle_z_world=position_z,
            max_vehicle_depth_m=self._max_vehicle_depth_m,
            soft_margin_m=self._depth_soft_margin_m,
            recovery_heave=self._depth_recovery_heave,
        )
        heave = depth_safety.command_heave
        self._depth_limit_active = depth_safety.limit_active
        self._depth_recovery_active = depth_safety.recovery_active
        self._vehicle_depth_m = depth_safety.vehicle_depth_m
        self._last_command = (forward, lateral, heave, yaw)
        msg = OverrideRCIn()
        msg.channels.fill(OverrideRCIn.CHAN_NOCHANGE)
        for index in range(8):
            msg.channels[index] = self.RC_NEUTRAL
        msg.channels[2] = self._axis_pwm(heave)
        msg.channels[3] = self._axis_pwm(-yaw)
        msg.channels[4] = self._axis_pwm(forward)
        msg.channels[5] = self._axis_pwm(lateral)
        self._rc_pub.publish(msg)

    def _publish_release(self) -> None:
        self._last_requested_command = (0.0, 0.0, 0.0, 0.0)
        self._last_command = (0.0, 0.0, 0.0, 0.0)
        msg = OverrideRCIn()
        msg.channels.fill(OverrideRCIn.CHAN_RELEASE)
        self._rc_pub.publish(msg)

    @classmethod
    def _axis_pwm(cls, value: float) -> int:
        return int(np.clip(round(cls.RC_NEUTRAL + float(np.clip(value, -1.0, 1.0)) * cls.RC_SPAN), 1100, 1900))

    def _transition(self, state: str) -> None:
        if state == self._state:
            return
        self.get_logger().info(f"homing state: {self._state} -> {state}")
        self._state = state
        self._state_started_wall = time.monotonic()
        if state in {"PROBE", "REPROBE"}:
            vehicle_depth_m = (
                None if self._position is None else max(0.0, -float(self._position[2]))
            )
            self._probe_heave_for_state = (
                select_auto_probe_heave(
                    vehicle_depth_m,
                    max_vehicle_depth_m=self._max_vehicle_depth_m,
                    magnitude=self._auto_probe_heave_magnitude,
                )
                if self._auto_source_depth
                else self._probe_heave
            )
            self.get_logger().info(
                "homing vertical probe: "
                f"heave={self._probe_heave_for_state:+.3f} "
                f"vehicle_depth={vehicle_depth_m if vehicle_depth_m is not None else float('nan'):.3f}m "
                f"limit={self._max_vehicle_depth_m:.3f}m"
            )

    def _publish_status(self) -> None:
        estimate = self._source_estimate
        distance = self._current_estimated_distance()
        source = None
        control_source = self._source_locked if self._source_locked is not None else self._source_smoothed
        if control_source is not None:
            source = [float(value) for value in control_source]
        now = time.monotonic()
        inputs_ready = self._ready(now)
        payload = {
            "state": self._state,
            "dry_run": self._dry_run,
            "control_output_active": (
                not self._dry_run
                and inputs_ready
                and self._state not in {"COMPLETE", "FAILED_TIMEOUT", "FAILED_ESTIMATE"}
            ),
            "inputs_ready": inputs_ready,
            "connected": self._connected or self._connection_grace_active(now),
            "armed": self._vehicle_armed_effective(now),
            "raw_connected": self._connected,
            "raw_armed": self._armed,
            "connection_grace_active": self._connection_grace_active(now),
            "audio_fresh": time.monotonic() - self._last_audio_wall < self._audio_timeout_s,
            "sample_count": len(self._samples),
            "probe_attempt": self._probe_attempt,
            "minimum_probe_legs": self._minimum_probe_legs,
            "estimated_source_world": source,
            "source_locked": self._source_locked is not None,
            "estimated_distance_m": distance,
            "amplitude_distance_m": self._current_amplitude_range(),
            "rms_residual_m": None if estimate is None else estimate.rms_residual_m,
            "condition_number": None if estimate is None else estimate.condition_number,
            "bias_range_rate_mps": None if estimate is None else estimate.bias_range_rate_mps,
            "phase_direction_world": (
                None
                if self._original_direction_world is None
                else [float(value) for value in self._original_direction_world]
            ),
            "control_direction_source": (
                "range_localizer"
                if self._source_locked is not None or self._estimate_usable()
                else ("phase_ekf" if self._direction_usable(time.monotonic()) else "unavailable")
            ),
            "command": {
                "forward": self._last_command[0],
                "lateral": self._last_command[1],
                "heave": self._last_command[2],
                "yaw": self._last_command[3],
            },
            "requested_command": {
                "forward": self._last_requested_command[0],
                "lateral": self._last_requested_command[1],
                "heave": self._last_requested_command[2],
                "yaw": self._last_requested_command[3],
            },
            "depth_safety": {
                "vehicle_depth_m": self._vehicle_depth_m,
                "tank_max_depth_m": self._tank_max_depth_m,
                "auto_source_depth": self._auto_source_depth,
                "pinger_expected_depth_m": self._pinger_expected_depth_m,
                "vehicle_target_depth_m": self._vehicle_target_depth_m,
                "max_vehicle_depth_m": self._max_vehicle_depth_m,
                "soft_margin_m": self._depth_soft_margin_m,
                "probe_heave": self._probe_heave_for_state,
                "limit_active": self._depth_limit_active,
                "recovery_active": self._depth_recovery_active,
            },
            "bearing_error_deg": math.degrees(self._last_bearing_rad),
            "yaw_rate_rad_s": self._yaw_rate_rad_s,
            "range_complete": self._range_complete,
            "arrival_complete": self._arrival_complete,
            "completion_reason": self._completion_reason,
            "arrival_radius_m": self._arrival_radius_m,
            "arrival_hold_s": self._arrival_hold_s,
            "success_range_m": self._success_range_m,
            "success_hold_s": self._success_hold_s,
            "amplitude_range_constant": self._amplitude_range_constant,
            "active_runtime_s": (
                None
                if self._active_started_wall is None
                else max(0.0, now - self._active_started_wall)
            ),
            "max_runtime_s": self._max_runtime_s,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._status_pub.publish(msg)


def main() -> int:
    rclpy.init()
    node = SingleHydrophoneHomingController()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except Exception:
        # Humble can surface SIGTERM-driven context shutdown as RCLError from
        # wait_set instead of ExternalShutdownException. Preserve real runtime
        # errors, but treat an already-invalid ROS context as normal shutdown.
        if rclpy.ok():
            raise
    finally:
        try:
            node._publish_release()
        except (Exception, KeyboardInterrupt):
            pass
        try:
            node.destroy_node()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (ExternalShutdownException, KeyboardInterrupt):
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
