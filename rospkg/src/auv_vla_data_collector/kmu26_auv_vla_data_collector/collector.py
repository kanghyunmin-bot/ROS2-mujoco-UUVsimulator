"""ROS 2 episode recorder for physical-robot U0 fine-tuning data."""

from __future__ import annotations

import json
import hashlib
import time
import os
import shutil
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from auv_dvl_a50_msg.msg import DVL
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn, State, RCIn, RCOut
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .contract import (
    ACTION_NAMES,
    STATE_NAMES,
    RcCommandTracker,
    body_velocity,
    build_state,
    sample_is_fresh,
    validate_sample_times,
)


@dataclass
class Latest:
    value: Any
    source_time: float
    received_time: float
    frame_id: str


def _stamp_to_seconds(message: Any, fallback: float) -> float:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback
    value = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
    return value


class VlaDataCollector(Node):
    def __init__(self) -> None:
        super().__init__("vla_data_collector")
        self._lock = threading.RLock()

        self._dataset_root = Path(
            self.declare_parameter("dataset_root", "~/vla_data/staging").value
        ).expanduser()
        self._rate_hz = float(self.declare_parameter("record_rate_hz", 10.0).value)
        self._max_sensor_age = float(
            self.declare_parameter("max_sensor_age_sec", 0.25).value
        )
        self._max_control_age = float(
            self.declare_parameter("max_control_age_sec", 0.5).value
        )
        self._jpeg_quality = int(self.declare_parameter("jpeg_quality", 95).value)
        self._collection_kind = str(
            self.declare_parameter("collection_kind", "connection_check").value
        )
        self._data_source = str(self.declare_parameter("data_source", "unknown").value)
        self._session_id = str(self.declare_parameter("session_id", "").value)
        self._expected_mode = str(
            self.declare_parameter("expected_mode", "STABILIZE").value
        )
        context_path = str(self.declare_parameter("provenance_file", "").value)
        self._provenance_context = (
            json.loads(Path(context_path).expanduser().read_text())
            if context_path
            else {}
        )
        if self._collection_kind not in ("connection_check", "task_demonstration"):
            raise ValueError("Invalid collection_kind")
        if self._data_source not in ("simulation", "real", "unknown"):
            raise ValueError("Invalid data_source")
        self._code_snapshot = {
            name: Path(__file__).with_name(name).read_text()
            for name in ("collector.py", "contract.py")
        }
        self._vehicle = None
        self._vehicle_at = -float("inf")
        self._episode_vehicle = None
        self._vehicle_samples = []
        self._rc_feedback = {}
        self._receipt_timestamps = []
        self._rc_axis_times = []
        self._clock_last = None
        self._clock_wall = time.monotonic()
        self._default_task = str(self.declare_parameter("default_task", "").value)
        self._depth_positive_down = bool(
            self.declare_parameter("depth_pose_z_is_positive_up", True).value
        )
        self._neutral_pwm = int(self.declare_parameter("neutral_pwm", 1500).value)
        self._pwm_span = int(self.declare_parameter("pwm_span", 300).value)
        action_channels = self.declare_parameter("action_channels", [5, 6, 3, 4]).value
        if self._rate_hz <= 0.0:
            raise ValueError("record_rate_hz must be positive")
        if self._max_sensor_age <= 0.0 or self._max_control_age <= 0.0:
            raise ValueError("Sensor and control age limits must be positive")
        if not 1 <= self._jpeg_quality <= 100:
            raise ValueError("jpeg_quality must be in [1, 100]")
        if len(action_channels) != 4 or any(
            int(value) < 1 or int(value) > 18 for value in action_channels
        ):
            raise ValueError("action_channels must contain [surge, sway, heave, yaw]")
        self._action_channel_indices = tuple(
            int(value) - 1 for value in action_channels
        )
        self._rc_tracker = RcCommandTracker(
            self._action_channel_indices, self._neutral_pwm, self._pwm_span
        )
        self._body_frame = str(self.declare_parameter("body_frame", "base_link").value)
        self._dvl_input_frame = str(
            self.declare_parameter("dvl_input_frame", "dvl_link").value
        )
        self._dvl_convention = str(
            self.declare_parameter("dvl_convention", "FRD").value
        )
        if self._dvl_convention not in ("FRD", "FLU"):
            raise ValueError("dvl_convention must be FRD or FLU")

        ego_topic = str(
            self.declare_parameter(
                "ego_image_topic", "/imx219/camera0/image_raw/compressed"
            ).value
        )
        release_topic = str(
            self.declare_parameter(
                "buoy_release_image_topic",
                "/imx219/camera1/image_raw/compressed",
            ).value
        )
        dvl_twist_topic = str(
            self.declare_parameter("dvl_twist_topic", "/dvl/twist").value
        )
        dvl_data_topic = str(
            self.declare_parameter("dvl_data_topic", "/dvl/data").value
        )
        imu_topic = str(self.declare_parameter("imu_topic", "/mavros/imu/data").value)
        depth_topic = str(self.declare_parameter("depth_topic", "/depth/pose").value)
        rc_topic = str(
            self.declare_parameter("rc_override_topic", "/mavros/rc/override").value
        )
        task_topic = str(
            self.declare_parameter(
                "task_description_topic", "/vla/task_description"
            ).value
        )
        self._topics = {
            "ego_image": ego_topic,
            "buoy_release_image": release_topic,
            "dvl_twist": dvl_twist_topic,
            "dvl_data": dvl_data_topic,
            "imu": imu_topic,
            "depth": depth_topic,
            "rc_override": rc_topic,
            "task_description": task_topic,
        }

        self._ego: Optional[Latest] = None
        self._release: Optional[Latest] = None
        self._dvl_twist: Optional[Latest] = None
        self._dvl_data: Optional[Latest] = None
        self._imu: Optional[Latest] = None
        self._depth: Optional[Latest] = None
        self._control: Optional[Latest] = None
        self._control_command = np.zeros(4, dtype=np.float32)
        self._control_update_mask = np.zeros(4, dtype=np.float32)
        self._task_description = self._default_task

        self._active = False
        self._episode_index = -1
        self._episode_task = ""
        self._recording_dir: Optional[Path] = None
        self._states: list[np.ndarray] = []
        self._actions: list[np.ndarray] = []
        self._rc_pwm: list[np.ndarray] = []
        self._rc_update_masks: list[np.ndarray] = []
        self._ros_timestamps: list[float] = []
        self._source_ages: list[np.ndarray] = []
        self._source_timestamps: list[np.ndarray] = []
        self._previous_sample_command = np.zeros(4, dtype=np.float32)
        self._last_skip_log_time = 0.0

        self.create_subscription(
            CompressedImage, ego_topic, self._on_ego_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CompressedImage,
            release_topic,
            self._on_release_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistWithCovarianceStamped,
            dvl_twist_topic,
            self._on_dvl_twist,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DVL, dvl_data_topic, self._on_dvl_data, qos_profile_sensor_data
        )
        self.create_subscription(Imu, imu_topic, self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped,
            depth_topic,
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(OverrideRCIn, rc_topic, self._on_rc_override, 20)
        self.create_subscription(
            State, "/mavros/state", self._on_vehicle_state, qos_profile_sensor_data
        )
        for topic, message_type in (("/mavros/rc/in", RCIn), ("/mavros/rc/out", RCOut)):
            self.create_subscription(
                message_type,
                topic,
                lambda message, key=topic: self._on_rc_feedback(key, message),
                qos_profile_sensor_data,
            )
        self._clock_watchdog = self.create_timer(
            0.1, self._watch_clock, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        self.create_subscription(String, task_topic, self._on_task_description, 10)

        self.create_service(Trigger, "~/start_episode", self._on_start_episode)
        self.create_service(SetBool, "~/stop_episode", self._on_stop_episode)
        self.create_service(Trigger, "~/discard_episode", self._on_discard_episode)
        self.create_timer(1.0 / self._rate_hz, self._record_sample)

        self._dataset_root.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Dataset staging root: {self._dataset_root}")
        self.get_logger().info(
            f"Action order {ACTION_NAMES}, RC channels {action_channels}"
        )
        self.get_logger().info(
            "Publish a task, then call ~/start_episode to begin recording"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def _interrupt_episode(self, reason: str) -> None:
        if self._active:
            if self._states:
                self._finish_episode(False, reason)
            else:
                self._on_discard_episode(Trigger.Request(), Trigger.Response())

    def _watch_clock(self) -> None:
        with self._lock:
            now, wall = self._now(), time.monotonic()
            backwards = self._clock_last is not None and now < self._clock_last
            if self._clock_last is None or now != self._clock_last:
                self._clock_wall = wall
            if backwards or wall - self._clock_wall > 1.0:
                self._interrupt_episode("clock_reset" if backwards else "clock_stalled")
                for name in (
                    "ego",
                    "release",
                    "imu",
                    "depth",
                    "dvl_data",
                    "dvl_twist",
                    "control",
                ):
                    setattr(self, "_" + name, None)
                self._rc_feedback.clear()
                self._rc_tracker = RcCommandTracker(
                    self._action_channel_indices, self._neutral_pwm, self._pwm_span
                )
            self._clock_last = now

    def _on_rc_feedback(self, key, message):
        with self._lock:
            self._rc_feedback[key] = {
                "source_time": _stamp_to_seconds(message, self._now()),
                "receipt_time": self._now(),
                "channels": list(message.channels),
            }

    def _on_vehicle_state(self, message: State) -> None:
        with self._lock:
            self._vehicle = message
            self._vehicle_at = time.monotonic()
            current = (message.connected, message.armed, message.mode)
            if (
                self._active
                and self._episode_vehicle is not None
                and current != self._episode_vehicle
            ):
                self._interrupt_episode("vehicle_state_changed")

    def _demonstration_ready(self) -> bool:
        state = self._vehicle
        return bool(
            state
            and state.connected
            and state.armed
            and state.mode == self._expected_mode
            and time.monotonic() - self._vehicle_at <= 2.0
            and self.count_publishers(self._topics["rc_override"]) == 1
            and self._data_source != "unknown"
            and self._session_id
            and self._provenance_context
        )

    def _decode_image(self, message: CompressedImage) -> Optional[np.ndarray]:
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            self.get_logger().error("Failed to decode a compressed camera image")
        return image

    def _latest(self, message: Any, value: Any) -> Latest:
        received = self._now()
        header = getattr(message, "header", None)
        frame_id = str(getattr(header, "frame_id", ""))
        return Latest(value, _stamp_to_seconds(message, received), received, frame_id)

    def _on_ego_image(self, message: CompressedImage) -> None:
        image = self._decode_image(message)
        if image is not None:
            with self._lock:
                self._ego = self._latest(message, image)

    def _on_release_image(self, message: CompressedImage) -> None:
        image = self._decode_image(message)
        if image is not None:
            with self._lock:
                self._release = self._latest(message, image)

    def _on_dvl_twist(self, message: TwistWithCovarianceStamped) -> None:
        velocity = message.twist.twist.linear
        try:
            value = body_velocity(
                [velocity.x, velocity.y, velocity.z],
                message.header.frame_id,
                self._dvl_input_frame,
                self._dvl_convention,
            )
        except ValueError as error:
            self.get_logger().warning(str(error))
            with self._lock:
                self._dvl_twist = None
            return
        if np.all(np.isfinite(value)):
            with self._lock:
                self._dvl_twist = self._latest(message, value)

    def _on_dvl_data(self, message: DVL) -> None:
        value = (float(message.altitude), bool(message.velocity_valid))
        with self._lock:
            self._dvl_data = self._latest(message, value)

    def _on_imu(self, message: Imu) -> None:
        if (
            message.header.frame_id != self._body_frame
            or message.orientation_covariance[0] < 0
        ):
            with self._lock:
                self._imu = None
            return
        angular = message.angular_velocity
        linear = message.linear_acceleration
        orientation = message.orientation
        value = (
            np.asarray([angular.x, angular.y, angular.z], dtype=np.float32),
            np.asarray([linear.x, linear.y, linear.z], dtype=np.float32),
            np.asarray(
                [orientation.w, orientation.x, orientation.y, orientation.z],
                dtype=np.float32,
            ),
        )
        with self._lock:
            self._imu = self._latest(message, value)

    def _on_depth(self, message: PoseWithCovarianceStamped) -> None:
        z_value = float(message.pose.pose.position.z)
        depth = -z_value if self._depth_positive_down else z_value
        if np.isfinite(depth):
            with self._lock:
                self._depth = self._latest(message, depth)

    def _on_rc_override(self, message: OverrideRCIn) -> None:
        with self._lock:
            command, update_mask = self._rc_tracker.update(
                message.channels, self._now()
            )
            self._control_command = command
            self._control_update_mask = update_mask
            selected_pwm = np.asarray(
                [
                    (
                        int(message.channels[index])
                        if index < len(message.channels)
                        else 65535
                    )
                    for index in self._action_channel_indices
                ],
                dtype=np.int32,
            )
            self._control = self._latest(message, selected_pwm)

    def _on_task_description(self, message: String) -> None:
        task = message.data.strip()
        if task:
            with self._lock:
                self._task_description = task
            self.get_logger().info(f"Task description set to: {task}")

    def _is_fresh(self, value: Optional[Latest], now: float, max_age: float) -> bool:
        return value is not None and sample_is_fresh(
            value.source_time, value.received_time, now, max_age
        )

    def _missing_start_inputs(self, now: float) -> list[str]:
        checks = {
            "ego camera": self._is_fresh(self._ego, now, self._max_sensor_age),
            "buoy-release camera": self._is_fresh(
                self._release, now, self._max_sensor_age
            ),
            "IMU": self._is_fresh(self._imu, now, self._max_sensor_age),
            "depth": self._is_fresh(self._depth, now, self._max_sensor_age),
            "RC override": self._rc_tracker.fresh(now, self._max_control_age),
        }
        if self._collection_kind == "task_demonstration":
            checks["armed/mode/single RC publisher/provenance"] = (
                self._demonstration_ready()
            )
        return [name for name, ready in checks.items() if not ready]

    def policy_observation(self, previous_command: np.ndarray) -> dict:
        """Build a fresh RGB/body-FLU policy input without requiring an RC publisher.

        Raises:
            ValueError: Required sensors or the instruction are unavailable/stale.
        """
        with self._lock:
            now = self._now()
            for name, value in (
                ("ego", self._ego),
                ("release", self._release),
                ("imu", self._imu),
                ("depth", self._depth),
            ):
                if not self._is_fresh(value, now, self._max_sensor_age):
                    raise ValueError(f"Missing/stale {name}")
            if not self._task_description:
                raise ValueError("Missing task description")
            raw_valid = self._is_fresh(self._dvl_data, now, self._max_sensor_age)
            altitude, velocity_valid = (
                self._dvl_data.value if raw_valid else (0.0, False)
            )
            dvl_valid = bool(
                raw_valid
                and velocity_valid
                and self._is_fresh(self._dvl_twist, now, self._max_sensor_age)
                and abs(self._dvl_twist.source_time - self._dvl_data.source_time)
                <= 1e-6
            )
            altitude_valid = bool(
                raw_valid and velocity_valid and np.isfinite(altitude) and altitude > 0
            )
            angular, linear, attitude = self._imu.value
            state = build_state(
                previous_command,
                self._dvl_twist.value if dvl_valid else np.zeros(3),
                angular,
                linear,
                attitude,
                self._depth.value,
                altitude if altitude_valid else 0.0,
                [1, 1, dvl_valid, altitude_valid],
            )
            slices = {
                "prev_command": (0, 4),
                "dvl_velocity": (4, 7),
                "angular_velocity": (7, 10),
                "linear_acceleration": (10, 13),
                "attitude": (13, 17),
                "depth": (17, 18),
                "altitude": (18, 19),
                "validity": (19, 23),
            }
            observation = {
                f"state.{key}": state[a:b][None].copy()
                for key, (a, b) in slices.items()
            }
            observation.update(
                {
                    "video.ego": cv2.cvtColor(self._ego.value, cv2.COLOR_BGR2RGB)[None],
                    "video.buoy_release": cv2.cvtColor(
                        self._release.value, cv2.COLOR_BGR2RGB
                    )[None],
                    "annotation.human.action.task_description": [
                        self._task_description
                    ],
                }
            )
            return observation

    def _next_episode_index(self) -> int:
        indices = []
        for path in self._dataset_root.glob("episode_*"):
            try:
                indices.append(int(path.name.rsplit("_", 1)[1]))
            except ValueError:
                continue
        return max(indices, default=-1) + 1

    def _on_start_episode(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        with self._lock:
            if self._active:
                response.success = False
                response.message = f"Episode {self._episode_index} is already recording"
                return response
            if not self._task_description:
                response.success = False
                response.message = (
                    "Publish a non-empty task description before recording"
                )
                return response
            missing = self._missing_start_inputs(self._now())
            if missing:
                response.success = False
                response.message = "Inputs are not fresh: " + ", ".join(missing)
                return response

            self._episode_index = self._next_episode_index()
            self._episode_task = self._task_description
            self._recording_dir = self._dataset_root / (
                f".recording_episode_{self._episode_index:06d}"
            )
            if self._recording_dir.exists():
                response.success = False
                response.message = (
                    f"Temporary directory already exists: {self._recording_dir}"
                )
                return response
            (self._recording_dir / "frames" / "ego").mkdir(parents=True)
            (self._recording_dir / "frames" / "buoy_release").mkdir(parents=True)
            self._vehicle_samples.clear()
            self._receipt_timestamps.clear()
            self._rc_axis_times.clear()
            v = self._vehicle
            self._episode_vehicle = (v.connected, v.armed, v.mode) if v else None
            self._states.clear()
            self._actions.clear()
            self._rc_pwm.clear()
            self._rc_update_masks.clear()
            self._ros_timestamps.clear()
            self._source_ages.clear()
            self._source_timestamps.clear()
            self._previous_sample_command = self._control_command.copy()
            self._active = True

            response.success = True
            response.message = (
                f"Started episode {self._episode_index}: {self._episode_task}"
            )
            self.get_logger().info(response.message)
            return response

    def _on_stop_episode(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        with self._lock:
            if not self._active:
                response.success = False
                response.message = "No episode is recording"
                return response
            if not self._states:
                response.success = False
                response.message = (
                    "No samples recorded yet; wait for data or discard the episode"
                )
                return response
            episode_path = self._finish_episode(bool(request.data), "operator_stop")
            response.success = True
            response.message = f"Saved {episode_path}"
            return response

    def _on_discard_episode(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        del request
        with self._lock:
            if not self._active or self._recording_dir is None:
                response.success = False
                response.message = "No episode is recording"
                return response
            target = self._recording_dir
            self._active = False
            self._recording_dir = None
            shutil.rmtree(target)
            response.success = True
            response.message = f"Discarded episode {self._episode_index}"
            self.get_logger().warning(response.message)
            return response

    def _warn_skipped(self, reason: str, now: float) -> None:
        if now - self._last_skip_log_time >= 2.0:
            self.get_logger().warning(f"Skipping sample: {reason}")
            self._last_skip_log_time = now

    def _record_sample(self) -> None:
        with self._lock:
            if not self._active or self._recording_dir is None:
                return
            now = self._now()
            if self._ros_timestamps:
                try:
                    validate_sample_times(
                        [self._ros_timestamps[-1], now], self._rate_hz
                    )
                except ValueError:
                    self._finish_episode(False, "sampling_discontinuity")
                    return
            if (
                self._collection_kind == "task_demonstration"
                and not self._demonstration_ready()
            ):
                self._interrupt_episode("control_context_lost")
                return
            if self._ego is None or self._release is None:
                self._warn_skipped("camera has never produced an image", now)
                return
            if not self._is_fresh(self._imu, now, self._max_sensor_age):
                self._warn_skipped("stale IMU", now)
                return
            if not self._is_fresh(self._depth, now, self._max_sensor_age):
                self._warn_skipped("stale depth", now)
                return
            if not self._rc_tracker.fresh(now, self._max_control_age):
                self._warn_skipped("stale RC override", now)
                return

            ego_valid = self._is_fresh(self._ego, now, self._max_sensor_age)
            release_valid = self._is_fresh(self._release, now, self._max_sensor_age)
            if self._collection_kind == "task_demonstration":
                repeated = bool(
                    self._source_timestamps
                    and (
                        self._ego.source_time <= self._source_timestamps[-1][0]
                        or self._release.source_time <= self._source_timestamps[-1][1]
                    )
                )
                if not ego_valid or not release_valid or repeated:
                    self._interrupt_episode("invalid_or_duplicate_camera")
                    return
            dvl_message_fresh = self._is_fresh(
                self._dvl_data, now, self._max_sensor_age
            )
            dvl_twist_fresh = self._is_fresh(self._dvl_twist, now, self._max_sensor_age)
            altitude, dvl_reported_valid = (
                self._dvl_data.value if self._dvl_data is not None else (0.0, False)
            )
            altitude_valid = bool(
                dvl_message_fresh
                and dvl_reported_valid
                and np.isfinite(altitude)
                and altitude > 0.0
            )
            dvl_valid = bool(
                dvl_twist_fresh
                and dvl_message_fresh
                and dvl_reported_valid
                and abs(self._dvl_twist.source_time - self._dvl_data.source_time)
                <= 1e-6
            )
            dvl_velocity = (
                self._dvl_twist.value if dvl_valid else np.zeros(3, dtype=np.float32)
            )
            altitude_value = float(altitude) if altitude_valid else 0.0
            angular_velocity, linear_acceleration, attitude = self._imu.value

            try:
                state = build_state(
                    self._previous_sample_command,
                    dvl_velocity,
                    angular_velocity,
                    linear_acceleration,
                    attitude,
                    float(self._depth.value),
                    altitude_value,
                    [ego_valid, release_valid, dvl_valid, altitude_valid],
                )
            except ValueError as error:
                self._warn_skipped(str(error), now)
                return

            frame_index = len(self._states)
            encode_options = [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            ego_ok, ego_jpeg = cv2.imencode(".jpg", self._ego.value, encode_options)
            release_ok, release_jpeg = cv2.imencode(
                ".jpg", self._release.value, encode_options
            )
            if not ego_ok or not release_ok:
                self._warn_skipped("JPEG encoding failed", now)
                return

            ego_path = (
                self._recording_dir / "frames" / "ego" / f"frame_{frame_index:06d}.jpg"
            )
            release_path = (
                self._recording_dir
                / "frames"
                / "buoy_release"
                / (f"frame_{frame_index:06d}.jpg")
            )
            ego_path.write_bytes(ego_jpeg.tobytes())
            release_path.write_bytes(release_jpeg.tobytes())

            source_ages = np.asarray(
                [
                    now - self._ego.source_time,
                    now - self._release.source_time,
                    now - self._imu.source_time,
                    now - self._depth.source_time,
                    now - self._dvl_twist.source_time if self._dvl_twist else np.nan,
                    now - self._dvl_data.source_time if self._dvl_data else np.nan,
                    now - self._control.received_time,
                ],
                dtype=np.float32,
            )
            source_timestamps = np.asarray(
                [
                    self._ego.source_time,
                    self._release.source_time,
                    self._imu.source_time,
                    self._depth.source_time,
                    self._dvl_twist.source_time if self._dvl_twist else np.nan,
                    self._dvl_data.source_time if self._dvl_data else np.nan,
                    self._control.source_time,
                ],
                dtype=np.float64,
            )
            self._receipt_timestamps.append(
                np.asarray(
                    [
                        value.received_time if value else np.nan
                        for value in (
                            self._ego,
                            self._release,
                            self._imu,
                            self._depth,
                            self._dvl_twist,
                            self._dvl_data,
                            self._control,
                        )
                    ],
                    dtype=np.float64,
                )
            )
            self._rc_axis_times.append(self._rc_tracker.updated_at.copy())
            v = self._vehicle
            self._vehicle_samples.append(
                {
                    "connected": bool(v and v.connected),
                    "armed": bool(v and v.armed),
                    "mode": v.mode if v else "unknown",
                    "state_receipt_age_wall_s": time.monotonic() - self._vehicle_at
                    if v
                    else None,
                    "rc_feedback": dict(self._rc_feedback),
                    "rc_publishers": self.count_publishers(self._topics["rc_override"]),
                }
            )
            self._states.append(state)
            self._actions.append(self._control_command.copy())
            self._rc_pwm.append(self._control.value.copy())
            self._rc_update_masks.append(self._control_update_mask.copy())
            self._ros_timestamps.append(now)
            self._source_ages.append(source_ages)
            self._source_timestamps.append(source_timestamps)
            self._previous_sample_command = self._control_command.copy()

    def _finish_episode(self, success: bool, termination_reason: str) -> Path:
        assert self._recording_dir is not None
        recording_dir = self._recording_dir
        frame_count = len(self._states)
        if frame_count == 0:
            raise RuntimeError("Cannot save an episode with no samples")

        np.savez_compressed(
            recording_dir / "samples.npz",
            receipt_timestamp=np.stack(self._receipt_timestamps),
            rc_axis_timestamp=np.stack(self._rc_axis_times),
            observation_state=np.stack(self._states).astype(np.float32),
            action=np.stack(self._actions).astype(np.float32),
            rc_pwm=np.stack(self._rc_pwm).astype(np.int32),
            rc_update_mask=np.stack(self._rc_update_masks).astype(np.float32),
            ros_timestamp=np.asarray(self._ros_timestamps, dtype=np.float64),
            source_age=np.stack(self._source_ages).astype(np.float32),
            source_timestamp=np.stack(self._source_timestamps).astype(np.float64),
        )
        code_dir = recording_dir / "collector_source"
        code_dir.mkdir()
        for name, source in self._code_snapshot.items():
            (code_dir / name).write_text(source)
        manifest = {
            "schema_version": 2,
            "provenance": {
                "data_source": self._data_source,
                "collection_kind": self._collection_kind,
                "session_id": self._session_id,
                "context": self._provenance_context,
                "use_sim_time": bool(self.get_parameter("use_sim_time").value),
                "collector_sha256": hashlib.sha256(
                    self._code_snapshot["collector.py"].encode()
                ).hexdigest(),
                "contract_sha256": hashlib.sha256(
                    self._code_snapshot["contract.py"].encode()
                ).hexdigest(),
                "action_channels": [i + 1 for i in self._action_channel_indices],
                "neutral_pwm": self._neutral_pwm,
                "pwm_span": self._pwm_span,
                "expected_mode": self._expected_mode,
                "max_sensor_age_sec": self._max_sensor_age,
                "max_control_age_sec": self._max_control_age,
                "action_semantics": "latest_requested_rc_at_observation; FCU acceptance unverified",
                "dvl_reference": "DVL acoustic origin; axes FLU; lever arm retained",
            },
            "episode_index": self._episode_index,
            "task": self._episode_task,
            "success": success,
            "termination_reason": termination_reason,
            "frames": frame_count,
            "fps": self._rate_hz,
            "created_utc": datetime.now(timezone.utc).isoformat(),
            "state_names": list(STATE_NAMES),
            "action_names": list(ACTION_NAMES),
            "state_conventions": {
                "body_vectors": "FLU",
                "dvl_input_convention": self._dvl_convention,
                "attitude": "quaternion_wxyz",
                "depth": "positive_down_m",
                "action": "normalized_rc_request_minus1_to_plus1; axis signs follow ArduSub",
            },
            "topics": self._topics,
            "source_frames": {
                "ego_image": self._ego.frame_id if self._ego else "",
                "buoy_release_image": self._release.frame_id if self._release else "",
                "dvl_twist": self._dvl_twist.frame_id if self._dvl_twist else "",
                "dvl_data": self._dvl_data.frame_id if self._dvl_data else "",
                "imu": self._imu.frame_id if self._imu else "",
                "depth": self._depth.frame_id if self._depth else "",
            },
        }
        (recording_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n", encoding="utf-8"
        )

        (recording_dir / "vehicle_state.jsonl").write_text(
            "".join(
                json.dumps(row, allow_nan=False) + "\n" for row in self._vehicle_samples
            )
        )
        final_path = self._dataset_root / f"episode_{self._episode_index:06d}"
        os.replace(recording_dir, final_path)
        self._active = False
        self._recording_dir = None
        self.get_logger().info(
            f"Saved episode {self._episode_index}: {frame_count} frames, success={success}"
        )
        return final_path


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = VlaDataCollector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if (
                node._active
            ):  # Preserve an interrupted recording instead of deleting it.
                with node._lock:
                    if node._states:
                        node._finish_episode(False, "node_shutdown")
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
