import json
import math
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field

import cv2
import numpy as np
import rclpy
from auv_msg.msg import AuvSetpoint
from auv_msg.msg import MissionStatus
from cv_bridge import CvBridge
from auv_dvl_a50_msg.msg import DVL
from auv_dvl_a50_msg.msg import CommandResponse
from auv_dvl_a50_msg.msg import ConfigCommand
from auv_dvl_a50_msg.msg import ConfigStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn
from geometry_msgs.msg import Vector3Stamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from robot_localization.srv import SetPose
from rclpy.executors import ExternalShutdownException
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import UInt8


PATH_MIN_DISTANCE_M = 0.01
WEB_CONTROL_TIMEOUT_S = 0.35
WEB_CONTROL_PERIOD_S = 0.05
DVL_MAX_FOM = 0.05
DVL_MIN_ALTITUDE_M = 0.05
DVL_MIN_VALID_BEAMS = 4
DVL_TWIST_STALE_AFTER_S = 0.75
DVL_CALIBRATION_TIMEOUT_S = 20.0
ATTITUDE_MAX_TILT_DEG = 10.0
STILLNESS_MAX_SPEED_MPS = 0.05
READY_MODES = {"ALT_HOLD", "POSHOLD", "GUIDED"}
DEFAULT_VISION_FRAME_TOPIC = "/vision/yolo/annotated/compressed"
COMPRESSED_IMAGE_TYPE = "sensor_msgs/msg/CompressedImage"
RAW_IMAGE_TYPE = "sensor_msgs/msg/Image"
VISION_IMAGE_TYPES = (COMPRESSED_IMAGE_TYPE, RAW_IMAGE_TYPE)
VISION_RAW_FRAME_MIN_INTERVAL_S = 0.1
VISION_RAW_FRAME_JPEG_QUALITY = 80


def _supported_image_type(topic_types: object) -> str:
    types = topic_types if isinstance(topic_types, (list, tuple)) else []
    return next((item for item in VISION_IMAGE_TYPES if item in types), "")


def _vision_image_topic_options(
    topic_names_and_types: dict[str, list[str]],
    selected_topic: str,
    selected_type: str,
) -> list[dict]:
    options = []
    for topic in sorted(topic_names_and_types):
        topic_type = _supported_image_type(topic_names_and_types[topic])
        if topic_type:
            options.append(
                {"topic": topic, "type": topic_type, "available": True}
            )
    if selected_topic and not any(
        item["topic"] == selected_topic for item in options
    ):
        options.append(
            {
                "topic": selected_topic,
                "type": selected_type,
                "available": False,
            }
        )
        options.sort(key=lambda item: item["topic"])
    return options


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_yaw(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def _start_forward_pose(
    x: float,
    y: float,
    yaw: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
) -> dict[str, float]:
    """Convert odom pose to a display frame with start-forward pointing up."""
    dx = x - origin_x
    dy = y - origin_y
    cosine = math.cos(origin_yaw)
    sine = math.sin(origin_yaw)
    return {
        # Display +x is the vehicle's right side at initialization.
        "x": sine * dx - cosine * dy,
        # Display +y is the vehicle's forward direction at initialization.
        "y": cosine * dx + sine * dy,
        "yaw": _wrap_yaw(yaw - origin_yaw),
    }


def _rpy_from_quaternion(x: float, y: float, z: float, w: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    return roll, pitch, _yaw_from_quaternion(x, y, z, w)


def _quaternion_from_yaw(yaw: float) -> dict[str, float]:
    half_yaw = yaw * 0.5
    return {
        "x": 0.0,
        "y": 0.0,
        "z": math.sin(half_yaw),
        "w": math.cos(half_yaw),
    }


def _finite_or_none(value: float) -> float | None:
    return value if math.isfinite(value) else None


def _dvl_quality_state(msg: DVL, valid_beams: int) -> tuple[bool, str]:
    if not bool(msg.velocity_valid):
        return False, "velocity invalid"
    if not math.isfinite(float(msg.fom)) or float(msg.fom) > DVL_MAX_FOM:
        return False, f"FOM {float(msg.fom):.3f} > {DVL_MAX_FOM:.3f}"
    if not math.isfinite(float(msg.altitude)) or float(msg.altitude) < DVL_MIN_ALTITUDE_M:
        return False, f"altitude {float(msg.altitude):.2f} m < {DVL_MIN_ALTITUDE_M:.2f} m"
    if valid_beams < DVL_MIN_VALID_BEAMS:
        return False, f"{valid_beams} valid beams < {DVL_MIN_VALID_BEAMS}"
    return True, "DVL good"


@dataclass
class TopicHealth:
    name: str
    stale_after: float = 1.0
    last_seen: float | None = None
    stamps: deque[float] = field(default_factory=lambda: deque(maxlen=120))

    def tick(self) -> None:
        now = time.monotonic()
        self.last_seen = now
        self.stamps.append(now)

    def snapshot(self) -> dict:
        now = time.monotonic()
        age = None if self.last_seen is None else now - self.last_seen
        return {
            "name": self.name,
            "alive": age is not None and age <= self.stale_after,
            "age": age,
            "hz": self._hz(now),
        }

    def _hz(self, now: float) -> float:
        recent = [stamp for stamp in self.stamps if now - stamp <= 2.0]
        if len(recent) < 2:
            return 0.0
        return (len(recent) - 1) / (recent[-1] - recent[0])


@dataclass(frozen=True)
class TopicConfig:
    odom: str = "/odometry/filtered"
    depth: str = "/depth/pose"
    mavros_state: str = "/mavros/state"
    pinger_homing_status: str = "/pinger_homing/status"
    hydrophone_direction: str = "/homing/direction"
    delta_range: str = "/audio_phase_estimator/delta_range_m"
    iq_magnitude: str = "/audio_phase_estimator/iq_magnitude"
    rc_mux_status: str = "/control/rc_override_mux/status"
    rc_output: str = "/mavros/rc/override"
    audio: str = "/audio"


class LocalizationRosNode(Node):
    def __init__(self, topic_config: TopicConfig | None = None):
        super().__init__("auv_web_gui_bridge")
        self._topic_config = topic_config or TopicConfig()
        self._lock = threading.Lock()
        self._health = {
            "odom": TopicHealth(self._topic_config.odom),
            "dvl": TopicHealth("/dvl/twist"),
            "dvl_data": TopicHealth("/dvl/data"),
            "depth": TopicHealth(self._topic_config.depth),
            "imu": TopicHealth("/mavros/imu/data"),
            "mavros_state": TopicHealth(self._topic_config.mavros_state, stale_after=2.0),
            "joy": TopicHealth("/joy", stale_after=0.5),
            "battery": TopicHealth("/battery", stale_after=3.0),
            "vision_camera": TopicHealth(
                DEFAULT_VISION_FRAME_TOPIC, stale_after=1.5
            ),
            "vision_bbox": TopicHealth("/vision/buoy_bbox", stale_after=1.0),
            "vision_mission_enable": TopicHealth(
                "/mission/control_enable", stale_after=5.0
            ),
            "vision_mission_state": TopicHealth("/mission/state", stale_after=3.0),
            "vision_rc_command": TopicHealth("/mission/rc_command", stale_after=1.0),
            "pinger_homing": TopicHealth(
                self._topic_config.pinger_homing_status, stale_after=1.5
            ),
            "hydrophone_direction": TopicHealth(
                self._topic_config.hydrophone_direction, stale_after=1.5
            ),
            "delta_range": TopicHealth(self._topic_config.delta_range, stale_after=1.5),
            "iq_magnitude": TopicHealth(self._topic_config.iq_magnitude, stale_after=1.5),
            "rc_mux": TopicHealth(self._topic_config.rc_mux_status, stale_after=1.0),
        }
        self._pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        self._velocity = {"x": 0.0, "y": 0.0, "z": 0.0}
        self._depth = {"z": 0.0}
        self._frames = {
            "odom": {"frame_id": "", "child_frame_id": ""},
            "depth": {"frame_id": ""},
        }
        self._attitude = {
            "roll_deg": None,
            "pitch_deg": None,
            "yaw_deg": None,
            "tilt_deg": None,
            "updated_at": "",
        }
        self._dvl_quality = {
            "good": False,
            "reason": "no DVL data",
            "velocity_valid": False,
            "fom": None,
            "altitude": None,
            "valid_beams": 0,
            "speed": None,
            "updated_at": "",
        }
        self._dvl_quality_samples: deque[dict] = deque(maxlen=240)
        self._tilt_samples: deque[dict] = deque(maxlen=240)
        self._dvl_twist_samples: deque[dict] = deque(maxlen=240)
        self._battery = {
            "voltage": None,
            "current": None,
            "temperature": None,
            "percentage": None,
            "present": False,
        }
        self._mavros_state = {
            "connected": False,
            "armed": False,
            "guided": False,
            "manual_input": False,
            "mode": "",
            "system_status": None,
            "updated_at": "",
        }
        self._guided = {
            "status": "Waiting for guided_navigation",
            "arrived": False,
            "mission_status": MissionStatus.READY,
            "mission_status_label": "READY",
            "active_target": None,
            "start_frame": None,
            "last_command": None,
            "updated_at": "",
        }
        self._joy = {"axes": [], "buttons": []}
        self._dvl_config: dict = {}
        self._dvl_calibration = {
            "operation_id": 0,
            "state": "idle",
            "message": "Ready",
            "success": None,
            "error_message": "",
            "started_at": "",
            "completed_at": "",
            "timeout_s": DVL_CALIBRATION_TIMEOUT_S,
            "deadline_monotonic": None,
        }
        self._pinger_homing_status = {
            "raw": "",
            "state": "",
            "dry_run": True,
            "control_output_active": False,
            "estimated_distance_m": None,
            "amplitude_distance_m": None,
            "bearing_error_deg": None,
        }
        self._hydrophone_direction = {
            "x": None,
            "y": None,
            "z": None,
            "bearing_rad": None,
        }
        self._delta_range_m: float | None = None
        self._iq_magnitude: float | None = None
        self._rc_mux_status = {
            "owner": "unknown",
            "conflict": False,
            "output_enabled": False,
            "publisher_count": 0,
        }
        self._dvl_events: deque[dict] = deque(maxlen=40)
        self._path: list[dict[str, float]] = []
        self._path_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
        self._path_frame = {
            "aligned": False,
            "origin_x": None,
            "origin_y": None,
            "origin_yaw": None,
            "x_axis": "right",
            "y_axis": "start_forward",
        }
        self._last_external_nav_reset_counter: int | None = None
        self._web_control = {
            "enabled": False,
            "active": False,
            "axes": {
                "forward": 0.0,
                "lateral": 0.0,
                "vertical": 0.0,
                "yaw": 0.0,
            },
            "last_command": 0.0,
            "last_publish": "",
            "neutral_burst": 0,
        }
        self._vision = {
            "frame_sequence": 0,
            "frame_stamp": 0.0,
            "frame_width": 0,
            "frame_height": 0,
            "frame_topic": DEFAULT_VISION_FRAME_TOPIC,
            "frame_type": COMPRESSED_IMAGE_TYPE,
            "frame_error": "",
            "detections": {},
            "mission_enabled": False,
            "mission_state": "UNKNOWN",
            "rc_channels": [],
        }
        self._vision_frame_data = b""
        self._vision_frame_content_type = "image/jpeg"
        self._vision_subscription_generation = 0
        self._vision_raw_encode_pending = False
        self._vision_last_raw_frame_at = 0.0
        self._vision_frame_encoder = ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="vision-frame-encoder",
        )
        self._cv_bridge = CvBridge()
        self._vision_frame_subscription = None

        sensor_qos = qos_profile_sensor_data
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._dvl_config_pub = self.create_publisher(
            ConfigCommand,
            "/dvl/config/command",
            command_qos,
        )
        self._web_joy_pub = self.create_publisher(Joy, "/joy", 10)
        self._guided_goal_pub = self.create_publisher(
            AuvSetpoint, "/guided/goal", command_qos
        )
        self._guided_cancel_pub = self.create_publisher(
            Empty, "/guided/cancel", command_qos
        )
        self._guided_waypoint_enable_pub = self.create_publisher(
            Bool, "/guided/waypoint_enable", command_qos
        )
        self._guided_recapture_pub = self.create_publisher(
            Empty, "/guided/recapture_start_frame", command_qos
        )
        self._vision_enable_pub = self.create_publisher(
            Bool, "/mission/control_enable", 10
        )
        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._set_pose_client = self.create_client(SetPose, "/set_pose")
        self.create_subscription(Odometry, self._topic_config.odom, self._on_odom, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, "/dvl/twist", self._on_dvl, sensor_qos)
        self.create_subscription(DVL, "/dvl/data", self._on_dvl_data, sensor_qos)
        self.create_subscription(
            CommandResponse,
            "/dvl/command/response",
            self._on_dvl_response,
            command_qos,
        )
        self.create_subscription(
            ConfigStatus,
            "/dvl/config/status",
            self._on_dvl_config,
            command_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._topic_config.depth,
            self._on_depth,
            sensor_qos,
        )
        self.create_subscription(BatteryState, "/battery", self._on_battery, sensor_qos)
        self.create_subscription(State, self._topic_config.mavros_state, self._on_mavros_state, 20)
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, sensor_qos)
        self.create_subscription(
            UInt8,
            "/mavros/odometry/reset_counter",
            self._on_external_nav_reset,
            state_qos,
        )
        self.create_subscription(
            String, "/guided/status", self._on_guided_status, state_qos
        )
        self.create_subscription(
            Bool, "/guided/arrived", self._on_guided_arrived, state_qos
        )
        self.create_subscription(
            MissionStatus,
            "/guided/mission_status",
            self._on_guided_mission_status,
            state_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/guided/active_target",
            self._on_guided_active_target,
            state_qos,
        )
        self.create_subscription(
            PoseStamped,
            "/guided/start_frame",
            self._on_guided_start_frame,
            state_qos,
        )
        self.create_subscription(Joy, "/joy", self._on_joy, 20)
        self._vision_frame_subscription = self._create_vision_frame_subscription(
            DEFAULT_VISION_FRAME_TOPIC,
            COMPRESSED_IMAGE_TYPE,
            self._vision_subscription_generation,
            sensor_qos,
        )
        self.create_subscription(
            Float32MultiArray, "/vision/buoy_bbox", self._on_vision_bbox, 20
        )
        self.create_subscription(
            Bool, "/mission/control_enable", self._on_vision_enable, 10
        )
        self.create_subscription(
            String, "/mission/state", self._on_vision_state, state_qos
        )
        self.create_subscription(
            OverrideRCIn,
            "/mission/rc_command",
            self._on_vision_rc_command,
            20,
        )
        self.create_subscription(
            String,
            self._topic_config.pinger_homing_status,
            self._on_pinger_homing,
            20,
        )
        self.create_subscription(
            Vector3Stamped,
            self._topic_config.hydrophone_direction,
            self._on_hydrophone_direction,
            20,
        )
        self.create_subscription(
            Float64, self._topic_config.delta_range, self._on_delta_range, 50
        )
        self.create_subscription(
            Float64, self._topic_config.iq_magnitude, self._on_iq_magnitude, 50
        )
        self.create_subscription(
            String, self._topic_config.rc_mux_status, self._on_rc_mux_status, 20
        )
        self.create_timer(WEB_CONTROL_PERIOD_S, self._publish_web_control)

    def publish_dvl_command(
        self,
        command: str,
        parameter_name: str = "",
        parameter_value: str = "",
    ) -> None:
        with self._lock:
            self._expire_dvl_calibration_locked(time.monotonic())
            if self._dvl_calibration["state"] == "calibrating":
                raise RuntimeError("DVL gyro calibration is in progress")
        self._publish_dvl_command(command, parameter_name, parameter_value)

    def _publish_dvl_command(
        self,
        command: str,
        parameter_name: str = "",
        parameter_value: str = "",
    ) -> None:
        msg = ConfigCommand()
        msg.command = command
        msg.parameter_name = parameter_name
        msg.parameter_value = parameter_value
        self._dvl_config_pub.publish(msg)
        self._append_dvl_event(
            "sent",
            command,
            parameter_name,
            parameter_value,
            None,
            "",
        )

    def start_dvl_gyro_calibration(
        self,
        timeout_s: float = DVL_CALIBRATION_TIMEOUT_S,
    ) -> dict:
        subscriber_count = self._dvl_config_pub.get_subscription_count()
        if subscriber_count <= 0:
            message = "DVL command subscriber is not available; start the DVL stack first"
            with self._lock:
                self._set_dvl_calibration_result_locked(
                    state="failed",
                    success=False,
                    error_message=message,
                )
            raise RuntimeError(message)

        now = time.monotonic()
        with self._lock:
            self._expire_dvl_calibration_locked(now)
            if self._dvl_calibration["state"] == "calibrating":
                raise RuntimeError("DVL gyro calibration is already in progress")
            operation_id = int(self._dvl_calibration["operation_id"]) + 1
            self._dvl_calibration = {
                "operation_id": operation_id,
                "state": "calibrating",
                "message": "Calibrating gyro; keep the vehicle completely still",
                "success": None,
                "error_message": "",
                "started_at": time.strftime("%H:%M:%S"),
                "completed_at": "",
                "timeout_s": float(timeout_s),
                "deadline_monotonic": now + float(timeout_s),
            }

        try:
            self._publish_dvl_command("calibrate_gyro")
        except Exception as exc:
            with self._lock:
                self._set_dvl_calibration_result_locked(
                    state="failed",
                    success=False,
                    error_message=f"Failed to publish calibration command: {exc}",
                )
            raise
        return self.dvl_calibration_status()

    def dvl_calibration_status(self) -> dict:
        with self._lock:
            self._expire_dvl_calibration_locked(time.monotonic())
            return self._dvl_calibration_snapshot_locked()

    def snapshot(self) -> dict:
        topic_names_and_types = dict(self.get_topic_names_and_types())
        rc_output_publishers = self.get_publishers_info_by_topic(
            self._topic_config.rc_output
        )
        audio_publishers = self.get_publishers_info_by_topic(self._topic_config.audio)
        graph = {
            "rc_output_publishers": len(rc_output_publishers),
            "rc_output_publisher_nodes": _endpoint_nodes(rc_output_publishers),
            "audio_publishers": len(audio_publishers),
            "audio_publisher_nodes": _endpoint_nodes(audio_publishers),
            "dvl_command_subscribers": self._dvl_config_pub.get_subscription_count(),
            "guided_goal_subscribers": self._guided_goal_pub.get_subscription_count(),
            "topic_types": {
                "odom": list(topic_names_and_types.get(self._topic_config.odom, [])),
                "depth": list(topic_names_and_types.get(self._topic_config.depth, [])),
                "mavros_state": list(
                    topic_names_and_types.get(self._topic_config.mavros_state, [])
                ),
                "audio": list(topic_names_and_types.get(self._topic_config.audio, [])),
            },
            "services": {
                "arming": self._arm_client.service_is_ready(),
                "set_mode": self._set_mode_client.service_is_ready(),
            },
        }
        with self._lock:
            vision_snapshot = self._vision_snapshot_locked()
            vision_snapshot["image_topics"] = _vision_image_topic_options(
                topic_names_and_types,
                str(self._vision["frame_topic"]),
                str(self._vision["frame_type"]),
            )
            return {
                "config": {
                    "topics": {
                        "odom": self._topic_config.odom,
                        "depth": self._topic_config.depth,
                        "mavros_state": self._topic_config.mavros_state,
                        "pinger_homing_status": self._topic_config.pinger_homing_status,
                        "hydrophone_direction": self._topic_config.hydrophone_direction,
                        "delta_range": self._topic_config.delta_range,
                        "iq_magnitude": self._topic_config.iq_magnitude,
                        "rc_mux_status": self._topic_config.rc_mux_status,
                        "rc_output": self._topic_config.rc_output,
                        "audio": self._topic_config.audio,
                    }
                },
                "topics": {name: item.snapshot() for name, item in self._health.items()},
                "pose": dict(self._pose),
                "velocity": dict(self._velocity),
                "depth": dict(self._depth),
                "frames": {
                    "odom": dict(self._frames["odom"]),
                    "depth": dict(self._frames["depth"]),
                },
                "attitude": dict(self._attitude),
                "dvl_quality": dict(self._dvl_quality),
                "precheck": self._precheck_snapshot_locked(),
                "battery": dict(self._battery),
                "joy": {
                    "axes": list(self._joy["axes"]),
                    "buttons": list(self._joy["buttons"]),
                },
                "mavros_state": dict(self._mavros_state),
                "guided": {
                    **self._guided,
                    "available": self._guided_goal_pub.get_subscription_count() > 0,
                },
                "dvl_config": dict(self._dvl_config),
                "dvl_calibration": self._dvl_calibration_snapshot_locked(),
                "dvl_events": list(self._dvl_events),
                "path": list(self._path),
                "path_count": len(self._path),
                "path_pose": dict(self._path_pose),
                "path_frame": dict(self._path_frame),
                "web_control": self._web_control_snapshot(),
                "vision": vision_snapshot,
                "pinger_homing_status": dict(self._pinger_homing_status),
                "hydrophone_direction": dict(self._hydrophone_direction),
                "delta_range_m": self._delta_range_m,
                "iq_magnitude": self._iq_magnitude,
                "rc_mux_status": dict(self._rc_mux_status),
                "graph": graph,
            }

    def latest_vision_frame(self) -> tuple[bytes, str, int, str]:
        with self._lock:
            return (
                self._vision_frame_data,
                self._vision_frame_content_type,
                int(self._vision["frame_sequence"]),
                str(self._vision["frame_topic"]),
            )

    def select_vision_frame_topic(self, topic: str) -> dict:
        requested_topic = str(topic).strip()
        if not requested_topic:
            raise ValueError("image topic is required")
        topic_names_and_types = dict(self.get_topic_names_and_types())
        topic_type = _supported_image_type(
            topic_names_and_types.get(requested_topic, [])
        )
        if not topic_type:
            raise ValueError(
                f"topic is not an available Image or CompressedImage: {requested_topic}"
            )

        with self._lock:
            current_topic = str(self._vision["frame_topic"])
            current_type = str(self._vision["frame_type"])
            generation = int(self._vision_subscription_generation)
        if requested_topic == current_topic and topic_type == current_type:
            return {"topic": current_topic, "type": current_type}

        next_generation = generation + 1
        new_subscription = self._create_vision_frame_subscription(
            requested_topic,
            topic_type,
            next_generation,
            qos_profile_sensor_data,
        )
        old_subscription = self._vision_frame_subscription
        self._vision_frame_subscription = new_subscription
        with self._lock:
            self._vision_subscription_generation = next_generation
            self._vision["frame_topic"] = requested_topic
            self._vision["frame_type"] = topic_type
            self._vision["frame_stamp"] = 0.0
            self._vision["frame_width"] = 0
            self._vision["frame_height"] = 0
            self._vision["frame_error"] = ""
            self._vision_frame_data = b""
            self._vision_frame_content_type = "image/jpeg"
            self._health["vision_camera"] = TopicHealth(
                requested_topic,
                stale_after=1.5,
            )
        if old_subscription is not None:
            self.destroy_subscription(old_subscription)
        return {"topic": requested_topic, "type": topic_type}

    def destroy_node(self) -> bool:
        self._vision_frame_encoder.shutdown(wait=True, cancel_futures=True)
        return super().destroy_node()

    def set_vision_control_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._vision_enable_pub.publish(msg)
        with self._lock:
            self._vision["mission_enabled"] = bool(enabled)

    def release_vision_control(self) -> None:
        self.set_vision_control_enabled(False)

    def clear_path(self) -> None:
        with self._lock:
            self._path.clear()

    def reset_path_frame(self, use_current_pose: bool = False) -> dict:
        with self._lock:
            self._path.clear()
            odom_seen = self._health["odom"].last_seen
            if use_current_pose and (
                odom_seen is None or time.monotonic() - odom_seen > 2.0
            ):
                raise RuntimeError(
                    "odometry is not fresh; cannot align the path view"
                )
            if use_current_pose:
                pose = self._pose
                self._set_path_frame_locked(
                    float(pose["x"]),
                    float(pose["y"]),
                    float(pose["yaw"]),
                )
            else:
                self._path_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
                self._path_frame.update(
                    {
                        "aligned": False,
                        "origin_x": None,
                        "origin_y": None,
                        "origin_yaw": None,
                    }
                )
            return {
                "path_pose": dict(self._path_pose),
                "path_frame": dict(self._path_frame),
            }

    def dvl_command_subscriber_count(self) -> int:
        return self._dvl_config_pub.get_subscription_count()

    def topic_publisher_count(self, topic: str) -> int:
        return len(self.get_publishers_info_by_topic(topic))

    def wait_for_odom_after(self, timestamp: float, timeout: float = 5.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                odom_seen = self._health["odom"].last_seen
            if odom_seen is not None and odom_seen >= timestamp:
                return True
            time.sleep(0.05)
        return False

    def wait_for_dvl_good(self, duration_s: float = 2.0, timeout_s: float = 10.0) -> tuple[bool, dict]:
        deadline = time.monotonic() + timeout_s
        last_snapshot = {}
        while time.monotonic() < deadline:
            with self._lock:
                ok, snapshot = self._dvl_good_window_locked(duration_s)
                last_snapshot = snapshot
            if ok:
                return True, snapshot
            time.sleep(0.05)
        with self._lock:
            ok, snapshot = self._dvl_good_window_locked(duration_s)
        return ok, snapshot or last_snapshot

    def wait_for_attitude_level(
        self,
        duration_s: float = 1.0,
        timeout_s: float = 5.0,
        max_tilt_deg: float = ATTITUDE_MAX_TILT_DEG,
    ) -> tuple[bool, dict]:
        deadline = time.monotonic() + timeout_s
        last_snapshot = {}
        while time.monotonic() < deadline:
            with self._lock:
                ok, snapshot = self._attitude_level_window_locked(duration_s, max_tilt_deg)
                last_snapshot = snapshot
            if ok:
                return True, snapshot
            time.sleep(0.05)
        with self._lock:
            ok, snapshot = self._attitude_level_window_locked(duration_s, max_tilt_deg)
        return ok, snapshot or last_snapshot

    def wait_for_mode_ready(
        self,
        allowed_modes: set[str] | None = None,
        timeout_s: float = 3.0,
    ) -> tuple[bool, dict]:
        allowed = allowed_modes or READY_MODES
        deadline = time.monotonic() + timeout_s
        snapshot = {}
        while time.monotonic() < deadline:
            with self._lock:
                snapshot = self._mode_snapshot_locked(allowed)
            if snapshot["ok"]:
                return True, snapshot
            time.sleep(0.05)
        with self._lock:
            snapshot = self._mode_snapshot_locked(allowed)
        return bool(snapshot.get("ok")), snapshot

    def wait_for_vehicle_still(
        self,
        duration_s: float = 1.0,
        timeout_s: float = 5.0,
        max_speed_mps: float = STILLNESS_MAX_SPEED_MPS,
    ) -> tuple[bool, dict]:
        deadline = time.monotonic() + timeout_s
        last_snapshot = {}
        while time.monotonic() < deadline:
            with self._lock:
                ok, snapshot = self._still_window_locked(duration_s, max_speed_mps)
                last_snapshot = snapshot
            if ok:
                return True, snapshot
            time.sleep(0.05)
        with self._lock:
            ok, snapshot = self._still_window_locked(duration_s, max_speed_mps)
        return ok, snapshot or last_snapshot

    def set_localization_origin(self) -> dict:
        now = time.monotonic()
        with self._lock:
            odom_seen = self._health["odom"].last_seen
            if odom_seen is None or now - odom_seen > 2.0:
                raise RuntimeError("odometry is not alive; cannot set localization origin")
            pose = dict(self._pose)

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = float(pose["z"])
        orientation = _quaternion_from_yaw(float(pose["yaw"]))
        msg.pose.pose.orientation.x = orientation["x"]
        msg.pose.pose.orientation.y = orientation["y"]
        msg.pose.pose.orientation.z = orientation["z"]
        msg.pose.pose.orientation.w = orientation["w"]
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[14] = 0.01
        msg.pose.covariance[21] = 0.01
        msg.pose.covariance[28] = 0.01
        msg.pose.covariance[35] = 0.01
        if not self._set_pose_client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError("robot_localization set_pose service is not available")

        request = SetPose.Request()
        request.pose = msg
        future = self._set_pose_client.call_async(request)
        deadline = time.monotonic() + 1.0
        while not future.done():
            if time.monotonic() >= deadline:
                raise RuntimeError("robot_localization set_pose service timed out")
            time.sleep(0.02)
        if future.exception() is not None:
            raise RuntimeError(f"robot_localization set_pose failed: {future.exception()}")

        with self._lock:
            self._pose = {
                "x": 0.0,
                "y": 0.0,
                "z": pose["z"],
                "yaw": pose["yaw"],
            }
            self._path.clear()
            self._set_path_frame_locked(0.0, 0.0, float(pose["yaw"]))
        self.get_logger().info(
            "Set localization origin: "
            f"previous x={pose['x']:.3f} "
            f"y={pose['y']:.3f} "
            f"z={pose['z']:.3f} "
            f"yaw={pose['yaw']:.3f}"
        )
        return {
            "previous_pose": pose,
            "new_pose": {
                "x": 0.0,
                "y": 0.0,
                "z": pose["z"],
                "yaw": pose["yaw"],
            },
        }

    def set_web_control_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._web_control["enabled"] = enabled
            self._web_control["active"] = False
            self._web_control["last_command"] = time.monotonic()
            if not enabled:
                self._web_control["neutral_burst"] = 8

    def neutralize_web_control(self, burst_count: int = 10) -> None:
        with self._lock:
            self._web_control["enabled"] = False
            self._web_control["active"] = False
            self._web_control["axes"] = {
                "forward": 0.0,
                "lateral": 0.0,
                "vertical": 0.0,
                "yaw": 0.0,
            }
            self._web_control["last_command"] = time.monotonic()
            self._web_control["neutral_burst"] = max(
                int(self._web_control["neutral_burst"]),
                int(burst_count),
            )

    def update_web_control_command(self, axes: dict, active: bool) -> None:
        clean_axes = {
            "forward": _clamp_axis(axes.get("forward", 0.0)),
            "lateral": _clamp_axis(axes.get("lateral", 0.0)),
            "vertical": _clamp_axis(axes.get("vertical", 0.0)),
            "yaw": _clamp_axis(axes.get("yaw", 0.0)),
        }
        with self._lock:
            self._web_control["active"] = bool(active)
            self._web_control["axes"] = clean_axes
            self._web_control["last_command"] = time.monotonic()

    def set_armed(self, armed: bool) -> bool:
        if not self._arm_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Arming service not available.")
            return False
        request = CommandBool.Request()
        request.value = bool(armed)
        self._arm_client.call_async(request)
        return True

    def set_mode(self, mode: str) -> bool:
        if not self._set_mode_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Set mode service not available.")
            return False
        request = SetMode.Request()
        request.custom_mode = mode
        self._set_mode_client.call_async(request)
        return True

    def publish_guided_goal(
        self,
        x: float,
        y: float,
        z: float,
        mode: int,
    ) -> dict:
        subscriber_count = self._guided_goal_pub.get_subscription_count()
        if subscriber_count <= 0:
            raise RuntimeError(
                "guided_navigation is unavailable on /guided/goal"
            )
        msg = AuvSetpoint()
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(z)
        msg.yaw = 0.0
        msg.mode = int(mode)
        self._guided_goal_pub.publish(msg)

        command = {
            "x": msg.x,
            "y": msg.y,
            "z": msg.z,
            "heading": "AUTO",
            "mode": msg.mode,
            "sent_at": time.strftime("%H:%M:%S"),
        }
        with self._lock:
            self._guided["last_command"] = command
        return {
            "goal": command,
            "subscriber_count": subscriber_count,
        }

    def cancel_guided_goal(self) -> int:
        subscriber_count = self._guided_cancel_pub.get_subscription_count()
        if subscriber_count <= 0:
            raise RuntimeError(
                "guided_navigation is unavailable on /guided/cancel"
            )
        self._guided_cancel_pub.publish(Empty())
        return subscriber_count

    def set_guided_waypoint_enabled(self, enabled: bool) -> int:
        subscriber_count = self._guided_waypoint_enable_pub.get_subscription_count()
        msg = Bool()
        msg.data = bool(enabled)
        self._guided_waypoint_enable_pub.publish(msg)
        return subscriber_count

    def recapture_guided_start_frame(self) -> int:
        subscriber_count = self._guided_recapture_pub.get_subscription_count()
        if subscriber_count <= 0:
            raise RuntimeError(
                "guided_navigation is unavailable on "
                "/guided/recapture_start_frame"
            )
        self._guided_recapture_pub.publish(Empty())
        return subscriber_count

    def _on_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self._lock:
            self._health["odom"].tick()
            self._pose = {
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw": yaw,
            }
            self._frames["odom"] = {
                "frame_id": msg.header.frame_id,
                "child_frame_id": msg.child_frame_id,
            }
            if not bool(self._path_frame["aligned"]):
                self._set_path_frame_locked(
                    float(pose.position.x),
                    float(pose.position.y),
                    yaw,
                )
            display_pose = _start_forward_pose(
                float(pose.position.x),
                float(pose.position.y),
                yaw,
                float(self._path_frame["origin_x"]),
                float(self._path_frame["origin_y"]),
                float(self._path_frame["origin_yaw"]),
            )
            self._path_pose = display_pose
            self._append_path_point(display_pose["x"], display_pose["y"])

    def _on_external_nav_reset(self, msg: UInt8) -> None:
        reset_counter = int(msg.data)
        with self._lock:
            if self._last_external_nav_reset_counter == reset_counter:
                return
            self._last_external_nav_reset_counter = reset_counter
            self._path.clear()
            self._path_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
            self._path_frame.update(
                {
                    "aligned": False,
                    "origin_x": None,
                    "origin_y": None,
                    "origin_yaw": None,
                }
            )
        self.get_logger().info(
            "Path view will realign start-forward up after ExternalNav reset "
            f"counter={reset_counter}"
        )

    def _set_path_frame_locked(self, x: float, y: float, yaw: float) -> None:
        self._path_frame.update(
            {
                "aligned": True,
                "origin_x": x,
                "origin_y": y,
                "origin_yaw": yaw,
            }
        )
        self._path_pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def _append_path_point(self, x: float, y: float) -> None:
        if not math.isfinite(x) or not math.isfinite(y):
            return
        if self._path:
            last = self._path[-1]
            if math.hypot(x - last["x"], y - last["y"]) < PATH_MIN_DISTANCE_M:
                return
        self._path.append({"x": x, "y": y})

    def _web_control_snapshot(self) -> dict:
        now = time.monotonic()
        last_command = self._web_control["last_command"]
        age = None if last_command == 0.0 else now - last_command
        return {
            "enabled": bool(self._web_control["enabled"]),
            "active": bool(self._web_control["active"]),
            "fresh": age is not None and age <= WEB_CONTROL_TIMEOUT_S,
            "age": age,
            "axes": dict(self._web_control["axes"]),
            "last_publish": self._web_control["last_publish"],
        }

    def _publish_web_control(self) -> None:
        with self._lock:
            enabled = bool(self._web_control["enabled"])
            neutral_burst = int(self._web_control["neutral_burst"])
            if not enabled and neutral_burst <= 0:
                return

            now = time.monotonic()
            fresh = now - self._web_control["last_command"] <= WEB_CONTROL_TIMEOUT_S
            active = enabled and bool(self._web_control["active"]) and fresh
            axes = dict(self._web_control["axes"]) if active else {
                "forward": 0.0,
                "lateral": 0.0,
                "vertical": 0.0,
                "yaw": 0.0,
            }
            if not enabled:
                self._web_control["neutral_burst"] = max(0, neutral_burst - 1)
            self._web_control["last_publish"] = time.strftime("%H:%M:%S")

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [
            axes["lateral"],
            axes["forward"],
            axes["yaw"],
            axes["vertical"],
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        msg.buttons = [0] * 12
        self._web_joy_pub.publish(msg)

    def _on_dvl(self, msg: TwistWithCovarianceStamped) -> None:
        linear = msg.twist.twist.linear
        speed = math.sqrt(linear.x * linear.x + linear.y * linear.y + linear.z * linear.z)
        with self._lock:
            self._health["dvl"].tick()
            self._velocity = {"x": linear.x, "y": linear.y, "z": linear.z}
            if math.isfinite(speed):
                self._dvl_twist_samples.append({"t": time.monotonic(), "speed": speed})

    def _on_dvl_data(self, msg: DVL) -> None:
        velocity = msg.velocity
        speed = math.sqrt(
            velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z)
        valid_beams = sum(1 for beam in msg.beams if bool(beam.valid))
        good, reason = _dvl_quality_state(msg, valid_beams)
        sample = {
            "t": time.monotonic(),
            "good": good,
            "reason": reason,
            "velocity_valid": bool(msg.velocity_valid),
            "fom": _finite_or_none(float(msg.fom)),
            "altitude": _finite_or_none(float(msg.altitude)),
            "valid_beams": valid_beams,
            "speed": _finite_or_none(speed),
        }
        with self._lock:
            self._health["dvl_data"].tick()
            self._dvl_quality = {
                **{key: value for key, value in sample.items() if key != "t"},
                "updated_at": time.strftime("%H:%M:%S"),
            }
            self._dvl_quality_samples.append(sample)

    def _on_dvl_response(self, msg: CommandResponse) -> None:
        self._append_dvl_event(
            "response",
            msg.response_to,
            "",
            str(msg.result),
            msg.success,
            msg.error_message,
        )
        if msg.response_to != "calibrate_gyro":
            return
        with self._lock:
            self._expire_dvl_calibration_locked(time.monotonic())
            if self._dvl_calibration["state"] != "calibrating":
                return
            if msg.success:
                self._set_dvl_calibration_result_locked(
                    state="completed",
                    success=True,
                    error_message="",
                )
            else:
                self._set_dvl_calibration_result_locked(
                    state="failed",
                    success=False,
                    error_message=msg.error_message or "DVL rejected gyro calibration",
                )

    def _on_dvl_config(self, msg: ConfigStatus) -> None:
        with self._lock:
            self._dvl_config = {
                "updated_at": time.strftime("%H:%M:%S"),
                "response_to": msg.response_to,
                "success": msg.success,
                "error_message": msg.error_message,
                "speed_of_sound": msg.speed_of_sound,
                "acoustic_enabled": msg.acoustic_enabled,
                "dark_mode_enabled": msg.dark_mode_enabled,
                "mounting_rotation_offset": msg.mounting_rotation_offset,
                "range_mode": msg.range_mode,
                "format": msg.format,
                "type": msg.type,
            }
        self._append_dvl_event(
            "config",
            msg.response_to,
            "",
            msg.range_mode,
            msg.success,
            msg.error_message,
        )

    def _on_depth(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._health["depth"].tick()
            self._depth = {"z": msg.pose.pose.position.z}
            self._frames["depth"] = {"frame_id": msg.header.frame_id}

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._health["battery"].tick()
            self._battery = {
                "voltage": _finite_or_none(msg.voltage),
                "current": _finite_or_none(msg.current),
                "temperature": _finite_or_none(msg.temperature),
                "percentage": _finite_or_none(msg.percentage),
                "present": bool(msg.present),
            }

    def _on_mavros_state(self, msg: State) -> None:
        with self._lock:
            self._health["mavros_state"].tick()
            self._mavros_state = {
                "connected": bool(msg.connected),
                "armed": bool(msg.armed),
                "guided": bool(msg.guided),
                "manual_input": bool(msg.manual_input),
                "mode": msg.mode,
                "system_status": int(msg.system_status),
                "updated_at": time.strftime("%H:%M:%S"),
            }

    def _on_guided_status(self, msg: String) -> None:
        with self._lock:
            self._guided["status"] = str(msg.data)
            self._guided["updated_at"] = time.strftime("%H:%M:%S")

    def _on_guided_arrived(self, msg: Bool) -> None:
        with self._lock:
            self._guided["arrived"] = bool(msg.data)
            self._guided["updated_at"] = time.strftime("%H:%M:%S")

    def _on_guided_mission_status(self, msg: MissionStatus) -> None:
        labels = {
            MissionStatus.READY: "READY",
            MissionStatus.RUNNING: "RUNNING",
            MissionStatus.COMPLETED: "COMPLETED",
        }
        with self._lock:
            self._guided["mission_status"] = int(msg.status)
            self._guided["mission_status_label"] = labels.get(
                int(msg.status), f"UNKNOWN ({int(msg.status)})"
            )
            self._guided["updated_at"] = time.strftime("%H:%M:%S")

    def _on_guided_active_target(self, msg: PoseStamped) -> None:
        pose = msg.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self._lock:
            self._guided["active_target"] = {
                "frame_id": msg.header.frame_id,
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw_rad": yaw,
                "yaw_deg": math.degrees(yaw),
            }
            self._guided["updated_at"] = time.strftime("%H:%M:%S")

    def _on_guided_start_frame(self, msg: PoseStamped) -> None:
        pose = msg.pose
        yaw = _yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        with self._lock:
            self._guided["start_frame"] = {
                "frame_id": msg.header.frame_id,
                "x": pose.position.x,
                "y": pose.position.y,
                "z": pose.position.z,
                "yaw_rad": yaw,
                "yaw_deg": math.degrees(yaw),
            }
            self._guided["updated_at"] = time.strftime("%H:%M:%S")

    def _on_imu(self, msg: Imu) -> None:
        orientation = msg.orientation
        roll, pitch, yaw = _rpy_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)
        tilt_deg = math.degrees(math.hypot(roll, pitch))
        with self._lock:
            self._health["imu"].tick()
            self._attitude = {
                "roll_deg": roll_deg,
                "pitch_deg": pitch_deg,
                "yaw_deg": yaw_deg,
                "tilt_deg": tilt_deg,
                "updated_at": time.strftime("%H:%M:%S"),
            }
            self._tilt_samples.append({"t": time.monotonic(), "tilt_deg": tilt_deg})

    def _on_joy(self, msg: Joy) -> None:
        with self._lock:
            self._health["joy"].tick()
            self._joy = {
                "axes": [round(value, 3) for value in msg.axes],
                "buttons": list(msg.buttons),
            }

    def _create_vision_frame_subscription(
        self,
        topic: str,
        topic_type: str,
        generation: int,
        qos: QoSProfile,
    ):
        if topic_type == COMPRESSED_IMAGE_TYPE:
            return self.create_subscription(
                CompressedImage,
                topic,
                lambda msg: self._on_vision_compressed_image(
                    msg,
                    topic,
                    generation,
                ),
                qos,
            )
        if topic_type == RAW_IMAGE_TYPE:
            return self.create_subscription(
                Image,
                topic,
                lambda msg: self._on_vision_raw_image(
                    msg,
                    topic,
                    generation,
                ),
                qos,
            )
        raise ValueError(f"unsupported image topic type: {topic_type}")

    def _on_vision_compressed_image(
        self,
        msg: CompressedImage,
        topic: str,
        generation: int,
    ) -> None:
        image_format = str(msg.format or "jpeg").lower()
        content_type = "image/png" if "png" in image_format else "image/jpeg"
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self._store_vision_frame(
            topic=topic,
            generation=generation,
            data=bytes(msg.data),
            content_type=content_type,
            stamp=stamp,
        )

    def _on_vision_raw_image(
        self,
        msg: Image,
        topic: str,
        generation: int,
    ) -> None:
        now = time.monotonic()
        with self._lock:
            if (
                topic != self._vision["frame_topic"]
                or generation != self._vision_subscription_generation
                or self._vision_raw_encode_pending
                or now - self._vision_last_raw_frame_at
                < VISION_RAW_FRAME_MIN_INTERVAL_S
            ):
                return
            self._vision_raw_encode_pending = True
            self._vision_last_raw_frame_at = now
        try:
            self._vision_frame_encoder.submit(
                self._encode_raw_vision_frame,
                msg,
                topic,
                generation,
            )
        except Exception as exc:
            with self._lock:
                self._vision_raw_encode_pending = False
                if generation == self._vision_subscription_generation:
                    self._vision["frame_error"] = str(exc)[:240]

    def _encode_raw_vision_frame(
        self,
        msg: Image,
        topic: str,
        generation: int,
    ) -> None:
        try:
            image = self._cv_bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="passthrough",
            )
            bgr_image = self._vision_image_to_bgr(image, str(msg.encoding))
            encoded_ok, encoded = cv2.imencode(
                ".jpg",
                bgr_image,
                [cv2.IMWRITE_JPEG_QUALITY, VISION_RAW_FRAME_JPEG_QUALITY],
            )
            if not encoded_ok:
                raise RuntimeError("OpenCV could not encode the selected image")
            stamp = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9
            )
            self._store_vision_frame(
                topic=topic,
                generation=generation,
                data=encoded.tobytes(),
                content_type="image/jpeg",
                stamp=stamp,
                width=int(msg.width),
                height=int(msg.height),
            )
        except Exception as exc:
            with self._lock:
                if (
                    topic == self._vision["frame_topic"]
                    and generation == self._vision_subscription_generation
                ):
                    self._vision["frame_error"] = (
                        f"{msg.encoding or 'unknown encoding'}: {exc}"
                    )[:240]
        finally:
            with self._lock:
                self._vision_raw_encode_pending = False

    @staticmethod
    def _vision_image_to_bgr(image: np.ndarray, encoding: str) -> np.ndarray:
        array = np.asarray(image)
        normalized_encoding = encoding.strip().lower()
        if array.ndim == 2:
            if array.dtype == np.uint8:
                return cv2.cvtColor(array, cv2.COLOR_GRAY2BGR)
            numeric = array.astype(np.float32, copy=False)
            valid = np.isfinite(numeric)
            if np.issubdtype(array.dtype, np.integer):
                valid &= numeric > 0
            scaled = np.zeros(array.shape, dtype=np.uint8)
            if np.any(valid):
                low, high = np.percentile(numeric[valid], (2.0, 98.0))
                if high <= low:
                    high = low + 1.0
                scaled[valid] = np.clip(
                    (numeric[valid] - low) * 255.0 / (high - low),
                    0.0,
                    255.0,
                ).astype(np.uint8)
            return cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)
        if array.ndim == 3 and array.shape[2] == 3:
            if normalized_encoding.startswith("rgb"):
                return cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
            return array
        if array.ndim == 3 and array.shape[2] == 4:
            conversion = (
                cv2.COLOR_RGBA2BGR
                if normalized_encoding.startswith("rgba")
                else cv2.COLOR_BGRA2BGR
            )
            return cv2.cvtColor(array, conversion)
        raise ValueError(
            f"unsupported image shape {array.shape} for {encoding or 'unknown'}"
        )

    def _store_vision_frame(
        self,
        *,
        topic: str,
        generation: int,
        data: bytes,
        content_type: str,
        stamp: float,
        width: int = 0,
        height: int = 0,
    ) -> None:
        if not data:
            return
        with self._lock:
            if (
                topic != self._vision["frame_topic"]
                or generation != self._vision_subscription_generation
            ):
                return
            self._health["vision_camera"].tick()
            self._vision_frame_data = data
            self._vision_frame_content_type = content_type
            self._vision["frame_sequence"] = int(self._vision["frame_sequence"]) + 1
            self._vision["frame_stamp"] = stamp
            self._vision["frame_error"] = ""
            if width > 0 and height > 0:
                self._vision["frame_width"] = width
                self._vision["frame_height"] = height

    def _on_vision_bbox(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 10:
            return
        values = [float(value) for value in msg.data[:10]]
        if not all(math.isfinite(value) for value in values):
            return
        (
            stamp,
            detected,
            class_id,
            confidence,
            center_x,
            center_y,
            width,
            height,
            image_width,
            image_height,
        ) = values
        now = time.monotonic()
        with self._lock:
            self._health["vision_bbox"].tick()
            previous_stamp = float(self._vision.get("detection_stamp", -1.0))
            if abs(stamp - previous_stamp) > 1e-6:
                self._vision["detections"] = {}
                self._vision["detection_stamp"] = stamp
            if self._vision["frame_topic"] == DEFAULT_VISION_FRAME_TOPIC:
                self._vision["frame_width"] = max(0, int(round(image_width)))
                self._vision["frame_height"] = max(0, int(round(image_height)))
            if detected >= 0.5 and image_width > 0.0 and image_height > 0.0:
                class_key = str(int(round(class_id)))
                self._vision["detections"][class_key] = {
                    "detected": True,
                    "class_id": int(round(class_id)),
                    "confidence": confidence,
                    "center_x": center_x,
                    "center_y": center_y,
                    "width": width,
                    "height": height,
                    "image_width": image_width,
                    "image_height": image_height,
                    "error_x": (center_x - image_width / 2.0) / (image_width / 2.0),
                    "error_y": (center_y - image_height / 2.0) / (image_height / 2.0),
                    "area_ratio": max(0.0, width * height / (image_width * image_height)),
                    "received_at": now,
                }

    def _on_vision_enable(self, msg: Bool) -> None:
        with self._lock:
            self._health["vision_mission_enable"].tick()
            self._vision["mission_enabled"] = bool(msg.data)

    def _on_vision_state(self, msg: String) -> None:
        with self._lock:
            self._health["vision_mission_state"].tick()
            self._vision["mission_state"] = str(msg.data or "UNKNOWN")

    def _on_vision_rc_command(self, msg: OverrideRCIn) -> None:
        with self._lock:
            self._health["vision_rc_command"].tick()
            self._vision["rc_channels"] = [int(value) for value in msg.channels]

    def _vision_snapshot_locked(self) -> dict:
        now = time.monotonic()
        detections = []
        for item in self._vision["detections"].values():
            clean = {key: value for key, value in item.items() if key != "received_at"}
            clean["age"] = max(0.0, now - float(item["received_at"]))
            detections.append(clean)
        detections.sort(key=lambda item: item["class_id"])
        return {
            "frame_sequence": int(self._vision["frame_sequence"]),
            "frame_stamp": float(self._vision["frame_stamp"]),
            "frame_width": int(self._vision["frame_width"]),
            "frame_height": int(self._vision["frame_height"]),
            "frame_topic": str(self._vision["frame_topic"]),
            "frame_type": str(self._vision["frame_type"]),
            "frame_error": str(self._vision["frame_error"]),
            "detections": detections,
            "mission_enabled": bool(self._vision["mission_enabled"]),
            "mission_state": str(self._vision["mission_state"]),
            "rc_channels": list(self._vision["rc_channels"]),
        }

    def _on_pinger_homing(self, msg: String) -> None:
        parsed = _json_object(msg.data)
        with self._lock:
            self._health["pinger_homing"].tick()
            self._pinger_homing_status = {
                "raw": msg.data[-1600:],
                "state": str(parsed.get("state", "")) if parsed else "",
                "dry_run": bool(parsed.get("dry_run", True)) if parsed else True,
                "control_output_active": (
                    bool(parsed.get("control_output_active", False)) if parsed else False
                ),
                "inputs_ready": bool(parsed.get("inputs_ready", False)) if parsed else False,
                "connected": bool(parsed.get("connected", False)) if parsed else False,
                "armed": bool(parsed.get("armed", False)) if parsed else False,
                "audio_fresh": bool(parsed.get("audio_fresh", False)) if parsed else False,
                "sample_count": _integer_or_zero(parsed.get("sample_count")) if parsed else 0,
                "probe_attempt": _integer_or_zero(parsed.get("probe_attempt")) if parsed else 0,
                "minimum_probe_legs": (
                    _integer_or_zero(parsed.get("minimum_probe_legs")) if parsed else 0
                ),
                "estimated_source_world": parsed.get("estimated_source_world") if parsed else None,
                "source_locked": bool(parsed.get("source_locked", False)) if parsed else False,
                "estimated_distance_m": (
                    _number_or_none(parsed.get("estimated_distance_m")) if parsed else None
                ),
                "amplitude_distance_m": (
                    _number_or_none(parsed.get("amplitude_distance_m")) if parsed else None
                ),
                "rms_residual_m": (
                    _number_or_none(parsed.get("rms_residual_m")) if parsed else None
                ),
                "condition_number": (
                    _number_or_none(parsed.get("condition_number")) if parsed else None
                ),
                "bias_range_rate_mps": (
                    _number_or_none(parsed.get("bias_range_rate_mps")) if parsed else None
                ),
                "control_direction_source": (
                    str(parsed.get("control_direction_source", "")) if parsed else ""
                ),
                "command": parsed.get("command", {}) if parsed else {},
                "requested_command": parsed.get("requested_command", {}) if parsed else {},
                "depth_safety": parsed.get("depth_safety", {}) if parsed else {},
                "bearing_error_deg": (
                    _number_or_none(parsed.get("bearing_error_deg")) if parsed else None
                ),
                "range_complete": bool(parsed.get("range_complete", False)) if parsed else False,
                "arrival_complete": bool(parsed.get("arrival_complete", False)) if parsed else False,
                "completion_reason": str(parsed.get("completion_reason", "")) if parsed else "",
                "arrival_radius_m": (
                    _number_or_none(parsed.get("arrival_radius_m")) if parsed else None
                ),
                "active_runtime_s": (
                    _number_or_none(parsed.get("active_runtime_s")) if parsed else None
                ),
                "max_runtime_s": (
                    _number_or_none(parsed.get("max_runtime_s")) if parsed else None
                ),
                "amplitude_range_constant": (
                    _number_or_none(parsed.get("amplitude_range_constant")) if parsed else None
                ),
            }

    def _on_hydrophone_direction(self, msg: Vector3Stamped) -> None:
        x = msg.vector.x
        y = msg.vector.y
        z = msg.vector.z
        bearing = math.atan2(y, x) if math.isfinite(x) and math.isfinite(y) else None
        with self._lock:
            self._health["hydrophone_direction"].tick()
            self._hydrophone_direction = {
                "x": _finite_or_none(x),
                "y": _finite_or_none(y),
                "z": _finite_or_none(z),
                "bearing_rad": bearing,
            }

    def _on_delta_range(self, msg: Float64) -> None:
        value = _finite_or_none(float(msg.data))
        with self._lock:
            self._health["delta_range"].tick()
            self._delta_range_m = value

    def _on_iq_magnitude(self, msg: Float64) -> None:
        value = _finite_or_none(float(msg.data))
        with self._lock:
            self._health["iq_magnitude"].tick()
            self._iq_magnitude = value

    def _on_rc_mux_status(self, msg: String) -> None:
        parsed = _json_object(msg.data)
        with self._lock:
            self._health["rc_mux"].tick()
            self._rc_mux_status = {
                "owner": str(parsed.get("owner", "unknown")),
                "conflict": bool(parsed.get("conflict", False)),
                "output_enabled": bool(parsed.get("output_enabled", False)),
                "publisher_count": _integer_or_zero(parsed.get("publisher_count")),
            }

    def _append_dvl_event(
        self,
        event_type: str,
        command: str,
        parameter_name: str,
        parameter_value: str,
        success: bool | None,
        error_message: str,
    ) -> None:
        with self._lock:
            self._dvl_events.append(
                {
                    "time": time.strftime("%H:%M:%S"),
                    "type": event_type,
                    "command": command,
                    "parameter_name": parameter_name,
                    "parameter_value": parameter_value,
                    "success": success,
                    "error_message": error_message,
                }
            )

    def _expire_dvl_calibration_locked(self, now: float) -> None:
        deadline = self._dvl_calibration.get("deadline_monotonic")
        if (
            self._dvl_calibration.get("state") == "calibrating"
            and isinstance(deadline, (int, float))
            and now >= float(deadline)
        ):
            self._set_dvl_calibration_result_locked(
                state="timeout",
                success=False,
                error_message="No calibrate_gyro ACK received within the timeout",
            )

    def _set_dvl_calibration_result_locked(
        self,
        state: str,
        success: bool,
        error_message: str,
    ) -> None:
        if state == "completed":
            message = "Calibration complete; DVL success ACK received"
        elif state == "timeout":
            message = "Calibration result unknown; DVL ACK timed out"
        else:
            message = error_message or "Gyro calibration failed"
        self._dvl_calibration.update(
            {
                "state": state,
                "message": message,
                "success": success,
                "error_message": error_message,
                "completed_at": time.strftime("%H:%M:%S"),
                "deadline_monotonic": None,
            }
        )

    def _dvl_calibration_snapshot_locked(self) -> dict:
        self._expire_dvl_calibration_locked(time.monotonic())
        return {
            key: value
            for key, value in self._dvl_calibration.items()
            if key != "deadline_monotonic"
        }

    def _precheck_snapshot_locked(self) -> dict:
        dvl_ok, dvl = self._dvl_good_window_locked(2.0)
        attitude_ok, attitude = self._attitude_level_window_locked(1.0, ATTITUDE_MAX_TILT_DEG)
        still_ok, still = self._still_window_locked(1.0, STILLNESS_MAX_SPEED_MPS)
        mode = self._mode_snapshot_locked(READY_MODES)
        return {
            "ready": dvl_ok and attitude_ok and mode["ok"] and still_ok,
            "dvl_good": dvl,
            "attitude_level": attitude,
            "mode_ready": mode,
            "vehicle_still": still,
        }

    def _dvl_good_window_locked(self, duration_s: float) -> tuple[bool, dict]:
        now = time.monotonic()
        recent = [
            sample for sample in self._dvl_quality_samples
            if now - float(sample["t"]) <= max(duration_s, 0.0)
        ]
        twist_age = self._health["dvl"].snapshot()["age"]
        latest = dict(self._dvl_quality)
        latest["twist_age"] = twist_age
        if not recent:
            latest["ok"] = False
            latest["reason"] = latest.get("reason") or "no recent DVL data"
            return False, latest
        has_full_window = (now - float(recent[0]["t"])) >= max(
            0.0, duration_s - DVL_TWIST_STALE_AFTER_S)
        latest_is_fresh = now - recent[-1]["t"] <= DVL_TWIST_STALE_AFTER_S
        twist_is_fresh = twist_age is not None and twist_age <= DVL_TWIST_STALE_AFTER_S
        all_good = all(bool(sample["good"]) for sample in recent)
        ok = has_full_window and latest_is_fresh and twist_is_fresh and all_good
        if not ok:
            if not has_full_window:
                latest["reason"] = f"waiting for {duration_s:.1f}s DVL good window"
            elif not latest_is_fresh:
                latest["reason"] = "DVL raw data stale"
            elif not twist_is_fresh:
                latest["reason"] = "DVL twist stale"
            elif not all_good:
                latest["reason"] = recent[-1].get("reason", "DVL degraded")
        latest["ok"] = ok
        latest["duration_s"] = duration_s
        return ok, latest

    def _attitude_level_window_locked(
        self,
        duration_s: float,
        max_tilt_deg: float,
    ) -> tuple[bool, dict]:
        now = time.monotonic()
        recent = [
            sample for sample in self._tilt_samples
            if now - float(sample["t"]) <= max(duration_s, 0.0)
        ]
        latest = dict(self._attitude)
        latest["max_tilt_deg"] = max_tilt_deg
        if not recent:
            latest["ok"] = False
            latest["reason"] = "no recent IMU attitude"
            return False, latest
        has_full_window = (now - float(recent[0]["t"])) >= max(0.0, duration_s - 0.25)
        max_recent_tilt = max(float(sample["tilt_deg"]) for sample in recent)
        ok = has_full_window and max_recent_tilt <= max_tilt_deg
        latest["ok"] = ok
        latest["recent_max_tilt_deg"] = max_recent_tilt
        latest["duration_s"] = duration_s
        latest["reason"] = (
            "level"
            if ok
            else (
                f"waiting for {duration_s:.1f}s level attitude"
                if not has_full_window
                else f"tilt {max_recent_tilt:.1f} deg > {max_tilt_deg:.1f} deg"
            )
        )
        return ok, latest

    def _mode_snapshot_locked(self, allowed_modes: set[str]) -> dict:
        topic = self._health["mavros_state"].snapshot()
        mode = str(self._mavros_state.get("mode") or "")
        ok = bool(topic["alive"]) and mode in allowed_modes
        return {
            "ok": ok,
            "mode": mode,
            "allowed_modes": sorted(allowed_modes),
            "topic_alive": bool(topic["alive"]),
            "reason": "mode ready" if ok else (
                "mavros state stale" if not topic["alive"] else f"mode {mode or '--'} not allowed"
            ),
        }

    def _still_window_locked(
        self,
        duration_s: float,
        max_speed_mps: float,
    ) -> tuple[bool, dict]:
        now = time.monotonic()
        recent = [
            sample for sample in self._dvl_twist_samples
            if now - float(sample["t"]) <= max(duration_s, 0.0)
        ]
        if not recent:
            return False, {
                "ok": False,
                "reason": "no recent DVL twist speed",
                "duration_s": duration_s,
                "max_speed_mps": max_speed_mps,
            }
        has_full_window = (now - float(recent[0]["t"])) >= max(0.0, duration_s - 0.25)
        max_recent_speed = max(float(sample["speed"]) for sample in recent)
        ok = has_full_window and max_recent_speed <= max_speed_mps
        return ok, {
            "ok": ok,
            "reason": (
                "still"
                if ok
                else (
                    f"waiting for {duration_s:.1f}s still window"
                    if not has_full_window
                    else f"DVL speed {max_recent_speed:.3f} m/s > {max_speed_mps:.3f} m/s"
                )
            ),
            "duration_s": duration_s,
            "max_speed_mps": max_speed_mps,
            "recent_max_speed_mps": max_recent_speed,
        }


class RosInterface:
    def __init__(self, topic_config: TopicConfig | None = None):
        self.topic_config = topic_config or TopicConfig()
        self.node: LocalizationRosNode | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._state_lock = threading.Lock()

    def start(self) -> None:
        if not rclpy.ok():
            # FastAPI/launch CLI arguments are not ROS arguments for this node.
            rclpy.init(args=[])
        self.node = LocalizationRosNode(self.topic_config)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self) -> None:
        executor = self._executor
        if executor is None:
            return
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    def stop(self) -> None:
        with self._state_lock:
            executor = self._executor
            node = self.node
            self._executor = None
            self.node = None
            if executor is not None:
                executor.shutdown()
            if node is not None:
                node.destroy_node()
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None
        if rclpy.ok():
            rclpy.shutdown()

    def status(self) -> dict:
        with self._state_lock:
            if self.node is None or not rclpy.ok():
                return {}
            try:
                return self.node.snapshot()
            except Exception:
                # SIGINT can invalidate the global ROS context before FastAPI's
                # shutdown hook closes an already-connected status WebSocket.
                if not rclpy.ok():
                    return {}
                raise

    def publish_dvl_command(
        self,
        command: str,
        parameter_name: str = "",
        parameter_value: str = "",
    ) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.publish_dvl_command(command, parameter_name, parameter_value)

    def start_dvl_gyro_calibration(
        self,
        timeout_s: float = DVL_CALIBRATION_TIMEOUT_S,
    ) -> dict:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.start_dvl_gyro_calibration(timeout_s)

    def reset_dvl_dead_reckoning(self) -> None:
        self.publish_dvl_command("reset_dead_reckoning")

    def clear_path(self) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.clear_path()

    def reset_path_frame(self, use_current_pose: bool = False) -> dict:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.reset_path_frame(use_current_pose)

    def dvl_command_subscriber_count(self) -> int:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.dvl_command_subscriber_count()

    def topic_publisher_count(self, topic: str) -> int:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.topic_publisher_count(topic)

    def set_localization_origin(self) -> dict:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.set_localization_origin()

    def wait_for_odom_after(self, timestamp: float, timeout: float = 5.0) -> bool:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.wait_for_odom_after(timestamp, timeout)

    def wait_for_dvl_good(
        self,
        duration_s: float = 2.0,
        timeout_s: float = 10.0,
    ) -> tuple[bool, dict]:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.wait_for_dvl_good(duration_s, timeout_s)

    def wait_for_attitude_level(
        self,
        duration_s: float = 1.0,
        timeout_s: float = 5.0,
        max_tilt_deg: float = ATTITUDE_MAX_TILT_DEG,
    ) -> tuple[bool, dict]:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.wait_for_attitude_level(duration_s, timeout_s, max_tilt_deg)

    def wait_for_mode_ready(
        self,
        allowed_modes: set[str] | None = None,
        timeout_s: float = 3.0,
    ) -> tuple[bool, dict]:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.wait_for_mode_ready(allowed_modes, timeout_s)

    def wait_for_vehicle_still(
        self,
        duration_s: float = 1.0,
        timeout_s: float = 5.0,
        max_speed_mps: float = STILLNESS_MAX_SPEED_MPS,
    ) -> tuple[bool, dict]:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.wait_for_vehicle_still(duration_s, timeout_s, max_speed_mps)

    def set_web_control_enabled(self, enabled: bool) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.set_web_control_enabled(enabled)

    def neutralize_web_control(self, burst_count: int = 10) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.neutralize_web_control(burst_count)

    def update_web_control_command(self, axes: dict, active: bool) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.update_web_control_command(axes, active)

    def set_armed(self, armed: bool) -> bool:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.set_armed(armed)

    def set_mode(self, mode: str) -> bool:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.set_mode(mode)

    def publish_guided_goal(
        self,
        x: float,
        y: float,
        z: float,
        mode: int,
    ) -> dict:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.publish_guided_goal(x, y, z, mode)

    def cancel_guided_goal(self) -> int:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.cancel_guided_goal()

    def set_guided_waypoint_enabled(self, enabled: bool) -> int:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.set_guided_waypoint_enabled(enabled)

    def recapture_guided_start_frame(self) -> int:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.recapture_guided_start_frame()

    def latest_vision_frame(self) -> tuple[bytes, str, int, str]:
        if self.node is None:
            return b"", "image/jpeg", 0, DEFAULT_VISION_FRAME_TOPIC
        return self.node.latest_vision_frame()

    def select_vision_frame_topic(
        self,
        topic: str,
        timeout_s: float = 2.0,
    ) -> dict:
        with self._state_lock:
            node = self.node
            executor = self._executor
        if node is None or executor is None:
            raise RuntimeError("ROS interface is not running")
        future = executor.create_task(
            self._select_vision_frame_topic_task,
            node,
            topic,
        )
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError("timed out while changing the image topic")
        error = future.exception()
        if error is not None:
            raise error
        result = future.result()
        if result["error"]:
            if result["invalid"]:
                raise ValueError(result["error"])
            raise RuntimeError(result["error"])
        return result["selected"]

    @staticmethod
    def _select_vision_frame_topic_task(
        node: LocalizationRosNode,
        topic: str,
    ) -> dict:
        try:
            return {
                "selected": node.select_vision_frame_topic(topic),
                "error": "",
                "invalid": False,
            }
        except Exception as exc:
            return {
                "selected": {},
                "error": str(exc),
                "invalid": isinstance(exc, ValueError),
            }

    def set_vision_control_enabled(self, enabled: bool) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.set_vision_control_enabled(enabled)

    def release_vision_control(self) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.release_vision_control()


def _clamp_axis(value: object) -> float:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return 0.0
    if not math.isfinite(numeric):
        return 0.0
    return max(-1.0, min(1.0, numeric))


def _json_object(text: str) -> dict:
    try:
        data = json.loads(text)
    except Exception:
        return {}
    return data if isinstance(data, dict) else {}


def _number_or_none(value: object) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _integer_or_zero(value: object) -> int:
    try:
        return int(value)
    except (TypeError, ValueError):
        return 0


def _endpoint_nodes(endpoints: list) -> list[dict[str, str]]:
    return [
        {
            "name": str(endpoint.node_name),
            "namespace": str(endpoint.node_namespace),
            "type": str(endpoint.topic_type),
        }
        for endpoint in endpoints
    ]
