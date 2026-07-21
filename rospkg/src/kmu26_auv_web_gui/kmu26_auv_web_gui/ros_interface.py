import json
import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import rclpy
from dvl_msgs.msg import DVL
from dvl_msgs.msg import CommandResponse
from dvl_msgs.msg import ConfigCommand
from dvl_msgs.msg import ConfigStatus
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
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64
from std_msgs.msg import String


PATH_MIN_DISTANCE_M = 0.01
PATH_MAX_POINTS = 1600
PATH_COMPACT_TARGET_POINTS = 1200
PATH_RECENT_POINTS = 800
PATH_MAX_SINGLE_JUMP_M = 25.0
WEB_CONTROL_TIMEOUT_S = 0.35
WEB_CONTROL_PERIOD_S = 0.05
DVL_MAX_FOM = 0.05
DVL_MIN_ALTITUDE_M = 0.05
DVL_MIN_VALID_BEAMS = 4
DVL_TWIST_STALE_AFTER_S = 0.75
ATTITUDE_MAX_TILT_DEG = 10.0
STILLNESS_MAX_SPEED_MPS = 0.05
READY_MODES = {"ALT_HOLD", "POSHOLD", "GUIDED"}


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


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


def _evenly_sample_path(
    points: list[dict[str, float]], target_count: int
) -> list[dict[str, float]]:
    """Keep the endpoints and overall shape of an older path section."""
    if target_count <= 0:
        return []
    if len(points) <= target_count:
        return list(points)
    if target_count == 1:
        return [points[0]]
    last_index = len(points) - 1
    return [
        points[round(index * last_index / (target_count - 1))]
        for index in range(target_count)
    ]


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
    phase_peak_candidates: str = "/pinger_homing/phase_peak_candidates"
    selected_frequency: str = "/pinger_homing/selected_frequency_hz"
    select_frequency: str = "/pinger_homing/select_frequency_hz"
    rc_mux_status: str = "/control/rc_override_mux/status"
    rc_output: str = "/mavros/rc/override"
    audio: str = "/audio"


class LocalizationRosNode(Node):
    def __init__(self, topic_config: TopicConfig | None = None):
        super().__init__("kmu26_auv_web_gui_bridge")
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
                "/vision/yolo/annotated/compressed", stale_after=1.5
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
            "phase_peak_candidates": TopicHealth(
                self._topic_config.phase_peak_candidates, stale_after=5.0
            ),
            "selected_frequency": TopicHealth(
                self._topic_config.selected_frequency, stale_after=30.0
            ),
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
        self._joy = {"axes": [], "buttons": []}
        self._dvl_config: dict = {}
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
        self._phase_peak_selection = {
            "candidates": [],
            "suggested_frequency_hz": None,
            "selected_frequency_hz": None,
            "confirmed": False,
            "confirmation_source": "",
            "selected_rank": None,
            "request_sequence": 0,
            "acknowledged_sequence": 0,
            "updated_at": "",
            "raw": "",
        }
        self._rc_mux_status = {
            "owner": "unknown",
            "conflict": False,
            "output_enabled": False,
            "publisher_count": 0,
        }
        self._dvl_events: deque[dict] = deque(maxlen=40)
        self._path: list[dict[str, float]] = []
        self._path_break_pending = False
        self._path_reset_count = 0
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
            "detections": {},
            "mission_enabled": False,
            "mission_state": "UNKNOWN",
            "rc_channels": [],
        }
        self._vision_frame_data = b""
        self._vision_frame_content_type = "image/jpeg"

        sensor_qos = qos_profile_sensor_data
        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._dvl_config_pub = self.create_publisher(
            ConfigCommand,
            "/dvl/config/command",
            sensor_qos,
        )
        self._web_joy_pub = self.create_publisher(Joy, "/joy", 10)
        self._vision_enable_pub = self.create_publisher(
            Bool, "/mission/control_enable", 10
        )
        self._phase_frequency_select_pub = self.create_publisher(
            Float64, self._topic_config.select_frequency, state_qos
        )
        self._arm_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self._set_pose_client = self.create_client(SetPose, "/set_pose")
        self.create_subscription(Odometry, self._topic_config.odom, self._on_odom, sensor_qos)
        self.create_subscription(TwistWithCovarianceStamped, "/dvl/twist", self._on_dvl, sensor_qos)
        self.create_subscription(DVL, "/dvl/data", self._on_dvl_data, sensor_qos)
        self.create_subscription(CommandResponse, "/dvl/command/response", self._on_dvl_response, sensor_qos)
        self.create_subscription(ConfigStatus, "/dvl/config/status", self._on_dvl_config, sensor_qos)
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._topic_config.depth,
            self._on_depth,
            sensor_qos,
        )
        self.create_subscription(BatteryState, "/battery", self._on_battery, sensor_qos)
        self.create_subscription(State, self._topic_config.mavros_state, self._on_mavros_state, 20)
        self.create_subscription(Imu, "/mavros/imu/data", self._on_imu, sensor_qos)
        self.create_subscription(Joy, "/joy", self._on_joy, 20)
        self.create_subscription(
            CompressedImage,
            "/vision/yolo/annotated/compressed",
            self._on_vision_image,
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
            String,
            self._topic_config.phase_peak_candidates,
            self._on_phase_peak_candidates,
            state_qos,
        )
        self.create_subscription(
            Float64,
            self._topic_config.selected_frequency,
            self._on_selected_frequency,
            state_qos,
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
            True,
            "",
        )

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
            "topic_types": {
                "odom": list(topic_names_and_types.get(self._topic_config.odom, [])),
                "imu": list(topic_names_and_types.get("/mavros/imu/data", [])),
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
                        "phase_peak_candidates": self._topic_config.phase_peak_candidates,
                        "selected_frequency": self._topic_config.selected_frequency,
                        "select_frequency": self._topic_config.select_frequency,
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
                "dvl_config": dict(self._dvl_config),
                "dvl_events": list(self._dvl_events),
                "path": list(self._path),
                "path_count": len(self._path),
                "path_diagnostics": {
                    "stored_points": len(self._path),
                    "max_points": PATH_MAX_POINTS,
                    "visual_reset_count": self._path_reset_count,
                },
                "web_control": self._web_control_snapshot(),
                "vision": self._vision_snapshot_locked(),
                "pinger_homing_status": dict(self._pinger_homing_status),
                "hydrophone_direction": dict(self._hydrophone_direction),
                "delta_range_m": self._delta_range_m,
                "iq_magnitude": self._iq_magnitude,
                "phase_peak_selection": {
                    "candidates": [
                        dict(candidate)
                        for candidate in self._phase_peak_selection["candidates"]
                    ],
                    **{
                        key: value
                        for key, value in self._phase_peak_selection.items()
                        if key != "candidates"
                    },
                },
                "rc_mux_status": dict(self._rc_mux_status),
                "graph": graph,
            }

    def latest_vision_frame(self) -> tuple[bytes, str, int]:
        with self._lock:
            return (
                self._vision_frame_data,
                self._vision_frame_content_type,
                int(self._vision["frame_sequence"]),
            )

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
            self._path_break_pending = False

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
            self._path_break_pending = False
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
            self._append_path_point(pose.position.x, pose.position.y)

    def _append_path_point(self, x: float, y: float) -> None:
        if not math.isfinite(x) or not math.isfinite(y):
            # Do not alter the localization/control state. Only prevent the
            # renderer from joining a later valid point to this broken sample.
            self._path_break_pending = True
            return
        if self._path_break_pending:
            self._path.clear()
            self._path_break_pending = False
            self._path_reset_count += 1
        if self._path:
            last = self._path[-1]
            distance = math.hypot(x - last["x"], y - last["y"])
            if distance > PATH_MAX_SINGLE_JUMP_M:
                # A localization reset or resume spike must not stretch the
                # visual map. The current pose remains exactly as received.
                self._path.clear()
                self._path_reset_count += 1
            elif distance < PATH_MIN_DISTANCE_M:
                return
        self._path.append({"x": x, "y": y})
        if len(self._path) > PATH_MAX_POINTS:
            recent_count = min(PATH_RECENT_POINTS, len(self._path))
            recent = self._path[-recent_count:]
            history = self._path[:-recent_count]
            history_budget = max(0, PATH_COMPACT_TARGET_POINTS - recent_count)
            self._path = _evenly_sample_path(history, history_budget) + recent

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

    def _on_vision_image(self, msg: CompressedImage) -> None:
        image_format = str(msg.format or "jpeg").lower()
        content_type = "image/png" if "png" in image_format else "image/jpeg"
        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        with self._lock:
            self._health["vision_camera"].tick()
            self._vision_frame_data = bytes(msg.data)
            self._vision_frame_content_type = content_type
            self._vision["frame_sequence"] = int(self._vision["frame_sequence"]) + 1
            self._vision["frame_stamp"] = stamp

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
                "acoustic_estimator_mode": (
                    str(parsed.get("acoustic_estimator_mode", "")) if parsed else ""
                ),
                "navigation_mode": (
                    str(parsed.get("navigation_mode", "")) if parsed else ""
                ),
                "odometry_required": (
                    bool(parsed.get("odometry_required", True)) if parsed else True
                ),
                "odometry_fresh": (
                    bool(parsed.get("odometry_fresh", False)) if parsed else False
                ),
                "imu_fresh": bool(parsed.get("imu_fresh", False)) if parsed else False,
                "depth_fresh": (
                    bool(parsed.get("depth_fresh", False)) if parsed else False
                ),
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

    def _on_phase_peak_candidates(self, msg: String) -> None:
        parsed = _json_object(msg.data)
        raw_candidates = parsed.get("candidates", [])
        candidates: list[dict] = []
        if isinstance(raw_candidates, list):
            for index, item in enumerate(raw_candidates[:10]):
                if not isinstance(item, dict):
                    continue
                frequency_hz = _number_or_none(item.get("frequency_hz"))
                if frequency_hz is None or frequency_hz <= 0.0:
                    continue
                rank = _integer_or_zero(item.get("rank")) or index + 1
                candidates.append(
                    {
                        "rank": rank,
                        "frequency_hz": frequency_hz,
                        "magnitude": _number_or_none(item.get("magnitude")),
                        "snr_db": _number_or_none(item.get("snr_db")),
                        "quality": item.get("quality"),
                        "support_frames": _integer_or_zero(item.get("support_frames")),
                        "support": _number_or_none(item.get("support")),
                        "selectable": bool(item.get("selectable", False)),
                    }
                )
        candidates.sort(key=lambda item: (int(item["rank"]), -float(item["frequency_hz"])))
        with self._lock:
            self._health["phase_peak_candidates"].tick()
            self._phase_peak_selection["candidates"] = candidates
            self._phase_peak_selection["suggested_frequency_hz"] = _number_or_none(
                parsed.get("suggested_frequency_hz")
            )
            self._phase_peak_selection["scanner_ready"] = bool(parsed.get("ready", candidates))
            self._phase_peak_selection["scanner_state"] = str(parsed.get("state", ""))
            self._phase_peak_selection["frames_seen"] = _integer_or_zero(
                parsed.get("frames_seen")
            )
            self._phase_peak_selection["required_support_frames"] = _integer_or_zero(
                parsed.get("required_support_frames")
            )
            self._phase_peak_selection["error"] = str(parsed.get("error", ""))
            self._phase_peak_selection["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
            self._phase_peak_selection["raw"] = msg.data[-2000:]

    def _on_selected_frequency(self, msg: Float64) -> None:
        frequency_hz = _number_or_none(msg.data)
        if frequency_hz is None or frequency_hz <= 0.0:
            return
        with self._lock:
            self._health["selected_frequency"].tick()
            self._phase_peak_selection["selected_frequency_hz"] = frequency_hz
            self._phase_peak_selection["confirmed"] = True
            self._phase_peak_selection["acknowledged_sequence"] = int(
                self._phase_peak_selection["request_sequence"]
            )
            if not self._phase_peak_selection["confirmation_source"]:
                self._phase_peak_selection["confirmation_source"] = "topic"
            self._phase_peak_selection["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")

    def request_phase_frequency(
        self,
        frequency_hz: float,
        source: str,
        rank: int | None = None,
    ) -> int:
        if not math.isfinite(frequency_hz) or frequency_hz <= 0.0:
            raise ValueError("frequency_hz must be finite and positive")
        message = Float64()
        message.data = float(frequency_hz)
        with self._lock:
            self._phase_peak_selection["request_sequence"] = int(
                self._phase_peak_selection["request_sequence"]
            ) + 1
            request_sequence = int(self._phase_peak_selection["request_sequence"])
            self._phase_peak_selection["selected_frequency_hz"] = None
            self._phase_peak_selection["confirmed"] = False
            self._phase_peak_selection["confirmation_source"] = str(source)
            self._phase_peak_selection["selected_rank"] = rank
            self._phase_peak_selection["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
        self._phase_frequency_select_pub.publish(message)
        return request_sequence

    def confirm_manual_phase_frequency(self, frequency_hz: float) -> None:
        self.request_phase_frequency(frequency_hz, "manual", None)
        with self._lock:
            # Manual confirmation is only exposed by the server when no usable
            # scanner result exists. The homing launch receives this exact Hz.
            self._phase_peak_selection["selected_frequency_hz"] = float(frequency_hz)
            self._phase_peak_selection["confirmed"] = True
            self._phase_peak_selection["acknowledged_sequence"] = int(
                self._phase_peak_selection["request_sequence"]
            )

    def phase_frequency_selected(self, request_sequence: int) -> bool:
        with self._lock:
            selected = _number_or_none(
                self._phase_peak_selection.get("selected_frequency_hz")
            )
            return (
                bool(self._phase_peak_selection.get("confirmed", False))
                and selected is not None
                and int(self._phase_peak_selection.get("acknowledged_sequence", 0))
                == int(request_sequence)
            )

    def clear_phase_frequency_selection(self) -> None:
        with self._lock:
            self._phase_peak_selection["selected_frequency_hz"] = None
            self._phase_peak_selection["confirmed"] = False
            self._phase_peak_selection["confirmation_source"] = ""
            self._phase_peak_selection["selected_rank"] = None

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
        success: bool,
        error_message: str,
    ) -> int:
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

    def request_phase_frequency(
        self,
        frequency_hz: float,
        source: str,
        rank: int | None = None,
    ) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        return self.node.request_phase_frequency(frequency_hz, source, rank)

    def confirm_manual_phase_frequency(self, frequency_hz: float) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.confirm_manual_phase_frequency(frequency_hz)

    def wait_for_phase_frequency_selection(
        self, request_sequence: int, timeout_s: float = 1.5
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.node is None:
                return False
            if self.node.phase_frequency_selected(request_sequence):
                return True
            time.sleep(0.02)
        return False

    def clear_phase_frequency_selection(self) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.clear_phase_frequency_selection()

    def reset_dvl_dead_reckoning(self) -> None:
        self.publish_dvl_command("reset_dead_reckoning")

    def clear_path(self) -> None:
        if self.node is None:
            raise RuntimeError("ROS interface is not running")
        self.node.clear_path()

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

    def latest_vision_frame(self) -> tuple[bytes, str, int]:
        if self.node is None:
            return b"", "image/jpeg", 0
        return self.node.latest_vision_frame()

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
