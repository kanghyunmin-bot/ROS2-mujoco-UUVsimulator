"""Lightweight MuJoCo <-> ROS2 bridge with real-robot topic compatibility.

This variant is intentionally trimmed down to match the interfaces observed on the
real vehicle and to reduce runtime overhead versus the original bridge.

Design goals
------------
1. Publish only the ROS2 topics that matter for the real robot bringup.
2. Preserve the public class/signature expected by the simulator.
3. Keep MAVROS-facing topic names compatible with the real robot stack.
4. Avoid heavyweight image/rendering/registry logic unless you add it back later.

Published topics
----------------
Core / simulator-facing:
  /imu/data
  /depth
  /bar30/pressure_pa
  /dvl/velocity
  /dvl/altitude
  /dvl/odometry
  /dvl/data
  /dvl/position
  /rovio/odometry
  /mujoco/ground_truth/pose
  /tf
  /tf_static
  /robot_description

Real robot compatibility / MAVROS-facing (full surface mode):
  /mavros/state
  /mavros/imu/data
  /mavros/imu/data_raw
  /mavros/imu/static_pressure
  /mavros/imu/atm_pressure
  /mavros/vfr_hud
  /mavros/local_position/pose
  /mavros/local_position/odom
  /mavros/local_position/velocity_local
  /mavros/vision_pose/pose
  /mavros/battery
  /mavros/rc/in
  /mavros/rc/out

Compat-only MAVROS output (enable_mavros_surface=False):
  /mavros/vfr_hud

Subscriptions / services
------------------------
  /cmd_vel (TwistStamped)
  /mavros/rc/override
  /mavros/setpoint_raw/local
  /mavros/cmd/arming
  /mavros/set_mode

Notes
-----
- This bridge does *not* try to emulate every MAVROS plugin topic.
- Stereo camera image publishing was removed for performance. Ping360 publishes
  a lightweight on-demand sonar image only when a subscriber is present.
- /mavros/imu/static_pressure defaults to an "internal" pressure stream to
  mimic the real robot behavior you observed. /mavros/imu/atm_pressure carries
  the external depth/Bar30-derived pressure used by your compatibility node.
- /dvl/data and /dvl/position are published on a best-effort basis using the
  installed dvl_msgs definitions, if present. Unknown fields are ignored safely.
"""

from __future__ import annotations

import json
import os
import time
from array import array
from pathlib import Path
from typing import Callable, Optional

import mujoco
import numpy as np

from .ping360_sim import PING360_GRADS_PER_REV, Ping360Config, Ping360Sample, Ping360Simulator
from .ros2_bridge_runtime import PublishQueue, PublisherDemandCache, StaticContextPublisher
from .sitl_transport import SitlTransport, VerticalEstimate


class Ros2Bridge:
    """Runtime adapter between MuJoCo state and ROS2 topics."""

    _POSITION_TARGET_TYPEMASK_X_IGNORE = 0x01
    _POSITION_TARGET_TYPEMASK_Y_IGNORE = 0x02
    _POSITION_TARGET_TYPEMASK_Z_IGNORE = 0x04
    _POSITION_TARGET_TYPEMASK_YAW_IGNORE = 0x400
    _MAV_CMD_CONDITION_YAW = 115

    def __init__(
        self,
        model: mujoco.MjModel,
        command_callback: Callable[[float, float, float, float], None],
        cmd_limit: float = 15.0,
        publish_images: bool = False,
        image_width: int = 640,
        image_height: int = 360,
        sensor_hz: float = 50.0,
        image_hz: float = 10.0,
        enable_sitl: bool = False,
        sitl_ip: str = "127.0.0.1",
        sitl_port: int = 9002,
        sitl_send_port: int = 9003,
        sitl_mavlink_endpoint: str = "",
        sitl_mavlink_servo_hz: float = 20.0,
        sitl_mavlink_target_sysid: int = 0,
        sitl_mavlink_target_compid: int = 0,
        sitl_mavlink_source_sysid: int = 255,
        sitl_mavlink_source_compid: int = 190,
        camera_calib_left: str = "",
        camera_calib_right: str = "",
        enable_ros: bool = True,
        enable_mavros_surface: bool = True,
        enable_ping360: bool = True,
        ping360_config_path: str = "",
        ping360_overrides: Optional[dict] = None,
    ) -> None:
        # Keep the original signature for drop-in compatibility.
        self._legacy_image_request = bool(publish_images)
        del image_width, image_height, image_hz, camera_calib_left, camera_calib_right

        self.model = model
        self.command_callback = command_callback
        self.cmd_limit = float(cmd_limit)
        self.sensor_dt = 1.0 / max(float(sensor_hz), 1e-6)
        self.next_sensor_t = 0.0
        self.last_pub_t = -1.0
        self.enable_sitl = bool(enable_sitl)
        self._enable_ros = bool(enable_ros)
        self._mavros_surface_enabled = bool(enable_mavros_surface)
        self._ros_ok = False
        self._ros_error_reported = False

        # Command shaping / timeout.
        self.cmd_timeout_s = float(np.clip(self._env_to_float("ROS2_UUV_CMD_TIMEOUT_S", 0.45), 0.1, 2.0))
        self.last_cmd_wall = -1.0
        self.cmd_active = False
        self._cmd_filter_norm = np.zeros(4, dtype=np.float64)
        self._cmd_filter_t = -1.0
        self._cmd_deadband_norm = float(np.clip(self._env_to_float("ROS2_UUV_CMD_DEADBAND", 0.0), 0.0, 0.2))
        self._cmd_slew_rate_norm = float(np.clip(self._env_to_float("ROS2_UUV_CMD_SLEW_RATE", 200.0), 0.0, 200.0))

        # RC mapping.
        self._mavros_rc_forward_channel = self._clamp_rc_channel(self._env_to_int("ROS2_UUV_MAVROS_RC_CH_FORWARD", 5) - 1)
        self._mavros_rc_sway_channel = self._clamp_rc_channel(self._env_to_int("ROS2_UUV_MAVROS_RC_CH_SWAY", 6) - 1)
        self._mavros_rc_yaw_channel = self._clamp_rc_channel(self._env_to_int("ROS2_UUV_MAVROS_RC_CH_YAW", 4) - 1)
        self._mavros_rc_heave_channel = self._clamp_rc_channel(self._env_to_int("ROS2_UUV_MAVROS_RC_CH_HEAVE", 3) - 1)
        self._mavros_rc_forward_invert = bool(self._env_to_int("ROS2_UUV_MAVROS_RC_INV_FORWARD", 0))
        self._mavros_rc_sway_invert = bool(self._env_to_int("ROS2_UUV_MAVROS_RC_INV_SWAY", 0))
        self._mavros_rc_yaw_invert = bool(self._env_to_int("ROS2_UUV_MAVROS_RC_INV_YAW", 0))
        self._mavros_rc_heave_invert = bool(self._env_to_int("ROS2_UUV_MAVROS_RC_INV_HEAVE", 1))
        self._mavros_rc_pwm_span = float(np.clip(self._env_to_float("ROS2_UUV_MAVROS_RC_PWM_SPAN", 300.0), 50.0, 700.0))
        self._mavros_rc_override_local_fallback = bool(
            self._env_to_int("ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK", 1)
        )

        # Setpoint emulation.
        self._mavros_setpoint_enabled = bool(self._env_to_int("ROS2_UUV_MAVROS_SETPOINT_ENABLE", 1))
        self._mavros_setpoint_pos_kp = float(self._env_to_float("ROS2_UUV_MAVROS_SETPOINT_POS_KP", 0.55))
        self._mavros_setpoint_heave_kp = float(self._env_to_float("ROS2_UUV_MAVROS_SETPOINT_HEAVE_KP", 0.55))
        self._mavros_setpoint_yaw_kp = float(self._env_to_float("ROS2_UUV_MAVROS_SETPOINT_YAW_KP", 1.2))
        self._mavros_setpoint_timeout_s = float(self._env_to_float("ROS2_UUV_MAVROS_SETPOINT_TIMEOUT_S", 1.0))
        self._mavros_setpoint_last_t = -1.0
        self._mavros_setpoint_pos = None
        self._mavros_setpoint_yaw = None
        self._mavros_pending_yaw_delta = 0.0

        # Battery + state emulation.
        self._mavros_mode = "MANUAL"
        self._mavros_armed = False
        self._mavros_state_pub_hz = float(self._env_to_float("ROS2_UUV_MAVROS_STATE_HZ", 20.0))
        self._mavros_state_next_t = 0.0
        self._mavros_battery_voltage = float(self._env_to_float("ROS2_UUV_MAVROS_BATTERY_VOLTAGE", 16.0))
        self._mavros_battery_current = float(self._env_to_float("ROS2_UUV_MAVROS_BATTERY_CURRENT", 0.0))
        self._mavros_battery_soc = float(self._env_to_float("ROS2_UUV_MAVROS_BATTERY_SOC", 100.0))
        self._mavros_last_rc_override = None
        self._mavros_last_rc_out = None

        # Physics / pressure model.
        self._bar30_surface_pressure_pa = self._env_to_clamped_float("ROS2_UUV_BAR30_SURFACE_PRESSURE_PA", 101325.0, 80000.0, 120000.0)
        self._bar30_water_density = self._env_to_clamped_float("ROS2_UUV_BAR30_WATER_DENSITY", float(self.model.opt.density), 900.0, 1200.0)
        self._bar30_gravity = self._env_to_clamped_float("ROS2_UUV_BAR30_GRAVITY", 9.80665, 9.5, 10.0)
        self._gravity_enu = np.array([0.0, 0.0, -self._bar30_gravity], dtype=np.float64)
        self._imu_acc_clip_mps2 = 16.0 * self._bar30_gravity
        self._static_pressure_source = str(os.environ.get("ROS2_UUV_STATIC_PRESSURE_SOURCE", "internal")).strip().lower()
        if self._static_pressure_source not in {"internal", "external"}:
            self._static_pressure_source = "internal"
        self._internal_pressure_pa = float(self._env_to_float("ROS2_UUV_INTERNAL_PRESSURE_PA", self._bar30_surface_pressure_pa))

        # DVL filtering.
        self._dvl_filter_alpha = self._env_to_clamped_float("ROS2_UUV_DVL_LPF_ALPHA", 1.0, 0.0, 1.0)
        self._dvl_vel_body_filt = None
        self._odom_pos = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self._last_odom_time = -1.0

        # Shared frame transforms.
        self._enu_to_ned = np.diag([1.0, -1.0, -1.0])
        self._bmj_to_frd = np.diag([1.0, -1.0, -1.0])
        self._bmj_to_flu = np.eye(3, dtype=np.float64)
        self._rovio_to_flu = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        self._base_to_rovio = self._rovio_to_flu.T

        # MuJoCo IDs / sensors.
        self.sensor_ids = {
            sname: sid
            for sname in ("imu_quat", "imu_gyro", "imu_acc", "dvl_vel_body", "dvl_altitude")
            if (sid := mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, sname)) >= 0
        }
        self._base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self._imu_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "imu_site")
        self._bar30_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "bar30_site")
        self._dvl_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "dvl_site")
        self._cam_left_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "cam_left_site")
        self._cam_right_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "cam_right_site")
        ping360_default_config = Path(__file__).resolve().parents[1] / "config" / "ping360.json"
        self._ping360_config = Ping360Config.from_file(
            ping360_config_path or ping360_default_config,
            ping360_overrides,
        )
        if not enable_ping360:
            self._ping360_config.enabled = False
        self._ping360_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            self._ping360_config.site_name,
        )
        self._ping360 = Ping360Simulator(self.model, self._ping360_config) if self._ping360_config.enabled else None
        self._ping360_image_lookup_key = None
        self._ping360_image_lookup = None

        # SITL transport.
        self._sitl_transport = None
        self._sitl_prev_vel_sim_t = None
        self._sitl_prev_vel_enu = None
        if self.enable_sitl:
            self._sitl_transport = SitlTransport(
                model=self.model,
                sitl_ip=sitl_ip,
                sitl_port=int(sitl_port),
                sitl_send_port=int(sitl_send_port),
                sitl_mavlink_endpoint=sitl_mavlink_endpoint,
                sitl_mavlink_servo_hz=float(sitl_mavlink_servo_hz),
                sitl_mavlink_target_sysid=int(sitl_mavlink_target_sysid),
                sitl_mavlink_target_compid=int(sitl_mavlink_target_compid),
                sitl_mavlink_source_sysid=int(sitl_mavlink_source_sysid),
                sitl_mavlink_source_compid=int(sitl_mavlink_source_compid),
                enu_to_ned=self._enu_to_ned,
                surface_pressure_pa=self._bar30_surface_pressure_pa,
                water_density=self._bar30_water_density,
                gravity=self._bar30_gravity,
                home_alt_m=float(self._env_to_float("ROS2_UUV_HOME_ALT_M", 0.0)),
                rangefinder_max_m=float(self._env_to_float("ROS2_UUV_SITL_RANGEFINDER_MAX_M", 30.0)),
                command_debug=bool(self._env_to_int("ROS2_UUV_SITL_CMD_DEBUG", 0)),
            )

        # ROS2 runtime init.
        self.node = None
        self.rclpy = None
        self._ros_context = None
        self._executor = None
        self.RCOut = None
        self.DVLMsg = None
        self.DVLDRMsg = None
        self._publisher_demand = PublisherDemandCache(default_probe_period_s=0.25)
        self._static_context_publisher = None
        self._static_tf_published = False
        self._robot_description_text = self._load_robot_description_text()
        self._robot_description_pub_period_s = 1.0
        self._robot_description_next_t = 0.0

        if self._enable_ros:
            self._init_ros()

    # ---------------------------------------------------------------------
    # ROS initialization / teardown
    # ---------------------------------------------------------------------
    def _init_ros(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import (
                PoseStamped,
                PoseWithCovarianceStamped,
                TransformStamped,
                TwistStamped,
                TwistWithCovarianceStamped,
            )
            from nav_msgs.msg import Odometry
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
            from sensor_msgs.msg import BatteryState, FluidPressure, Image, Imu, LaserScan, Range
            from std_msgs.msg import Float32, String
            from tf2_msgs.msg import TFMessage
            from mavros_msgs.msg import OverrideRCIn, PositionTarget, VfrHud
            from mavros_msgs.msg import State as MavrosState
            from mavros_msgs.srv import CommandBool as MavrosCommandBool
            from mavros_msgs.srv import CommandLong as MavrosCommandLong
            from mavros_msgs.srv import SetMode as MavrosSetMode

            try:
                from mavros_msgs.msg import RCOut
            except Exception:
                RCOut = None

            # Optional DVL messages. If they are unavailable we still run.
            try:
                from dvl_msgs.msg import DVL as DVLMsg
                from dvl_msgs.msg import DVLDR as DVLDRMsg
            except Exception:
                DVLMsg = None
                DVLDRMsg = None
            try:
                from rclpy.signals import SignalHandlerOptions
            except Exception:
                SignalHandlerOptions = None
        except ImportError as exc:
            raise RuntimeError(
                "ROS2 packages not found. Install rclpy + sensor_msgs + geometry_msgs + nav_msgs + mavros_msgs."
            ) from exc

        self.rclpy = rclpy
        self.SingleThreadedExecutor = SingleThreadedExecutor
        self.PoseStamped = PoseStamped
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.TransformStamped = TransformStamped
        self.TwistStamped = TwistStamped
        self.TwistWithCovarianceStamped = TwistWithCovarianceStamped
        self.Odometry = Odometry
        self.QoSProfile = QoSProfile
        self.DurabilityPolicy = DurabilityPolicy
        self.HistoryPolicy = HistoryPolicy
        self.ReliabilityPolicy = ReliabilityPolicy
        self.BatteryState = BatteryState
        self.FluidPressure = FluidPressure
        self.Image = Image
        self.Imu = Imu
        self.LaserScan = LaserScan
        self.Range = Range
        self.Float32 = Float32
        self.String = String
        self.TFMessage = TFMessage
        self.OverrideRCIn = OverrideRCIn
        self.PositionTarget = PositionTarget
        self.RCOut = RCOut
        self.VfrHud = VfrHud
        self.MavrosState = MavrosState
        self.MavrosCommandBool = MavrosCommandBool
        self.MavrosCommandLong = MavrosCommandLong
        self.MavrosSetMode = MavrosSetMode
        self.DVLMsg = DVLMsg
        self.DVLDRMsg = DVLDRMsg

        self._ros_context = Context()
        init_kwargs = {"args": None, "context": self._ros_context}
        if SignalHandlerOptions is not None:
            init_kwargs["signal_handler_options"] = SignalHandlerOptions.NO
        self.rclpy.init(**init_kwargs)
        self.node = Node("uuv_mujoco_bridge", context=self._ros_context)
        self._executor = self.SingleThreadedExecutor(context=self._ros_context)
        self._executor.add_node(self.node)

        # Publishers: only the small set used by the real robot stack.
        q10 = 10
        q1 = 1
        tf_qos = self.QoSProfile(depth=50)
        latched_qos = self.QoSProfile(
            depth=1,
            history=self.HistoryPolicy.KEEP_LAST,
            reliability=self.ReliabilityPolicy.RELIABLE,
            durability=self.DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # Core topics.
        self.pub_imu = self.node.create_publisher(self.Imu, "/imu/data", q10)
        self.pub_depth = self.node.create_publisher(self.Float32, "/depth", q10)
        self.pub_depth_pose = self.node.create_publisher(self.PoseWithCovarianceStamped, "/depth/pose", q10)
        self.pub_bar30_pressure = self.node.create_publisher(self.Float32, "/bar30/pressure_pa", q10)
        self.pub_dvl_velocity = self.node.create_publisher(self.TwistStamped, "/dvl/velocity", q10)
        self.pub_dvl_twist = self.node.create_publisher(self.TwistWithCovarianceStamped, "/dvl/twist", q10)
        self.pub_dvl_altitude = self.node.create_publisher(self.Range, "/dvl/altitude", q10)
        self.pub_dvl_odometry = self.node.create_publisher(self.Odometry, "/dvl/odometry", q10)
        self.pub_rovio_odometry = self.node.create_publisher(self.Odometry, "/rovio/odometry", q10)
        self.pub_ground_truth = self.node.create_publisher(self.PoseStamped, "/mujoco/ground_truth/pose", q10)
        self.pub_ping360_image = self.node.create_publisher(self.Image, "/ping360/image", q1)
        self.pub_ping360_scan = self.node.create_publisher(self.LaserScan, "/ping360/scan", q10)
        self.pub_ping360_status = self.node.create_publisher(self.String, "/ping360/status", q10)

        # MAVROS-compatible surface.
        self.pub_mavros_vfr_hud = self.node.create_publisher(self.VfrHud, "/mavros/vfr_hud", q10)
        if self._mavros_surface_enabled:
            self.pub_mavros_state = self.node.create_publisher(self.MavrosState, "/mavros/state", q10)
            self.pub_mavros_imu_data = self.node.create_publisher(self.Imu, "/mavros/imu/data", q10)
            self.pub_mavros_imu_data_raw = self.node.create_publisher(self.Imu, "/mavros/imu/data_raw", q10)
            self.pub_mavros_imu_static_pressure = self.node.create_publisher(self.FluidPressure, "/mavros/imu/static_pressure", q10)
            self.pub_mavros_imu_atm_pressure = self.node.create_publisher(self.FluidPressure, "/mavros/imu/atm_pressure", q10)
            self.pub_mavros_local_pose = self.node.create_publisher(self.PoseStamped, "/mavros/local_position/pose", q10)
            self.pub_mavros_local_odom = self.node.create_publisher(self.Odometry, "/mavros/local_position/odom", q10)
            self.pub_mavros_local_vel = self.node.create_publisher(self.TwistStamped, "/mavros/local_position/velocity_local", q10)
            self.pub_mavros_vision_pose = self.node.create_publisher(self.PoseStamped, "/mavros/vision_pose/pose", q10)
            self.pub_mavros_battery = self.node.create_publisher(self.BatteryState, "/mavros/battery", q10)
            self.pub_mavros_rc_in = self.node.create_publisher(self.OverrideRCIn, "/mavros/rc/in", q10)
            self.pub_mavros_rc_out = self.node.create_publisher(self.RCOut, "/mavros/rc/out", q10) if self.RCOut else None
        else:
            self.pub_mavros_state = None
            self.pub_mavros_imu_data = None
            self.pub_mavros_imu_data_raw = None
            self.pub_mavros_imu_static_pressure = None
            self.pub_mavros_imu_atm_pressure = None
            self.pub_mavros_local_pose = None
            self.pub_mavros_local_odom = None
            self.pub_mavros_local_vel = None
            self.pub_mavros_vision_pose = None
            self.pub_mavros_battery = None
            self.pub_mavros_rc_in = None
            self.pub_mavros_rc_out = None

        # DVL real-robot topic compatibility.
        self.pub_dvl_data = self.node.create_publisher(self.DVLMsg, "/dvl/data", q10) if self.DVLMsg else None
        self.pub_dvl_position = self.node.create_publisher(self.DVLDRMsg, "/dvl/position", q10) if self.DVLDRMsg else None

        # TF / description.
        self.pub_tf = self.node.create_publisher(self.TFMessage, "/tf", tf_qos)
        self.pub_tf_static = self.node.create_publisher(self.TFMessage, "/tf_static", latched_qos)
        self.pub_robot_description = self.node.create_publisher(self.String, "/robot_description", latched_qos)

        # Subscriptions.
        self.sub_cmd_vel = self.node.create_subscription(self.TwistStamped, "/cmd_vel", self._on_cmd_vel_stamped, q10)
        self.sub_ping360_config = self.node.create_subscription(self.String, "/ping360/config", self._on_ping360_config, q10)
        if self._mavros_surface_enabled:
            self.sub_mavros_rc_override = self.node.create_subscription(self.OverrideRCIn, "/mavros/rc/override", self._on_mavros_rc_override, q10)
            self.sub_mavros_setpoint = self.node.create_subscription(self.PositionTarget, "/mavros/setpoint_raw/local", self._on_mavros_setpoint, q10)
        else:
            self.sub_mavros_rc_override = None
            self.sub_mavros_setpoint = None

        # Services.
        if self._mavros_surface_enabled:
            self.srv_mavros_cmd_arming = self.node.create_service(self.MavrosCommandBool, "/mavros/cmd/arming", self._on_mavros_cmd_arming)
            self.srv_mavros_set_mode = self.node.create_service(self.MavrosSetMode, "/mavros/set_mode", self._on_mavros_set_mode)
            self.srv_mavros_command_long = self.node.create_service(self.MavrosCommandLong, "/mavros/cmd/command", self._on_mavros_command_long)
        else:
            self.srv_mavros_cmd_arming = None
            self.srv_mavros_set_mode = None
            self.srv_mavros_command_long = None

        self._static_tf_specs = self._build_static_tf_specs()
        self._static_context_publisher = StaticContextPublisher(
            tf_static_pub=self.pub_tf_static,
            robot_description_pub=self.pub_robot_description,
            string_factory=self.String,
            build_tf_message=self._build_tf_message,
            safe_publish=self._safe_publish,
            static_tf_specs=self._static_tf_specs,
            robot_description_text=self._robot_description_text,
            robot_description_pub_period_s=self._robot_description_pub_period_s,
        )
        self._ros_ok = True
        if self._mavros_surface_enabled:
            self.node.get_logger().info(
                "ROS2 bridge active (lightweight real-robot interface, full MAVROS surface): "
                "/cmd_vel(TwistStamped), /imu/data, /dvl/*, /rovio/odometry, "
                "/ping360/image, /ping360/scan, /ping360/status, /ping360/config, "
                "/mavros/state, /mavros/imu/*, /mavros/vfr_hud, /mavros/local_position/*, "
                "/mavros/vision_pose/pose, /mavros/battery, /mavros/rc/in, /mavros/rc/out, "
                "/mavros/rc/override, /tf, /robot_description"
            )
        else:
            self.node.get_logger().info(
                "ROS2 bridge active (lightweight real-robot interface, compat MAVROS surface): "
                "/cmd_vel(TwistStamped), /imu/data, /dvl/*, /rovio/odometry, "
                "/ping360/image, /ping360/scan, /ping360/status, /ping360/config, "
                "/mavros/vfr_hud, /tf, /robot_description"
            )
        if self._legacy_image_request:
            self.node.get_logger().warn(
                "Legacy image publishing flags were requested, but the lightweight bridge "
                "does not publish /stereo/* topics."
            )

    # ---------------------------------------------------------------------
    # Utilities
    # ---------------------------------------------------------------------
    @staticmethod
    def _env_to_int(env_name: str, default: int) -> int:
        value = os.getenv(env_name)
        if not value:
            return default
        try:
            return int(value)
        except ValueError:
            return default

    @staticmethod
    def _env_to_float(env_name: str, default: float) -> float:
        value = os.getenv(env_name)
        if not value:
            return float(default)
        try:
            return float(value)
        except ValueError:
            return float(default)

    def _env_to_clamped_float(self, env_name: str, default: float, min_value: float, max_value: float) -> float:
        return float(np.clip(self._env_to_float(env_name, default), min_value, max_value))

    @staticmethod
    def _finite_or_zero(value: float) -> float:
        return float(value) if np.isfinite(value) else 0.0

    @staticmethod
    def _clamp_rc_channel(value: int) -> int:
        return max(0, min(17, int(value)))

    @staticmethod
    def _wrap_angle_rad(angle_rad: float) -> float:
        a = float(angle_rad)
        while a > np.pi:
            a -= 2.0 * np.pi
        while a < -np.pi:
            a += 2.0 * np.pi
        return a

    @staticmethod
    def _rc_channel_value(channels, index: int) -> int:
        return int(channels[index]) if index < len(channels) else 0

    def _rc_to_norm(self, pwm_value: int, invert: bool = False) -> float:
        pwm = int(pwm_value)
        if pwm <= 0 or pwm < 800 or pwm > 2200:
            return 0.0
        norm = (float(pwm) - 1500.0) / max(self._mavros_rc_pwm_span, 1e-6)
        if invert:
            norm = -norm
        return float(np.clip(norm, -1.0, 1.0))

    def _apply_cmd_deadband(self, value: float) -> float:
        value = self._finite_or_zero(value)
        if abs(value) <= self._cmd_deadband_norm:
            return 0.0
        return float(np.clip(value, -1.0, 1.0))

    def _handle_normalized_cmd(self, fwd_norm: float, sway_norm: float, yaw_norm: float, heave_norm: float) -> None:
        now = time.monotonic()
        raw = np.array(
            [
                self._apply_cmd_deadband(fwd_norm),
                self._apply_cmd_deadband(sway_norm),
                self._apply_cmd_deadband(yaw_norm),
                self._apply_cmd_deadband(heave_norm),
            ],
            dtype=np.float64,
        )
        if self._cmd_filter_t < 0.0:
            self._cmd_filter_norm = raw
        else:
            dt = max(0.0, now - self._cmd_filter_t)
            delta = raw - self._cmd_filter_norm
            max_delta = self._cmd_slew_rate_norm * dt
            self._cmd_filter_norm = self._cmd_filter_norm + np.clip(delta, -max_delta, max_delta)
        self._cmd_filter_t = now
        self.last_cmd_wall = now
        self.cmd_active = True
        self.command_callback(
            float(self._cmd_filter_norm[0] * self.cmd_limit),
            float(self._cmd_filter_norm[1] * self.cmd_limit),
            float(self._cmd_filter_norm[2] * self.cmd_limit),
            float(self._cmd_filter_norm[3] * self.cmd_limit),
        )

    def _clear_cmd(self) -> None:
        self._cmd_filter_t = time.monotonic()
        self._cmd_filter_norm = np.zeros(4, dtype=np.float64)
        self.command_callback(0.0, 0.0, 0.0, 0.0)
        self.cmd_active = False

    def _is_ros_context_shutdown_error(self, exc: Exception) -> bool:
        msg = str(exc).lower()
        return (
            "context is not valid" in msg
            or "context is invalid" in msg
            or "rcl_shutdown" in msg
            or "rcl_init() was not called" in msg
        )

    def _safe_publish(self, publisher, msg, label: str) -> bool:
        if publisher is None:
            return True
        try:
            publisher.publish(msg)
            return True
        except Exception as exc:
            if not self._ros_error_reported:
                self._ros_error_reported = True
                print(f"[ros2_bridge] publish blocked ({label}): {exc}", flush=True)
            self._ros_ok = False
            return False

    @staticmethod
    def _sensor_slice(model: mujoco.MjModel, sensor_ids: dict, name: str, data: mujoco.MjData) -> np.ndarray | None:
        sid = sensor_ids.get(name, -1)
        if sid < 0:
            return None
        adr = int(model.sensor_adr[sid])
        dim = int(model.sensor_dim[sid])
        return np.array(data.sensordata[adr : adr + dim], dtype=np.float64)

    @staticmethod
    def _quat_wxyz_to_rotmat(quat: np.ndarray) -> np.ndarray:
        w, x, y, z = quat
        n = float(np.linalg.norm([w, x, y, z]))
        if n <= 1e-12:
            return np.eye(3, dtype=np.float64)
        w, x, y, z = w / n, x / n, y / n, z / n
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _rotmat_to_quat_wxyz(rot: np.ndarray) -> np.ndarray:
        m = rot
        tr = float(m[0, 0] + m[1, 1] + m[2, 2])
        if tr > 0.0:
            s = np.sqrt(tr + 1.0) * 2.0
            w = 0.25 * s
            x = (m[2, 1] - m[1, 2]) / s
            y = (m[0, 2] - m[2, 0]) / s
            z = (m[1, 0] - m[0, 1]) / s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        q = np.array([w, x, y, z], dtype=np.float64)
        n = float(np.linalg.norm(q))
        return q / max(n, 1e-12)

    def _quat_to_yaw(self, quat: np.ndarray) -> float:
        w, x, y, z = quat
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return float(np.arctan2(siny_cosp, cosy_cosp))

    @staticmethod
    def _pressure_abs_from_depth_m(depth_m: float, surface_pressure_pa: float, rho: float, gravity: float) -> float:
        depth = float(max(0.0, depth_m))
        return float(surface_pressure_pa + rho * gravity * depth)

    # ---------------------------------------------------------------------
    # DVL best-effort compatibility builders
    # ---------------------------------------------------------------------
    @staticmethod
    def _set_nested_xyz(msg, attr: str, xyz: np.ndarray) -> None:
        if not hasattr(msg, attr):
            return
        obj = getattr(msg, attr)
        for axis, value in zip(("x", "y", "z"), xyz.tolist()):
            if hasattr(obj, axis):
                setattr(obj, axis, float(value))

    @staticmethod
    def _set_first_attr(msg, names, value) -> None:
        for name in names:
            if hasattr(msg, name):
                setattr(msg, name, value)
                return

    def _build_dvl_msg(self, stamp, vel_body_flu: np.ndarray | None, altitude_m: float | None) -> object | None:
        if self.DVLMsg is None:
            return None
        msg = self.DVLMsg()
        header = getattr(msg, "header", None)
        if header is not None:
            header.stamp = stamp
            if hasattr(header, "frame_id"):
                header.frame_id = "dvl_link"
        if vel_body_flu is not None:
            self._set_nested_xyz(msg, "velocity", vel_body_flu)
            self._set_nested_xyz(msg, "vel", vel_body_flu)
            self._set_first_attr(msg, ("velocity_x", "vx", "surge_velocity"), float(vel_body_flu[0]))
            self._set_first_attr(msg, ("velocity_y", "vy", "sway_velocity"), float(vel_body_flu[1]))
            self._set_first_attr(msg, ("velocity_z", "vz", "heave_velocity"), float(vel_body_flu[2]))
        if altitude_m is not None and np.isfinite(altitude_m):
            self._set_first_attr(msg, ("altitude", "range", "height"), float(altitude_m))
        self._set_first_attr(msg, ("valid", "is_valid", "bottom_lock"), True)
        return msg

    def _build_dvldr_msg(self, stamp, position_flu: np.ndarray, quat_ros: np.ndarray) -> object | None:
        if self.DVLDRMsg is None:
            return None
        msg = self.DVLDRMsg()
        header = getattr(msg, "header", None)
        if header is not None:
            header.stamp = stamp
            if hasattr(header, "frame_id"):
                header.frame_id = "odom"
        self._set_nested_xyz(msg, "position", position_flu)
        self._set_nested_xyz(msg, "pose", position_flu)
        self._set_first_attr(msg, ("x", "position_x"), float(position_flu[0]))
        self._set_first_attr(msg, ("y", "position_y"), float(position_flu[1]))
        self._set_first_attr(msg, ("z", "position_z"), float(position_flu[2]))
        if hasattr(msg, "roll") or hasattr(msg, "pitch") or hasattr(msg, "yaw"):
            # Derive yaw only; roll/pitch are less critical for compatibility.
            self._set_first_attr(msg, ("yaw",), float(self._quat_to_yaw(quat_ros)))
            self._set_first_attr(msg, ("roll",), 0.0)
            self._set_first_attr(msg, ("pitch",), 0.0)
        return msg

    # ---------------------------------------------------------------------
    # Message builders
    # ---------------------------------------------------------------------
    def _build_pose(self, stamp, frame_id: str, pos: np.ndarray, quat: np.ndarray):
        msg = self.PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])
        msg.pose.orientation.w = float(quat[0])
        msg.pose.orientation.x = float(quat[1])
        msg.pose.orientation.y = float(quat[2])
        msg.pose.orientation.z = float(quat[3])
        return msg

    def _build_twist(self, stamp, frame_id: str, linear: np.ndarray):
        msg = self.TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.twist.linear.x = float(linear[0])
        msg.twist.linear.y = float(linear[1])
        msg.twist.linear.z = float(linear[2])
        return msg

    def _build_twist_cov(
        self,
        stamp,
        frame_id: str,
        linear: np.ndarray,
        angular: np.ndarray | None = None,
        linear_cov_diag: tuple[float, float, float] | None = None,
        angular_cov_diag: tuple[float, float, float] | None = None,
    ):
        msg = self.TwistWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.twist.twist.linear.x = float(linear[0])
        msg.twist.twist.linear.y = float(linear[1])
        msg.twist.twist.linear.z = float(linear[2])
        ang = np.zeros(3, dtype=np.float64) if angular is None else np.asarray(angular, dtype=np.float64)
        msg.twist.twist.angular.x = float(ang[0])
        msg.twist.twist.angular.y = float(ang[1])
        msg.twist.twist.angular.z = float(ang[2])
        if linear_cov_diag is not None:
            msg.twist.covariance[0] = float(linear_cov_diag[0])
            msg.twist.covariance[7] = float(linear_cov_diag[1])
            msg.twist.covariance[14] = float(linear_cov_diag[2])
        if angular_cov_diag is not None:
            msg.twist.covariance[21] = float(angular_cov_diag[0])
            msg.twist.covariance[28] = float(angular_cov_diag[1])
            msg.twist.covariance[35] = float(angular_cov_diag[2])
        return msg

    def _build_imu(self, stamp, quat: np.ndarray, gyro: np.ndarray, acc: np.ndarray, frame_id: str = "imu_link"):
        msg = self.Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.orientation.w = float(quat[0])
        msg.orientation.x = float(quat[1])
        msg.orientation.y = float(quat[2])
        msg.orientation.z = float(quat[3])
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration.x = float(acc[0])
        msg.linear_acceleration.y = float(acc[1])
        msg.linear_acceleration.z = float(acc[2])
        msg.orientation_covariance[0] = 1e-4
        msg.orientation_covariance[4] = 1e-4
        msg.orientation_covariance[8] = 1e-4
        msg.angular_velocity_covariance[0] = 5e-4
        msg.angular_velocity_covariance[4] = 5e-4
        msg.angular_velocity_covariance[8] = 5e-4
        msg.linear_acceleration_covariance[0] = 1e-2
        msg.linear_acceleration_covariance[4] = 1e-2
        msg.linear_acceleration_covariance[8] = 1e-2
        return msg

    @staticmethod
    def _apply_real_mavros_imu_covariance(msg) -> None:
        # Median covariance from the April 1 /mavros/imu/data bag.
        for idx in (0, 4, 8):
            msg.orientation_covariance[idx] = 1e-4
            msg.angular_velocity_covariance[idx] = 1.2184700254281e-7
            msg.linear_acceleration_covariance[idx] = 9.0e-8

    def _build_depth_pose(self, stamp, depth_m: float):
        msg = self.PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        # Real depth node uses z-up odom convention; positive depth is -z.
        msg.pose.pose.position.z = -float(max(0.0, depth_m))
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance[0] = 0.0
        msg.pose.covariance[7] = 0.0
        msg.pose.covariance[14] = 0.05
        return msg

    def _build_odom(self, stamp, frame_id: str, child_frame: str, pos: np.ndarray, quat: np.ndarray, vel: np.ndarray):
        msg = self.Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.w = float(quat[0])
        msg.pose.pose.orientation.x = float(quat[1])
        msg.pose.pose.orientation.y = float(quat[2])
        msg.pose.pose.orientation.z = float(quat[3])
        msg.twist.twist.linear.x = float(vel[0])
        msg.twist.twist.linear.y = float(vel[1])
        msg.twist.twist.linear.z = float(vel[2])
        return msg

    def _build_pressure(self, stamp, pressure_pa: float, frame_id: str = "base_link"):
        msg = self.FluidPressure()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.fluid_pressure = float(pressure_pa)
        msg.variance = 0.0
        return msg

    def _build_range(self, stamp, distance_m: float, frame_id: str = "dvl_link"):
        msg = self.Range()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.radiation_type = self.Range.ULTRASOUND
        msg.field_of_view = 0.25
        msg.min_range = 0.05
        msg.max_range = 30.0
        msg.range = float("inf") if distance_m < 0.0 else float(distance_m)
        return msg

    def _build_ping360_image(self, stamp, sample: Ping360Sample):
        rendered = self._render_ping360_polar_image(sample)
        msg = self.Image()
        msg.header.stamp = stamp
        msg.header.frame_id = self._ping360_config.frame_id
        msg.height = int(rendered.shape[0])
        msg.width = int(rendered.shape[1])
        msg.encoding = "mono8"
        msg.is_bigendian = 0
        msg.step = int(rendered.shape[1])
        msg.data = array("B", rendered.astype(np.uint8, copy=False).tobytes())
        return msg

    def _build_ping360_scan(self, stamp, sample: Ping360Sample):
        msg = self.LaserScan()
        msg.header.stamp = stamp
        msg.header.frame_id = self._ping360_config.frame_id
        angle_increment = 2.0 * np.pi / float(PING360_GRADS_PER_REV)
        msg.angle_min = 0.0
        msg.angle_max = 2.0 * np.pi - angle_increment
        msg.angle_increment = angle_increment
        msg.time_increment = float(sample.settings.profile_period_s)
        msg.scan_time = float(sample.settings.scan_period_s)
        msg.range_min = float(self._ping360_config.min_range_m)
        msg.range_max = float(sample.settings.effective_range_m)
        ranges = np.asarray(sample.ranges_m, dtype=np.float32)
        ranges = np.where(np.isfinite(ranges), ranges, np.inf).astype(np.float32, copy=False)
        msg.ranges = ranges.tolist()
        msg.intensities = np.asarray(sample.intensities, dtype=np.float32).tolist()
        return msg

    def _build_ping360_status(self, stamp, sample: Ping360Sample):
        msg = self.String()
        payload = sample.status_dict()
        payload["updated"] = bool(sample.updated)
        payload["stamp"] = {
            "sec": int(getattr(stamp, "sec", 0)),
            "nanosec": int(getattr(stamp, "nanosec", 0)),
        }
        msg.data = json.dumps(payload, sort_keys=True)
        return msg

    def _ping360_polar_lookup(self, image_size: int, number_of_samples: int) -> dict[str, np.ndarray]:
        size = int(np.clip(image_size, 128, 1200))
        samples = max(1, int(number_of_samples))
        key = (size, samples)
        if self._ping360_image_lookup_key == key and self._ping360_image_lookup is not None:
            return self._ping360_image_lookup

        yy, xx = np.indices((size, size), dtype=np.float32)
        center = (float(size) - 1.0) * 0.5
        radius_px = max(center - 1.0, 1.0)
        dx = xx - center
        dy = center - yy
        rr = np.sqrt(dx * dx + dy * dy) / radius_px
        mask = rr <= 1.0
        angle = np.mod(np.arctan2(dy, dx), 2.0 * np.pi)
        grad_float = angle * float(PING360_GRADS_PER_REV) / (2.0 * np.pi)
        angle_idx = np.floor(grad_float + 0.5).astype(np.int16) % PING360_GRADS_PER_REV
        range_idx = np.clip(np.floor(rr * float(samples - 1) + 0.5), 0, samples - 1).astype(np.int32)

        ring_width = max(1.2 / radius_px, 0.002)
        ring_mask = np.zeros_like(mask, dtype=bool)
        for frac in (0.25, 0.50, 0.75, 1.0):
            ring_mask |= mask & (np.abs(rr - frac) <= ring_width)
        spoke_dist = np.abs(((grad_float + 25.0) % 50.0) - 25.0)
        spoke_mask = mask & (rr > 0.04) & (spoke_dist <= 0.45)

        lookup = {
            "mask": mask,
            "angle_idx": angle_idx,
            "range_idx": range_idx,
            "ring_mask": ring_mask,
            "spoke_mask": spoke_mask,
            "rr": rr,
        }
        self._ping360_image_lookup_key = key
        self._ping360_image_lookup = lookup
        return lookup

    def _render_ping360_polar_image(self, sample: Ping360Sample) -> np.ndarray:
        raw = np.asarray(sample.image, dtype=np.uint8)
        size = int(np.clip(self._ping360_config.image_size_px, 128, 1200))
        lookup = self._ping360_polar_lookup(size, raw.shape[1])
        out = np.zeros((size, size), dtype=np.uint8)
        mask = lookup["mask"]
        out[mask] = 5

        angle_idx = lookup["angle_idx"]
        range_idx = lookup["range_idx"]
        values = raw[angle_idx[mask], range_idx[mask]].astype(np.float32)
        noise_gate = float(self._ping360_config.noise_floor) + 0.75 * float(self._ping360_config.speckle_std)
        display_gain = float(max(self._ping360_config.image_display_gain, 0.0))
        returns = np.clip((values - noise_gate) * display_gain, 0.0, 255.0).astype(np.uint8)
        out[mask] = np.maximum(out[mask], returns)

        out[lookup["spoke_mask"]] = np.maximum(out[lookup["spoke_mask"]], 18)
        out[lookup["ring_mask"]] = np.maximum(out[lookup["ring_mask"]], 30)

        def grad_distance(a: np.ndarray, grad: int) -> np.ndarray:
            forward = (a.astype(np.int16) - int(grad)) % PING360_GRADS_PER_REV
            backward = (int(grad) - a.astype(np.int16)) % PING360_GRADS_PER_REV
            return np.minimum(forward, backward)

        rr = lookup["rr"]
        for boundary_grad in (sample.settings.start_angle_grad, sample.settings.stop_angle_grad):
            boundary_mask = mask & (rr > 0.06) & (grad_distance(angle_idx, int(boundary_grad)) <= 1)
            out[boundary_mask] = np.maximum(out[boundary_mask], 55)

        sweep_mask = mask & (rr > 0.04) & (grad_distance(angle_idx, int(sample.angle_grad)) <= 1)
        out[sweep_mask] = np.maximum(out[sweep_mask], 145)

        center_mask = mask & (rr <= 0.018)
        out[center_mask] = 180
        out[~mask] = 0
        return out

    def _build_battery(self, stamp):
        msg = self.BatteryState()
        msg.header.stamp = stamp
        msg.header.frame_id = "base_link"
        msg.voltage = float(self._mavros_battery_voltage)
        msg.current = float(self._mavros_battery_current)
        msg.percentage = float(np.clip(self._mavros_battery_soc, 0.0, 100.0) / 100.0)
        msg.charge = -1.0
        msg.capacity = -1.0
        return msg

    def _build_mavros_state(self, stamp):
        msg = self.MavrosState()
        header = getattr(msg, "header", None)
        if header is not None:
            header.stamp = stamp
            if hasattr(header, "frame_id"):
                header.frame_id = "base_link"
        if hasattr(msg, "connected"):
            msg.connected = True
        if hasattr(msg, "armed"):
            msg.armed = bool(self._mavros_armed)
        if hasattr(msg, "guided"):
            msg.guided = self._mavros_mode == "GUIDED"
        if hasattr(msg, "manual_input"):
            msg.manual_input = True
        if hasattr(msg, "mode"):
            msg.mode = str(self._mavros_mode)
        if hasattr(msg, "system_status"):
            msg.system_status = 0
        return msg

    def _build_vfr_hud(self, stamp, compatibility_altitude: float):
        msg = self.VfrHud()
        if hasattr(msg, "header"):
            msg.header.stamp = stamp
            msg.header.frame_id = "base_link"
        # Real robot compatibility quirk: vfr_hud.altitude is used downstream
        # as a pressure-like passthrough input by vfr2atm_pressure.
        if hasattr(msg, "airspeed"):
            msg.airspeed = 0.0
        if hasattr(msg, "groundspeed"):
            msg.groundspeed = 0.0
        if hasattr(msg, "heading"):
            msg.heading = 0
        if hasattr(msg, "throttle"):
            msg.throttle = 0.0
        if hasattr(msg, "altitude"):
            msg.altitude = float(compatibility_altitude)
        if hasattr(msg, "climb"):
            msg.climb = 0.0
        return msg

    def _build_tf_message(self, stamp, specs) -> object | None:
        transforms = []
        for parent, child, translation, quat in specs:
            t = self.TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = float(translation[0])
            t.transform.translation.y = float(translation[1])
            t.transform.translation.z = float(translation[2])
            t.transform.rotation.w = float(quat[0])
            t.transform.rotation.x = float(quat[1])
            t.transform.rotation.y = float(quat[2])
            t.transform.rotation.z = float(quat[3])
            transforms.append(t)
        if not transforms:
            return None
        msg = self.TFMessage()
        msg.transforms = transforms
        return msg

    @staticmethod
    def _quat_identity() -> np.ndarray:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    @staticmethod
    def _quat_x_180() -> np.ndarray:
        return np.array([0.0, 1.0, 0.0, 0.0], dtype=np.float64)

    @classmethod
    def _camera_optical_quat(cls) -> np.ndarray:
        rot_parent_child = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=np.float64)
        return cls._rotmat_to_quat_wxyz(rot_parent_child)

    def _site_local_pose(self, site_id: int, fallback_pos: np.ndarray | None = None, fallback_quat: np.ndarray | None = None):
        pos = np.zeros(3, dtype=np.float64) if fallback_pos is None else np.asarray(fallback_pos, dtype=np.float64)
        quat = self._quat_identity() if fallback_quat is None else np.asarray(fallback_quat, dtype=np.float64)
        if site_id >= 0:
            pos = np.asarray(self.model.site_pos[site_id], dtype=np.float64)
            quat = np.asarray(self.model.site_quat[site_id], dtype=np.float64)
        return pos, quat

    def _build_static_tf_specs(self):
        specs = []
        zero = np.zeros(3, dtype=np.float64)
        ident = self._quat_identity()
        x180 = self._quat_x_180()
        optical_quat = self._camera_optical_quat()

        specs.append(("map", "map_ned", zero, x180))
        specs.append(("odom", "odom_ned", zero, x180))
        specs.append(("base_link", "base_link_frd", zero, x180))
        specs.append(("base_link", "fcu_link", zero, ident))
        specs.append(("base_link", "auv_link", zero, ident))

        imu_pos, imu_quat = self._site_local_pose(self._imu_site_id)
        specs.append(("base_link", "imu_link", imu_pos, imu_quat))

        bar30_pos, bar30_quat = self._site_local_pose(self._bar30_site_id, fallback_pos=np.array([0.0, 0.0, -0.06], dtype=np.float64))
        specs.append(("base_link", "bar30_link", bar30_pos, bar30_quat))

        dvl_pos, dvl_quat = self._site_local_pose(self._dvl_site_id, fallback_pos=np.array([0.0, 0.0, -0.1], dtype=np.float64), fallback_quat=x180)
        specs.append(("base_link", "dvl", dvl_pos, dvl_quat))
        specs.append(("dvl", "dvl_link", zero, ident))

        ping360_pos, ping360_quat = self._site_local_pose(
            self._ping360_site_id,
            fallback_pos=np.array([0.0, 0.0, 0.205], dtype=np.float64),
            fallback_quat=ident,
        )
        specs.append(("base_link", self._ping360_config.frame_id, ping360_pos, ping360_quat))

        cam_left_pos, _ = self._site_local_pose(self._cam_left_site_id, fallback_pos=np.array([0.1493, -0.0225, -0.02], dtype=np.float64))
        cam_right_pos, _ = self._site_local_pose(self._cam_right_site_id, fallback_pos=np.array([0.1493, 0.0225, -0.02], dtype=np.float64))
        camera_link_pos = 0.5 * (cam_left_pos + cam_right_pos)
        specs.append(("base_link", "camera_link", camera_link_pos, ident))
        specs.append(("camera_link", "stereo_left", cam_left_pos - camera_link_pos, ident))
        specs.append(("camera_link", "stereo_right", cam_right_pos - camera_link_pos, ident))
        specs.append(("stereo_left", "stereo_left_optical", zero, optical_quat))
        specs.append(("stereo_right", "stereo_right_optical", zero, optical_quat))
        return specs

    def _load_robot_description_text(self) -> str:
        workspace_root = Path(__file__).resolve().parents[2]
        urdf_candidates = (
            workspace_root / "rospkg" / "kmu26_auv" / "urdf" / "rov.urdf",
            workspace_root / "kmu26_auv" / "urdf" / "rov.urdf",
        )
        mesh_candidates = (
            workspace_root / "rospkg" / "kmu26_auv" / "meshes",
            workspace_root / "kmu26_auv" / "meshes",
        )
        urdf_path = next((path for path in urdf_candidates if path.is_file()), urdf_candidates[0])
        mesh_root = next((path for path in mesh_candidates if path.is_dir()), mesh_candidates[0])
        mesh_uri = f"{mesh_root.resolve().as_uri()}/"
        try:
            text = urdf_path.read_text(encoding="utf-8")
        except OSError:
            return (
                '<robot name="uuv_sim">'
                '<link name="auv_link"><visual><geometry><box size="0.7 0.5 0.3" /></geometry>'
                '<material name="fallback"><color rgba="0.5 0.5 0.5 1.0" /></material>'
                "</visual></link></robot>"
            )
        for prefix in (
            "package://hit25_auv/meshes/",
            "package://hit25_auv_ros2/meshes/",
            "package://kmu26_auv/meshes/",
        ):
            text = text.replace(prefix, mesh_uri)
        return text

    def _publish_static_context(self, stamp, sim_t: float) -> bool:
        if self._static_context_publisher is None:
            return True
        return self._static_context_publisher.publish(stamp, sim_t)

    # ---------------------------------------------------------------------
    # Subscriptions / services
    # ---------------------------------------------------------------------
    def _on_ping360_config(self, msg) -> None:
        try:
            payload = json.loads(str(getattr(msg, "data", "") or "{}"))
        except json.JSONDecodeError as exc:
            if self.node is not None:
                self.node.get_logger().warn(f"invalid /ping360/config JSON: {exc}")
            return
        if not isinstance(payload, dict):
            if self.node is not None:
                self.node.get_logger().warn("/ping360/config must be a JSON object")
            return

        known = set(Ping360Config.__dataclass_fields__)
        current = {
            field_name: getattr(self._ping360_config, field_name)
            for field_name in Ping360Config.__dataclass_fields__
        }
        current.update({key: value for key, value in payload.items() if key in known})
        next_config = Ping360Config(**current)
        self._ping360_config = next_config
        self._ping360_site_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_SITE,
            next_config.site_name,
        )
        self._ping360 = Ping360Simulator(self.model, next_config) if next_config.enabled else None
        self._ping360_image_lookup_key = None
        self._ping360_image_lookup = None
        if self.node is not None:
            settings = self._ping360.settings.as_dict() if self._ping360 is not None else {}
            self.node.get_logger().info(
                "updated /ping360/config "
                f"range={settings.get('effective_range_m', next_config.requested_range_m):.3f}m "
                f"steps={settings.get('num_steps', next_config.num_steps)} "
                f"sector={settings.get('start_angle_grad', next_config.start_angle_grad)}.."
                f"{settings.get('stop_angle_grad', next_config.stop_angle_grad)} grad"
            )

    def _on_cmd_vel_stamped(self, msg) -> None:
        twist = getattr(msg, "twist", msg)
        fwd = float(np.clip(getattr(twist.linear, "x", 0.0), -1.0, 1.0))
        left = float(np.clip(getattr(twist.linear, "y", 0.0), -1.0, 1.0))
        up = float(np.clip(getattr(twist.linear, "z", 0.0), -1.0, 1.0))
        yaw = float(np.clip(getattr(twist.angular, "z", 0.0), -1.0, 1.0))
        if self._sitl_transport is not None:
            self._sitl_transport.send_body_velocity_setpoint(forward_mps=fwd, left_mps=left, up_mps=up, yaw_rate_rad_s=yaw)
            return
        self._handle_normalized_cmd(fwd, left, -yaw, -up)

    def _on_mavros_rc_override(self, msg) -> None:
        channels = getattr(msg, "channels", None)
        if channels is None or len(channels) == 0:
            return
        if self._sitl_transport is not None:
            try:
                self._sitl_transport.send_rc_override([int(v) for v in channels[:8]])
            except Exception:
                pass
        fwd = self._rc_to_norm(self._rc_channel_value(channels, self._mavros_rc_forward_channel), self._mavros_rc_forward_invert)
        sway = self._rc_to_norm(self._rc_channel_value(channels, self._mavros_rc_sway_channel), self._mavros_rc_sway_invert)
        yaw = self._rc_to_norm(self._rc_channel_value(channels, self._mavros_rc_yaw_channel), self._mavros_rc_yaw_invert)
        heave = self._rc_to_norm(self._rc_channel_value(channels, self._mavros_rc_heave_channel), self._mavros_rc_heave_invert)
        if self._sitl_transport is None or self._mavros_rc_override_local_fallback:
            self._handle_normalized_cmd(fwd, sway, yaw, heave)
        try:
            rc_in = self.OverrideRCIn()
            for i in range(18):
                rc_in.channels[i] = int(channels[i]) if i < len(channels) else 0
            self._mavros_last_rc_override = rc_in
        except Exception:
            self._mavros_last_rc_override = None

    def _on_sitl_servo_output_for_ros(self, pwm_values: list[int]) -> None:
        if not self._mavros_surface_enabled or self.RCOut is None:
            self._mavros_last_rc_out = None
            return
        try:
            rc_out = self.RCOut()
            header = getattr(rc_out, "header", None)
            if header is not None and self.node is not None:
                header.stamp = self.node.get_clock().now().to_msg()
                if hasattr(header, "frame_id"):
                    header.frame_id = "fcu"
            channels = [int(v) for v in list(pwm_values)[:18]]
            if hasattr(rc_out, "channels"):
                rc_out.channels = channels
            self._mavros_last_rc_out = rc_out
        except Exception:
            self._mavros_last_rc_out = None

    def _on_mavros_setpoint(self, msg) -> None:
        if not self._mavros_setpoint_enabled:
            return
        frame_local_ned = getattr(self.PositionTarget, "FRAME_LOCAL_NED", 1)
        coordinate_frame = int(getattr(msg, "coordinate_frame", frame_local_ned))
        if coordinate_frame != frame_local_ned:
            return
        tmask = int(getattr(msg, "type_mask", 0))
        target_ned = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        has_pos = False
        pos = getattr(msg, "position", None)
        if pos is None:
            pos = getattr(msg, "position_", None)
        if pos is not None:
            if not (tmask & self._POSITION_TARGET_TYPEMASK_X_IGNORE):
                target_ned[0] = float(getattr(pos, "x", 0.0))
                has_pos = True
            if not (tmask & self._POSITION_TARGET_TYPEMASK_Y_IGNORE):
                target_ned[1] = float(getattr(pos, "y", 0.0))
                has_pos = True
            if not (tmask & self._POSITION_TARGET_TYPEMASK_Z_IGNORE):
                target_ned[2] = float(getattr(pos, "z", 0.0))
                has_pos = True
        if has_pos:
            self._mavros_setpoint_pos = target_ned.copy()
        if not (tmask & self._POSITION_TARGET_TYPEMASK_YAW_IGNORE):
            self._mavros_setpoint_yaw = float(getattr(msg, "yaw", 0.0))
        self._mavros_setpoint_last_t = time.monotonic()

    def _on_mavros_cmd_arming(self, request, response):
        arm_value = bool(getattr(request, "value", False))
        forward_ok = True
        if self._sitl_transport is not None:
            forward_ok = bool(self._sitl_transport.send_arm_command(arm_value))
        if forward_ok:
            self._mavros_armed = arm_value
        if hasattr(response, "success"):
            response.success = bool(forward_ok)
        if hasattr(response, "result"):
            response.result = 0 if forward_ok else 1
        return response

    def _on_mavros_set_mode(self, request, response):
        mode = str(getattr(request, "custom_mode", ""))
        forward_ok = True
        if mode:
            if self._sitl_transport is not None:
                forward_ok = bool(self._sitl_transport.send_set_mode(mode))
            if forward_ok:
                self._mavros_mode = mode
        if hasattr(response, "mode_sent"):
            response.mode_sent = bool(forward_ok and mode)
        if hasattr(response, "success"):
            response.success = bool(forward_ok)
        return response

    def _on_mavros_command_long(self, request, response):
        command = int(getattr(request, "command", 0))
        if command == self._MAV_CMD_CONDITION_YAW:
            angle_deg = float(getattr(request, "param1", 0.0))
            is_relative = float(getattr(request, "param4", 0.0))
            direction = float(getattr(request, "param3", 0.0))
            direction = 1.0 if direction >= 0.0 else -1.0
            delta_rad = np.deg2rad(angle_deg * direction)
            if is_relative > 0.5:
                self._mavros_setpoint_yaw = None
                self._mavros_pending_yaw_delta += float(delta_rad)
            else:
                self._mavros_setpoint_yaw = float(np.deg2rad(angle_deg))
                self._mavros_pending_yaw_delta = 0.0
            self._mavros_setpoint_last_t = time.monotonic()
        if hasattr(response, "success"):
            response.success = True
        if hasattr(response, "result"):
            response.result = 0
        return response

    # ---------------------------------------------------------------------
    # State estimation helpers
    # ---------------------------------------------------------------------
    def _estimate_base_accel_enu(self, sim_t: float, base_vel_enu: np.ndarray) -> np.ndarray:
        accel_enu = np.zeros(3, dtype=np.float64)
        prev_t = self._sitl_prev_vel_sim_t
        prev_vel = self._sitl_prev_vel_enu
        if prev_t is not None and prev_vel is not None:
            dt = sim_t - float(prev_t)
            if 1.0e-4 <= dt <= 0.2:
                accel_fd = (base_vel_enu - prev_vel) / dt
                if np.all(np.isfinite(accel_fd)):
                    accel_enu = np.clip(accel_fd, -self._imu_acc_clip_mps2, self._imu_acc_clip_mps2)
        self._sitl_prev_vel_sim_t = sim_t
        self._sitl_prev_vel_enu = base_vel_enu.copy()
        return accel_enu

    def _estimate_vertical_truth(self, base_pos_enu: np.ndarray, base_vel_enu: np.ndarray) -> VerticalEstimate:
        depth_m = float(max(0.0, -base_pos_enu[2]))
        pos_ned = self._enu_to_ned @ base_pos_enu
        vel_ned = self._enu_to_ned @ base_vel_enu
        pos_ned[2] = depth_m
        vel_ned[2] = -base_vel_enu[2]
        pressure_pa = self._pressure_abs_from_depth_m(depth_m, self._bar30_surface_pressure_pa, self._bar30_water_density, self._bar30_gravity)
        return VerticalEstimate(depth_m=depth_m, pressure_pa=pressure_pa, pos_ned=pos_ned, vel_ned=vel_ned, alt_m=-depth_m)

    def _imu_vectors_in_body(self, data: mujoco.MjData, gyro: np.ndarray | None) -> np.ndarray | None:
        gyro_bmj = np.array(gyro, dtype=np.float64) if gyro is not None else None
        if self._base_id < 0 or self._imu_site_id < 0 or gyro_bmj is None:
            return gyro_bmj
        try:
            base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
            imu_rot_enu = data.site_xmat[self._imu_site_id].reshape(3, 3).copy()
            rot_bmj_imu = base_rot_enu.T @ imu_rot_enu
            return rot_bmj_imu @ gyro_bmj
        except Exception:
            return gyro_bmj

    def _specific_force_body(self, data: mujoco.MjData, acc_sensor_bmj: np.ndarray | None, base_vel_enu: np.ndarray, sim_t: float) -> np.ndarray | None:
        if self._base_id < 0:
            return None
        try:
            base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
        except Exception:
            return None

        if acc_sensor_bmj is not None and np.all(np.isfinite(acc_sensor_bmj)):
            specific_force_bmj = np.asarray(acc_sensor_bmj, dtype=np.float64)
        else:
            lin_acc_enu = self._estimate_base_accel_enu(sim_t, base_vel_enu)
            lin_acc_bmj = base_rot_enu.T @ lin_acc_enu
            gravity_bmj = base_rot_enu.T @ self._gravity_enu
            specific_force_bmj = lin_acc_bmj - gravity_bmj
        return np.nan_to_num(np.clip(specific_force_bmj, -self._imu_acc_clip_mps2, self._imu_acc_clip_mps2), nan=0.0, posinf=0.0, neginf=0.0)

    def _dvl_velocity_body(self, data: mujoco.MjData, dvl_vel_sensor: np.ndarray | None, gyro_bmj: np.ndarray | None) -> np.ndarray | None:
        if dvl_vel_sensor is None:
            return None
        vel_body = np.array(dvl_vel_sensor, dtype=np.float64)
        if self._base_id >= 0 and self._dvl_site_id >= 0:
            try:
                base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
                dvl_rot_enu = data.site_xmat[self._dvl_site_id].reshape(3, 3).copy()
                rot_bmj_dvl = base_rot_enu.T @ dvl_rot_enu
                vel_body = rot_bmj_dvl @ vel_body
                if gyro_bmj is not None:
                    r_enu = data.site_xpos[self._dvl_site_id] - data.xpos[self._base_id]
                    r_body = base_rot_enu.T @ r_enu
                    vel_body = vel_body - np.cross(gyro_bmj, r_body)
            except Exception:
                pass
        vel_body = np.nan_to_num(vel_body, nan=0.0, posinf=0.0, neginf=0.0)
        alpha = float(self._dvl_filter_alpha)
        if alpha <= 0.0:
            self._dvl_vel_body_filt = vel_body
            return vel_body
        if self._dvl_vel_body_filt is None:
            self._dvl_vel_body_filt = vel_body
        else:
            self._dvl_vel_body_filt = ((1.0 - alpha) * self._dvl_vel_body_filt) + (alpha * vel_body)
        return self._dvl_vel_body_filt.copy()

    def _apply_mavros_setpoint(self, base_pos_enu: np.ndarray, base_rot_enu: np.ndarray) -> None:
        if not self._mavros_setpoint_enabled or self._sitl_transport is not None:
            return
        if self._mavros_setpoint_last_t < 0.0:
            return
        if (time.monotonic() - self._mavros_setpoint_last_t) > self._mavros_setpoint_timeout_s:
            self._mavros_setpoint_pos = None
            self._mavros_setpoint_yaw = None
            self._mavros_pending_yaw_delta = 0.0
            self._clear_cmd()
            return
        if self._mavros_pending_yaw_delta != 0.0:
            current_yaw_ned = self._quat_to_yaw(self._rotmat_to_quat_wxyz(self._enu_to_ned @ base_rot_enu))
            if self._mavros_setpoint_yaw is None:
                self._mavros_setpoint_yaw = float(current_yaw_ned)
            self._mavros_setpoint_yaw = float(self._mavros_setpoint_yaw + self._mavros_pending_yaw_delta)
            self._mavros_pending_yaw_delta = 0.0

        fwd_cmd = sway_cmd = yaw_cmd = heave_cmd = 0.0
        if self._mavros_setpoint_pos is not None:
            target_enu = self._enu_to_ned @ self._mavros_setpoint_pos
            err_world = target_enu - base_pos_enu
            err_body = base_rot_enu.T @ err_world
            fwd_cmd = self._mavros_setpoint_pos_kp * err_body[0]
            sway_cmd = self._mavros_setpoint_pos_kp * err_body[1]
            heave_cmd = -self._mavros_setpoint_heave_kp * err_body[2]
        if self._mavros_setpoint_yaw is not None:
            current_yaw_ned = self._quat_to_yaw(self._rotmat_to_quat_wxyz(self._enu_to_ned @ base_rot_enu))
            yaw_err = self._wrap_angle_rad(self._mavros_setpoint_yaw - current_yaw_ned)
            yaw_cmd = self._mavros_setpoint_yaw_kp * yaw_err
        self._handle_normalized_cmd(float(fwd_cmd), float(sway_cmd), float(yaw_cmd), float(heave_cmd))

    # ---------------------------------------------------------------------
    # Public API used by simulator loop
    # ---------------------------------------------------------------------
    def set_sitl_servo_handler(self, callback: Optional[Callable[[list[int]], None]]) -> None:
        if self._sitl_transport is not None:
            def wrapped_servo_handler(pwm_values: list[int]) -> None:
                self._on_sitl_servo_output_for_ros(pwm_values)
                if callback is not None:
                    callback(pwm_values)

            self._sitl_transport.set_servo_handler(wrapped_servo_handler)

    def spin_once(self) -> None:
        if self.enable_sitl and self._sitl_transport is not None:
            self._sitl_transport.poll_servo()
        if not self._enable_ros or not self._ros_ok:
            if self.cmd_active and (time.monotonic() - self.last_cmd_wall > self.cmd_timeout_s):
                self._clear_cmd()
            return
        try:
            if self._executor is None:
                return
            self._executor.spin_once(timeout_sec=0.0)
        except Exception as exc:
            if not self._ros_error_reported:
                self._ros_error_reported = True
                print(f"[ros2_bridge] callbacks disabled: {exc}", flush=True)
            self._ros_ok = False
            return
        if self.cmd_active and (time.monotonic() - self.last_cmd_wall > self.cmd_timeout_s):
            self._clear_cmd()

    def publish(self, data: mujoco.MjData) -> None:
        sim_t = float(data.time)
        if sim_t <= self.last_pub_t:
            return
        if self.last_pub_t >= 0.0 and sim_t + 1e-9 < (self.last_pub_t + self.sensor_dt):
            return
        self.last_pub_t = sim_t

        if self.enable_sitl and self._sitl_transport is not None:
            self._sitl_transport.poll_servo()

        if self._base_id < 0:
            return

        # Shared truth state.
        try:
            base_pos_enu = np.array(data.xpos[self._base_id], dtype=np.float64)
            base_rot_enu = data.xmat[self._base_id].reshape(3, 3).copy()
            quat_base = np.array(data.xquat[self._base_id], dtype=np.float64)
        except Exception:
            return
        try:
            cvel = np.array(data.cvel[self._base_id], dtype=np.float64)
            base_vel_enu = cvel[3:6].copy() if cvel.size >= 6 else np.zeros(3, dtype=np.float64)
        except Exception:
            base_vel_enu = np.zeros(3, dtype=np.float64)

        gyro = self._sensor_slice(self.model, self.sensor_ids, "imu_gyro", data)
        acc_sensor = self._sensor_slice(self.model, self.sensor_ids, "imu_acc", data)
        dvl_vel_sensor = self._sensor_slice(self.model, self.sensor_ids, "dvl_vel_body", data)
        dvl_altitude_sensor = self._sensor_slice(self.model, self.sensor_ids, "dvl_altitude", data)
        dvl_altitude_m = float(dvl_altitude_sensor[0]) if dvl_altitude_sensor is not None and dvl_altitude_sensor.size > 0 else None
        if self._sitl_transport is not None and dvl_altitude_m is None:
            try:
                dvl_altitude_m = self._sitl_transport.sitl_rangefinder_from_model(data)
            except Exception:
                dvl_altitude_m = None

        gyro_bmj = self._imu_vectors_in_body(data, gyro)
        acc_sensor_bmj = np.array(acc_sensor, dtype=np.float64) if acc_sensor is not None else None
        if acc_sensor_bmj is not None and self._imu_site_id >= 0:
            try:
                imu_rot_enu = data.site_xmat[self._imu_site_id].reshape(3, 3).copy()
                rot_bmj_imu = base_rot_enu.T @ imu_rot_enu
                acc_sensor_bmj = rot_bmj_imu @ acc_sensor_bmj
            except Exception:
                pass
        acc_bmj = self._specific_force_body(data, acc_sensor_bmj, base_vel_enu, sim_t)
        dvl_vel_body_bmj = self._dvl_velocity_body(data, dvl_vel_sensor, gyro_bmj)
        vertical_est = self._estimate_vertical_truth(base_pos_enu, base_vel_enu)
        bar30_pressure_pa = float(vertical_est.pressure_pa)

        if self._sitl_transport is not None and gyro_bmj is not None and acc_bmj is not None:
            gyro_frd = self._bmj_to_frd @ gyro_bmj
            acc_frd = self._bmj_to_frd @ acc_bmj
            rot_ned_bfrd = self._enu_to_ned @ base_rot_enu @ self._bmj_to_frd.T
            quat_ned_bfrd = self._rotmat_to_quat_wxyz(rot_ned_bfrd)
            self._sitl_transport.send_state(
                sim_t,
                gyro_frd,
                acc_frd,
                vertical_est,
                quat_ned_bfrd,
                rangefinder_distance_m=dvl_altitude_m,
                pressure_pa=bar30_pressure_pa,
            )

        if not self._enable_ros or not self._ros_ok:
            return

        try:
            stamp = self.node.get_clock().now().to_msg()
        except Exception as exc:
            if not self._ros_error_reported:
                self._ros_error_reported = True
                print(f"[ros2_bridge] timestamp acquisition failed: {exc}", flush=True)
            self._ros_ok = False
            return

        if not self._publish_static_context(stamp, sim_t):
            return

        # Derived ROS-frame values.
        quat_ros = self._rotmat_to_quat_wxyz(base_rot_enu @ self._bmj_to_flu.T)
        gyro_ros = self._bmj_to_flu @ gyro_bmj if gyro_bmj is not None else np.zeros(3, dtype=np.float64)
        acc_ros = self._bmj_to_flu @ acc_bmj if acc_bmj is not None else np.zeros(3, dtype=np.float64)
        dvl_vel_body_ros = self._bmj_to_flu @ dvl_vel_body_bmj if dvl_vel_body_bmj is not None else None
        dvl_vel_body_frd = self._bmj_to_frd @ dvl_vel_body_bmj if dvl_vel_body_bmj is not None else None

        # Integrate DVL odometry.
        if dvl_vel_body_bmj is not None:
            dt = self.sensor_dt if self._last_odom_time < 0.0 else float(np.clip(sim_t - self._last_odom_time, 1e-4, 0.2))
            self._last_odom_time = sim_t
            rot_world_body = self._quat_wxyz_to_rotmat(quat_base)
            vel_world = rot_world_body @ dvl_vel_body_bmj
            self._odom_pos += vel_world * dt
        else:
            self._last_odom_time = -1.0

        self._apply_mavros_setpoint(base_pos_enu, base_rot_enu)

        # Decide what static_pressure should emulate.
        if self._static_pressure_source == "external":
            static_pressure_pa = bar30_pressure_pa
        else:
            static_pressure_pa = self._internal_pressure_pa

        jobs = PublishQueue(self._publisher_demand, sim_t)
        zero_vel = np.zeros(3, dtype=np.float64)

        imu_msg = None
        imu_raw_msg = None
        mavros_imu_msg = None
        mavros_imu_raw_msg = None
        depth_msg = None
        depth_pose_msg = None
        baro_msg = None
        ground_truth_msg = None
        dvl_velocity_msg = None
        dvl_twist_msg = None
        dvl_altitude_msg = None
        mavros_state_msg = None
        mavros_vfr_hud_msg = None
        mavros_static_pressure_msg = None
        mavros_atm_pressure_msg = None
        mavros_battery_msg = None
        mavros_local_pose_msg = None
        mavros_vision_pose_msg = None
        odom_local_msg = None
        rovio_odom_msg = None
        dvl_data_msg = None
        dvl_pos_msg = None
        tf_msg = None
        ping360_sample = None
        ping360_image_msg = None
        ping360_scan_msg = None
        ping360_status_msg = None

        def get_imu_msg():
            nonlocal imu_msg
            if imu_msg is None:
                imu_msg = self._build_imu(stamp, quat_ros, gyro_ros, acc_ros)
            return imu_msg

        def get_imu_raw_msg():
            nonlocal imu_raw_msg
            if imu_raw_msg is None:
                imu_raw_msg = self._build_imu(stamp, quat_ros, gyro_ros, acc_ros)
            return imu_raw_msg

        def get_mavros_imu_msg():
            nonlocal mavros_imu_msg
            if mavros_imu_msg is None:
                # The April 1 real bag uses frame_id=fcu_link, but MAVROS has
                # already converted IMU vectors into ROS FLU convention.
                mavros_imu_msg = self._build_imu(stamp, quat_ros, gyro_ros, acc_ros, frame_id="fcu_link")
                self._apply_real_mavros_imu_covariance(mavros_imu_msg)
            return mavros_imu_msg

        def get_mavros_imu_raw_msg():
            nonlocal mavros_imu_raw_msg
            if mavros_imu_raw_msg is None:
                mavros_imu_raw_msg = self._build_imu(stamp, quat_ros, gyro_ros, acc_ros, frame_id="fcu_link")
                self._apply_real_mavros_imu_covariance(mavros_imu_raw_msg)
            return mavros_imu_raw_msg

        def get_depth_msg():
            nonlocal depth_msg
            if depth_msg is None:
                depth_msg = self.Float32()
                depth_msg.data = float(vertical_est.depth_m)
            return depth_msg

        def get_depth_pose_msg():
            nonlocal depth_pose_msg
            if depth_pose_msg is None:
                depth_pose_msg = self._build_depth_pose(stamp, vertical_est.depth_m)
            return depth_pose_msg

        def get_baro_msg():
            nonlocal baro_msg
            if baro_msg is None:
                baro_msg = self.Float32()
                baro_msg.data = float(bar30_pressure_pa)
            return baro_msg

        def get_ground_truth_msg():
            nonlocal ground_truth_msg
            if ground_truth_msg is None:
                ground_truth_msg = self._build_pose(stamp, "world", base_pos_enu, quat_ros)
            return ground_truth_msg

        def get_dvl_velocity_msg():
            nonlocal dvl_velocity_msg
            if dvl_vel_body_ros is None:
                return None
            if dvl_velocity_msg is None:
                dvl_velocity_msg = self._build_twist(stamp, "base_link", dvl_vel_body_ros)
            return dvl_velocity_msg

        def get_dvl_twist_msg():
            nonlocal dvl_twist_msg
            if dvl_vel_body_frd is None:
                return None
            if dvl_twist_msg is None:
                dvl_twist_msg = self._build_twist_cov(
                    stamp,
                    "dvl",
                    dvl_vel_body_frd,
                    linear_cov_diag=(4.696386440627975e-6, 1.173283067146258e-6, 1.566586860235475e-7),
                    angular_cov_diag=(0.0, 0.0, 0.0),
                )
            return dvl_twist_msg

        def get_dvl_altitude_msg():
            nonlocal dvl_altitude_msg
            if dvl_altitude_m is None:
                return None
            if dvl_altitude_msg is None:
                dvl_altitude_msg = self._build_range(stamp, float(dvl_altitude_m))
            return dvl_altitude_msg

        def get_mavros_state_msg():
            nonlocal mavros_state_msg
            if mavros_state_msg is None:
                mavros_state_msg = self._build_mavros_state(stamp)
            return mavros_state_msg

        def get_mavros_vfr_hud_msg():
            nonlocal mavros_vfr_hud_msg
            if mavros_vfr_hud_msg is None:
                mavros_vfr_hud_msg = self._build_vfr_hud(stamp, bar30_pressure_pa)
            return mavros_vfr_hud_msg

        def get_mavros_static_pressure_msg():
            nonlocal mavros_static_pressure_msg
            if mavros_static_pressure_msg is None:
                mavros_static_pressure_msg = self._build_pressure(stamp, static_pressure_pa)
            return mavros_static_pressure_msg

        def get_mavros_atm_pressure_msg():
            nonlocal mavros_atm_pressure_msg
            if mavros_atm_pressure_msg is None:
                mavros_atm_pressure_msg = self._build_pressure(stamp, bar30_pressure_pa)
            return mavros_atm_pressure_msg

        def get_mavros_battery_msg():
            nonlocal mavros_battery_msg
            if mavros_battery_msg is None:
                mavros_battery_msg = self._build_battery(stamp)
            return mavros_battery_msg

        def get_mavros_local_pose_msg():
            nonlocal mavros_local_pose_msg
            if mavros_local_pose_msg is None:
                mavros_local_pose_msg = self._build_pose(stamp, "map", base_pos_enu, quat_ros)
            return mavros_local_pose_msg

        def get_mavros_vision_pose_msg():
            nonlocal mavros_vision_pose_msg
            if mavros_vision_pose_msg is None:
                mavros_vision_pose_msg = self._build_pose(stamp, "map", base_pos_enu, quat_ros)
            return mavros_vision_pose_msg

        def get_odom_local_msg():
            nonlocal odom_local_msg
            if odom_local_msg is None:
                odom_local_msg = self._build_odom(
                    stamp,
                    "odom",
                    "base_link",
                    self._odom_pos,
                    quat_ros,
                    dvl_vel_body_ros if dvl_vel_body_ros is not None else zero_vel,
                )
            return odom_local_msg

        rot_world_body = self._quat_wxyz_to_rotmat(quat_base)
        rot_world_rovio = rot_world_body @ self._base_to_rovio
        quat_rovio = self._rotmat_to_quat_wxyz(rot_world_rovio)

        def get_rovio_odom_msg():
            nonlocal rovio_odom_msg
            if rovio_odom_msg is None:
                rovio_odom_msg = self._build_odom(
                    stamp,
                    "odom",
                    "base_link",
                    self._odom_pos,
                    quat_rovio,
                    dvl_vel_body_ros if dvl_vel_body_ros is not None else zero_vel,
                )
            return rovio_odom_msg

        def get_dvl_data_msg():
            nonlocal dvl_data_msg
            if dvl_data_msg is None:
                dvl_data_msg = self._build_dvl_msg(stamp, dvl_vel_body_ros, dvl_altitude_m)
            return dvl_data_msg

        def get_dvl_pos_msg():
            nonlocal dvl_pos_msg
            if dvl_pos_msg is None:
                dvl_pos_msg = self._build_dvldr_msg(stamp, self._odom_pos, quat_ros)
            return dvl_pos_msg

        map_to_odom = base_pos_enu - self._odom_pos
        tf_specs = [
            ("map", "odom", map_to_odom, self._quat_identity()),
            ("odom", "base_link", self._odom_pos, quat_ros),
        ]

        def get_tf_msg():
            nonlocal tf_msg
            if tf_msg is None:
                tf_msg = self._build_tf_message(stamp, tf_specs)
            return tf_msg

        def get_ping360_sample():
            nonlocal ping360_sample
            if self._ping360 is None or not self._ping360.active:
                return None
            if ping360_sample is None:
                ping360_sample = self._ping360.update(data, sim_t)
            return ping360_sample

        def get_ping360_image_msg():
            nonlocal ping360_image_msg
            sample = get_ping360_sample()
            if sample is None:
                return None
            if ping360_image_msg is None:
                ping360_image_msg = self._build_ping360_image(stamp, sample)
            return ping360_image_msg

        def get_ping360_scan_msg():
            nonlocal ping360_scan_msg
            sample = get_ping360_sample()
            if sample is None:
                return None
            if ping360_scan_msg is None:
                ping360_scan_msg = self._build_ping360_scan(stamp, sample)
            return ping360_scan_msg

        def get_ping360_status_msg():
            nonlocal ping360_status_msg
            sample = get_ping360_sample()
            if sample is None:
                return None
            if ping360_status_msg is None:
                ping360_status_msg = self._build_ping360_status(stamp, sample)
            return ping360_status_msg

        # Core topics.
        jobs.add(self.pub_imu, "/imu/data", get_imu_msg, on_demand=True)
        jobs.add(self.pub_depth, "/depth", get_depth_msg, on_demand=True)
        jobs.add(self.pub_depth_pose, "/depth/pose", get_depth_pose_msg, on_demand=True)
        jobs.add(self.pub_bar30_pressure, "/bar30/pressure_pa", get_baro_msg, on_demand=True)
        jobs.add(self.pub_ground_truth, "/mujoco/ground_truth/pose", get_ground_truth_msg, on_demand=True)
        if self._ping360_config.publish_image:
            jobs.add(self.pub_ping360_image, "/ping360/image", get_ping360_image_msg, on_demand=True)
        if self._ping360_config.publish_scan:
            jobs.add(self.pub_ping360_scan, "/ping360/scan", get_ping360_scan_msg, on_demand=True)
        if self._ping360_config.publish_status:
            jobs.add(self.pub_ping360_status, "/ping360/status", get_ping360_status_msg, on_demand=True)

        if dvl_vel_body_ros is not None:
            jobs.add(self.pub_dvl_velocity, "/dvl/velocity", get_dvl_velocity_msg, on_demand=True)
            jobs.add(self.pub_dvl_twist, "/dvl/twist", get_dvl_twist_msg, on_demand=True)
            if self._mavros_surface_enabled:
                jobs.add(self.pub_mavros_local_vel, "/mavros/local_position/velocity_local", get_dvl_velocity_msg, on_demand=True)
        if dvl_altitude_m is not None:
            jobs.add(self.pub_dvl_altitude, "/dvl/altitude", get_dvl_altitude_msg, on_demand=True)

        # MAVROS surface.
        if self._mavros_surface_enabled and self._mavros_state_pub_hz > 0.0 and sim_t + 1e-9 >= self._mavros_state_next_t:
            self._mavros_state_next_t = sim_t + 1.0 / float(self._mavros_state_pub_hz)
            jobs.add(self.pub_mavros_state, "/mavros/state", get_mavros_state_msg, on_demand=True)
        jobs.add(self.pub_mavros_vfr_hud, "/mavros/vfr_hud", get_mavros_vfr_hud_msg, on_demand=True)
        if self._mavros_surface_enabled:
            jobs.add(self.pub_mavros_imu_data, "/mavros/imu/data", get_mavros_imu_msg, on_demand=True)
            jobs.add(self.pub_mavros_imu_data_raw, "/mavros/imu/data_raw", get_mavros_imu_raw_msg, on_demand=True)
            jobs.add(self.pub_mavros_imu_static_pressure, "/mavros/imu/static_pressure", get_mavros_static_pressure_msg, on_demand=True)
            jobs.add(self.pub_mavros_imu_atm_pressure, "/mavros/imu/atm_pressure", get_mavros_atm_pressure_msg, on_demand=True)
            jobs.add(self.pub_mavros_battery, "/mavros/battery", get_mavros_battery_msg, on_demand=True)
            jobs.add(self.pub_mavros_local_pose, "/mavros/local_position/pose", get_mavros_local_pose_msg, on_demand=True)
            jobs.add(self.pub_mavros_vision_pose, "/mavros/vision_pose/pose", get_mavros_vision_pose_msg, on_demand=True)

        jobs.add(self.pub_dvl_odometry, "/dvl/odometry", get_odom_local_msg, on_demand=True)
        if self._mavros_surface_enabled:
            jobs.add(self.pub_mavros_local_odom, "/mavros/local_position/odom", get_odom_local_msg, on_demand=True)

        # /rovio/odometry compatibility.
        jobs.add(self.pub_rovio_odometry, "/rovio/odometry", get_rovio_odom_msg, on_demand=True)

        if self._mavros_surface_enabled and self._mavros_last_rc_override is not None:
            jobs.add(self.pub_mavros_rc_in, "/mavros/rc/in", self._mavros_last_rc_override, on_demand=True)
        if self._mavros_surface_enabled and self._mavros_last_rc_out is not None:
            jobs.add(self.pub_mavros_rc_out, "/mavros/rc/out", self._mavros_last_rc_out, on_demand=True)

        # Real robot DVL custom-topic compatibility.
        jobs.add(self.pub_dvl_data, "/dvl/data", get_dvl_data_msg, on_demand=True)
        jobs.add(self.pub_dvl_position, "/dvl/position", get_dvl_pos_msg, on_demand=True)

        # Dynamic TF.
        jobs.add(self.pub_tf, "/tf", get_tf_msg, on_demand=True)

        if not jobs.flush(self._safe_publish):
            return

    def reset_odometry(self) -> None:
        self._odom_pos = np.zeros(3, dtype=np.float64)
        self._last_odom_time = -1.0

    def shutdown(self) -> None:
        if self._sitl_transport is not None:
            self._sitl_transport.shutdown()
            self._sitl_transport = None
        if not self._enable_ros:
            return
        try:
            if self._executor is not None and self.node is not None:
                self._executor.remove_node(self.node)
        except Exception:
            pass
        try:
            if self._executor is not None:
                self._executor.shutdown(timeout_sec=0.0)
        except Exception:
            pass
        try:
            if self.node is not None:
                self.node.destroy_node()
        except Exception:
            pass
        try:
            if self._ros_context is not None and self._ros_context.ok():
                self._ros_context.shutdown()
        except Exception:
            pass
