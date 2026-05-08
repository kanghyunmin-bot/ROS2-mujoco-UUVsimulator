"""ROS node used by the MuJoCo UUV control GUI."""

from __future__ import annotations

from dataclasses import replace

from .config import *
from .helpers import (
    make_rc_override_message,
    make_rc_release_message,
    normalize_backend_name,
    padded_rc_channels,
    quaternion_to_euler_deg,
    severity_name,
)
from .models import TelemetrySnapshot
from .runtime import *

class UuvGuiNode(Node):
    def __init__(self, namespace: str, backend: str):
        super().__init__("uuv_control_gui")
        ns = namespace.rstrip("/")
        self._base_ns = ns if ns else ""
        self._backend_preference = normalize_backend_name(backend)
        self._backend_detected = (
            self._backend_preference
            if self._backend_preference != BACKEND_AUTO
            else DEFAULT_AUTO_BACKEND
        )

        self._lock = threading.Lock()
        self._snapshot = TelemetrySnapshot()
        self._last_wall = {}
        self._mode_request_in_flight = False
        self._vehicle_info_in_flight = False
        self._vehicle_info_supported = False
        self._last_graph_probe_wall = -1.0
        self._last_mode_seen = ""
        self._last_armed_seen: Optional[bool] = None
        self._rc_override_subscribers = 0

        if HAVE_MAVROS_MSGS:
            self._rc_override_pub = self.create_publisher(OverrideRCIn, self._topic("rc/override"), 10)
            self._arm_client = self.create_client(CommandBool, self._topic("cmd/arming"))
            self._mode_client = self.create_client(SetMode, self._topic("set_mode"))
            self._vehicle_info_client = self.create_client(VehicleInfoGet, self._topic("vehicle_info_get"))
        else:
            self._rc_override_pub = None
            self._arm_client = None
            self._mode_client = None
            self._vehicle_info_client = None

        state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )

        self._ping360_config_pub = self.create_publisher(String, "/ping360/config", 10)
        self.create_subscription(String, "/ping360/status", self._on_ping360_status, 10)

        self.create_subscription(Imu, self._topic("imu/data"), self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Imu, "/imu/data", self._on_imu, qos_profile_sensor_data)
        self.create_subscription(BatteryState, self._topic("battery"), self._on_battery, best_effort_qos)
        self.create_subscription(BatteryState, "/battery", self._on_battery, best_effort_qos)
        if HAVE_MAVROS_MSGS:
            self.create_subscription(State, self._topic("state"), self._on_state, state_qos)
            self.create_subscription(
                PoseStamped, self._topic("local_position/pose"), self._on_pose, qos_profile_sensor_data
            )
            self.create_subscription(
                Odometry, self._topic("local_position/odom"), self._on_local_odom, qos_profile_sensor_data
            )
            self.create_subscription(
                TwistStamped,
                self._topic("local_position/velocity_body"),
                self._on_velocity_body,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                TwistStamped,
                self._topic("local_position/velocity_local"),
                self._on_velocity_local,
                qos_profile_sensor_data,
            )
        self.create_subscription(Odometry, "/rovio/odometry", self._on_rovio_odom, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/dvl/odometry", self._on_dvl_odom, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, "/dvl/velocity", self._on_dvl_velocity, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/mujoco/ground_truth/pose", self._on_ground_truth_pose, qos_profile_sensor_data)
        if HAVE_MAVROS_MSGS:
            self.create_subscription(RCOut, self._topic("rc/out"), self._on_rc_out, 20)
            self.create_subscription(OverrideRCIn, self._topic("rc/in"), self._on_rc_in, 20)
            self.create_subscription(StatusText, self._topic("statustext/recv"), self._on_status_text, best_effort_qos)

        self.create_subscription(Float32, "/depth", self._on_depth, best_effort_qos)
        self.create_subscription(Float32, "/bar30/pressure_pa", self._on_bar30_pressure, best_effort_qos)
        if HAVE_MAVROS_MSGS:
            self.create_subscription(
                FluidPressure,
                self._topic("imu/atm_pressure"),
                self._on_atm_pressure,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                FluidPressure,
                self._topic("imu/static_pressure"),
                self._on_static_pressure,
                qos_profile_sensor_data,
            )

        self._push_event(
            f"GUI attached to {self._base_ns or '/mavros'} "
            f"(RC override via {self._topic('rc/override')}; SITL stack uses ArduSub closed-loop)"
        )
        if not HAVE_MAVROS_MSGS:
            self._push_event("mavros_msgs not available in this Python env: MAVROS arm/mode/RC features disabled")
        self._probe_backend(force=True)

    def _topic(self, suffix: str) -> str:
        if not self._base_ns:
            return f"/{suffix.lstrip('/')}"
        return f"{self._base_ns}/{suffix.lstrip('/')}"

    def _safe_count_publishers(self, topic: str) -> int:
        try:
            return int(self.count_publishers(topic))
        except Exception:
            return 0

    def _safe_count_subscribers(self, topic: str) -> int:
        try:
            return int(self.count_subscribers(topic))
        except Exception:
            return 0

    @staticmethod
    def _service_ready(client) -> int:
        if client is None:
            return 0
        try:
            return 1 if client.service_is_ready() else 0
        except Exception:
            return 0

    def _effective_backend(self) -> str:
        if self._backend_preference != BACKEND_AUTO:
            return self._backend_preference
        return self._backend_detected

    def _active_layout(self) -> RcLayout:
        backend = self._effective_backend()
        return RC_LAYOUTS.get(backend, RC_LAYOUTS[DEFAULT_AUTO_BACKEND])

    def backend_label(self) -> str:
        backend = self._effective_backend()
        layout = self._active_layout()
        if self._backend_preference == BACKEND_AUTO:
            return f"auto->{backend} ({layout.label})"
        return f"{backend} ({layout.label})"

    def rc_mapping_summary(self) -> str:
        return self._active_layout().summary

    def vehicle_info_supported(self) -> bool:
        return bool(self._vehicle_info_supported)

    def probe_backend(self) -> None:
        self._probe_backend()

    def _probe_backend(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and self._last_graph_probe_wall >= 0.0 and (now - self._last_graph_probe_wall) < 1.0:
            return
        self._last_graph_probe_wall = now

        mavros_score = 0
        sim_score = 0

        vehicle_info_services = self._service_ready(self._vehicle_info_client)
        arm_services = self._service_ready(self._arm_client)
        mode_services = self._service_ready(self._mode_client)
        rc_out_publishers = self._safe_count_publishers(self._topic("rc/out"))
        state_publishers = self._safe_count_publishers(self._topic("state"))
        pose_publishers = self._safe_count_publishers(self._topic("local_position/pose"))
        velocity_body_publishers = self._safe_count_publishers(self._topic("local_position/velocity_body"))
        rc_in_publishers = self._safe_count_publishers(self._topic("rc/in"))
        velocity_local_publishers = self._safe_count_publishers(
            self._topic("local_position/velocity_local")
        )
        bridge_imu_publishers = self._safe_count_publishers("/imu/data")
        bridge_battery_publishers = self._safe_count_publishers("/battery")
        bridge_rovio_publishers = self._safe_count_publishers("/rovio/odometry")
        bridge_dvl_odom_publishers = self._safe_count_publishers("/dvl/odometry")
        bridge_dvl_velocity_publishers = self._safe_count_publishers("/dvl/velocity")
        bridge_depth_publishers = self._safe_count_publishers("/depth")
        rc_override_subscribers = self._safe_count_subscribers(self._topic("rc/override"))
        self._rc_override_subscribers = rc_override_subscribers

        if vehicle_info_services > 0:
            mavros_score += 3
        if arm_services > 0:
            mavros_score += 1
        if mode_services > 0:
            mavros_score += 1
        if rc_out_publishers > 0:
            mavros_score += 3
        if state_publishers > 0:
            mavros_score += 1
        if pose_publishers > 0:
            mavros_score += 1
        if velocity_body_publishers > 0:
            mavros_score += 3
        if rc_in_publishers > 0:
            sim_score += 3
        if velocity_local_publishers > 0:
            sim_score += 2
        if bridge_imu_publishers > 0:
            sim_score += 2
        if bridge_rovio_publishers > 0:
            sim_score += 3
        if bridge_dvl_odom_publishers > 0:
            sim_score += 2
        if bridge_dvl_velocity_publishers > 0:
            sim_score += 1
        if bridge_depth_publishers > 0:
            sim_score += 1
        if bridge_battery_publishers > 0:
            sim_score += 1
        if rc_override_subscribers > 0:
            sim_score += 2

        self._vehicle_info_supported = vehicle_info_services > 0
        if self._backend_preference != BACKEND_AUTO:
            return

        next_backend = self._backend_detected
        if mavros_score > 0 and (
            mavros_score > sim_score
            or (
                mavros_score == sim_score
                and (rc_out_publishers > 0 or velocity_body_publishers > 0 or vehicle_info_services > 0)
            )
        ):
            next_backend = BACKEND_MAVROS
        elif sim_score > 0:
            next_backend = BACKEND_SIM_BRIDGE
        else:
            next_backend = DEFAULT_AUTO_BACKEND

        if next_backend != self._backend_detected:
            self._backend_detected = next_backend
            self._push_event(
                f"backend -> {self.backend_label()} ({self.rc_mapping_summary()})"
            )

    def _touch(self, key: str) -> None:
        self._last_wall[key] = time.monotonic()

    def _push_event(self, text: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        with self._lock:
            self._snapshot.events.appendleft(f"[{stamp}] {text}")

    def push_event(self, text: str) -> None:
        self._push_event(text)

    def snapshot(self) -> TelemetrySnapshot:
        now = time.monotonic()
        with self._lock:
            snap = replace(
                self._snapshot,
                rc_out=list(self._snapshot.rc_out),
                events=deque(self._snapshot.events, maxlen=TELEMETRY_EVENT_LIMIT),
            )

        snap.state_age_s = now - self._last_wall.get("state", math.inf)
        snap.imu_age_s = now - self._last_wall.get("imu", math.inf)
        snap.pose_age_s = now - self._last_wall.get("pose", math.inf)
        snap.depth_age_s = now - self._last_wall.get("depth", math.inf)
        snap.rc_age_s = now - self._last_wall.get("rc_out", math.inf)
        snap.ping360_age_s = now - self._last_wall.get("ping360", math.inf)
        return snap

    def request_vehicle_info(self) -> None:
        self._probe_backend()
        if not self._vehicle_info_supported:
            return
        if self._vehicle_info_in_flight:
            return
        try:
            ready = self._vehicle_info_client.service_is_ready()
        except Exception:
            return
        if not ready:
            return
        req = VehicleInfoGet.Request()
        req.sysid = 1
        req.compid = 1
        req.get_all = False
        future = self._vehicle_info_client.call_async(req)
        self._vehicle_info_in_flight = True
        future.add_done_callback(self._on_vehicle_info_response)

    def _on_vehicle_info_response(self, future) -> None:
        self._vehicle_info_in_flight = False
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"vehicle_info_get failed: {exc}")
            return
        if not resp.success or not resp.vehicles:
            return
        info = resp.vehicles[0]
        with self._lock:
            self._snapshot.vehicle_mode = info.mode
            self._snapshot.mode_id = int(info.mode_id)
            self._snapshot.autopilot_name = f"autopilot={info.autopilot}, type={info.type}"

    def arm(self, value: bool) -> None:
        if self._arm_client is None:
            self._push_event("arm service unavailable in current Python env")
            return
        try:
            ready = self._arm_client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            self._push_event("arm service unavailable")
            return
        req = CommandBool.Request()
        req.value = bool(value)
        future = self._arm_client.call_async(req)
        future.add_done_callback(
            lambda fut: self._on_arm_response(fut, "arm" if value else "disarm")
        )

    def _on_arm_response(self, future, action: str) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"{action} failed: {exc}")
            return
        self._push_event(f"{action}: success={resp.success}, result={resp.result}")

    def set_mode(self, mode: str) -> None:
        if self._mode_request_in_flight:
            return
        if self._mode_client is None:
            self._push_event("set_mode service unavailable in current Python env")
            return
        try:
            ready = self._mode_client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            self._push_event("set_mode service unavailable")
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        self._mode_request_in_flight = True
        future.add_done_callback(lambda fut: self._on_mode_response(fut, mode))

    def _on_mode_response(self, future, mode: str) -> None:
        self._mode_request_in_flight = False
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"set_mode {mode} failed: {exc}")
            return
        self._push_event(f"set_mode {mode}: mode_sent={resp.mode_sent}")

    def publish_rc_override(
        self,
        *,
        yaw: float,
        heave: float,
        forward: float,
        lateral: float,
        pitch: float = 0.0,
        roll: float = 0.0,
    ) -> None:
        del pitch
        del roll
        if self._rc_override_pub is None:
            return
        self._probe_backend(force=True)
        msg = make_rc_override_message(
            self._active_layout(),
            yaw=yaw,
            heave=heave,
            forward=forward,
            lateral=lateral,
        )
        if self._rc_override_subscribers > 0:
            self._rc_override_pub.publish(msg)

    def publish_rc_release(self) -> None:
        if self._rc_override_pub is None:
            return
        msg = make_rc_release_message()
        self._rc_override_pub.publish(msg)

    def publish_rc_channels(self, channels: Iterable[int]) -> bool:
        if self._rc_override_pub is None:
            return False
        msg = OverrideRCIn()
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * RC_MESSAGE_CHANNEL_COUNT
        for idx, value in enumerate(list(channels)[:RC_MESSAGE_CHANNEL_COUNT]):
            msg.channels[idx] = int(value)
        self._rc_override_pub.publish(msg)
        return True

    def publish_ping360_config(
        self,
        *,
        range_m: float,
        num_steps: int,
        gain: int,
        interface_mode: str,
        frequency_khz: int,
        start_angle_grad: int,
        stop_angle_grad: int,
    ) -> None:
        payload = {
            "requested_range_m": float(range_m),
            "num_steps": int(num_steps),
            "gain_setting": int(gain),
            "interface_mode": str(interface_mode),
            "transmit_frequency_khz": int(frequency_khz),
            "start_angle_grad": int(start_angle_grad),
            "stop_angle_grad": int(stop_angle_grad),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self._ping360_config_pub.publish(msg)
        self._push_event(
            "ping360 config -> "
            f"range={range_m:g}m step={num_steps} gain={gain} "
            f"{interface_mode} {frequency_khz}kHz sector={start_angle_grad}..{stop_angle_grad}grad"
        )

    def _on_state(self, msg: State) -> None:
        self._touch("state")
        with self._lock:
            self._snapshot.connected = bool(msg.connected)
            self._snapshot.armed = bool(msg.armed)
            self._snapshot.guided = bool(msg.guided)
            self._snapshot.manual_input = bool(msg.manual_input)
            self._snapshot.mode = msg.mode
            self._snapshot.system_status = int(msg.system_status)

        if msg.mode != self._last_mode_seen:
            self._push_event(f"mode -> {msg.mode}")
            self._last_mode_seen = msg.mode
        if self._last_armed_seen is None or bool(msg.armed) != self._last_armed_seen:
            self._push_event(f"armed -> {msg.armed}")
            self._last_armed_seen = bool(msg.armed)

    def _on_imu(self, msg: Imu) -> None:
        self._touch("imu")
        q = msg.orientation
        roll_deg, pitch_deg, yaw_deg = quaternion_to_euler_deg(q.w, q.x, q.y, q.z)
        with self._lock:
            self._snapshot.roll_deg = roll_deg
            self._snapshot.pitch_deg = pitch_deg
            self._snapshot.yaw_deg = yaw_deg
            self._snapshot.ang_vel_xyz = (
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            )
            self._snapshot.lin_acc_xyz = (
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            )

    def _on_battery(self, msg: BatteryState) -> None:
        with self._lock:
            self._snapshot.battery_voltage = msg.voltage
            self._snapshot.battery_current = msg.current
            self._snapshot.battery_percent = msg.percentage

    def _on_pose(self, msg: PoseStamped) -> None:
        self._touch("pose")
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        with self._lock:
            self._snapshot.position_xyz = (x, y, z)
            depth_age = time.monotonic() - self._last_wall.get("depth", math.inf)
            if not math.isfinite(self._snapshot.depth_m) or depth_age > 1.0:
                self._snapshot.depth_m = max(0.0, -z)
                self._snapshot.depth_source = "local_position.pose.z"

    def _on_odom(self, msg: Odometry, source: str) -> None:
        self._touch("pose")
        pose = msg.pose.pose
        twist = msg.twist.twist
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        with self._lock:
            self._snapshot.position_xyz = (x, y, z)
            self._snapshot.velocity_xyz = (
                twist.linear.x,
                twist.linear.y,
                twist.linear.z,
            )
            self._snapshot.velocity_source = source
            depth_age = time.monotonic() - self._last_wall.get("depth", math.inf)
            if not math.isfinite(self._snapshot.depth_m) or depth_age > 1.0:
                self._snapshot.depth_m = max(0.0, -z)
                self._snapshot.depth_source = f"{source}.pose.z"

    def _on_local_odom(self, msg: Odometry) -> None:
        self._on_odom(msg, self._topic("local_position/odom"))

    def _on_rovio_odom(self, msg: Odometry) -> None:
        self._on_odom(msg, "/rovio/odometry")

    def _on_dvl_odom(self, msg: Odometry) -> None:
        self._on_odom(msg, "/dvl/odometry")

    def _on_velocity(self, msg: TwistStamped, source: str) -> None:
        with self._lock:
            self._snapshot.velocity_xyz = (
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
            )
            self._snapshot.velocity_source = source

    def _on_velocity_body(self, msg: TwistStamped) -> None:
        self._on_velocity(msg, self._topic("local_position/velocity_body"))

    def _on_velocity_local(self, msg: TwistStamped) -> None:
        self._on_velocity(msg, self._topic("local_position/velocity_local"))

    def _on_dvl_velocity(self, msg: TwistStamped) -> None:
        self._on_velocity(msg, "/dvl/velocity")

    def _on_ground_truth_pose(self, msg: PoseStamped) -> None:
        self._touch("pose")
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        with self._lock:
            if not math.isfinite(self._snapshot.position_xyz[0]):
                self._snapshot.position_xyz = (x, y, z)
            depth_age = time.monotonic() - self._last_wall.get("depth", math.inf)
            if not math.isfinite(self._snapshot.depth_m) or depth_age > 1.0:
                self._snapshot.depth_m = max(0.0, -z)
                self._snapshot.depth_source = "/mujoco/ground_truth/pose.z"

    def _on_rc_out(self, msg: RCOut) -> None:
        self._touch("rc_out")
        with self._lock:
            self._snapshot.rc_out = padded_rc_channels(msg.channels)
            self._snapshot.rc_feedback_source = self._topic("rc/out")

    def _on_rc_in(self, msg: OverrideRCIn) -> None:
        self._touch("rc_out")
        with self._lock:
            self._snapshot.rc_out = padded_rc_channels(
                getattr(msg, "channels", []),
                sanitize_override_markers=True,
            )
            self._snapshot.rc_feedback_source = self._topic("rc/in")

    def _on_status_text(self, msg: StatusText) -> None:
        self._push_event(f"{severity_name(int(msg.severity))}: {msg.text}")

    def _on_depth(self, msg: Float32) -> None:
        self._touch("depth")
        with self._lock:
            self._snapshot.depth_m = float(msg.data)
            self._snapshot.depth_source = "/depth"

    def _on_bar30_pressure(self, msg: Float32) -> None:
        self._on_pressure_value(float(msg.data), "/bar30/pressure_pa")

    def _on_ping360_status(self, msg: String) -> None:
        self._touch("ping360")
        try:
            payload = json.loads(str(msg.data))
            settings = payload.get("settings", {}) if isinstance(payload, dict) else {}
        except json.JSONDecodeError:
            settings = {}
            payload = {}

        try:
            effective_range = float(settings.get("effective_range_m", math.nan))
            requested_range = float(settings.get("requested_range_m", math.nan))
            resolution_cm = float(settings.get("range_resolution_m", math.nan)) * 100.0
            angular_resolution = float(settings.get("angular_resolution_deg", math.nan))
            scan_period = float(settings.get("scan_period_s", math.nan))
            angle_deg = float(payload.get("angle_deg", math.nan))
            num_steps = int(settings.get("num_steps", 0))
            start_grad = int(settings.get("start_angle_grad", 0))
            stop_grad = int(settings.get("stop_angle_grad", 399))
            flags = settings.get("quality_flags", [])
        except (TypeError, ValueError):
            effective_range = requested_range = resolution_cm = angular_resolution = scan_period = angle_deg = math.nan
            num_steps = 0
            start_grad = 0
            stop_grad = 399
            flags = []

        if math.isfinite(effective_range):
            summary = (
                f"ping360: req={requested_range:.2f}m eff={effective_range:.2f}m "
                f"res={resolution_cm:.2f}cm step={num_steps}/{angular_resolution:.1f}deg "
                f"sector={start_grad}..{stop_grad}grad scan={scan_period:.1f}s "
                f"angle={angle_deg:.1f}deg"
            )
        else:
            summary = "ping360: waiting for status"
        if isinstance(flags, list) and flags:
            summary += " flags=" + ",".join(str(flag) for flag in flags[:4])
        with self._lock:
            self._snapshot.ping360_summary = summary

    def _on_atm_pressure(self, msg: FluidPressure) -> None:
        self._on_pressure_value(float(msg.fluid_pressure), self._topic("imu/atm_pressure"))

    def _on_static_pressure(self, msg: FluidPressure) -> None:
        self._on_pressure_value(float(msg.fluid_pressure), self._topic("imu/static_pressure"))

    def _on_pressure_value(self, pressure_pa: float, source: str) -> None:
        self._touch("depth")
        rho = 997.0
        g = 9.80665
        approx_depth = max(0.0, (pressure_pa - 101325.0) / (rho * g))
        with self._lock:
            self._snapshot.pressure_pa = pressure_pa
            if not math.isfinite(self._snapshot.depth_m):
                self._snapshot.depth_m = approx_depth
                self._snapshot.depth_source = f"{source} (approx)"
