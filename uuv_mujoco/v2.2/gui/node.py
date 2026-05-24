"""ROS node used by the MuJoCo UUV control GUI."""

from __future__ import annotations

from dataclasses import replace

from .config import *
from .helpers import (
    clamp_axis,
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
        self._arm_request_in_flight = False
        self._rc_override_subscribers = 0
        self._manual_control_subscribers = 0
        self._one_shot_timers = []
        self._vehicle_connected_since_wall = -1.0
        self._control_request_timeout_s = float(os.environ.get("UUV_GUI_CONTROL_REQUEST_TIMEOUT_S", "20.0"))
        self._control_request_retry_s = float(os.environ.get("UUV_GUI_CONTROL_REQUEST_RETRY_S", "0.5"))
        self._require_arm_mode_settle = os.environ.get(
            "UUV_GUI_REQUIRE_ARM_MODE_EKF_SETTLE",
            "1",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self._arm_mode_settle_s = float(os.environ.get("UUV_GUI_ARM_MODE_EKF_SETTLE_S", "3.0"))
        self._initial_depth_hold_opt_in = os.environ.get(
            "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE",
            "0",
        ).strip().lower() not in {"0", "false", "no", "off"}
        self._initial_depth_release_pending = False
        self._initial_depth_release_in_flight = False
        self._initial_depth_release_reason = ""

        if HAVE_MAVROS_MSGS:
            self._rc_override_pub = self.create_publisher(OverrideRCIn, self._topic("rc/override"), 10)
            self._manual_control_pub = self.create_publisher(ManualControl, self._topic("manual_control/send"), 10)
            self._arm_client = self.create_client(CommandBool, self._topic("cmd/arming"))
            self._mode_client = self.create_client(SetMode, self._topic("set_mode"))
            self._vehicle_info_client = self.create_client(VehicleInfoGet, self._topic("vehicle_info_get"))
        else:
            self._rc_override_pub = None
            self._manual_control_pub = None
            self._arm_client = None
            self._mode_client = None
            self._vehicle_info_client = None

        if HAVE_STD_SRVS and self._initial_depth_hold_opt_in:
            self._initial_depth_release_client = self.create_client(
                Trigger,
                "/mujoco/release_initial_depth_hold",
            )
        else:
            self._initial_depth_release_client = None

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
            self.create_subscription(RCIn, self._topic("rc/in"), self._on_rc_in, 20)
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
            f"(pilot joystick via {self._topic('manual_control/send')}; "
            f"rosbag replay via {self._topic('rc/override')})"
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

    def control_readiness(self, snap: TelemetrySnapshot) -> tuple[str, str]:
        state_fresh = bool(snap.connected) and math.isfinite(snap.state_age_s) and snap.state_age_s < 3.0
        arm_ready = self._service_ready(self._arm_client) > 0
        mode_ready = self._service_ready(self._mode_client) > 0
        rc_ready = self._manual_control_subscribers > 0 or self._rc_override_subscribers > 0
        depth_ready = math.isfinite(snap.depth_m) and math.isfinite(snap.depth_age_s) and snap.depth_age_s < 3.0
        settle_left_s = self._arm_mode_settle_left_s()

        if not state_fresh:
            return "WAIT: vehicle", "NotReady.TLabel"
        if settle_left_s > 0.0:
            return f"WAIT: EKF settle {settle_left_s:.0f}s", "NotReady.TLabel"
        if not arm_ready or not mode_ready:
            return "WAIT: arm/mode", "NotReady.TLabel"
        if not rc_ready:
            return "CMD READY / RC WAIT", "Limited.TLabel"
        if not depth_ready:
            return "CMD READY / DEPTH WAIT", "Limited.TLabel"
        return "READY", "Ready.TLabel"

    def vehicle_info_supported(self) -> bool:
        return bool(self._vehicle_info_supported)

    def probe_backend(self) -> None:
        self._probe_backend()

    def _probe_backend(self, force: bool = False) -> None:
        now = time.monotonic()
        if not force and self._last_graph_probe_wall >= 0.0 and (now - self._last_graph_probe_wall) < 3.0:
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
        manual_control_subscribers = self._safe_count_subscribers(self._topic("manual_control/send"))
        self._rc_override_subscribers = rc_override_subscribers
        self._manual_control_subscribers = manual_control_subscribers

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
        if manual_control_subscribers > 0:
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
                rc_in=list(self._snapshot.rc_in),
                rc_out=list(self._snapshot.rc_out),
                events=deque(self._snapshot.events, maxlen=TELEMETRY_EVENT_LIMIT),
            )

        snap.state_age_s = now - self._last_wall.get("state", math.inf)
        snap.imu_age_s = now - self._last_wall.get("imu", math.inf)
        snap.pose_age_s = now - self._last_wall.get("pose", math.inf)
        snap.depth_age_s = now - self._last_wall.get("depth", math.inf)
        snap.rc_in_age_s = now - self._last_wall.get("rc_in", math.inf)
        snap.rc_out_age_s = now - self._last_wall.get("rc_out", math.inf)
        snap.rc_age_s = min(snap.rc_in_age_s, snap.rc_out_age_s)
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

    def _call_trigger_service(self, client, label: str, on_success=None, on_done=None) -> bool:
        try:
            ready = client is not None and client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            self._push_event(f"{label}: service unavailable")
            return False
        future = client.call_async(Trigger.Request())

        def _done(fut) -> None:
            try:
                resp = fut.result()
            except Exception as exc:
                self._push_event(f"{label} failed: {exc}")
                if on_done is not None:
                    on_done(False)
                return
            ok = bool(getattr(resp, "success", False))
            message = str(getattr(resp, "message", ""))
            self._push_event(f"{label}: success={ok} {message}".strip())
            if on_done is not None:
                on_done(ok)
                return
            if ok and on_success is not None:
                on_success()

        future.add_done_callback(_done)
        return True

    def _vehicle_ready_for_initial_depth_release(self) -> bool:
        with self._lock:
            armed = bool(self._snapshot.armed)
            mode = str(self._snapshot.mode).upper()
        if not armed:
            return False
        if self._initial_depth_release_reason == "ALT_HOLD":
            return mode == "ALT_HOLD"
        return True

    def _try_release_initial_depth_hold(self) -> None:
        if not self._initial_depth_hold_opt_in:
            return
        if not self._initial_depth_release_pending:
            return
        if self._initial_depth_release_in_flight:
            return
        if not self._vehicle_ready_for_initial_depth_release():
            return
        self._initial_depth_release_in_flight = True

        def _on_release_done(ok: bool) -> None:
            self._initial_depth_release_in_flight = False
            self._initial_depth_release_pending = not bool(ok)
            if ok:
                self._initial_depth_release_reason = ""

        started = self._call_trigger_service(
            self._initial_depth_release_client,
            "initial depth release",
            on_done=_on_release_done,
        )
        if not started:
            self._initial_depth_release_in_flight = False
            self._initial_depth_release_pending = True

    def _request_initial_depth_release_when_armed(self, reason: str) -> None:
        if not self._initial_depth_hold_opt_in:
            return
        self._initial_depth_release_pending = True
        self._initial_depth_release_reason = str(reason).strip() or "unknown"
        self._push_event(f"initial depth hold release waiting for armed state ({reason})")
        self._try_release_initial_depth_hold()

    def request_initial_depth_release_when_armed(self, reason: str) -> None:
        self._request_initial_depth_release_when_armed(reason)

    def _schedule_once(self, delay_s: float, callback) -> None:
        holder = {}

        def _timer_cb() -> None:
            timer = holder.get("timer")
            if timer is not None:
                try:
                    timer.cancel()
                except Exception:
                    pass
            try:
                callback()
            finally:
                if timer in self._one_shot_timers:
                    self._one_shot_timers.remove(timer)

        timer = self.create_timer(max(0.0, float(delay_s)), _timer_cb)
        holder["timer"] = timer
        self._one_shot_timers.append(timer)

    def _state_age_s(self) -> float:
        return time.monotonic() - self._last_wall.get("state", math.inf)

    def _fresh_vehicle_state(self) -> tuple[bool, bool, str]:
        with self._lock:
            return bool(self._snapshot.connected), bool(self._snapshot.armed), str(self._snapshot.mode)

    def _arm_mode_settle_left_s(self) -> float:
        if not self._require_arm_mode_settle:
            return 0.0
        with self._lock:
            connected_since = float(self._vehicle_connected_since_wall)
        if connected_since <= 0.0:
            return float(max(0.0, self._arm_mode_settle_s))
        age_s = time.monotonic() - connected_since
        return float(max(0.0, self._arm_mode_settle_s - age_s))

    def _arm_mode_gate_reason(self, *, arm_value: Optional[bool] = None, mode: str = "") -> str:
        if arm_value is False:
            return ""
        requested_mode = str(mode or "").upper()
        if requested_mode in {"", "MANUAL"} and arm_value is None:
            return ""

        snap = self.snapshot()
        if not bool(snap.connected) or not math.isfinite(snap.state_age_s) or snap.state_age_s >= 3.0:
            return "waiting for fresh vehicle state"
        if math.isfinite(snap.depth_age_s) and snap.depth_age_s >= 3.0:
            return "waiting for fresh Bar30/depth feedback"
        if not math.isfinite(snap.depth_age_s):
            return "waiting for Bar30/depth feedback"
        if math.isfinite(snap.imu_age_s) and snap.imu_age_s >= 3.0:
            return "waiting for fresh IMU feedback"
        if not math.isfinite(snap.imu_age_s):
            return "waiting for IMU feedback"

        settle_left_s = self._arm_mode_settle_left_s()
        if settle_left_s > 0.0:
            return f"waiting EKF/ExternalNav settle ({settle_left_s:.1f}s left)"
        return ""

    def _arm_target_reached(self, value: bool) -> bool:
        connected, armed, _mode = self._fresh_vehicle_state()
        return connected and self._state_age_s() < 3.0 and armed == bool(value)

    def _mode_target_reached(self, mode: str) -> bool:
        connected, _armed, current_mode = self._fresh_vehicle_state()
        return connected and self._state_age_s() < 3.0 and current_mode.upper() == str(mode).upper()

    def _retry_arm_request(self, value: bool, deadline: float, attempt: int) -> None:
        if time.monotonic() >= deadline:
            self._push_event(f"arm target timeout: armed={value}")
            return
        self._schedule_once(
            self._control_request_retry_s,
            lambda: self._send_arm_request(value, deadline, attempt + 1),
        )

    def _retry_mode_request(self, mode: str, deadline: float, attempt: int) -> None:
        if time.monotonic() >= deadline:
            self._push_event(f"set_mode target timeout: {mode}")
            return
        self._schedule_once(
            self._control_request_retry_s,
            lambda: self._send_mode_request(mode, deadline, attempt + 1),
        )

    def _send_arm_request(self, value: bool, deadline: Optional[float] = None, attempt: int = 1) -> None:
        if deadline is None:
            deadline = time.monotonic() + self._control_request_timeout_s
        if self._arm_target_reached(value):
            self._push_event(f"arm target reached: armed={value}")
            if value:
                self._try_release_initial_depth_hold()
            return
        gate_reason = self._arm_mode_gate_reason(arm_value=bool(value))
        if gate_reason:
            if time.monotonic() >= deadline:
                self._push_event(f"arm blocked: {gate_reason}")
                return
            if attempt == 1 or attempt % 4 == 0:
                self._push_event(f"arm delayed: {gate_reason}")
            self._retry_arm_request(value, deadline, attempt)
            return
        if self._arm_request_in_flight:
            self._retry_arm_request(value, deadline, attempt)
            return
        if self._arm_client is None:
            self._push_event("arm service unavailable in current Python env")
            self._retry_arm_request(value, deadline, attempt)
            return
        try:
            ready = self._arm_client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            if attempt == 1 or attempt % 4 == 0:
                self._push_event("arm service unavailable; waiting")
            self._retry_arm_request(value, deadline, attempt)
            return
        req = CommandBool.Request()
        req.value = bool(value)
        future = self._arm_client.call_async(req)
        self._arm_request_in_flight = True
        future.add_done_callback(
            lambda fut: self._on_arm_response(
                fut,
                "arm" if value else "disarm",
                bool(value),
                deadline,
                attempt,
            )
        )

    def arm(self, value: bool) -> None:
        deadline = time.monotonic() + self._control_request_timeout_s
        self._send_arm_request(value, deadline, 1)

    def _on_arm_response(
        self,
        future,
        action: str,
        target_value: bool,
        deadline: float,
        attempt: int,
    ) -> None:
        self._arm_request_in_flight = False
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"{action} failed: {exc}")
            self._retry_arm_request(target_value, deadline, attempt)
            return
        self._push_event(f"{action}: success={resp.success}, result={resp.result}, attempt={attempt}")
        if self._arm_target_reached(target_value):
            self._push_event(f"arm target reached: armed={target_value}")
            if target_value:
                self._try_release_initial_depth_hold()
            return
        self._retry_arm_request(target_value, deadline, attempt)

    def set_mode(self, mode: str) -> None:
        if str(mode).upper() == "ALT_HOLD":
            self._request_initial_depth_release_when_armed("before_ALT_HOLD")
        deadline = time.monotonic() + self._control_request_timeout_s
        self._send_mode_request(mode, deadline, 1)

    def _send_mode_request(self, mode: str, deadline: float, attempt: int) -> None:
        if self._mode_target_reached(mode):
            self._push_event(f"mode target reached: {mode}")
            return
        if str(mode).upper() == "ALT_HOLD" and self._initial_depth_hold_opt_in:
            self._try_release_initial_depth_hold()
            if self._initial_depth_release_pending or self._initial_depth_release_in_flight:
                if time.monotonic() >= deadline:
                    self._push_event("set_mode ALT_HOLD not sent: initial depth hold release incomplete")
                    return
                if attempt == 1 or attempt % 4 == 0:
                    self._push_event("set_mode ALT_HOLD delayed: releasing initial depth hold first")
                self._retry_mode_request(mode, deadline, attempt)
                return
        gate_reason = self._arm_mode_gate_reason(mode=mode)
        if gate_reason:
            if time.monotonic() >= deadline:
                self._push_event(f"set_mode {mode} blocked: {gate_reason}")
                return
            if attempt == 1 or attempt % 4 == 0:
                self._push_event(f"set_mode {mode} delayed: {gate_reason}")
            self._retry_mode_request(mode, deadline, attempt)
            return
        if self._mode_request_in_flight:
            self._retry_mode_request(mode, deadline, attempt)
            return
        if self._mode_client is None:
            self._push_event("set_mode service unavailable in current Python env")
            self._retry_mode_request(mode, deadline, attempt)
            return
        try:
            ready = self._mode_client.service_is_ready()
        except Exception:
            ready = False
        if not ready:
            if attempt == 1 or attempt % 4 == 0:
                self._push_event("set_mode service unavailable; waiting")
            self._retry_mode_request(mode, deadline, attempt)
            return
        req = SetMode.Request()
        req.base_mode = 0
        req.custom_mode = mode
        future = self._mode_client.call_async(req)
        self._mode_request_in_flight = True
        future.add_done_callback(lambda fut: self._on_mode_response(fut, mode, deadline, attempt))

    def _on_mode_response(self, future, mode: str, deadline: float, attempt: int) -> None:
        self._mode_request_in_flight = False
        try:
            resp = future.result()
        except Exception as exc:
            self._push_event(f"set_mode {mode} failed: {exc}")
            self._retry_mode_request(mode, deadline, attempt)
            return
        self._push_event(f"set_mode {mode}: mode_sent={resp.mode_sent}, attempt={attempt}")
        if self._mode_target_reached(mode):
            self._push_event(f"mode target reached: {mode}")
            return
        self._retry_mode_request(mode, deadline, attempt)

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
        self._rc_override_pub.publish(msg)

    def publish_manual_control(
        self,
        *,
        yaw: float,
        heave: float,
        forward: float,
        lateral: float,
    ) -> None:
        if self._manual_control_pub is None:
            return
        msg = ManualControl()
        msg.x = clamp_axis(forward)
        msg.y = clamp_axis(lateral)
        # The local bridge accepts normalized heave [-1, +1] and converts it to
        # MAVLink MANUAL_CONTROL z [0, 1000] with 500 as neutral.
        msg.z = clamp_axis(heave)
        msg.r = clamp_axis(yaw)
        msg.buttons = 0
        self._manual_control_pub.publish(msg)

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

    def publish_ping360_enabled(self, enabled: bool) -> None:
        msg = String()
        msg.data = json.dumps({"enabled": bool(enabled)}, sort_keys=True)
        self._ping360_config_pub.publish(msg)
        self._push_event(f"ping360 sonar -> {'on' if enabled else 'off'}")

    def _on_state(self, msg: State) -> None:
        self._touch("state")
        now = time.monotonic()
        with self._lock:
            was_connected = bool(self._snapshot.connected)
            self._snapshot.connected = bool(msg.connected)
            self._snapshot.armed = bool(msg.armed)
            self._snapshot.guided = bool(msg.guided)
            self._snapshot.manual_input = bool(msg.manual_input)
            self._snapshot.mode = msg.mode
            self._snapshot.system_status = int(msg.system_status)
            if bool(msg.connected) and not was_connected:
                self._vehicle_connected_since_wall = now
            elif not bool(msg.connected):
                self._vehicle_connected_since_wall = -1.0

        if msg.mode != self._last_mode_seen:
            self._push_event(f"mode -> {msg.mode}")
            self._last_mode_seen = msg.mode
        if self._last_armed_seen is None or bool(msg.armed) != self._last_armed_seen:
            self._push_event(f"armed -> {msg.armed}")
            self._last_armed_seen = bool(msg.armed)
        self._try_release_initial_depth_hold()

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
            self._snapshot.rc_out_source = self._topic("rc/out")
            self._snapshot.rc_feedback_source = self._topic("rc/out")

    def _on_rc_in(self, msg: RCIn) -> None:
        self._touch("rc_in")
        with self._lock:
            self._snapshot.rc_in = padded_rc_channels(
                getattr(msg, "channels", []),
                sanitize_override_markers=True,
            )
            self._snapshot.rc_in_source = self._topic("rc/in")
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

        active = payload.get("active") if isinstance(payload, dict) else None
        enabled = payload.get("enabled") if isinstance(payload, dict) else None
        active_bool = bool(active) if isinstance(active, bool) else None
        enabled_bool = bool(enabled) if isinstance(enabled, bool) else None

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

        if active_bool is False:
            summary = "ping360: off" if enabled_bool is False else "ping360: inactive"
        elif math.isfinite(effective_range):
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
            self._snapshot.ping360_enabled = enabled_bool
            self._snapshot.ping360_active = active_bool

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
