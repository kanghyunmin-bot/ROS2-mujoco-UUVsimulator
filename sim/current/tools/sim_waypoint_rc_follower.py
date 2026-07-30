#!/usr/bin/env python3
"""Execute mission-package waypoints through ArduSub STABILIZE RC control.

The real mission packages publish absolute ``mavros_msgs/PositionTarget``
waypoints.  ArduSub 4.1 SITL accepts those targets in GUIDED but, with the
current simulation sensor stack, does not generate horizontal motor output.
ALT_HOLD also does not accept a new depth target in this stack.  This
simulation-only adapter leaves waypoint generation untouched and follows the
same XYZ waypoints using STABILIZE RC with odometry feedback.
"""

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import OverrideRCIn, PositionTarget, State
from mavros_msgs.srv import SetMode
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class SimWaypointRcFollower(Node):
    def __init__(self) -> None:
        super().__init__("sim_waypoint_rc_follower")
        self.declare_parameter(
            "raw_odometry_topic", "/mavros/local_position/odom"
        )
        self.declare_parameter("control_odometry_topic", "/homing/sim_odometry")
        self.declare_parameter("waypoint_topic", "/waypoint")
        self.declare_parameter("waypoint_enable_topic", "/guided/waypoint_enable")
        self.declare_parameter("status_topic", "/guided/status")
        self.declare_parameter("arrived_topic", "/guided/arrived")
        self.declare_parameter("start_frame_topic", "/guided/start_frame")
        self.declare_parameter("rc_override_topic", "/mavros/rc/override")
        self.declare_parameter("fcu_state_topic", "/mavros/state")
        self.declare_parameter("set_mode_service", "/mavros/set_mode")
        self.declare_parameter("required_mode", "STABILIZE")
        self.declare_parameter("enabled_at_start", True)
        self.declare_parameter("arrival_tolerance_m", 0.13)
        self.declare_parameter("depth_tolerance_m", 0.07)
        self.declare_parameter("proportional_gain", 0.24)
        self.declare_parameter("depth_proportional_gain", 0.35)
        self.declare_parameter("maximum_axis_command", 0.38)
        self.declare_parameter("minimum_axis_command", 0.055)
        self.declare_parameter("maximum_heave_command", 0.45)
        self.declare_parameter("minimum_heave_command", 0.28)
        self.declare_parameter("slow_radius_m", 1.25)
        self.declare_parameter("rc_pwm_span", 400.0)
        self.declare_parameter("forward_sign", 1.0)
        # ArduSub RC6 is FRD-right while the simulation odometry is ROS FLU-left.
        self.declare_parameter("lateral_sign", -1.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("odometry_timeout_s", 0.5)
        self.declare_parameter("mode_request_interval_s", 2.0)
        self.declare_parameter("control_rate_hz", 20.0)

        self.arrival_tolerance_m = max(
            0.03, float(self.get_parameter("arrival_tolerance_m").value)
        )
        self.depth_tolerance_m = max(
            0.02, float(self.get_parameter("depth_tolerance_m").value)
        )
        self.proportional_gain = max(
            0.01, float(self.get_parameter("proportional_gain").value)
        )
        self.depth_proportional_gain = max(
            0.01, float(self.get_parameter("depth_proportional_gain").value)
        )
        self.maximum_axis_command = clamp(
            float(self.get_parameter("maximum_axis_command").value), 0.05, 0.8
        )
        self.minimum_axis_command = clamp(
            float(self.get_parameter("minimum_axis_command").value),
            0.0,
            self.maximum_axis_command,
        )
        self.maximum_heave_command = clamp(
            float(self.get_parameter("maximum_heave_command").value), 0.1, 0.8
        )
        self.minimum_heave_command = clamp(
            float(self.get_parameter("minimum_heave_command").value),
            0.0,
            self.maximum_heave_command,
        )
        self.slow_radius_m = max(
            self.arrival_tolerance_m,
            float(self.get_parameter("slow_radius_m").value),
        )
        self.rc_pwm_span = clamp(
            float(self.get_parameter("rc_pwm_span").value), 50.0, 500.0
        )
        self.forward_sign = 1.0 if (
            float(self.get_parameter("forward_sign").value) >= 0.0
        ) else -1.0
        self.lateral_sign = 1.0 if (
            float(self.get_parameter("lateral_sign").value) >= 0.0
        ) else -1.0
        self.target_timeout_s = max(
            0.1, float(self.get_parameter("target_timeout_s").value)
        )
        self.odometry_timeout_s = max(
            0.1, float(self.get_parameter("odometry_timeout_s").value)
        )
        self.mode_request_interval_s = max(
            0.5, float(self.get_parameter("mode_request_interval_s").value)
        )
        self.required_mode = str(self.get_parameter("required_mode").value)
        if not self.required_mode:
            raise ValueError("required_mode must not be empty")
        control_rate_hz = clamp(
            float(self.get_parameter("control_rate_hz").value), 5.0, 50.0
        )

        reliable_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE
        )
        sensor_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.rc_publisher = self.create_publisher(
            OverrideRCIn,
            str(self.get_parameter("rc_override_topic").value),
            reliable_qos,
        )
        self.status_publisher = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), latched_qos
        )
        self.arrived_publisher = self.create_publisher(
            Bool, str(self.get_parameter("arrived_topic").value), latched_qos
        )
        self.start_frame_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter("start_frame_topic").value),
            latched_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("raw_odometry_topic").value),
            self.on_raw_odometry,
            sensor_qos,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("control_odometry_topic").value),
            self.on_control_odometry,
            sensor_qos,
        )
        self.create_subscription(
            PositionTarget,
            str(self.get_parameter("waypoint_topic").value),
            self.on_waypoint,
            reliable_qos,
        )
        self.create_subscription(
            Bool,
            str(self.get_parameter("waypoint_enable_topic").value),
            self.on_enable,
            reliable_qos,
        )
        self.create_subscription(
            State,
            str(self.get_parameter("fcu_state_topic").value),
            self.on_fcu_state,
            reliable_qos,
        )
        self.mode_client = self.create_client(
            SetMode, str(self.get_parameter("set_mode_service").value)
        )

        self.enabled = bool(self.get_parameter("enabled_at_start").value)
        self.control_odometry: Optional[Odometry] = None
        self.control_odometry_ns = 0
        self.target: Optional[PositionTarget] = None
        self.target_ns = 0
        self.start_frame: Optional[PoseStamped] = None
        self.fcu_mode = ""
        self.fcu_connected = False
        self.fcu_armed = False
        self.mode_request_pending = False
        self.last_mode_request_ns = 0
        self.was_arrived = False
        self.last_status_ns = 0

        self.timer = self.create_timer(1.0 / control_rate_hz, self.control)
        self.publish_status("WAITING: odometry and waypoint")
        self.get_logger().info(
            "Simulation waypoint executor ready: mission /waypoint -> "
            f"ArduSub {self.required_mode} RC XYZ feedback control"
        )

    def on_raw_odometry(self, message: Odometry) -> None:
        if self.start_frame is not None:
            return
        start = PoseStamped()
        start.header = message.header
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose = message.pose.pose
        self.start_frame = start
        self.start_frame_publisher.publish(start)
        self.get_logger().info(
            "Captured simulation start frame: "
            f"({start.pose.position.x:.3f}, {start.pose.position.y:.3f}, "
            f"{start.pose.position.z:.3f}) frame={start.header.frame_id!r}"
        )

    def on_control_odometry(self, message: Odometry) -> None:
        self.control_odometry = message
        self.control_odometry_ns = self.get_clock().now().nanoseconds

    def on_waypoint(self, message: PositionTarget) -> None:
        self.target = message
        self.target_ns = self.get_clock().now().nanoseconds

    def on_enable(self, message: Bool) -> None:
        self.enabled = bool(message.data)
        if not self.enabled:
            self.target = None
            self.release_rc()
            self.publish_status("IDLE: waypoint execution disabled")
        else:
            self.publish_status("WAITING: waypoint execution enabled")

    def on_fcu_state(self, message: State) -> None:
        self.fcu_connected = bool(message.connected)
        self.fcu_armed = bool(message.armed)
        self.fcu_mode = message.mode

    def request_control_mode_if_needed(self, now_ns: int) -> None:
        if (
            not self.enabled
            or not self.fcu_connected
            or self.fcu_mode == self.required_mode
            or self.mode_request_pending
            or not self.mode_client.service_is_ready()
        ):
            return
        if (
            self.last_mode_request_ns
            and (now_ns - self.last_mode_request_ns) * 1.0e-9
            < self.mode_request_interval_s
        ):
            return
        request = SetMode.Request()
        request.base_mode = 0
        request.custom_mode = self.required_mode
        self.mode_request_pending = True
        self.last_mode_request_ns = now_ns
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.on_mode_response)
        self.get_logger().info(f"Requested FCU mode {self.required_mode}")

    def on_mode_response(self, future) -> None:
        self.mode_request_pending = False
        try:
            if not future.result().mode_sent:
                self.get_logger().warning(
                    f"FCU rejected {self.required_mode} mode request"
                )
        except Exception as error:  # noqa: BLE001 - ROS future surfaces RPC errors
            self.get_logger().warning(
                f"{self.required_mode} mode request failed: {error}"
            )

    def control(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self.request_control_mode_if_needed(now_ns)

        if not self.enabled:
            return
        if self.fcu_mode != self.required_mode:
            self.publish_neutral()
            self.throttled_status(
                now_ns, f"WAITING: FCU mode is {self.fcu_mode or 'unknown'}"
            )
            return
        if not self.fcu_armed:
            self.publish_neutral()
            self.throttled_status(now_ns, "WAITING: FCU is disarmed")
            return
        if (
            self.control_odometry is None
            or (now_ns - self.control_odometry_ns) * 1.0e-9
            > self.odometry_timeout_s
        ):
            self.publish_neutral()
            self.throttled_status(now_ns, "DEGRADED: control odometry is stale")
            return
        if (
            self.target is None
            or (now_ns - self.target_ns) * 1.0e-9 > self.target_timeout_s
        ):
            self.publish_neutral()
            self.throttled_status(now_ns, "WAITING: waypoint is stale")
            return

        pose = self.control_odometry.pose.pose
        dx = self.target.position.x - pose.position.x
        dy = self.target.position.y - pose.position.y
        dz = self.target.position.z - pose.position.z
        distance = math.hypot(dx, dy)
        planar_arrived = distance <= self.arrival_tolerance_m
        depth_arrived = abs(dz) <= self.depth_tolerance_m
        arrived = planar_arrived and depth_arrived
        if arrived:
            self.publish_neutral()
            if not self.was_arrived:
                self.arrived_publisher.publish(Bool(data=True))
            self.was_arrived = True
            self.throttled_status(
                now_ns,
                f"ARRIVED: waypoint distance={distance:.3f} m "
                f"depth_error={dz:.3f} m",
            )
            return

        if self.was_arrived:
            self.arrived_publisher.publish(Bool(data=False))
        self.was_arrived = False

        orientation = pose.orientation
        yaw = yaw_from_quaternion(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        forward_error = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral_error = -math.sin(yaw) * dx + math.cos(yaw) * dy
        scale = clamp(distance / self.slow_radius_m, 0.25, 1.0)
        axis_limit = max(self.minimum_axis_command, self.maximum_axis_command * scale)
        if planar_arrived:
            forward = 0.0
            lateral = 0.0
        else:
            forward = clamp(
                self.proportional_gain * forward_error,
                -axis_limit,
                axis_limit,
            )
            lateral = clamp(
                self.proportional_gain * lateral_error,
                -axis_limit,
                axis_limit,
            )
            if (
                abs(forward) < self.minimum_axis_command
                and abs(forward_error) > 0.03
            ):
                forward = math.copysign(
                    self.minimum_axis_command, forward_error
                )
            if (
                abs(lateral) < self.minimum_axis_command
                and abs(lateral_error) > 0.03
            ):
                lateral = math.copysign(
                    self.minimum_axis_command, lateral_error
                )
        if depth_arrived:
            heave = 0.0
        else:
            heave = clamp(
                self.depth_proportional_gain * dz,
                -self.maximum_heave_command,
                self.maximum_heave_command,
            )
            if abs(heave) < self.minimum_heave_command:
                heave = math.copysign(self.minimum_heave_command, dz)

        message = self.neutral_message()
        message.channels[2] = self.axis_pwm(heave)
        message.channels[4] = self.axis_pwm(self.forward_sign * forward)
        message.channels[5] = self.axis_pwm(self.lateral_sign * lateral)
        self.rc_publisher.publish(message)
        self.throttled_status(
            now_ns,
            "MOVING: "
            f"distance={distance:.2f} m depth_error={dz:.2f} m "
            f"body_cmd=({forward:.2f},{lateral:.2f},{heave:.2f})",
        )

    def axis_pwm(self, value: float) -> int:
        return int(round(1500.0 + self.rc_pwm_span * clamp(value, -1.0, 1.0)))

    @staticmethod
    def neutral_message() -> OverrideRCIn:
        message = OverrideRCIn()
        message.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        message.channels[0] = 1500
        message.channels[1] = 1500
        message.channels[2] = 1500
        message.channels[3] = 1500
        message.channels[4] = 1500
        message.channels[5] = 1500
        return message

    def publish_neutral(self) -> None:
        self.rc_publisher.publish(self.neutral_message())

    def release_rc(self) -> None:
        message = OverrideRCIn()
        message.channels = [OverrideRCIn.CHAN_RELEASE] * 18
        self.rc_publisher.publish(message)

    def throttled_status(self, now_ns: int, value: str) -> None:
        if not self.last_status_ns or (now_ns - self.last_status_ns) >= 500_000_000:
            self.publish_status(value)
            self.last_status_ns = now_ns

    def publish_status(self, value: str) -> None:
        self.status_publisher.publish(String(data=value))

    def close(self) -> None:
        if rclpy.ok():
            self.release_rc()


def main() -> None:
    rclpy.init()
    node = SimWaypointRcFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
