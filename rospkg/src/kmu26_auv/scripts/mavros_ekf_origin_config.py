#!/usr/bin/python3

import math
import time

import rclpy
from geographic_msgs.msg import GeoPointStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandHome
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data


class MavrosEkfOriginConfig(Node):
    """Give a GPS-less Pixhawk a fixed internal EKF geographic anchor."""

    def __init__(self):
        super().__init__("mavros_ekf_origin_config")

        self.state_topic = self.declare_parameter(
            "state_topic", "/mavros/state"
        ).value
        self.origin_set_topic = self.declare_parameter(
            "origin_set_topic", "/mavros/global_position/set_gp_origin"
        ).value
        self.origin_feedback_topic = self.declare_parameter(
            "origin_feedback_topic", "/mavros/global_position/gp_origin"
        ).value
        self.local_position_topic = self.declare_parameter(
            "local_position_topic", "/mavros/local_position/odom"
        ).value
        self.set_home_service = self.declare_parameter(
            "set_home_service", "/mavros/cmd/set_home"
        ).value

        self.latitude = float(self.declare_parameter("latitude", 37.0).value)
        self.longitude = float(self.declare_parameter("longitude", 127.0).value)
        self.altitude = float(self.declare_parameter("altitude", 0.0).value)
        self.require_disarmed = bool(
            self.declare_parameter("require_disarmed", True).value
        )
        self.retry_interval_s = max(
            0.5, float(self.declare_parameter("retry_interval_s", 2.0).value)
        )
        self.set_home = bool(self.declare_parameter("set_home", False).value)

        if (
            not math.isfinite(self.latitude)
            or not math.isfinite(self.longitude)
            or not math.isfinite(self.altitude)
            or not -90.0 <= self.latitude <= 90.0
            or not -180.0 <= self.longitude <= 180.0
            or (abs(self.latitude) < 1.0e-9 and abs(self.longitude) < 1.0e-9)
        ):
            raise ValueError(
                "EKF origin must use a finite, non-zero valid latitude/longitude"
            )

        origin_feedback_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.origin_pub = self.create_publisher(
            GeoPointStamped, self.origin_set_topic, qos_profile_sensor_data
        )
        self.create_subscription(State, self.state_topic, self._state_cb, 10)
        self.create_subscription(
            GeoPointStamped,
            self.origin_feedback_topic,
            self._origin_feedback_cb,
            origin_feedback_qos,
        )
        self.create_subscription(
            Odometry,
            self.local_position_topic,
            self._local_position_cb,
            qos_profile_sensor_data,
        )
        self.home_client = self.create_client(CommandHome, self.set_home_service)
        self.timer = self.create_timer(0.25, self._timer_cb)

        self.connected = False
        self.armed = False
        self.origin_confirmed = False
        self.local_position_seen = False
        self.home_confirmed = not self.set_home
        self.home_future = None
        self.last_origin_publish = 0.0
        self.first_origin_publish = 0.0
        self.last_wait_log = 0.0

        self.get_logger().info(
            "Internal EKF origin ready: "
            f"lat={self.latitude:.6f}, lon={self.longitude:.6f}, "
            f"alt={self.altitude:.1f} m; local ROS/Guided coordinates remain 0,0"
        )

    def _state_cb(self, msg):
        if self.connected and not msg.connected:
            self._reset_connection()
        self.connected = bool(msg.connected)
        self.armed = bool(msg.armed)

    def _reset_connection(self):
        self.origin_confirmed = False
        self.local_position_seen = False
        self.home_confirmed = not self.set_home
        self.home_future = None
        self.last_origin_publish = 0.0
        self.first_origin_publish = 0.0

    def _origin_feedback_cb(self, msg):
        latitude = float(msg.position.latitude)
        longitude = float(msg.position.longitude)
        if not math.isfinite(latitude) or not math.isfinite(longitude):
            return
        if abs(latitude) < 1.0e-9 and abs(longitude) < 1.0e-9:
            return
        if not self.origin_confirmed:
            self.get_logger().info(
                "Pixhawk EKF origin confirmed: "
                f"lat={latitude:.6f}, lon={longitude:.6f}; "
                "this is only an internal geographic anchor"
            )
        self.origin_confirmed = True

    def _local_position_cb(self, _msg):
        if not self.local_position_seen:
            self.get_logger().info(
                "Pixhawk local position is active; Guided may use its captured start frame"
            )
        self.local_position_seen = True
        self.origin_confirmed = True

    def _publish_origin(self, now):
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "earth"
        msg.position.latitude = self.latitude
        msg.position.longitude = self.longitude
        msg.position.altitude = self.altitude
        self.origin_pub.publish(msg)
        self.last_origin_publish = now
        if self.first_origin_publish == 0.0:
            self.first_origin_publish = now
            self.get_logger().info(
                "Published fixed internal EKF origin to Pixhawk"
            )

    def _request_home(self):
        if self.home_confirmed or self.home_future is not None:
            return
        if not self.home_client.service_is_ready():
            return

        request = CommandHome.Request()
        request.current_gps = False
        request.yaw = 0.0
        request.latitude = self.latitude
        request.longitude = self.longitude
        request.altitude = self.altitude
        self.home_future = self.home_client.call_async(request)
        self.home_future.add_done_callback(self._home_response_cb)

    def _home_response_cb(self, future):
        self.home_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warning(f"Pixhawk home request failed: {exc}")
            return
        if response.success:
            self.home_confirmed = True
            self.get_logger().info(
                "Pixhawk home set to the same internal origin"
            )
        else:
            self.get_logger().warning(
                f"Pixhawk rejected home request (result={response.result})"
            )

    def _timer_cb(self):
        if not self.connected or self.local_position_seen:
            return

        now = time.monotonic()
        if self.require_disarmed and self.armed:
            if now - self.last_wait_log >= 5.0:
                self.get_logger().warning(
                    "Waiting to set EKF origin: Pixhawk must be disarmed"
                )
                self.last_wait_log = now
            return

        if (
            self.origin_pub.get_subscription_count() > 0
            and not self.origin_confirmed
            and now - self.last_origin_publish >= self.retry_interval_s
        ):
            self._publish_origin(now)

        if self.first_origin_publish > 0.0 and now - self.first_origin_publish >= 1.0:
            self._request_home()


def main(args=None):
    rclpy.init(args=args)
    node = MavrosEkfOriginConfig()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
