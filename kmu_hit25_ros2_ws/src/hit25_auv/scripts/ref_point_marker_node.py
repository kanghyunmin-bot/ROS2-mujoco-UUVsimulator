#!/usr/bin/env python3
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
import tf_transformations as tft
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

try:
    from waterlinked_a50_ros_driver.msg import DvlIntegratedData
    _HAVE_DVL = True
except Exception:  # pragma: no cover
    DvlIntegratedData = None
    _HAVE_DVL = False

try:
    from hit25_auv.msg import AuvSetpoint
    _HAVE_SETPOINT = True
except Exception:  # pragma: no cover
    AuvSetpoint = None
    _HAVE_SETPOINT = False


class RefPointMarkerNode(Node):
    def __init__(self):
        super().__init__("ref_point_marker_node")

        self.parent_frame = self.declare_parameter("parent_frame", "odom").value
        self.auv_frame = self.declare_parameter("auv_frame", "auv_link").value
        self.marker_topic = self.declare_parameter("marker_topic", "/ref_point_marker").value
        self.dvl_use_odom = bool(self.declare_parameter("dvl_use_odom", True).value)
        self.dvl_odom_topic = self.declare_parameter("dvl_odom_topic", "/dvl/odometry").value
        self.dvl_ned_to_flu = bool(self.declare_parameter("dvl_ned_to_flu", True).value)
        self.dvl_topic = self.declare_parameter("dvl_topic", "/dvl/integrated_data").value

        self.max_path_points = int(self.declare_parameter("max_path_points", 2000).value)
        self.path_points = []
        self.last_path_pos = None
        self.min_dist_sq = 0.05 * 0.05

        self.buoy_id_counter = 0
        self.gate_id_counter = 0
        self.qr_id_counter = 0
        self.hydro_id_counter = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)

        if _HAVE_SETPOINT:
            self.setpoint_sub = self.create_subscription(
                AuvSetpoint, "/auv_setpoint", self.setpoint_callback, 10
            )
        else:
            self.get_logger().error("AuvSetpoint msg not available; setpoint markers disabled.")
            self.setpoint_sub = None

        if self.dvl_use_odom:
            self.dvl_sub = self.create_subscription(
                Odometry, self.dvl_odom_topic, self.dvl_odom_callback, 10
            )
        elif _HAVE_DVL:
            self.dvl_sub = self.create_subscription(
                DvlIntegratedData, self.dvl_topic, self.dvl_callback, 10
            )
        else:
            self.get_logger().warn("DVL msg not available; path markers disabled.")
            self.dvl_sub = None

        self.buoy_sub = self.create_subscription(
            PoseStamped, "/buoy_obstacle_pose", self.buoy_pose_callback, 10
        )
        self.gate_sub = self.create_subscription(
            PoseStamped, "/gate_obstacle_pose", self.gate_pose_callback, 10
        )
        self.qr_sub = self.create_subscription(
            PoseStamped, "/qr_obstacle_pose", self.qr_pose_callback, 10
        )
        self.hydro_sub = self.create_subscription(
            PoseStamped, "/hydrophone_obstacle_pose", self.hydro_pose_callback, 10
        )

        path_source = self.dvl_odom_topic if self.dvl_use_odom else self.dvl_topic
        self.get_logger().info(f"[Marker] Initialized. Path Source: {path_source}")

    @staticmethod
    def yaw_to_quat(yaw):
        q = tft.quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

    @staticmethod
    def quat_to_yaw(q):
        q_list = [q.x, q.y, q.z, q.w]
        _, _, yaw = tft.euler_from_quaternion(q_list)
        return yaw

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def dvl_callback(self, msg):
        curr_x = msg.x
        curr_y = -msg.y
        curr_z = -msg.z

        if self.last_path_pos is not None:
            dx = curr_x - self.last_path_pos[0]
            dy = curr_y - self.last_path_pos[1]
            dz = curr_z - self.last_path_pos[2]
            dist_sq = dx * dx + dy * dy + dz * dz
            if dist_sq < self.min_dist_sq:
                return

        self.last_path_pos = (curr_x, curr_y, curr_z)
        self.update_and_publish_path(curr_x, curr_y, curr_z)

    def dvl_odom_callback(self, msg: Odometry):
        curr_x = float(msg.pose.pose.position.x)
        curr_y = float(msg.pose.pose.position.y)
        curr_z = float(msg.pose.pose.position.z)

        if self.dvl_ned_to_flu:
            curr_y = -curr_y
            curr_z = -curr_z

        if self.last_path_pos is not None:
            dx = curr_x - self.last_path_pos[0]
            dy = curr_y - self.last_path_pos[1]
            dz = curr_z - self.last_path_pos[2]
            dist_sq = dx * dx + dy * dy + dz * dz
            if dist_sq < self.min_dist_sq:
                return

        self.last_path_pos = (curr_x, curr_y, curr_z)
        self.update_and_publish_path(curr_x, curr_y, curr_z)

    def setpoint_callback(self, msg):
        try:
            p_x = 0.0
            p_y = 0.0
            p_z = 0.0
            yaw_global = 0.0

            if msg.mode == 0:
                p_x = msg.x
                p_y = msg.y
                p_z = msg.z
                yaw_global = self.wrap_angle(msg.yaw)
            elif msg.mode == 1:
                transform = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    self.auv_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.1),
                )
                t = transform.transform.translation
                q = transform.transform.rotation

                auv_x, auv_y, auv_z = t.x, t.y, t.z
                yaw_auv = self.quat_to_yaw(q)

                cos_yaw = math.cos(yaw_auv)
                sin_yaw = math.sin(yaw_auv)

                p_x = auv_x + (cos_yaw * msg.x - sin_yaw * msg.y)
                p_y = auv_y + (sin_yaw * msg.x + cos_yaw * msg.y)
                p_z = auv_z + msg.z
                yaw_global = self.wrap_angle(yaw_auv + msg.yaw)
            else:
                return

            self.publish_ref_arrow_marker(p_x, p_y, p_z, yaw_global)
        except Exception:
            pass

    def update_and_publish_path(self, x, y, z):
        self.path_points.append((x, y, z))
        if len(self.path_points) > self.max_path_points:
            self.path_points.pop(0)

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.parent_frame
        marker.ns = "auv_actual_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8
        marker.points = [Point(x=px, y=py, z=pz) for (px, py, pz) in self.path_points]
        marker.lifetime = rclpy.duration.Duration().to_msg()

        self.marker_pub.publish(marker)

    def publish_ref_arrow_marker(self, x, y, z, yaw):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.parent_frame
        marker.ns = "ref_setpoint"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = Point(x=x, y=y, z=z)
        marker.pose.orientation = self.yaw_to_quat(yaw)
        marker.scale.x = 0.6
        marker.scale.y = 0.08
        marker.scale.z = 0.08
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = Duration(seconds=0.5).to_msg()
        self.marker_pub.publish(marker)

    def publish_obstacle_marker(self, msg, ns, shape, sx, sy, sz, color, marker_id):
        marker = Marker()
        marker.header = msg.header
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = marker_id
        marker.type = shape
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=3.0).to_msg()
        self.marker_pub.publish(marker)

    def buoy_pose_callback(self, msg):
        self.publish_obstacle_marker(msg, "buoy", Marker.SPHERE, 0.3, 0.3, 0.3, (1.0, 1.0, 0.0), self.buoy_id_counter)
        self.buoy_id_counter += 1

    def gate_pose_callback(self, msg):
        self.publish_obstacle_marker(msg, "gate", Marker.CYLINDER, 0.2, 0.2, 1.0, (0.0, 1.0, 0.0), self.gate_id_counter)
        self.gate_id_counter += 1

    def qr_pose_callback(self, msg):
        self.publish_obstacle_marker(msg, "qr", Marker.CUBE, 0.1, 0.2, 0.2, (1.0, 1.0, 1.0), self.qr_id_counter)
        self.qr_id_counter += 1

    def hydro_pose_callback(self, msg):
        self.publish_obstacle_marker(msg, "hydrophone", Marker.SPHERE, 0.1, 0.1, 0.1, (1.0, 0.0, 0.0), self.hydro_id_counter)
        self.hydro_id_counter += 1


def main():
    rclpy.init()
    node = RefPointMarkerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
