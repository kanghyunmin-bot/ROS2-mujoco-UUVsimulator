#!/usr/bin/env python3
import rclpy
import tf2_ros
import tf_transformations as tft
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from hit25_auv.map_odom_broadcaster import MapOdomBroadcaster

try:
    from waterlinked_a50_ros_driver.msg import DvlIntegratedData
    _HAVE_DVL = True
except Exception:  # pragma: no cover
    DvlIntegratedData = None
    _HAVE_DVL = False


class RovTfBroadcasterNode(Node):
    def __init__(self):
        super().__init__("rov_tf_broadcaster")
        self.get_logger().info("[RovTf] Initializing ROV TF Manager...")

        try:
            self.map_odom = MapOdomBroadcaster(self)
            self.get_logger().info("[RovTf] MapOdomBroadcaster loaded.")
        except Exception as e:
            self.get_logger().error(f"[RovTf] Failed to load MapOdomBroadcaster: {e}")
            self.map_odom = None

        self.odom_frame = self._param("odom_frame", "odom")
        self.base_frame = self._param("base_frame", "auv_link")
        self.dvl_frame = self._param("dvl_frame", "dvl")
        self.dvl_use_odom = bool(self._param("dvl_use_odom", True))
        self.dvl_odom_topic = self._param("dvl_odom_topic", "/dvl/odometry")
        self.dvl_ned_to_flu = bool(self._param("dvl_ned_to_flu", True))

        self.dvl_x = self._param("dvl_x", 0.0)
        self.dvl_y = self._param("dvl_y", 0.0)
        self.dvl_z = self._param("dvl_z", 0.0)
        self.dvl_r = self._param("dvl_roll", 0.0)
        self.dvl_p = self._param("dvl_pitch", 0.0)
        self.dvl_yaw_offset = self._param("dvl_yaw", 0.0)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        if self.dvl_use_odom:
            self.dvl_sub = self.create_subscription(
                Odometry, self.dvl_odom_topic, self.dvl_odom_callback, 10
            )
            self.get_logger().info(
                f"[RovTf] Using odometry source {self.dvl_odom_topic} (ned_to_flu={self.dvl_ned_to_flu})."
            )
        elif _HAVE_DVL:
            self.dvl_sub = self.create_subscription(
                DvlIntegratedData, "/dvl/integrated_data", self.dvl_callback, 10
            )
        else:
            self.get_logger().error("DVL msg not available; RovTfBroadcaster disabled.")
            self.dvl_sub = None

        self.get_logger().info("[RovTf] Ready. Managing Full TF Tree.")

    def _param(self, name, default):
        if self.has_parameter(name):
            return self.get_parameter(name).value
        return self.declare_parameter(name, default).value

    def dvl_callback(self, msg):
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            current_time = self.get_clock().now().to_msg()
        else:
            current_time = msg.header.stamp

        pos_x = msg.x
        pos_y = -msg.y
        pos_z = -msg.z
        yaw = -msg.yaw_rad

        t_base = TransformStamped()
        t_base.header.stamp = current_time
        t_base.header.frame_id = self.odom_frame
        t_base.child_frame_id = self.base_frame
        t_base.transform.translation.x = float(pos_x)
        t_base.transform.translation.y = float(pos_y)
        t_base.transform.translation.z = float(pos_z)

        q = tft.quaternion_from_euler(0, 0, yaw)
        t_base.transform.rotation.x = float(q[0])
        t_base.transform.rotation.y = float(q[1])
        t_base.transform.rotation.z = float(q[2])
        t_base.transform.rotation.w = float(q[3])

        t_dvl = TransformStamped()
        t_dvl.header.stamp = current_time
        t_dvl.header.frame_id = self.base_frame
        t_dvl.child_frame_id = self.dvl_frame
        t_dvl.transform.translation.x = float(self.dvl_x)
        t_dvl.transform.translation.y = float(self.dvl_y)
        t_dvl.transform.translation.z = float(self.dvl_z)

        q_dvl = tft.quaternion_from_euler(self.dvl_r, self.dvl_p, self.dvl_yaw_offset)
        t_dvl.transform.rotation.x = float(q_dvl[0])
        t_dvl.transform.rotation.y = float(q_dvl[1])
        t_dvl.transform.rotation.z = float(q_dvl[2])
        t_dvl.transform.rotation.w = float(q_dvl[3])

        self.tf_broadcaster.sendTransform([t_base, t_dvl])

    def dvl_odom_callback(self, msg: Odometry):
        current_time = msg.header.stamp
        if current_time.sec == 0 and current_time.nanosec == 0:
            current_time = self.get_clock().now().to_msg()

        pos_x = float(msg.pose.pose.position.x)
        pos_y = float(msg.pose.pose.position.y)
        pos_z = float(msg.pose.pose.position.z)
        q = msg.pose.pose.orientation
        yaw = float(tft.euler_from_quaternion([q.x, q.y, q.z, q.w])[2])

        if self.dvl_ned_to_flu:
            pos_y = -pos_y
            pos_z = -pos_z
            yaw = -yaw

        t_base = TransformStamped()
        t_base.header.stamp = current_time
        t_base.header.frame_id = self.odom_frame
        t_base.child_frame_id = self.base_frame
        t_base.transform.translation.x = pos_x
        t_base.transform.translation.y = pos_y
        t_base.transform.translation.z = pos_z

        q_base = tft.quaternion_from_euler(0, 0, yaw)
        t_base.transform.rotation.x = float(q_base[0])
        t_base.transform.rotation.y = float(q_base[1])
        t_base.transform.rotation.z = float(q_base[2])
        t_base.transform.rotation.w = float(q_base[3])

        t_dvl = TransformStamped()
        t_dvl.header.stamp = current_time
        t_dvl.header.frame_id = self.base_frame
        t_dvl.child_frame_id = self.dvl_frame
        t_dvl.transform.translation.x = float(self.dvl_x)
        t_dvl.transform.translation.y = float(self.dvl_y)
        t_dvl.transform.translation.z = float(self.dvl_z)

        q_dvl = tft.quaternion_from_euler(self.dvl_r, self.dvl_p, self.dvl_yaw_offset)
        t_dvl.transform.rotation.x = float(q_dvl[0])
        t_dvl.transform.rotation.y = float(q_dvl[1])
        t_dvl.transform.rotation.z = float(q_dvl[2])
        t_dvl.transform.rotation.w = float(q_dvl[3])

        self.tf_broadcaster.sendTransform([t_base, t_dvl])


def main():
    rclpy.init()
    node = RovTfBroadcasterNode()
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
