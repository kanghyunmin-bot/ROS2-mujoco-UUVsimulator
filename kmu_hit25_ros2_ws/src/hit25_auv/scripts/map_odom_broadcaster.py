#!/usr/bin/env python3
import math
import threading
import numpy as np

import tf2_ros
import tf_transformations as tft
from geometry_msgs.msg import TransformStamped


class MapOdomBroadcaster:
    """Broadcasts map -> odom TF with adjustable offset."""

    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()

        self._load_parameters()

        self.x_off = self.init_x_off
        self.y_off = self.init_y_off
        self.z_off = self.init_z_off
        self.yaw_off = self.init_yaw_off

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self._node)
        self._setup_interfaces()

        self._timer = self._node.create_timer(1.0 / self.publish_rate, self.publish_tf)

        self._node.get_logger().info(
            f"MapOdomBroadcaster initialized. Publishing '{self.map_frame}' -> '{self.odom_frame}' at {self.publish_rate} Hz."
        )

    def _param(self, name, default):
        if self._node.has_parameter(name):
            return self._node.get_parameter(name).value
        return self._node.declare_parameter(name, default).value

    def _load_parameters(self):
        self.map_frame = self._param("map_frame", "map")
        self.odom_frame = self._param("odom_frame", "odom")

        self.init_x_off = self._param("x_off", 0.0)
        self.init_y_off = self._param("y_off", 0.0)
        self.init_z_off = self._param("z_off", 0.0)
        self.init_yaw_off = self._param("yaw_off", 0.0)

        self.publish_rate = self._param("publish_rate", 20.0)
        self.mode = self._param("mode", "dynamic")

        self.enable_service = self._param("enable_set_offset_service", False)
        self.service_name = self._param("set_offset_service_name", "set_map_odom_offset")
        self.enable_topic = self._param("enable_offset_cmd_topic", False)
        self.topic_name = self._param("offset_cmd_topic_name", "map_odom_offset_cmd")

    def _setup_interfaces(self):
        if self.enable_service:
            self._node.get_logger().warn(
                f"Service '{self.service_name}' enabled but NOT implemented."
            )
        if self.enable_topic:
            self._node.get_logger().warn(
                f"Topic '{self.topic_name}' enabled but NOT implemented."
            )

    def publish_tf(self):
        with self._lock:
            x, y, z, yaw = self.x_off, self.y_off, self.z_off, self.yaw_off

        t = TransformStamped()
        t.header.stamp = self._node.get_clock().now().to_msg()
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        q = tft.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])

        self._tf_broadcaster.sendTransform(t)

    def set_identity(self):
        self._node.get_logger().info(
            f"Resetting '{self.map_frame}' -> '{self.odom_frame}' to Identity."
        )
        with self._lock:
            self.x_off = 0.0
            self.y_off = 0.0
            self.z_off = 0.0
            self.yaw_off = 0.0

    def set_offset(self, x, y, z, yaw):
        self._node.get_logger().info(
            f"Setting map->odom offset: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}"
        )
        with self._lock:
            self.x_off = x
            self.y_off = y
            self.z_off = z
            self.yaw_off = self.wrap_angle(yaw)

    def apply_drift_correction(self, delta_yaw, dx=0.0, dy=0.0, dz=0.0):
        with self._lock:
            self.x_off += dx
            self.y_off += dy
            self.z_off += dz
            self.yaw_off = self.wrap_angle(self.yaw_off + delta_yaw)

    def get_tf_matrix(self, from_frame, to_frame):
        from_frame_norm = from_frame.lstrip('/')
        to_frame_norm = to_frame.lstrip('/')

        with self._lock:
            x, y, z, yaw = self.x_off, self.y_off, self.z_off, self.yaw_off

        req = (from_frame_norm, to_frame_norm)
        ref_map = self.map_frame
        ref_odom = self.odom_frame

        if req == (ref_map, ref_odom):
            T_trans = tft.translation_matrix([x, y, z])
            T_rot = tft.euler_matrix(0, 0, yaw)
            return np.dot(T_trans, T_rot)
        if req == (ref_odom, ref_map):
            T_trans = tft.translation_matrix([x, y, z])
            T_rot = tft.euler_matrix(0, 0, yaw)
            M_map_odom = np.dot(T_trans, T_rot)
            return np.linalg.inv(M_map_odom)
        if from_frame_norm == to_frame_norm:
            return np.eye(4)

        self._node.get_logger().warn(
            f"get_tf_matrix: Unsupported transform request: '{from_frame}' -> '{to_frame}'"
        )
        return np.eye(4)

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
