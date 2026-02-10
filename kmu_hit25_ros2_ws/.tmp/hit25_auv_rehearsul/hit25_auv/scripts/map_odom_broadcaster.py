#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
map_odom_broadcaster.py

This module defines the MapOdomBroadcaster class.
It is intended to be imported by another main ROS node (e.g., main_node.py).

This class is responsible for:
1. Broadcasting ONLY the transform from 'map' to 'odom'.
2. Maintaining an internal offset (x, y, z, yaw) for this transform.
3. Providing methods to update this offset (e.g., set_identity, set_offset).
4. Providing a utility to get the 4x4 numpy transform matrix.

NOTE: This module does NOT broadcast any other transforms
(e.g., odom -> base_link). It is assumed 'odom -> base_link'
is handled by a different node (like controller.py).
"""

import rospy
import threading
import math
import numpy as np

# TF
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped

# TODO: Define and import custom service/message types if used
# from std_srvs.srv import Empty, EmptyResponse # Example for simple trigger
# from geometry_msgs.msg import PoseStamped # Example for offset command


class MapOdomBroadcaster(object):
    """
    Manages and broadcasts the 'map' -> 'odom' TF transform.
    """
    def __init__(self):
        """
        Initializes the MapOdomBroadcaster.
        - Loads ROS parameters (frame names, initial offsets, publish rate).
        - Initializes the TF broadcaster.
        - Initializes internal state (offsets) and a threading.Lock.
        - Starts a rospy.Timer for periodic TF publication.
        - (Optional) Initializes service or topic interfaces for offset updates.
        """
        rospy.loginfo("Initializing MapOdomBroadcaster...")

        # --- Concurrency ---
        self.lock = threading.Lock()

        # --- Load Parameters ---
        self.load_parameters()

        # --- Internal State Variables (protected by self.lock) ---
        # These represent the map -> odom transform
        self.x_off = self.init_x_off
        self.y_off = self.init_y_off
        self.z_off = self.init_z_off
        self.yaw_off = self.init_yaw_off

        # --- ROS Communications ---
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # --- Optional Service/Topic Interfaces ---
        self.setup_interfaces()

        # --- TF Publish Loop ---
        self.tf_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_tf
        )
        
        rospy.loginfo(
            f"MapOdomBroadcaster initialized. Publishing '{self.map_frame}' -> "
            f"'{self.odom_frame}' at {self.publish_rate} Hz."
        )

    def load_parameters(self):
        """
        Loads parameters from the ROS parameter server.
        """
        # Frame names
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")

        # Initial offset (map -> odom)
        self.init_x_off = rospy.get_param("~x_off", 0.0)
        self.init_y_off = rospy.get_param("~y_off", 0.0)
        self.init_z_off = rospy.get_param("~z_off", 0.0)
        self.init_yaw_off = rospy.get_param("~yaw_off", 0.0) # rad

        # Publish rate
        self.publish_rate = rospy.get_param("~publish_rate", 20.0) # Hz
        self.mode = rospy.get_param("~mode", "dynamic") # "dynamic" or "static"

        # Optional interfaces
        self.enable_service = rospy.get_param("~enable_set_offset_service", False)
        self.service_name = rospy.get_param("~set_offset_service_name", "set_map_odom_offset")
        self.enable_topic = rospy.get_param("~enable_offset_cmd_topic", False)
        self.topic_name = rospy.get_param("~offset_cmd_topic_name", "map_odom_offset_cmd")

    def setup_interfaces(self):
        """
        Initializes optional ROS services or subscribers for offset updates.
        """
        if self.enable_service:
            # TODO: Define a custom service (e.g., SetMapOdomOffset.srv)
            # with fields: float64 x, float64 y, float64 z, float64 yaw
            # For now, we'll log a warning.
            rospy.logwarn(
                f"Service '{self.service_name}' enabled but NOT implemented. "
                "A custom service definition is required."
            )
            # Example:
            # from your_project_msgs.srv import SetMapOdomOffset
            # self.offset_service = rospy.Service(
            #     self.service_name, SetMapOdomOffset, self.handle_set_offset_srv
            # )

        if self.enable_topic:
            # TODO: Define a custom message (e.g., geometry_msgs/Pose)
            rospy.logwarn(
                f"Topic '{self.topic_name}' enabled but NOT implemented. "
                "A message type must be chosen for the callback."
            )
            # Example:
            # from geometry_msgs.msg import Pose
            # self.offset_sub = rospy.Subscriber(
            #     self.topic_name, Pose, self.offset_cmd_callback
            # )

    def publish_tf(self, event=None):
        """
        Periodic callback to publish the map -> odom TF.
        Uses a thread-safe snapshot of the current offset.
        """
        
        # 1. Snapshot state variables (minimize lock time)
        with self.lock:
            x, y, z, yaw = self.x_off, self.y_off, self.z_off, self.yaw_off

        # 2. Create TransformStamped message
        stamp = rospy.Time.now()
        
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        
        q = tft.quaternion_from_euler(0, 0, yaw) # ENU: Rz(yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # 3. Broadcast the transform
        self.tf_broadcaster.sendTransform(t)
        
        # Note: If mode is "static", this timer-based broadcast is
        # technically still "dynamic". A true static implementation
        # would use tf2_ros.StaticTransformBroadcaster, but this
        # timer-based approach allows the offset to be updated at runtime.

    # --- Offset Setting/Update Methods ---

    def set_identity(self):
        """
        Resets the map -> odom transform to be an identity transform (no offset).
        """
        rospy.loginfo(f"Resetting '{self.map_frame}' -> '{self.odom_frame}' to Identity.")
        with self.lock:
            self.x_off = 0.0
            self.y_off = 0.0
            self.z_off = 0.0
            self.yaw_off = 0.0

    def set_offset(self, x, y, z, yaw):
        """
        Sets the map -> odom offset to new absolute values.
        This is thread-safe.
        """
        rospy.loginfo(
            f"Setting new map->odom offset: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={yaw:.2f}"
        )
        with self.lock:
            self.x_off = x
            self.y_off = y
            self.z_off = z
            self.yaw_off = self.wrap_angle(yaw)

    def apply_drift_correction(self, delta_yaw, dx=0.0, dy=0.0, dz=0.0):
        """
        Applies a delta (correction) to the existing map -> odom offset.
        Useful for incrementally correcting drift.
        This is thread-safe.
        """
        with self.lock:
            self.x_off += dx
            self.y_off += dy
            self.z_off += dz
            self.yaw_off = self.wrap_angle(self.yaw_off + delta_yaw)
            
            rospy.logdebug(f"Applied drift: dX={dx}, dY={dy}, dZ={dz}, dYaw={delta_yaw}")
            rospy.logdebug(f"New offset: x={self.x_off}, yaw={self.yaw_off}")

    # --- (Optional) Service/Topic Handlers ---

    def handle_set_offset_srv(self, req):
        """
        ROS Service callback stub to set the offset.
        """
        # TODO: Implement this based on your custom .srv file
        # Example (if req has x, y, z, yaw fields):
        # try:
        #     self.set_offset(req.x, req.y, req.z, req.yaw)
        #     return {'success': True, 'message': 'Offset updated'}
        # except Exception as e:
        #     rospy.logerr(f"Failed to set offset via service: {e}")
        #     return {'success': False, 'message': str(e)}
        
        rospy.logwarn("handle_set_offset_srv called but not implemented.")
        # Must return the correct response type for your service
        return {} 

    def offset_cmd_callback(self, msg):
        """
        ROS Topic callback stub to set or update the offset.
        """
        # TODO: Implement this based on your chosen message type
        # Example (if msg is geometry_msgs/Pose):
        # x = msg.position.x
        # y = msg.position.y
        # z = msg.position.z
        # q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        # (roll, pitch, yaw) = tft.euler_from_quaternion(q)
        # self.set_offset(x, y, z, yaw)
        
        rospy.logwarn("offset_cmd_callback called but not implemented.")

    # --- TF Matrix Utility ---

    def get_tf_matrix(self, from_frame, to_frame):
        """
        Returns the 4x4 homogeneous transform (numpy.ndarray)
        for 'map' -> 'odom' or 'odom' -> 'map'.
        
        This method is thread-safe.
        """
        # Normalize frame names (remove leading '/')
        from_frame_norm = from_frame.lstrip('/')
        to_frame_norm = to_frame.lstrip('/')
        
        # Get a thread-safe snapshot of the dynamic state
        with self.lock:
            x, y, z, yaw = self.x_off, self.y_off, self.z_off, self.yaw_off

        # --- Match request ---
        req = (from_frame_norm, to_frame_norm)
        ref_map = self.map_frame
        ref_odom = self.odom_frame

        if req == (ref_map, ref_odom):
            # Request: map -> odom
            T_trans = tft.translation_matrix([x, y, z])
            T_rot = tft.euler_matrix(0, 0, yaw)
            return np.dot(T_trans, T_rot)
            
        elif req == (ref_odom, ref_map):
            # Request: odom -> map
            T_trans = tft.translation_matrix([x, y, z])
            T_rot = tft.euler_matrix(0, 0, yaw)
            M_map_odom = np.dot(T_trans, T_rot)
            # Return the inverse
            return np.linalg.inv(M_map_odom)
            
        elif from_frame_norm == to_frame_norm:
            # Request: map -> map or odom -> odom
            return np.eye(4)
            
        else:
            rospy.logwarn(
                f"get_tf_matrix: Unsupported transform request: "
                f"'{from_frame}' -> '{to_frame}'"
            )
            return np.eye(4)

    # --- Static Utility Methods ---

    @staticmethod
    def wrap_angle(angle):
        """
        Wraps an angle in radians to the range [-pi, +pi].
        """
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def clamp(value, min_val, max_val):
        """
        Clamps a value to the specified [min_val, max_val] range.
        """
        return max(min_val, min(value, max_val))


# --- Main execution ---
# This file is intended to be a module, so no 'if __name__ == "__main__":'
# block with rospy.init_node() or rospy.spin() is included.
# The class should be imported and instantiated by a main node.