#!/usr/bin/env python3
import math
import threading

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time

from geometry_msgs.msg import TransformStamped
from mavros_msgs.msg import OverrideRCIn
from nav_msgs.msg import Odometry
import tf2_ros
import tf_transformations as tft

from hit25_auv.profilers import TrapezoidalScalarProfiler, TrapezoidalVectorProfiler

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


class PositionController:
    def __init__(self, node):
        self._node = node
        self._lock = threading.Lock()
        self._node.get_logger().info("Initializing PositionController (ROS 2)...")

        self.load_parameters()

        self.curr_pos = [0.0, 0.0, 0.0]
        self.curr_yaw = 0.0
        self.target_pos = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        self.last_dvl_stamp = Time()

        self.xy_profiler = TrapezoidalVectorProfiler(self.max_vel_xy, self.max_xy_accel)
        self.yaw_profiler = TrapezoidalScalarProfiler(self.max_yaw_rate, self.max_yaw_accel, is_angle=True)
        self._initialized_profiler = False
        self._initialized_target = False

        self.rc_override_pub = self._node.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)

        self.dvl_sub = None
        self.dvl_odom_sub = None
        if self.dvl_use_odom:
            self.dvl_odom_sub = self._node.create_subscription(
                Odometry, self.dvl_odom_topic, self.dvl_odom_callback, 10
            )
            self._node.get_logger().info(
                f"Using DVL odometry from {self.dvl_odom_topic} (ned_to_flu={self.dvl_ned_to_flu})."
            )
        else:
            if _HAVE_DVL:
                self.dvl_sub = self._node.create_subscription(
                    DvlIntegratedData, "/dvl/integrated_data", self.dvl_callback, 10
                )
            else:
                self._node.get_logger().error("DVL driver msg not available; controller disabled.")

        if _HAVE_SETPOINT:
            self.setpoint_sub = self._node.create_subscription(
                AuvSetpoint, "/auv_setpoint", self.setpoint_callback, 10
            )
        else:
            self._node.get_logger().error("AuvSetpoint msg not available; controller disabled.")
            self.setpoint_sub = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self._node)
        self.control_timer = self._node.create_timer(1.0 / self.control_rate, self.control_step)

    def _param(self, name, default):
        if self._node.has_parameter(name):
            return self._node.get_parameter(name).value
        return self._node.declare_parameter(name, default).value

    def load_parameters(self):
        self.enable_profile_yaw = self._param("enable_profile_yaw", True)
        self.enable_profile_xy = self._param("enable_profile_xy", True)

        self.control_rate = self._param("control_rate", 20.0)
        self.odom_frame = self._param("odom_frame", "odom")
        self.auv_link_frame = self._param("auv_link_frame", "auv_link")
        self.dvl_frame = self._param("dvl_frame", "dvl")
        self.dvl_timeout = Duration(seconds=float(self._param("dvl_timeout", 0.7)))
        self.dvl_use_odom = bool(self._param("dvl_use_odom", True))
        self.dvl_odom_topic = self._param("dvl_odom_topic", "/dvl/odometry")
        self.dvl_ned_to_flu = bool(self._param("dvl_ned_to_flu", True))

        self.Kp_x = self._param("Kp_x", 0.8)
        self.Kp_y = self._param("Kp_y", 0.8)
        self.Kp_z = self._param("Kp_z", 1.0)
        self.Kp_yaw = self._param("Kp_yaw", 1.0)

        self.K_ff_xy = self._param("K_ff_xy", 0.0)
        self.K_ff_yaw = self._param("K_ff_yaw", 0.0)

        self.max_vel_xy = self._param("max_vel_xy", 1.0)
        self.max_vel_z = self._param("max_vel_z", 0.5)
        self.max_yaw_rate = self._param("max_yaw_rate", 0.5)
        self.max_yaw_accel = self._param("max_yaw_accel", 0.3)
        self.max_xy_accel = self._param("max_xy_accel", 0.2)

        self.ch_throttle = 2
        self.ch_forward = 4
        self.ch_lateral = 5
        self.ch_yaw = 3
        self.pwm_neutral = 1500
        self.dvl_x = self._param("dvl_x", 0.0)
        self.dvl_y = self._param("dvl_y", 0.0)
        self.dvl_z = self._param("dvl_z", 0.0)
        self.dvl_roll = self._param("dvl_roll", 0.0)
        self.dvl_pitch = self._param("dvl_pitch", 0.0)
        self.dvl_yaw = self._param("dvl_yaw", 0.0)

        if hasattr(self, 'xy_profiler'):
            self.xy_profiler.max_vel = self.max_vel_xy
            self.xy_profiler.max_accel = self.max_xy_accel
        if hasattr(self, 'yaw_profiler'):
            self.yaw_profiler.max_vel = self.max_yaw_rate
            self.yaw_profiler.max_accel = self.max_yaw_accel

    def dvl_callback(self, msg):
        with self._lock:
            self.curr_pos = [msg.x, -msg.y, -msg.z]
            self.curr_yaw = self.wrap_angle(-msg.yaw_rad)
            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                self.last_dvl_stamp = self._node.get_clock().now()
            else:
                self.last_dvl_stamp = Time.from_msg(msg.header.stamp)

            if not self._initialized_profiler:
                self.xy_profiler.reset([self.curr_pos[0], self.curr_pos[1]])
                self.yaw_profiler.reset(self.curr_yaw)
                self._initialized_profiler = True

    def dvl_odom_callback(self, msg: Odometry):
        with self._lock:
            pos = msg.pose.pose.position
            rot = msg.pose.pose.orientation
            yaw = tft.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])[2]

            x, y, z = pos.x, pos.y, pos.z
            if self.dvl_ned_to_flu:
                x, y, z = x, -y, -z
                yaw = self.wrap_angle(-yaw)
            else:
                yaw = self.wrap_angle(yaw)

            self.curr_pos = [x, y, z]
            self.curr_yaw = yaw

            if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
                self.last_dvl_stamp = self._node.get_clock().now()
            else:
                self.last_dvl_stamp = Time.from_msg(msg.header.stamp)

            if not self._initialized_profiler:
                self.xy_profiler.reset([self.curr_pos[0], self.curr_pos[1]])
                self.yaw_profiler.reset(self.curr_yaw)
                self._initialized_profiler = True

    def setpoint_callback(self, msg):
        with self._lock:
            if msg.mode == 0:
                self.target_pos = [msg.x, msg.y, msg.z]
                self.target_yaw = self.wrap_angle(msg.yaw)
            elif msg.mode == 1:
                cx, cy, cz = self.curr_pos
                cyaw = self.curr_yaw
                self.target_pos = [cx + msg.x, cy + msg.y, cz + msg.z]
                self.target_yaw = self.wrap_angle(cyaw + msg.yaw)
            self._node.get_logger().info(
                f"New Target: Pos={self.target_pos}, Yaw={self.target_yaw:.2f}"
            )

    def control_step(self):
        if not self._initialized_profiler:
            return

        with self._lock:
            curr_pos = list(self.curr_pos)
            curr_yaw = self.curr_yaw
            target_pos = list(self.target_pos)
            target_yaw = self.target_yaw
            last_dvl_stamp = self.last_dvl_stamp

        now = self._node.get_clock().now()
        dt = 1.0 / self.control_rate

        if (now - last_dvl_stamp) > self.dvl_timeout:
            self.publish_rc([self.pwm_neutral] * 18)
            self.publish_tf(now)
            return

        # Initialize target to current position if not done yet
        if not self._initialized_target:
            self.target_pos = list(self.curr_pos)
            self.target_yaw = self.curr_yaw
            self._initialized_target = True
            if self._initialized_profiler:
                self.xy_profiler.reset([self.curr_pos[0], self.curr_pos[1]])
                self.yaw_profiler.reset(self.curr_yaw)
            self._node.get_logger().info(f"Initialized target to current pose: {self.target_pos}, Yaw={self.target_yaw:.2f}")

        if self.enable_profile_xy:
            ref_pos_xy, ref_vel_xy = self.xy_profiler.update([target_pos[0], target_pos[1]], dt)
            current_k_ff_xy = self.K_ff_xy
        else:
            ref_pos_xy = [target_pos[0], target_pos[1]]
            ref_vel_xy = [0.0, 0.0]
            current_k_ff_xy = 0.0

        if self.enable_profile_yaw:
            ref_yaw, ref_yaw_vel = self.yaw_profiler.update(target_yaw, dt)
            current_k_ff_yaw = self.K_ff_yaw
        else:
            ref_yaw = target_yaw
            ref_yaw_vel = 0.0
            current_k_ff_yaw = 0.0

        ex = ref_pos_xy[0] - curr_pos[0]
        ey = ref_pos_xy[1] - curr_pos[1]
        ez = target_pos[2] - curr_pos[2]
        e_yaw = self.wrap_angle(ref_yaw - curr_yaw)

        vx_cmd = (self.Kp_x * ex) + (ref_vel_xy[0] * current_k_ff_xy)
        vy_cmd = (self.Kp_y * ey) + (ref_vel_xy[1] * current_k_ff_xy)
        vz_cmd = self.Kp_z * ez
        yaw_rate_cmd = (self.Kp_yaw * e_yaw) + (ref_yaw_vel * current_k_ff_yaw)

        vx_cmd = self.clamp(vx_cmd, -self.max_vel_xy, self.max_vel_xy)
        vy_cmd = self.clamp(vy_cmd, -self.max_vel_xy, self.max_vel_xy)
        heave_cmd = self.clamp(-vz_cmd, -self.max_vel_z, self.max_vel_z)
        yaw_cmd = self.clamp(yaw_rate_cmd, -self.max_yaw_rate, self.max_yaw_rate)

        c_yaw = math.cos(-curr_yaw)
        s_yaw = math.sin(-curr_yaw)
        surge_cmd = vx_cmd * c_yaw - vy_cmd * s_yaw
        sway_cmd = vx_cmd * s_yaw + vy_cmd * c_yaw

        axis_forward = self.clamp(surge_cmd / self.max_vel_xy, -1.0, 1.0)
        axis_lateral = self.clamp(sway_cmd / self.max_vel_xy, -1.0, 1.0)
        axis_heave = self.clamp(heave_cmd / self.max_vel_z, -1.0, 1.0)
        axis_yaw = self.clamp(yaw_cmd / self.max_yaw_rate, -1.0, 1.0)

        pwm_channels = [self.pwm_neutral] * 18
        pwm_channels[self.ch_throttle] = self.scale_axis_to_pwm(axis_heave)
        pwm_channels[self.ch_forward] = self.scale_axis_to_pwm(axis_forward)
        pwm_channels[self.ch_lateral] = self.scale_axis_to_pwm(-axis_lateral)
        pwm_channels[self.ch_yaw] = self.scale_axis_to_pwm(-axis_yaw)

        self.publish_rc(pwm_channels)
        self.publish_tf(now)

    def publish_rc(self, channels):
        rc_msg = OverrideRCIn()
        rc_msg.channels = list(channels)
        self.rc_override_pub.publish(rc_msg)

    def publish_tf(self, stamp):
        transforms = []
        with self._lock:
            p = self.curr_pos
            y = self.curr_yaw

        t1 = TransformStamped()
        t1.header.stamp = stamp.to_msg()
        t1.header.frame_id = self.odom_frame
        t1.child_frame_id = self.auv_link_frame
        t1.transform.translation.x = float(p[0])
        t1.transform.translation.y = float(p[1])
        t1.transform.translation.z = float(p[2])
        q = tft.quaternion_from_euler(0, 0, y)
        t1.transform.rotation.x = float(q[0])
        t1.transform.rotation.y = float(q[1])
        t1.transform.rotation.z = float(q[2])
        t1.transform.rotation.w = float(q[3])
        transforms.append(t1)

        t2 = TransformStamped()
        t2.header.stamp = stamp.to_msg()
        t2.header.frame_id = self.auv_link_frame
        t2.child_frame_id = self.dvl_frame
        t2.transform.translation.x = float(self.dvl_x)
        t2.transform.translation.y = float(self.dvl_y)
        t2.transform.translation.z = float(self.dvl_z)
        q2 = tft.quaternion_from_euler(self.dvl_roll, self.dvl_pitch, self.dvl_yaw)
        t2.transform.rotation.x = float(q2[0])
        t2.transform.rotation.y = float(q2[1])
        t2.transform.rotation.z = float(q2[2])
        t2.transform.rotation.w = float(q2[3])
        transforms.append(t2)

        self.tf_broadcaster.sendTransform(transforms)

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def clamp(v, min_v, max_v):
        return max(min_v, min(v, max_v))

    @staticmethod
    def scale_axis_to_pwm(val):
        return int(max(1200, min(1500 + val * 300, 1800)))


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('position_controller')
    
    try:
        controller = PositionController(node)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
