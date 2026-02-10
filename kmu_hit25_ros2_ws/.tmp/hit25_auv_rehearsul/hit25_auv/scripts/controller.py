#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
controller.py
[Update] Feed-Forward 게인(K_ff)을 파라미터화 하여 조절 가능하도록 수정
"""

import rospy
import threading
import math
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, TransformStamped
from mavros_msgs.msg import OverrideRCIn
import tf2_ros
import tf.transformations as tft

# Profiler Import
try:
    from profilers import TrapezoidalScalarProfiler, TrapezoidalVectorProfiler
except ImportError:
    rospy.logfatal("Could not import 'profilers.py'.")
    raise

# DVL Message Import
try:
    from waterlinked_a50_ros_driver.msg import DvlIntegratedData
except ImportError:
    class DvlIntegratedData: pass
try:
    from hit25_auv.msg import AuvSetpoint
except ImportError:
    class AuvSetpoint: pass

class PositionController(object):
    def __init__(self):
        rospy.loginfo("Initializing PositionController (Tunable Feed-Forward)...")
        self.lock = threading.Lock()
        self.load_parameters()

        # State Variables
        self.curr_pos = [0.0, 0.0, 0.0]
        self.curr_yaw = 0.0
        self.target_pos = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        self.last_dvl_stamp = rospy.Time(0)

        # Profilers
        self.xy_profiler = TrapezoidalVectorProfiler(self.max_vel_xy, self.max_xy_accel)
        self.yaw_profiler = TrapezoidalScalarProfiler(self.max_yaw_rate, self.max_yaw_accel, is_angle=True)
        self._initialized_profiler = False

        # ROS Setup
        self.rc_override_pub = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.dvl_sub = rospy.Subscriber("/dvl/integrated_data", DvlIntegratedData, self.dvl_callback, queue_size=10)
        self.setpoint_sub = rospy.Subscriber("/auv_setpoint", AuvSetpoint, self.setpoint_callback, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.control_timer = rospy.Timer(rospy.Duration(1.0 / self.control_rate), self.control_step)
        
    def load_parameters(self):
        # Flags
        self.enable_profile_yaw = rospy.get_param("~enable_profile_yaw", True)
        self.enable_profile_xy  = rospy.get_param("~enable_profile_xy", True)

        # Basic Params
        self.control_rate = rospy.get_param("~control_rate", 20.0)
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.auv_link_frame = rospy.get_param("~auv_link_frame", "auv_link")
        self.dvl_frame = rospy.get_param("~dvl_frame", "dvl")
        self.dvl_timeout = rospy.Duration(rospy.get_param("~dvl_timeout", 0.7))

        # Control Gains (P)
        self.Kp_x = rospy.get_param("~Kp_x", 0.8)
        self.Kp_y = rospy.get_param("~Kp_y", 0.8)
        self.Kp_z = rospy.get_param("~Kp_z", 1.0)
        self.Kp_yaw = rospy.get_param("~Kp_yaw", 1.0)
        
        # [NEW] Feed-Forward Gains (0.0 = OFF, 1.0 = Full Prediction)
        self.K_ff_xy = rospy.get_param("~K_ff_xy", 0.0)   # 기본값 0.0 (OFF)
        self.K_ff_yaw = rospy.get_param("~K_ff_yaw", 0.0) # 기본값 0.0 (OFF)
        
        # Limits
        self.max_vel_xy = rospy.get_param("~max_vel_xy", 1.0)
        self.max_vel_z = rospy.get_param("~max_vel_z", 0.5)
        self.max_yaw_rate = rospy.get_param("~max_yaw_rate", 0.5)
        self.max_yaw_accel = rospy.get_param("~max_yaw_accel", 0.3) 
        self.max_xy_accel = rospy.get_param("~max_xy_accel", 0.2) 

        # ETC
        self.ch_throttle=2; self.ch_forward=4; self.ch_lateral=5; self.ch_yaw=3
        self.pwm_neutral=1500
        self.dvl_x=0.0; self.dvl_y=0.0; self.dvl_z=0.0
        self.dvl_roll=0.0; self.dvl_pitch=0.0; self.dvl_yaw=0.0
        
        # Hot-reload support
        if hasattr(self, 'xy_profiler'):
            self.xy_profiler.max_vel = self.max_vel_xy
            self.xy_profiler.max_accel = self.max_xy_accel
        if hasattr(self, 'yaw_profiler'):
            self.yaw_profiler.max_vel = self.max_yaw_rate
            self.yaw_profiler.max_accel = self.max_yaw_accel

    # (Callbacks 생략 - 기존과 동일)
    def dvl_callback(self, msg):
        with self.lock:
            self.curr_pos = [msg.x, -msg.y, -msg.z]
            self.curr_yaw = self.wrap_angle(-msg.yaw_rad)
            self.last_dvl_stamp = msg.header.stamp
            if not self._initialized_profiler:
                self.xy_profiler.reset([self.curr_pos[0], self.curr_pos[1]])
                self.yaw_profiler.reset(self.curr_yaw)
                self._initialized_profiler = True

    def setpoint_callback(self, msg):
        with self.lock:
            if msg.mode == 0:
                self.target_pos = [msg.x, msg.y, msg.z]
                self.target_yaw = self.wrap_angle(msg.yaw)
            elif msg.mode == 1:
                cx, cy, cz = self.curr_pos; cyaw = self.curr_yaw
                self.target_pos = [cx + msg.x, cy + msg.y, cz + msg.z]
                self.target_yaw = self.wrap_angle(cyaw + msg.yaw)
            rospy.loginfo(f"New Target: Pos={self.target_pos}, Yaw={self.target_yaw:.2f}")

    def control_step(self, event=None):
        with self.lock:
            if not self._initialized_profiler: return
            curr_pos = list(self.curr_pos); curr_yaw = self.curr_yaw
            target_pos = list(self.target_pos); target_yaw = self.target_yaw
            last_dvl_stamp = self.last_dvl_stamp

        now = rospy.Time.now()
        dt = 1.0 / self.control_rate

        if (now - last_dvl_stamp) > self.dvl_timeout:
            self.publish_rc([self.pwm_neutral]*18)
            self.publish_tf(now)
            return

        # === 1. Profiling & Feed-Forward Selection ===
        
        if self.enable_profile_xy:
            ref_pos_xy, ref_vel_xy = self.xy_profiler.update([target_pos[0], target_pos[1]], dt)
            # [수정] 파라미터 값 사용
            current_k_ff_xy = self.K_ff_xy 
        else:
            ref_pos_xy = [target_pos[0], target_pos[1]]
            ref_vel_xy = [0.0, 0.0]
            current_k_ff_xy = 0.0

        if self.enable_profile_yaw:
            ref_yaw, ref_yaw_vel = self.yaw_profiler.update(target_yaw, dt)
            # [수정] 파라미터 값 사용
            current_k_ff_yaw = self.K_ff_yaw
        else:
            ref_yaw = target_yaw
            ref_yaw_vel = 0.0
            current_k_ff_yaw = 0.0

        # === 2. Control Calculation ===
        
        ex = ref_pos_xy[0] - curr_pos[0]
        ey = ref_pos_xy[1] - curr_pos[1]
        ez = target_pos[2] - curr_pos[2]
        e_yaw = self.wrap_angle(ref_yaw - curr_yaw)

        # P-Term + Feed-Forward Term
        vx_cmd = (self.Kp_x * ex) + (ref_vel_xy[0] * current_k_ff_xy)
        vy_cmd = (self.Kp_y * ey) + (ref_vel_xy[1] * current_k_ff_xy)
        vz_cmd = self.Kp_z * ez
        yaw_rate_cmd = (self.Kp_yaw * e_yaw) + (ref_yaw_vel * current_k_ff_yaw)

        # ... (이후 Limit, Publish 부분은 기존과 완벽히 동일) ...
        vx_cmd = self.clamp(vx_cmd, -self.max_vel_xy, self.max_vel_xy)
        vy_cmd = self.clamp(vy_cmd, -self.max_vel_xy, self.max_vel_xy)
        heave_cmd = self.clamp(-vz_cmd, -self.max_vel_z, self.max_vel_z)
        yaw_cmd = self.clamp(yaw_rate_cmd, -self.max_yaw_rate, self.max_yaw_rate)

        c_yaw = math.cos(-curr_yaw); s_yaw = math.sin(-curr_yaw)
        surge_cmd = vx_cmd * c_yaw - vy_cmd * s_yaw
        sway_cmd  = vx_cmd * s_yaw + vy_cmd * c_yaw

        axis_forward = self.clamp(surge_cmd / self.max_vel_xy, -1.0, 1.0)
        axis_lateral = self.clamp(sway_cmd / self.max_vel_xy, -1.0, 1.0)
        axis_heave   = self.clamp(heave_cmd / self.max_vel_z, -1.0, 1.0)
        axis_yaw     = self.clamp(yaw_cmd / self.max_yaw_rate, -1.0, 1.0)

        pwm_channels = [self.pwm_neutral] * 18
        pwm_channels[self.ch_throttle] = self.scale_axis_to_pwm(axis_heave)
        pwm_channels[self.ch_forward] = self.scale_axis_to_pwm(axis_forward)
        pwm_channels[self.ch_lateral] = self.scale_axis_to_pwm(-axis_lateral)
        pwm_channels[self.ch_yaw] = self.scale_axis_to_pwm(-axis_yaw)
        
        self.publish_rc(pwm_channels)
        self.publish_tf(now)

    def publish_rc(self, channels):
        rc_msg = OverrideRCIn(); rc_msg.channels = channels; self.rc_override_pub.publish(rc_msg)
    def publish_tf(self, stamp):
        transforms = []
        with self.lock: p = self.curr_pos; y = self.curr_yaw
        t1 = TransformStamped(); t1.header.stamp = stamp; t1.header.frame_id = self.odom_frame; t1.child_frame_id = self.auv_link_frame
        t1.transform.translation.x = p[0]; t1.transform.translation.y = p[1]; t1.transform.translation.z = p[2]
        q = tft.quaternion_from_euler(0, 0, y)
        t1.transform.rotation.x = q[0]; t1.transform.rotation.y = q[1]; t1.transform.rotation.z = q[2]; t1.transform.rotation.w = q[3]
        transforms.append(t1)
        t2 = TransformStamped(); t2.header.stamp = stamp; t2.header.frame_id = self.auv_link_frame; t2.child_frame_id = self.dvl_frame
        t2.transform.translation.x = self.dvl_x; t2.transform.translation.y = self.dvl_y; t2.transform.translation.z = self.dvl_z
        q2 = tft.quaternion_from_euler(self.dvl_roll, self.dvl_pitch, self.dvl_yaw)
        t2.transform.rotation.x = q2[0]; t2.transform.rotation.y = q2[1]; t2.transform.rotation.z = q2[2]; t2.transform.rotation.w = q2[3]
        transforms.append(t2)
        self.tf_broadcaster.sendTransform(transforms)

    @staticmethod
    def wrap_angle(angle): return (angle + math.pi) % (2 * math.pi) - math.pi
    @staticmethod
    def clamp(v, min_v, max_v): return max(min_v, min(v, max_v))
    @staticmethod
    def scale_axis_to_pwm(val): return int(max(1200, min(1500 + val * 300, 1800)))