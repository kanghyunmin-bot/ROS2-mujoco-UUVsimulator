#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# mission_5.py (Return to Home and Disarm with Launch-File Configurable Start)

import rospy
import math
import sys
from enum import Enum, auto

import os
import rospkg

# --- Path Setup ---
try:
    pkg_path = rospkg.RosPack().get_path('hit25_auv')
    scripts_dir = os.path.join(pkg_path, 'scripts')
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
except Exception as e:
    pass

from auv_setpoint_sender import AUVSetpointSender
import tf
from tf.transformations import euler_from_quaternion

# --- Message and Service Imports ---
from hit25_auv.msg import MissionStatus
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped # ★ [추가] 이 줄을 추가하세요.

class MissionState(Enum):
    WAIT_FOR_TRIGGER, MOVE_TO_HOME, DISARMING, MISSION_COMPLETE = (auto() for i in range(4))

class Mission5Node:
    # --- Mission Parameters & Constants ---
    HOME_POINT = (1.5, 0.0)
    TARGET_DEPTH = -0.5
    DIST_TOLERANCE = 0.3
    XY_TIMEOUT = 20.0

    def __init__(self, control_frequency: float = 20.0):
        rospy.init_node('mission_5_node', anonymous=True)
        
        # ★★★ 핵심 수정: 'manual_start' 파라미터를 읽어와서 시작 모드를 결정합니다. ★★★
        # rosparam get /mission_5_node/manual_start 로 확인 가능
        # launch 파일에서 <param name="manual_start" type="bool" value="true"/> 로 설정
        self.is_manual_start = rospy.get_param("~manual_start", False)

        if self.is_manual_start:
            rospy.loginfo("[Mission 5] MANUAL START mode activated.")
        else:
            rospy.loginfo("[Mission 5] AUTO START mode activated. Waiting for Mission 4.")

        self.rate = rospy.Rate(control_frequency)
        self.state = MissionState.WAIT_FOR_TRIGGER

        # --- ROS Connections ---
        self.comm = AUVSetpointSender('/auv_setpoint')
        self.tf_listener = tf.TransformListener()
        
        self.mission_pub = rospy.Publisher('/mission_5', MissionStatus, queue_size=10)
        # ★ [추가] 아래 Publisher를 추가하세요.
        self.marker_pub = rospy.Publisher('/mission_5/home_marker', PoseStamped, queue_size=1, latch=True)

        # ★★★ 핵심 수정: 수동 시작 모드가 아닐 때만 Mission 4를 구독합니다. ★★★
        if not self.is_manual_start:
            self.mission_4_sub = rospy.Subscriber('/mission_4', MissionStatus, self._mission_4_callback)

        rospy.loginfo("[Mission 5] Waiting for /mavros/cmd/arming service...")
        try:
            rospy.wait_for_service('/mavros/cmd/arming', timeout=10.0)
            self.arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            rospy.loginfo("[Mission 5] MAVROS arming service connected.")
        except rospy.ROSException as e:
            rospy.logerr(f"[Mission 5] Failed to connect to MAVROS arming service: {e}")
            self.arming_client = None

        # --- Internal State Variables ---
        self.mission_msg = MissionStatus()
        self.mission_msg.status = MissionStatus.READY
        self._current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.TF_SOURCE_FRAME = 'odom'
        self.TF_TARGET_FRAME = 'auv_link'
        self.mission_start_trigger = False
        self.move_start_time = None
        
        rospy.loginfo("[Mission 5] Ready.")

    def _mission_4_callback(self, msg: MissionStatus):
        if msg.status == MissionStatus.COMPLETED and not self.mission_start_trigger:
            rospy.loginfo("[Mission 5] Trigger Received: Mission 4 Completed.")
            self.mission_start_trigger = True
            
    def get_current_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.TF_SOURCE_FRAME, self.TF_TARGET_FRAME, rospy.Time(0))
            (_, _, yaw) = euler_from_quaternion(rot)
            self._current_pose.update({'x': trans[0], 'y': trans[1], 'yaw': yaw})
        except: pass
        return self._current_pose
    # ★ [추가] 아래 함수 전체를 여기에 추가하세요.
    def _publish_home_marker(self):
        """HOME_POINT 위치에 RViz 마커를 발행합니다."""
        marker_msg = PoseStamped()
        marker_msg.header.frame_id = self.TF_SOURCE_FRAME
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.pose.position.x = self.HOME_POINT[0]
        marker_msg.pose.position.y = self.HOME_POINT[1]
        marker_msg.pose.position.z = self.TARGET_DEPTH
        marker_msg.pose.orientation.w = 1.0
        self.marker_pub.publish(marker_msg)
        rospy.loginfo(f"[Mission 5] Home marker published at {self.HOME_POINT}")

    # ( _handle_move_to_home, _handle_disarming 함수는 이전과 동일 )
    def _handle_move_to_home(self, pose):
        tx, ty = self.HOME_POINT
        self.comm.send_setpoint(tx, ty, self.TARGET_DEPTH, 0.0, is_relative=False)
        if self.move_start_time is None: self.move_start_time = rospy.Time.now()
        dist_to_home = math.hypot(pose['x'] - tx, pose['y'] - ty)
        elapsed_time = (rospy.Time.now() - self.move_start_time).to_sec()
        if dist_to_home < self.DIST_TOLERANCE:
            rospy.loginfo(f"[Mission 5] Reached Home Point ({tx}, {ty}).")
            self.state = MissionState.DISARMING
        elif elapsed_time > self.XY_TIMEOUT:
            rospy.logwarn(f"[Mission 5] Timeout while moving to home. Proceeding with disarm.")
            self.state = MissionState.DISARMING

    def _handle_disarming(self):
        rospy.loginfo("[Mission 5] Attempting to disarm vehicle...")
        if self.arming_client is None:
            rospy.logerr("[Mission 5] Arming service is not available. Cannot disarm.")
            self.state = MissionState.MISSION_COMPLETE
            return
        try:
            response = self.arming_client(False)
            if response.success: rospy.loginfo("Vehicle successfully disarmed.")
            else: rospy.logerr("Failed to disarm vehicle, but completing mission anyway.")
        except rospy.ServiceException as e: rospy.logerr(f"Service call failed: {e}")
        self.state = MissionState.MISSION_COMPLETE

    def run(self):
        """메인 루프: 미션의 전체 흐름을 제어합니다."""
        
        # =========================================================
        # [Option 1] Mission 4 완료 시 자동 시작 (활성화됨)
        # =========================================================
        rospy.loginfo("[Mission 5] Waiting for Mission 4 completion trigger...")
        while not rospy.is_shutdown() and not self.mission_start_trigger:
            self.mission_pub.publish(self.mission_msg) # READY 상태 계속 발행
            self.rate.sleep()
        # =========================================================

        # =========================================================
        # [Option 2] Keyboard Input Trigger (Enter) - 주석 처리됨
        # =========================================================
        #rospy.loginfo("[Mission 5] Waiting for Enter key to start manually...")
        #while not rospy.is_shutdown():
        #    self.mission_pub.publish(self.mission_msg) # READY 상태 계속 발행
        #    rospy.logwarn_once("!!! [Mission 5] Press Enter to START Return-to-Home !!!")
        #    try:
        #        if sys.version_info[0] < 3: raw_input()
        #        else: input()
        #        break # Enter가 입력되면 루프 탈출
        #    except (NameError, EOFError):
        #        if rospy.is_shutdown(): return
        #        break
        # =========================================================
        
        if rospy.is_shutdown(): return

        # --- 2. 미션 시작 ---
        rospy.loginfo("[Mission 5] Mission Started! Returning to home.")
        
        # ★ [추가] 이 줄을 추가하여 마커를 발행하게 합니다.
        self._publish_home_marker()
        
        self.mission_msg.status = MissionStatus.RUNNING
        self.state = MissionState.MOVE_TO_HOME        

        # --- 3. 메인 상태 머신 루프 ---
        while not rospy.is_shutdown() and self.state != MissionState.MISSION_COMPLETE:
            self.mission_pub.publish(self.mission_msg) # RUNNING
            pose = self.get_current_pose()
            if self.state == MissionState.MOVE_TO_HOME: self._handle_move_to_home(pose)
            elif self.state == MissionState.DISARMING: self._handle_disarming()
            self.rate.sleep()

        # --- 4. 임무 완료 후 무한 대기 ---
        if self.mission_msg.status != MissionStatus.COMPLETED:
            rospy.loginfo("[Mission 5] Mission Complete.")
            self.mission_msg.status = MissionStatus.COMPLETED
        rospy.loginfo("[Mission 5] Node will now remain active, publishing final status.")
        while not rospy.is_shutdown():
            self.mission_pub.publish(self.mission_msg)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        mission_node = Mission5Node()
        mission_node.run()
    except rospy.ROSInterruptException:
        pass