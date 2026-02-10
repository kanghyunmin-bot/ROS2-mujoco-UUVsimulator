#!/usr/bin/env python
# -*- coding: utf-8 -*-
# gate_mission.py

import rospy
import time
import math
import sys
from enum import Enum, auto
from typing import List, Tuple, Dict, Any, Union

import os
import rospkg
try:
    pkg_path = rospkg.RosPack().get_path('hit25_auv')
    scripts_dir = os.path.join(pkg_path, 'scripts')
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
except:
    pass

import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8

try:
    from hit25_auv.msg import MissionStatus
except ImportError:
    class MissionStatus:
        READY=0; RUNNING=1; COMPLETED=2; status=0

from auv_setpoint_sender import AUVSetpointSender
from vision_converter import VisionConverter

class GateState(Enum):
    IDLE = auto()
    SEARCH_LATERAL = auto()
    ALIGN_CENTER = auto()
    PASS_THROUGH = auto()
    COMPLETED = auto()

class GateMissionNode:
    # --- Constants ---
    
    SEARCH_STEP_Y = -0.2       
    SEARCH_INTERVAL = 2.0      
    
    TIMEOUT_SEARCH = 10.0      
    TIMEOUT_ALIGN  = 4.0      
    TIMEOUT_PASS   = 10.0      

    # [하드코딩 위치]
    HARDCODED_LEFT_PILLAR  = (6.3, 0.6)   
    HARDCODED_RIGHT_PILLAR = (6.3, -0.6)  
    
    # [거리 설정 변경]
    # 기존 1.4m 이동 = (게이트까지 0.9m) + (게이트 뒤 0.5m)라고 가정
    VISION_DETECT_DIST_X = 0.9   # 비전 인식 시 현재 위치에서 게이트까지의 추정 거리
    PASS_OFFSET_FROM_GATE = 0.2  # [요청사항] 게이트 위치로부터 더 이동할 거리 (m)

    ESTIMATED_PILLAR_OFFSET_Y = 0.5 

    MAX_X_LIMIT = 20.0
    MAX_Y_LIMIT = 0.5 
    
    GATE_OBJ_NAME = "Gate"     
    PILLAR_CONFIDENCE_THRES = 0.6
    
    TARGET_DEPTH = -0.5

    def __init__(self, control_frequency: float = 10.0):
        rospy.init_node('gate_mission_node', anonymous=True)
        rospy.loginfo("[GateMission] Initializing...")

        self.rate = rospy.Rate(control_frequency)
        self.state = GateState.IDLE
        
        self.comm = AUVSetpointSender('/auv_setpoint')
        self.vision = VisionConverter('/hit25/vision')
        
        self.mission1_sub = rospy.Subscriber('/mission_1', MissionStatus, self.mission1_callback)
        self.mission1_finished = False

        self.mission_status_pub = rospy.Publisher('/mission_2', MissionStatus, queue_size=10)
        self.status_msg = MissionStatus()
        self.status_msg.status = MissionStatus.READY
        
        self.gate_pose_pub = rospy.Publisher('/gate_obstacle_pose', PoseStamped, queue_size=10)

        self.tf_listener = tf.TransformListener()
        self.TF_SOURCE_FRAME = 'odom' 
        self.TF_TARGET_FRAME = 'auv_link' 
        
        self._current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        
        self.last_move_time = rospy.Time(0)
        self.search_start_y = 0.0
        
        self.left_pillar_last_y = None   
        self.right_pillar_first_y = None 
        
        self.found_left = False
        self.found_right = False
        
        # [중요] 게이트 좌표 저장 변수 (Y: 중심, X: 게이트 자체의 X좌표)
        self.target_gate_y = None 
        self.target_gate_x = None # 게이트의 절대 X 위치

        self.final_pass_target_x = None
        self.action_start_time = None
        
        rospy.loginfo("[GateMission] Ready.")

    def mission1_callback(self, msg):
        if msg.status == MissionStatus.COMPLETED:
            if not self.mission1_finished:
                rospy.loginfo("[GateMission] Detected Mission 1 COMPLETE signal.")
                self.mission1_finished = True

    def get_current_pose(self) -> Dict[str, float]:
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.TF_SOURCE_FRAME, self.TF_TARGET_FRAME, rospy.Time(0)
            )
            (_, _, yaw) = euler_from_quaternion(rot)
            self._current_pose['x'] = trans[0]
            self._current_pose['y'] = trans[1]
            self._current_pose['yaw'] = yaw
            return self._current_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return self._current_pose

    def _check_safety_limits(self, x, y):
        if abs(x) > self.MAX_X_LIMIT or abs(y) > self.MAX_Y_LIMIT:
            rospy.logwarn(f"[SAFETY] Limit Reached! X:{x:.2f}, Y:{y:.2f}")
            return False
        return True

    def _publish_gate_pose(self, x, y):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.TF_SOURCE_FRAME 
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = self.TARGET_DEPTH
        pose_msg.pose.orientation.w = 1.0
        self.gate_pose_pub.publish(pose_msg)

    def _detect_gate_pillar(self, current_pose):
        objects = self.vision.find_objects_by_name(self.GATE_OBJ_NAME)
        valid_objs = [obj for obj in objects if obj.confidence > self.PILLAR_CONFIDENCE_THRES]
        
        if not valid_objs:
            return

        target_obj = max(valid_objs, key=lambda o: o.confidence)
        screen_width = self.vision._latest_data.get('screen_width', 640)
        center_x = target_obj.center_x
        offset_from_center = center_x - (screen_width / 2.0)
        current_y_abs = current_pose['y']
        
        rospy.loginfo(f"[VISION] Gate Pillar detected at AUV_Y={current_y_abs:.2f}, Offset={offset_from_center:.1f}")

        if not self.found_right:
            if self.found_left and (self.left_pillar_last_y is not None):
                 if abs(self.left_pillar_last_y - current_y_abs) > 0.8:
                     self.found_right = True
                     self.right_pillar_first_y = current_y_abs
                     rospy.loginfo(f"[LOGIC] Right Pillar Start Found: {current_y_abs:.2f}")
                     return

            self.left_pillar_last_y = current_y_abs
            self.found_left = True

    def _use_hardcoded_and_align(self):
        left_pt = self.HARDCODED_LEFT_PILLAR
        right_pt = self.HARDCODED_RIGHT_PILLAR
        
        calc_center_y = (left_pt[1] + right_pt[1]) / 2.0
        self.target_gate_y = calc_center_y
        
        # [수정] 하드코딩된 X좌표를 게이트 위치로 설정 (3.5m)
        self.target_gate_x = left_pt[0] 
        
        self._publish_gate_pose(left_pt[0], left_pt[1])
        rospy.sleep(0.05)
        self._publish_gate_pose(right_pt[0], right_pt[1])
        
        rospy.logwarn(f"[TIMEOUT/SAFETY] Using Hardcoded Gate X:{self.target_gate_x:.2f}, Y:{self.target_gate_y:.2f}")
        
        self.state = GateState.ALIGN_CENTER
        self.action_start_time = None 

    def run(self):
        rospy.loginfo("[GateMission] Waiting for Trigger...")
        
        # =============================
        # [Trigger Mode 1] Automatic Trigger (Mission 1 Completed)
        # =============================

        while not rospy.is_shutdown():
            if self.mission1_finished:
                rospy.loginfo("[GateMission] Triggered by Mission 1 (COMPLETED)!")
                break
            
            # 대기 상태 유지 및 상태 발행
            self.mission_status_pub.publish(self.status_msg)
            self.rate.sleep()

        # =============================
        # [Trigger Mode 2] Manual Trigger (Enter Key) - 필요 시 주석 해제하여 사용
        # =============================
        
        # while not rospy.is_shutdown():
        #     rospy.logwarn_once("!!! [Press Enter to Start Gate Mission] !!!")
        #     try:
        #         if sys.version_info[0] < 3: raw_input()
        #         else: input()
        #         break
        #     except:
        #         break
        #     self.rate.sleep()
        
        # =============================

        rospy.loginfo("[GateMission] Started!")
        self.status_msg.status = MissionStatus.RUNNING
        self.mission_status_pub.publish(self.status_msg)
        
        start_pose = self.get_current_pose()
        self.search_start_y = start_pose['y']
        
        self.state = GateState.SEARCH_LATERAL
        self.last_move_time = rospy.Time.now()
        
        # 변수 초기화
        self.target_gate_y = None
        self.target_gate_x = None
        self.final_pass_target_x = None
        self.action_start_time = None 

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose()
            self.mission_status_pub.publish(self.status_msg)

            # --- A. 측면 탐색 ---
            if self.state == GateState.SEARCH_LATERAL:
                if self.action_start_time is None:
                    self.action_start_time = rospy.Time.now()

                self._detect_gate_pillar(current_pose)
                
                if self.found_left and self.found_right:
                    rospy.loginfo("[SEARCH] Both pillars detected! Calculating Center.")
                    center_y = (self.left_pillar_last_y + self.right_pillar_first_y) / 2.0
                    self.target_gate_y = center_y
                    
                    # [수정] 비전 인식 시 게이트 절대 X 위치 추정 (현재위치 + 0.9m)
                    self.target_gate_x = current_pose['x'] + self.VISION_DETECT_DIST_X
                    
                    est_left_y = center_y + self.ESTIMATED_PILLAR_OFFSET_Y
                    est_right_y = center_y - self.ESTIMATED_PILLAR_OFFSET_Y
                    
                    rospy.loginfo(f"[RESULT] Center Y: {center_y:.2f}, Est Gate X: {self.target_gate_x:.2f}")
                    
                    self._publish_gate_pose(self.target_gate_x, est_left_y)
                    rospy.sleep(0.05)
                    self._publish_gate_pose(self.target_gate_x, est_right_y)
                    
                    self.state = GateState.ALIGN_CENTER
                    self.action_start_time = None
                    continue

                elapsed = (rospy.Time.now() - self.action_start_time).to_sec()
                if elapsed > self.TIMEOUT_SEARCH:
                    rospy.logwarn(f"[TIMEOUT] Search timed out ({elapsed:.1f}s). Force ALIGN.")
                    self._use_hardcoded_and_align()
                    continue

                if (rospy.Time.now() - self.last_move_time).to_sec() > self.SEARCH_INTERVAL:
                    next_y = current_pose['y'] + self.SEARCH_STEP_Y
                    if not self._check_safety_limits(current_pose['x'], next_y):
                        rospy.logwarn("[SEARCH] Safety limit reached.")
                        self._use_hardcoded_and_align()
                    else:
                        rospy.loginfo(f"[SEARCH] Moving Lateral to Y={next_y:.2f}")
                        self.comm.send_setpoint(
                            x=current_pose['x'], 
                            y=next_y, 
                            z=self.TARGET_DEPTH, 
                            yaw=0.0, 
                            is_relative=False
                        )
                        self.last_move_time = rospy.Time.now()
            
            # --- B. 중심 정렬 ---
            elif self.state == GateState.ALIGN_CENTER:
                if self.action_start_time is None:
                    self.action_start_time = rospy.Time.now()

                # 안전장치: 혹시라도 좌표가 없으면 하드코딩 사용
                if self.target_gate_y is None or self.target_gate_x is None:
                    self.target_gate_y = (self.HARDCODED_LEFT_PILLAR[1] + self.HARDCODED_RIGHT_PILLAR[1]) / 2.0
                    self.target_gate_x = self.HARDCODED_LEFT_PILLAR[0]

                target_y = self.target_gate_y
                
                # X 유지, Y 이동
                self.comm.send_setpoint(
                    x=current_pose['x'], 
                    y=target_y, 
                    z=self.TARGET_DEPTH, 
                    yaw=0.0, 
                    is_relative=False
                )
                
                dist_y = abs(current_pose['y'] - target_y)
                
                if dist_y < 0.1: 
                    rospy.loginfo(f"[ALIGN] Aligned. Approaching...")
                    time.sleep(1.0)
                    self.final_pass_target_x = None 
                    self.state = GateState.PASS_THROUGH
                    self.action_start_time = None
                    continue

                elapsed = (rospy.Time.now() - self.action_start_time).to_sec()
                if elapsed > self.TIMEOUT_ALIGN:
                    rospy.logwarn(f"[TIMEOUT] Align timed out ({elapsed:.1f}s). Force PASS.")
                    self.final_pass_target_x = None
                    self.state = GateState.PASS_THROUGH
                    self.action_start_time = None
            
            # --- C. 통과 (Pass Through) ---
            elif self.state == GateState.PASS_THROUGH:
                if self.action_start_time is None:
                    self.action_start_time = rospy.Time.now()

                # [핵심 수정] 목표 지점 = 게이트 X 위치 + 0.5m
                if self.final_pass_target_x is None:
                    if self.target_gate_x is None: # 만약의 경우 대비
                        self.target_gate_x = current_pose['x'] + self.VISION_DETECT_DIST_X

                    self.final_pass_target_x = self.target_gate_x + self.PASS_OFFSET_FROM_GATE
                    
                    self.comm.send_setpoint(
                        x=self.final_pass_target_x, 
                        y=self.target_gate_y, 
                        z=self.TARGET_DEPTH, 
                        yaw=0.0, 
                        is_relative=False
                    )
                    rospy.loginfo(f"[PASS] Setpoint sent: GateX={self.target_gate_x:.2f} -> TargetX={self.final_pass_target_x:.2f}")

                dist_x = abs(current_pose['x'] - self.final_pass_target_x)

                if dist_x < 0.2:
                    rospy.loginfo("[PASS] Gate Passed Successfully.")
                    self.state = GateState.COMPLETED
                    self.action_start_time = None
                    continue

                elapsed = (rospy.Time.now() - self.action_start_time).to_sec()
                if elapsed > self.TIMEOUT_PASS:
                    rospy.logwarn(f"[TIMEOUT] Pass timed out ({elapsed:.1f}s). Force COMPLETE.")
                    self.state = GateState.COMPLETED
                    self.action_start_time = None
            
            # --- D. 완료 ---
            elif self.state == GateState.COMPLETED:
                if self.status_msg.status != MissionStatus.COMPLETED:
                    self.status_msg.status = MissionStatus.COMPLETED
                    self.mission_status_pub.publish(self.status_msg)
                    rospy.loginfo("[GateMission] Mission 2 Completed. Stopping Setpoints.")
                
                # [수정] 완료 시 더 이상 setpoint 명령을 보내지 않음 (주석 처리)
                # self.comm.send_setpoint(
                #     x=current_pose['x'], 
                #     y=current_pose['y'], 
                #     z=self.TARGET_DEPTH, 
                #     yaw=0.0, 
                #     is_relative=False
                # )
                
            self.rate.sleep()

if __name__ == "__main__":
    try:
        node = GateMissionNode(control_frequency=10.0)
        node.run()
    except rospy.ROSInterruptException:
        pass