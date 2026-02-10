#!/usr/bin/env python
# -*- coding: utf-8 -*-
# hydrophone_mission.py (Relentless Pursuer with Trigger Toggle & Status Publishing)

import rospy
import math
import sys
import statistics
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
from hit25_auv.msg import MissionStatus
from hit25_interpreter.msg import HydrophoneData
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool  # <--- [추가] 이 줄을 추가하세요

class MissionState(Enum):
    NONE = auto()
    WAIT_FOR_TRIGGER = auto()
    MOVE_TO_SEARCH_POINT = auto()
    SCAN_YAW = auto()
    ALIGN_BEST_ANGLE = auto()
    MOVE_FORWARD = auto()
    STATION_KEEPING = auto()
    MOVE_TO_FAILSAFE_POINT = auto()
    # Phase 2
    PRECISION_SEARCH_START = auto()
    PRECISION_SEARCH_LATERAL = auto()
    PRECISION_SEARCH_FORWARD = auto()
    MOVE_TO_FINAL_TARGET = auto()

class HydrophoneMissionNode:
    # --- Constants (변경 없음) ---
    SEARCH_POINT = (10.0, 1.0)
    FAILSAFE_POINT = (13.0, 1.0)
    TARGET_DEPTH = -0.5
    SCAN_STEP_DEG = 30
    DIST_TOLERANCE = 0.1
    YAW_TOLERANCE_RAD = math.radians(7.0)
    STABILIZE_TIME = 1.5
    YAW_TIMEOUT = 4.0
    XY_TIMEOUT = 10.0
    FORWARD_STEP_DIST = 0.5
    SCORE_DROP_THRESHOLD = 15.0 
    SCORE_RISE_THRESHOLD = 20.0
    FREQ_STD_THRESHOLD = 25.0
    PRECISION_SEARCH_DISTANCE = 0.5
    TARGET_FREQUENCY = 21164
    FREQUENCY_TOLERANCE = 1000.0
    MOVING_AVG_SAMPLES = 10
    MAX_RETRY_COUNT = 2

    # >>> [추가] 완료 조건 상수 <<<
    BOUND_X_MIN = -2.0
    BOUND_X_MAX = 2.0
    BOUND_Y_MIN = 0.0
    BOUND_Y_MAX = 4.0
    MAX_SCAN_CYCLES = 5  # (초기 1회 + 추가 4회 = 총 5회)

    def __init__(self, control_frequency: float = 20.0):
        rospy.init_node('hydrophone_mission_node', anonymous=True)
        rospy.loginfo("[Hydrophone] Initializing Mission 4 Node (Relentless Pursuer Mode).")

        self.rate = rospy.Rate(control_frequency)
        self.state = MissionState.WAIT_FOR_TRIGGER

        # --- ROS Connections ---
        self.comm = AUVSetpointSender('/auv_setpoint')
        self.tf_listener = tf.TransformListener()
        self.hydro_sub = rospy.Subscriber('/hydrophone/data', HydrophoneData, self._hydro_callback)
        self.mission_pub = rospy.Publisher('/mission_4', MissionStatus, queue_size=10)
        self.hydro_pose_pub = rospy.Publisher('/hydrophone_obstacle_pose', PoseStamped, queue_size=10)

        # ▼▼▼ [추가] 수동 종료 트리거 구독자 ▼▼▼
        self.stop_sub = rospy.Subscriber('/mission_4/stop_trigger', Bool, self.stop_trigger_callback)      
        ### <<< 추가됨: Mission 3 트리거 구독자
        self.mission_3_sub = rospy.Subscriber('/mission_3', MissionStatus, self.mission_3_callback)
        self.mission_start_trigger = False

        self.mission_msg = MissionStatus()
        self.mission_msg.status = MissionStatus.READY # 초기 상태를 READY로 명시
        self._current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.TF_SOURCE_FRAME = 'odom'
        self.TF_TARGET_FRAME = 'auv_link'
        self.latest_freq, self.latest_db, self.last_msg_time = 0.0, 0.0, rospy.Time(0)
        self.last_sent_setpoint, self.last_sent_time = None, rospy.Time(0)
        self.scan_angles_rad = []
        self._generate_scan_angles(is_wide_scan=True)
        self.current_scan_idx, self.scan_data, self.best_yaw, self.prev_best_score = 0, [], 0.0, -9999.0
        self.scan_iteration = 0
        self.target_forward_pose = {'x': 0, 'y': 0}
        self.stabilize_start_time, self.move_start_time = None, None
        self.detection_failure_count = 0
        
        self.precision_search_center = None
        self.precision_sub_state = None
        self.precision_search_data = {'lateral': [], 'forward': []}
        self.final_target_pose = None
        
        rospy.loginfo("[Hydrophone] Ready.")
    
    ### <<< 추가됨: Mission 3 콜백 함수
    def mission_3_callback(self, msg: MissionStatus):
        if msg.status == MissionStatus.COMPLETED and not self.mission_start_trigger:
            rospy.loginfo("[Hydrophone] Trigger Received: Mission 3 Completed.")
            self.mission_start_trigger = True

# ▼▼▼ [추가] 수동 종료 콜백 함수 ▼▼▼
    def stop_trigger_callback(self, msg: Bool):
        if msg.data:
            rospy.logwarn("!!! [Hydrophone] MANUAL STOP TRIGGER RECEIVED !!!")
            # 상태를 STATION_KEEPING으로 바꾸면 메인 루프가 종료됨
            self.state = MissionState.STATION_KEEPING
            self.mission_msg.status = MissionStatus.COMPLETED

    # ... (이하 모든 핸들러 및 헬퍼 함수는 원래 주셨던 코드와 100% 동일)
    def _hydro_callback(self, msg):
        self.latest_freq, self.latest_db, self.last_msg_time = msg.frequency, msg.decibels, rospy.Time.now()

    def _generate_scan_angles(self, is_wide_scan, center_yaw=0.0):
        self.scan_angles_rad = []
        if is_wide_scan:
            rospy.loginfo("[Hydrophone] Generating WIDE scan angles (360 deg).")
            start_deg, end_deg = -180, 180
        else:
            center_deg = math.degrees(self.wrap_angle(center_yaw))
            rospy.loginfo(f"[Hydrophone] Generating NARROW scan angles (+-90 deg around {center_deg:.1f} deg).")
            start_deg, end_deg = int(center_deg - 90), int(center_deg + 90)
        for deg in range(start_deg, end_deg, self.SCAN_STEP_DEG):
            self.scan_angles_rad.append(self.wrap_angle(math.radians(deg)))

    def get_current_pose(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.TF_SOURCE_FRAME, self.TF_TARGET_FRAME, rospy.Time(0))
            (_, _, yaw) = euler_from_quaternion(rot)
            self._current_pose.update({'x': trans[0], 'y': trans[1], 'yaw': yaw})
        except: pass
        return self._current_pose

    def _send_absolute_setpoint(self, x, y, z, yaw):
        current = (x, y, z, yaw)
        now = rospy.Time.now()
        if self.last_sent_setpoint != current or (now - self.last_sent_time).to_sec() > 1.0:
            self.comm.send_setpoint(x, y, z, yaw, is_relative=False)
            self.last_sent_setpoint, self.last_sent_time = current, now

    def _measure_signal_quality(self) -> dict:
        db_samples, freq_samples = [], []
        for _ in range(self.MOVING_AVG_SAMPLES):
            curr_db, curr_freq = (self.latest_db, self.latest_freq) if (rospy.Time.now() - self.last_msg_time).to_sec() <= 1.0 else (0.0, 0.0)
            db_samples.append(curr_db); freq_samples.append(curr_freq); rospy.sleep(0.05)
        
        if not db_samples: return {'score': -9999.0, 'freq_std': 999.0}
        avg_db = statistics.mean(db_samples)
        if avg_db < -65.0: return {'score': -9999.0, 'freq_std': 999.0}
        
        avg_freq = statistics.mean(freq_samples)
        freq_std = statistics.stdev(freq_samples) if len(freq_samples) > 1 else 0.0
        if abs(avg_freq - self.TARGET_FREQUENCY) > self.FREQUENCY_TOLERANCE: return {'score': -9999.0, 'freq_std': freq_std}
        
        penalty = freq_std
        return {'score': avg_db - penalty, 'freq_std': freq_std}

    def _handle_move_to_search_point(self, pose):
        tx, ty = self.SEARCH_POINT; self._send_absolute_setpoint(tx, ty, self.TARGET_DEPTH, 0.0)
        if self.move_start_time is None: self.move_start_time = rospy.Time.now()
        dist = math.hypot(pose['x'] - tx, pose['y'] - ty)
        if dist < self.DIST_TOLERANCE or (rospy.Time.now() - self.move_start_time).to_sec() > self.XY_TIMEOUT:
            self.state, self.current_scan_idx, self.scan_data = MissionState.SCAN_YAW, 0, []
            self.stabilize_start_time, self.move_start_time = None, None

    def _handle_scan_yaw(self, pose):
        if self.current_scan_idx < len(self.scan_angles_rad):
            target_yaw = self.scan_angles_rad[self.current_scan_idx]
            self._send_absolute_setpoint(pose['x'], pose['y'], self.TARGET_DEPTH, target_yaw)
            if self.move_start_time is None: self.move_start_time = rospy.Time.now()
            if abs(self.wrap_angle(target_yaw - pose['yaw'])) < self.YAW_TOLERANCE_RAD or (rospy.Time.now() - self.move_start_time).to_sec() > self.YAW_TIMEOUT:
                if self.stabilize_start_time is None: self.stabilize_start_time = rospy.Time.now()
                if (rospy.Time.now() - self.stabilize_start_time).to_sec() >= self.STABILIZE_TIME:
                    quality = self._measure_signal_quality()
                    self.scan_data.append({'yaw': target_yaw, 'score': quality['score']})
                    if quality['score'] > -9000.0:
                        # ★ 여기부터 변경 ★
                        offset = self.FORWARD_STEP_DIST  # 0.5m
                        mx = pose['x'] + offset * math.cos(target_yaw)
                        my = pose['y'] + offset * math.sin(target_yaw)
                        self._publish_hydrophone_pose(mx, my)
                        # ★ 여기까지 변경 ★
                    self.current_scan_idx += 1; self.stabilize_start_time, self.move_start_time = None, None
            else: self.stabilize_start_time = None; return
        else:
            sorted_candidates = sorted([d for d in self.scan_data if d['score'] > -9000.0], 
                                       key=lambda x: x['score'], reverse=True)

            if not sorted_candidates:
                self.detection_failure_count += 1
                rospy.logwarn(f"Scan failed. No valid signal. Failures: {self.detection_failure_count}")
                if self.detection_failure_count > self.MAX_RETRY_COUNT:
                    rospy.logerr("Relentless pursuit failed. No signal found after multiple rescans. Moving to failsafe point.")
                    self.state = MissionState.MOVE_TO_FAILSAFE_POINT
                    self.move_start_time = None
                    return
                rospy.loginfo("Retrying with a 360-degree scan at the current location.")
                self.current_scan_idx, self.scan_data = 0, []
                self._generate_scan_angles(is_wide_scan=True) 
                return

            self.detection_failure_count = 0
            best_result = sorted_candidates[0]
            self.best_yaw, score = best_result['yaw'], best_result['score']
            
            if self.scan_iteration > 0 and score < (self.prev_best_score - self.SCORE_DROP_THRESHOLD):
                rospy.logwarn(f"Signal drop! Score: {self.prev_best_score:.2f} -> {score:.2f}. Initiating 360 rescan.")
                self.current_scan_idx, self.scan_data = 0, []; self._generate_scan_angles(is_wide_scan=True)
                self.prev_best_score = -9999.0; return            

            self.prev_best_score = score
            self.state, self.stabilize_start_time, self.move_start_time = MissionState.ALIGN_BEST_ANGLE, None, None

    def _handle_align_best_angle(self, pose):
        self._send_absolute_setpoint(pose['x'], pose['y'], self.TARGET_DEPTH, self.best_yaw)
        if self.move_start_time is None: self.move_start_time = rospy.Time.now()
        
        is_aligned = abs(self.wrap_angle(self.best_yaw - pose['yaw'])) < self.YAW_TOLERANCE_RAD
        is_timeout = (rospy.Time.now() - self.move_start_time).to_sec() > self.YAW_TIMEOUT

        if is_aligned or is_timeout:
            if not self.stabilize_start_time: self.stabilize_start_time = rospy.Time.now()
            if (rospy.Time.now() - self.stabilize_start_time).to_sec() > 1.0:
                quality = self._measure_signal_quality()
                score_increase = quality['score'] - self.prev_best_score
                rospy.loginfo(f"Aligned. Quality check: Score Rise={score_increase:.2f}, Freq_std={quality['freq_std']:.2f}")

                if self.scan_iteration > 0 and score_increase > self.SCORE_RISE_THRESHOLD and quality['freq_std'] < self.FREQ_STD_THRESHOLD:
                    rospy.logwarn("!!! High score rise and signal quality detected! Switching to Phase 2. !!!")
                    self.state = MissionState.PRECISION_SEARCH_START
                    self.move_start_time = None; self.stabilize_start_time = None; return

                self.target_forward_pose = {'x': pose['x'] + self.FORWARD_STEP_DIST * math.cos(self.best_yaw),
                                            'y': pose['y'] + self.FORWARD_STEP_DIST * math.sin(self.best_yaw)}
                self.state, self.stabilize_start_time, self.move_start_time = MissionState.MOVE_FORWARD, None, None
        else:
            self.stabilize_start_time = None

    def _handle_move_forward(self, pose):
            tx, ty = self.target_forward_pose['x'], self.target_forward_pose['y']
            self._send_absolute_setpoint(tx, ty, self.TARGET_DEPTH, self.best_yaw)
            
            if self.move_start_time is None: self.move_start_time = rospy.Time.now()
            
            dist = math.hypot(pose['x'] - tx, pose['y'] - ty)
            
            # 목표 지점 도달 또는 시간 초과 시 다음 스캔 준비
            if dist < self.DIST_TOLERANCE or (rospy.Time.now() - self.move_start_time).to_sec() > self.XY_TIMEOUT:
                self.scan_iteration += 1
                
                # ▼▼▼ [변경] 기존의 종료 조건(범위 이탈, 횟수 제한) 코드를 모두 삭제함 ▼▼▼
                
                # 바로 다음 스캔 준비
                self.current_scan_idx, self.scan_data = 0, []
                self._generate_scan_angles(is_wide_scan=False, center_yaw=self.best_yaw)
                self.state, self.stabilize_start_time, self.move_start_time = MissionState.SCAN_YAW, None, None
            
    def _handle_precision_search_start(self, pose):
        rospy.loginfo("--- Starting Precision Search Phase ---")
        self.precision_search_center = pose.copy()
        self.precision_search_data = {'lateral': [], 'forward': []}
        self.precision_sub_state = 'move_left'
        self.state = MissionState.PRECISION_SEARCH_LATERAL
        self.move_start_time = None

    def _handle_precision_search_lateral(self, pose):
        center_x, center_y, center_yaw = self.precision_search_center['x'], self.precision_search_center['y'], self.precision_search_center['yaw']
        dist = self.PRECISION_SEARCH_DISTANCE
        left_yaw = self.wrap_angle(center_yaw + math.pi/2)
        target_pos = {'x': 0, 'y': 0}
        if self.precision_sub_state == 'move_left':
            target_pos = {'x': center_x + dist * math.cos(left_yaw), 'y': center_y + dist * math.sin(left_yaw)}
        elif self.precision_sub_state == 'move_right':
            target_pos = {'x': center_x - dist * math.cos(left_yaw), 'y': center_y - dist * math.sin(left_yaw)}
        elif self.precision_sub_state in ['return_to_center_1', 'return_to_center_2']:
            target_pos = {'x': center_x, 'y': center_y}

        self._send_absolute_setpoint(target_pos['x'], target_pos['y'], self.TARGET_DEPTH, center_yaw)
        quality = self._measure_signal_quality()
        self.precision_search_data['lateral'].append({'x': pose['x'], 'y': pose['y'], 'score': quality['score']})
        
        if self.move_start_time is None: self.move_start_time = rospy.Time.now()
        
        dist_to_target = math.hypot(pose['x'] - target_pos['x'], pose['y'] - target_pos['y'])
        if dist_to_target < self.DIST_TOLERANCE or (rospy.Time.now() - self.move_start_time).to_sec() > self.XY_TIMEOUT:
            self.move_start_time = None
            if self.precision_sub_state == 'move_left': self.precision_sub_state = 'return_to_center_1'
            elif self.precision_sub_state == 'return_to_center_1': self.precision_sub_state = 'move_right'
            elif self.precision_sub_state == 'move_right': self.precision_sub_state = 'return_to_center_2'
            elif self.precision_sub_state == 'return_to_center_2':
                self.state = MissionState.PRECISION_SEARCH_FORWARD; self.precision_sub_state = 'move_forward'
    
    def _handle_precision_search_forward(self, pose):
        if not self.precision_search_data['lateral']: best_lat_pos = self.precision_search_center
        else:
            best_lat_point = max(self.precision_search_data['lateral'], key=lambda p: p['score'])
            best_lat_pos = {'x': best_lat_point['x'], 'y': best_lat_point['y'], 'yaw': self.precision_search_center['yaw']}
        center_x, center_y, center_yaw = best_lat_pos['x'], best_lat_pos['y'], best_lat_pos['yaw']
        dist = self.PRECISION_SEARCH_DISTANCE
        target_pos = {'x': 0, 'y': 0}
        if self.precision_sub_state == 'move_forward':
            target_pos = {'x': center_x + dist * math.cos(center_yaw), 'y': center_y + dist * math.sin(center_yaw)}
        elif self.precision_sub_state == 'move_backward':
            target_pos = {'x': center_x - dist * math.cos(center_yaw), 'y': center_y - dist * math.sin(center_yaw)}
        elif self.precision_sub_state in ['return_to_center_3', 'return_to_center_4']:
            target_pos = {'x': center_x, 'y': center_y}
        self._send_absolute_setpoint(target_pos['x'], target_pos['y'], self.TARGET_DEPTH, center_yaw)
        quality = self._measure_signal_quality()
        self.precision_search_data['forward'].append({'x': pose['x'], 'y': pose['y'], 'score': quality['score']})
        if self.move_start_time is None: self.move_start_time = rospy.Time.now()
        dist_to_target = math.hypot(pose['x'] - target_pos['x'], pose['y'] - target_pos['y'])
        if dist_to_target < self.DIST_TOLERANCE or (rospy.Time.now() - self.move_start_time).to_sec() > self.XY_TIMEOUT:
            self.move_start_time = None
            if self.precision_sub_state == 'move_forward': self.precision_sub_state = 'return_to_center_3'
            elif self.precision_sub_state == 'return_to_center_3': self.precision_sub_state = 'move_backward'
            elif self.precision_sub_state == 'move_backward': self.precision_sub_state = 'return_to_center_4'
            elif self.precision_sub_state == 'return_to_center_4':
                if not self.precision_search_data['forward']: self.final_target_pose = best_lat_pos
                else:
                    final_point = max(self.precision_search_data['forward'], key=lambda p: p['score'])
                    self.final_target_pose = {'x': final_point['x'], 'y': final_point['y']}
                rospy.loginfo(f"Precision search complete. Final target: {self.final_target_pose}")
                self.state = MissionState.MOVE_TO_FINAL_TARGET

    def _handle_move_to_final_target(self, pose):
        if not self.final_target_pose: self.state = MissionState.STATION_KEEPING; return
        tx, ty = self.final_target_pose['x'], self.final_target_pose['y']
        self._send_absolute_setpoint(tx, ty, self.TARGET_DEPTH, pose['yaw'])
        if math.hypot(pose['x']-tx, pose['y']-ty) < self.DIST_TOLERANCE:
            self.state = MissionState.STATION_KEEPING

    def _handle_station_keeping(self, pose):
        safe_pose = self.final_target_pose if self.final_target_pose else pose
        self._send_absolute_setpoint(safe_pose['x'], safe_pose['y'], self.TARGET_DEPTH, pose['yaw'])
        if self.mission_msg.status != MissionStatus.COMPLETED:
            self.mission_msg.status = MissionStatus.COMPLETED
            rospy.loginfo("[Hydrophone] Mission Completed or Stopped. Station Keeping.")

    def _handle_move_to_failsafe_point(self, pose):
        tx, ty = self.FAILSAFE_POINT
        self._send_absolute_setpoint(tx, ty, self.TARGET_DEPTH, 0.0)
        if self.move_start_time is None: self.move_start_time = rospy.Time.now()
        if math.hypot(pose['x']-tx, pose['y']-ty) < self.DIST_TOLERANCE or (rospy.Time.now() - self.move_start_time).to_sec() > self.XY_TIMEOUT:
            self.state = MissionState.STATION_KEEPING
    
    def _publish_hydrophone_pose(self, x, y):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.TF_SOURCE_FRAME
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.pose.position.x = x; pose_msg.pose.position.y = y; pose_msg.pose.position.z = self.TARGET_DEPTH
        pose_msg.pose.orientation.w = 1.0
        self.hydro_pose_pub.publish(pose_msg)
    
    def run(self):
        ### <<< 변경됨: 트리거 토글 기능
        # =========================================================
        # [Option 1] ROS Topic Trigger (/mission_3 == 2) - 활성화됨
        # =========================================================
        rospy.loginfo("[Hydrophone] Waiting for /mission_3 completion Trigger...")
        while not rospy.is_shutdown():
            self.mission_pub.publish(self.mission_msg) # READY 상태 계속 발행
            if hasattr(self, 'mission_start_trigger') and self.mission_start_trigger:
                rospy.loginfo("[Hydrophone] Mission 4 START Triggered!")
                break
            self.rate.sleep()
        # =========================================================

        # =========================================================
        # [Option 2] Keyboard Input Trigger (Enter) - 주석 처리됨
        # =========================================================
        # rospy.loginfo("[Hydrophone] Waiting for Enter input...")
        # while not rospy.is_shutdown():
        #     self.mission_pub.publish(self.mission_msg) # READY 상태 계속 발행
        #     rospy.logwarn_once("!!! [Mission 4] Press Enter to Start (Hydrophone Search) !!!")
        #     try:
        #         if sys.version_info[0] < 3: raw_input()
        #         else: input()
        #         break
        #     except (NameError, EOFError): break
        # =========================================================
        
        rospy.loginfo("[Hydrophone] Mission 4 Started!")
        self.mission_msg.status = MissionStatus.RUNNING # 상태를 RUNNING으로 전환
        self.state = MissionState.MOVE_TO_SEARCH_POINT
        
        while not rospy.is_shutdown() and self.state != MissionState.STATION_KEEPING:
            pose = self.get_current_pose()
            self.mission_pub.publish(self.mission_msg) # RUNNING 상태 계속 발행
            
            if self.state == MissionState.MOVE_TO_SEARCH_POINT: self._handle_move_to_search_point(pose)
            elif self.state == MissionState.SCAN_YAW: self._handle_scan_yaw(pose)
            elif self.state == MissionState.ALIGN_BEST_ANGLE: self._handle_align_best_angle(pose)
            elif self.state == MissionState.MOVE_FORWARD: self._handle_move_forward(pose)
            elif self.state == MissionState.MOVE_TO_FAILSAFE_POINT: self._handle_move_to_failsafe_point(pose)
            elif self.state == MissionState.PRECISION_SEARCH_START: self._handle_precision_search_start(pose)
            elif self.state == MissionState.PRECISION_SEARCH_LATERAL: self._handle_precision_search_lateral(pose)
            elif self.state == MissionState.PRECISION_SEARCH_FORWARD: self._handle_precision_search_forward(pose)
            elif self.state == MissionState.MOVE_TO_FINAL_TARGET: self._handle_move_to_final_target(pose)
            
            self.rate.sleep()
        
        final_pose = self.get_current_pose()
        self._handle_station_keeping(final_pose)
        self.mission_pub.publish(self.mission_msg) # 마지막 COMPLETED 상태 발행

    @staticmethod
    def wrap_angle(angle): 
        return (angle + math.pi) % (2 * math.pi) - math.pi

if __name__ == "__main__":
    node = HydrophoneMissionNode()
    node.run()