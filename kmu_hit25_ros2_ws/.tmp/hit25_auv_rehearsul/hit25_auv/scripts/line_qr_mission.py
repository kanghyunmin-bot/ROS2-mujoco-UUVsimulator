#!/usr/bin/env python
# -*- coding: utf-8 -*-
# line_qr_mission.py 
# (1m 전진 -> QR 스캔 -> [실패시 QR 앞 0.7m 이동] -> QR 정렬)

import rospy
import time
import math
import sys
from enum import Enum, auto
from typing import Dict

# >>> sys.path 조정 >>>
import os
import rospkg

pkg_path = rospkg.RosPack().get_path('hit25_auv')
scripts_dir = os.path.join(pkg_path, 'scripts')
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)
# <<< sys.path 조정 <<<

# --- Communication & Message Imports ---
from auv_setpoint_sender import AUVSetpointSender
import tf
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped 

from vision_converter import VisionConverter

try:
    from hit25_auv.msg import MissionStatus, YawControlDebug
except ImportError:
    pass


# --- PID Controller Class ---
class PIDController:
    def __init__(self, kp, ki, kd, output_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None

    def compute(self, error):
        current_time = time.time()
        if self.last_time is None:
            dt = 0.05
        else:
            dt = current_time - self.last_time

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.output_limit), -self.output_limit)

        self.prev_error = error
        self.last_time = current_time
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = None


# --- Mission State Definition ---
class MissionState(Enum):
    NONE = auto()
    WAIT_FOR_TRIGGER = auto()       # 시작 대기
    MOVE_FORWARD_1M = auto()        # 1m 전진
    INIT_SCAN_QR = auto()           # QR 스캔 준비
    SCANNING_QR = auto()            # QR Yaw 스캔
    MOVE_TO_HARDCODED = auto()      # QR 앞 접근 지점으로 이동
    ALIGN_TO_QR = auto()            # QR 정렬 (PID)
    MISSION_COMPLETE = auto()       # 종료


class LineQRMissionNode:
    # --- Constants ---

    TARGET_QR_NAME = 'Qr'

    # [Scanning Parameters]
    SCAN_START_DEG = -65
    SCAN_END_DEG = 65
    SCAN_STEP_DEG = 15

    # [Depth]
    TARGET_DEPTH = -0.5

    # [Tolerances]
    YAW_TOLERANCE_RAD = math.radians(4.0)
    POS_TOLERANCE_M = 0.2

    # [Vision PID]
    VISION_YAW_KP = 0.05 
    VISION_YAW_KI = 0.000
    VISION_YAW_KD = 0.1

    VISION_ALIGN_THRESHOLD_PIXEL = 20
    VISION_ALIGN_STABLE_TIME = 4.0
    VISION_PID_PERIOD = 3
    
    # [Initial Move]
    # INITIAL_MOVE_DIST = -1.0
    INITIAL_MOVE_DIST = 1.0


    # [Hardcoded Position & Offset]
    # 실제 수조 상의 QR 위치 (장애물 좌표)
    HARDCODED_QR_X = 9.8
    HARDCODED_QR_Y = -0.5
    
    # QR 인식 실패 시 QR로 부터 떨어진 접근 거리 (m)
    APPROACH_OFFSET = 1.0 

    def __init__(self, control_frequency: float = 20.0):
        rospy.init_node('line_qr_mission_node', anonymous=True)
        rospy.loginfo("[LineQR] Initializing Mission Node.")

        self.rate = rospy.Rate(control_frequency)
        self.state = MissionState.WAIT_FOR_TRIGGER

        # --- Modules ---
        self.comm = AUVSetpointSender('/auv_setpoint')
        self.vision = VisionConverter('/hit25/vision')

        # Publishers
        self.mission_pub = rospy.Publisher('/mission_3', MissionStatus, queue_size=10)
        self.mission_msg = MissionStatus()
        self.mission_msg.status = MissionStatus.READY

        self.debug_pub = rospy.Publisher('/yaw_control_debug', YawControlDebug, queue_size=10)
        self.debug_msg = YawControlDebug()
        
        self.qr_pose_pub = rospy.Publisher('/qr_obstacle_pose', PoseStamped, queue_size=10, latch=True)

        # [Trigger Subscriber]
        # /mission_2 토픽 구독
        self.mission_2_sub = rospy.Subscriber('/mission_2', MissionStatus, self.mission_2_callback)
        self.mission_start_trigger = False # 트리거 플래그

        # TF Listener
        self.tf_listener = tf.TransformListener()
        self._current_pose: Dict[str, float] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.TF_SOURCE_FRAME = 'odom'
        self.TF_TARGET_FRAME = 'auv_link'

        # --- Internal Variables ---
        self.pid = PIDController(
            self.VISION_YAW_KP,
            self.VISION_YAW_KI,
            self.VISION_YAW_KD,
            output_limit=0.5
        )
        
        # Move 관련 변수
        self.initial_move_target = None
        self.initial_move_start_time = None
        
        self.fallback_move_start_time = None
        self.has_moved_to_hardcoded = False 

        # Scan 관련
        self.scan_angles = []
        self.current_scan_idx = 0
        self.scan_start_time = None

        # Align 관련
        self.align_start_time = None
        self.last_pid_time = rospy.Time.now()

        rospy.loginfo("[LineQR] Ready. Waiting for Trigger.")

    # --- Callback Methods ---
    def mission_2_callback(self, msg):
        """ /mission_2 메시지를 받아 상태가 COMPLETED(2)이면 시작 플래그 설정 """
        # MissionStatus.COMPLETED 는 보통 2로 매핑됨
        if msg.status == 2:
            if not self.mission_start_trigger:
                rospy.loginfo("[LineQR] Trigger Received: Mission 2 Completed.")
                self.mission_start_trigger = True

    # --- Utility Methods ---
    def get_current_pose(self) -> Dict[str, float]:
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                self.TF_SOURCE_FRAME, self.TF_TARGET_FRAME, rospy.Time(0)
            )
            (roll, pitch, yaw) = euler_from_quaternion(rot)
            self._current_pose['x'] = trans[0]
            self._current_pose['y'] = trans[1]
            self._current_pose['yaw'] = yaw
            return self._current_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return self._current_pose

    def _publish_debug(self, target_x, target_yaw, current_pose, error_distance=0.0):
        self.debug_msg.target_yaw = target_yaw
        self.debug_msg.current_yaw = current_pose['yaw']
        self.debug_msg.error_yaw = self.wrap_angle(target_yaw - current_pose['yaw'])
        self.debug_msg.target_x = target_x
        self.debug_msg.current_x = current_pose['x']
        self.debug_msg.error_distance = error_distance
        self.debug_pub.publish(self.debug_msg)

    def _publish_qr_pose(self, x, y):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.TF_SOURCE_FRAME # odom
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = self.TARGET_DEPTH
        pose_msg.pose.orientation.w = 1.0 # 회전 없음
        
        self.qr_pose_pub.publish(pose_msg)

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _generate_scan_angles(self, center_yaw):
        angles = []
        for deg in range(self.SCAN_START_DEG, self.SCAN_END_DEG + 1, self.SCAN_STEP_DEG):
            rad = math.radians(deg)
            angles.append(self.wrap_angle(center_yaw + rad))
        return angles

    # --- State Handler Methods ---
    
    def _handle_move_forward_1m(self, current_pose):
        if self.initial_move_target is None:
            self.initial_move_start_time = rospy.Time.now()
            yaw = current_pose['yaw']
            tx = current_pose['x'] + self.INITIAL_MOVE_DIST * math.cos(yaw)
            ty = current_pose['y'] + self.INITIAL_MOVE_DIST * math.sin(yaw)
            self.initial_move_target = (tx, ty)
            rospy.loginfo(f"[MOVE] Moving Forward 1m to ({tx:.2f}, {ty:.2f})")
            
        target_x, target_y = self.initial_move_target
        
        self.comm.send_setpoint(target_x, target_y, self.TARGET_DEPTH, current_pose['yaw'], is_relative=False)
        
        dist = math.sqrt((current_pose['x'] - target_x)**2 + (current_pose['y'] - target_y)**2)
        self._publish_debug(target_x, current_pose['yaw'], current_pose, dist)
        
        if dist < self.POS_TOLERANCE_M:
            rospy.loginfo("[MOVE] 1m Forward Complete.")
            self.state = MissionState.INIT_SCAN_QR
        elif (rospy.Time.now() - self.initial_move_start_time).to_sec() > 15.0:
             rospy.logwarn("[MOVE] 1m Move Timeout. Force Next State.")
             self.state = MissionState.INIT_SCAN_QR

    def _init_scan_qr(self, current_pose):
        self.scan_angles = self._generate_scan_angles(current_pose['yaw'])
        self.current_scan_idx = 0
        self.scan_start_time = None
        self.state = MissionState.SCANNING_QR
        rospy.loginfo("[STATE] Start Scanning for QR")

    def _handle_scanning_qr(self, current_pose):
        objects = self.vision.find_objects_by_name(self.TARGET_QR_NAME)

        if objects:
            rospy.loginfo("[VISION] Found QR! Switching to ALIGN.")
            self.pid.reset()
            self.state = MissionState.ALIGN_TO_QR
            self.last_pid_time = rospy.Time.now()
            self.align_start_time = None
            return

        if self.current_scan_idx >= len(self.scan_angles):
            rospy.logwarn("[SCAN] QR NOT Found.")
            
            if not self.has_moved_to_hardcoded:
                rospy.logwarn(f"-> Moving to Hardcoded QR pos (Offset -{self.APPROACH_OFFSET}m).")
                self.state = MissionState.MOVE_TO_HARDCODED
                self._publish_qr_pose(self.HARDCODED_QR_X, self.HARDCODED_QR_Y)
            else:
                rospy.logerr("-> Already visited hardcoded pos. Mission Failed.")
                self.state = MissionState.MISSION_COMPLETE
            return

        if self.scan_start_time is None:
            self.scan_start_time = rospy.Time.now()

        target_yaw = self.scan_angles[self.current_scan_idx]

        self.comm.send_setpoint(
            current_pose['x'], current_pose['y'], self.TARGET_DEPTH, target_yaw, is_relative=False
        )
        self._publish_debug(current_pose['x'], target_yaw, current_pose)

        yaw_error = self.wrap_angle(target_yaw - current_pose['yaw'])
        elapsed = (rospy.Time.now() - self.scan_start_time).to_sec()

        if abs(yaw_error) < self.YAW_TOLERANCE_RAD:
            if elapsed > 2.0:
                self.current_scan_idx += 1
                self.scan_start_time = None
        elif elapsed > 5.0:
            rospy.logwarn("[SCAN] Timeout rotating to angle. Skipping.")
            self.current_scan_idx += 1
            self.scan_start_time = None

    def _handle_move_to_hardcoded(self, current_pose):
        target_x = self.HARDCODED_QR_X - self.APPROACH_OFFSET
        target_y = self.HARDCODED_QR_Y
        
        if self.fallback_move_start_time is None:
            self.fallback_move_start_time = rospy.Time.now()
            rospy.loginfo(f"[FALLBACK] Moving to Approach Point: ({target_x:.2f}, {target_y:.2f})")

        self.comm.send_setpoint(target_x, target_y, self.TARGET_DEPTH, 0.0, is_relative=False)
        
        dist = math.sqrt((current_pose['x'] - target_x)**2 + (current_pose['y'] - target_y)**2)
        self._publish_debug(target_x, 0.0, current_pose, dist)
        
        is_timeout = (rospy.Time.now() - self.fallback_move_start_time).to_sec() > 20.0

        if dist < self.POS_TOLERANCE_M or is_timeout:
            if is_timeout: rospy.logwarn("[FALLBACK] Timeout.")
            else: rospy.loginfo("[FALLBACK] Arrived at Approach Point.")
            
            self.has_moved_to_hardcoded = True
            self.state = MissionState.INIT_SCAN_QR

    def _handle_align_qr(self, current_pose):
        objects = self.vision.find_objects_by_name(self.TARGET_QR_NAME)

        if not objects:
            rospy.logwarn("[ALIGN] Lost QR. Mission Aborted.")
            self.state = MissionState.MISSION_COMPLETE 
            return

        target_obj = max(objects, key=lambda b: b.confidence)
        
        estimated_qr_dist = 1.5
        est_qr_x = current_pose['x'] + estimated_qr_dist * math.cos(current_pose['yaw'])
        est_qr_y = current_pose['y'] + estimated_qr_dist * math.sin(current_pose['yaw'])
        self._publish_qr_pose(est_qr_x, est_qr_y)
        
        screen_width = self.vision._latest_data.get('screen_width', 640)
        screen_center_x = screen_width / 2.0
        error_pixel = screen_center_x - target_obj.center_x

        current_time = rospy.Time.now()
        dt = (current_time - self.last_pid_time).to_sec()

        yaw_control_output = 0.0
        if dt >= self.VISION_PID_PERIOD:
            yaw_control_output = self.pid.compute(error_pixel)
            self.last_pid_time = current_time

        new_target_yaw = current_pose['yaw'] + yaw_control_output

        self.comm.send_setpoint(
            current_pose['x'], current_pose['y'], self.TARGET_DEPTH, new_target_yaw, is_relative=False
        )
        self._publish_debug(current_pose['x'], new_target_yaw, current_pose)

        if abs(error_pixel) < self.VISION_ALIGN_THRESHOLD_PIXEL:
            if self.align_start_time is None:
                self.align_start_time = rospy.Time.now()
                rospy.loginfo("[ALIGN] QR Centered. Stabilizing...")

            if (rospy.Time.now() - self.align_start_time).to_sec() > self.VISION_ALIGN_STABLE_TIME:
                rospy.loginfo("[ALIGN] QR Align Complete. Mission Complete.")
                self.state = MissionState.MISSION_COMPLETE
        else:
            self.align_start_time = None

    def run(self):
        self.mission_msg.status = MissionStatus.READY
        
        # =========================================================
        # [Option 1] ROS Topic Trigger (/mission_2 == 2) - 활성화됨
        # =========================================================
        rospy.loginfo("[LineQR] Waiting for /mission_2 completion Trigger...")
        while not rospy.is_shutdown():
            self.mission_pub.publish(self.mission_msg)
            
            if self.mission_start_trigger:
                rospy.loginfo("[LineQR] Mission 3 START Triggered!")
                break
            self.rate.sleep()
        # =========================================================

        # =========================================================
        # [Option 2] Keyboard Input Trigger (Enter) - 주석 처리됨
        # =========================================================
        # rospy.loginfo("[LineQR] Waiting for Enter input...")
        # while not rospy.is_shutdown():
        #     self.mission_pub.publish(self.mission_msg)
        #     rospy.logwarn_once("!!! [Mission 3] Press Enter to Start !!!")
        #     try:
        #         if sys.version_info[0] < 3: raw_input()
        #         else: input()
        #         break
        #     except (NameError, EOFError): break
        # =========================================================

        rospy.loginfo("[LineQR] Started!")
        self.mission_msg.status = MissionStatus.RUNNING
        self.state = MissionState.MOVE_FORWARD_1M 

        while not rospy.is_shutdown():
            current_pose = self.get_current_pose()
            self.mission_pub.publish(self.mission_msg)

            if self.state == MissionState.MOVE_FORWARD_1M:
                self._handle_move_forward_1m(current_pose)

            elif self.state == MissionState.INIT_SCAN_QR:
                self._init_scan_qr(current_pose)

            elif self.state == MissionState.SCANNING_QR:
                self._handle_scanning_qr(current_pose)
                
            elif self.state == MissionState.MOVE_TO_HARDCODED:
                self._handle_move_to_hardcoded(current_pose)

            elif self.state == MissionState.ALIGN_TO_QR:
                self._handle_align_qr(current_pose)

            elif self.state == MissionState.MISSION_COMPLETE:
                if self.mission_msg.status != MissionStatus.COMPLETED:
                    self.mission_msg.status = MissionStatus.COMPLETED
                    rospy.loginfo("[LineQR] Mission Complete / Aborted.")
                
                # [수정] 미션 완료 시에는 setpoint 발행 중단 (다른 노드와 간섭 방지)
                # self.comm.send_setpoint(...)  <- 제거됨

            self.rate.sleep()

if __name__ == "__main__":
    try:
        try:
            from hit25_auv.msg import MissionStatus, YawControlDebug
        except ImportError:
            rospy.logerr("MissionStatus.msg not found!")
            sys.exit(1)

        node = LineQRMissionNode(control_frequency=20.0)
        node.run()
    except rospy.ROSInterruptException:
        pass