#!/usr/bin/env python3
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# buoy_mission.py

import rclpy
from rclpy.executors import ExternalShutdownException
import time
import math
import sys
from enum import Enum, auto
from typing import List, Tuple, Dict, Any, Union
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

import tf2_ros
from tf_transformations import euler_from_quaternion

# --- 1. Communication & Message Imports ---
from hit25_auv.auv_setpoint_sender import AUVSetpointSender 

# ★ [추가] PoseStamped는 기존에 있었고, Path를 추가 임포트합니다.
from geometry_msgs.msg import PoseStamped 
from nav_msgs.msg import Path 
from std_msgs.msg import Int32

# [Custom Messages & Vision Import]
from hit25_auv.msg import MissionStatus 
from hit25_auv.msg import YawControlDebug 
from hit25_auv.vision_converter import VisionConverter 

# [Calculation Modules]
from hit25_auv.object_detector import ObjectDetector 
from hit25_auv.trajectory_calculator import TrajectoryCalculator 

# --- 2. PID Controller Class ---
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

# --- 3. Mission State Definition ---
class MissionState(Enum):
    NONE = auto()             
    BUOY_INIT = auto()        
    BUOY_MOVE_TO_POINT = auto() 
    BUOY_YAW_SCAN = auto()          
    BUOY_ALIGN_VISION = auto()      
    BUOY_CALCULATE_PATH = auto()    
    BUOY_PATH_FOLLOWING = auto()    
    MISSION_COMPLETE = auto()   

class BuoyMissionNode(Node):
    # --- Constants ---
    # 관측점 (이동/스캔용)
    OBSERVATION_POINTS = [
        (0.5, 0.0),  # P1
        (0.5, 0.0),   # P2
        (0.5, 0.0)    # P3
    ]
    
    # [하드코딩 부표 위치] (마지막으로 주신 값 유지)
    HARDCODED_BUOY_POSITIONS = [
        (0.8+0.5, -0.8 - 0.4), # 1번 부표
        (3.3+0.5, +0.6 + 0.7),  # 2번 부표
        (5.8+0.3, -1.1 - 0.3)  # 3번 부표
    ]
    
    YAW_SCAN_DEGREES = [0]
    YAW_SCAN_RADIANS = [math.radians(deg) for deg in YAW_SCAN_DEGREES]
    
    YAW_TOLERANCE_RAD = math.radians(4.0) 
    MOVE_YAW_TARGET = 0.0 

    # [Vision Constants]
    VISION_YAW_KP = 0.002  
    VISION_YAW_KI = 0.000
    VISION_YAW_KD = 0.001
    
    VISION_ALIGN_THRESHOLD_PIXEL = 20 
    VISION_ALIGN_STABLE_TIME = 8.0    
    VISION_PID_PERIOD = 5.0 

    # [Safety Constants]
    IGNORE_DUPLICATE_ANGLE_RAD = math.radians(10.0) 
    
    # [Path Following Constants]
    PATH_FOLLOW_TOLERANCE = 0.2 
    
    # ★ [추가] 경로 추종 전체 타임아웃 (40초)
    TIMEOUT_PATH_TOTAL = 40.0

    # [목표 수심 설정]
    TARGET_DEPTH = -0.5

    def __init__(self, control_frequency: float = 20.0):
        super().__init__('buoy_mission_node')
        self.get_logger().info("[BuoyMission] Initializing.")

        self.rate = self.create_rate(control_frequency)
        self.state = MissionState.NONE
        
        # --- Modules ---
        self.comm = AUVSetpointSender(self, '/auv_setpoint') 
        self.vision = VisionConverter(self, '/hit25/vision') 
        
        # 계산 모듈
        self.detector = ObjectDetector(max_intersection_diameter=1.0)
        # 오프셋 0.2m 설정 (직선 문제 해결용)
        self.traj_calc = TrajectoryCalculator(offset_m=0.2) 

        # Publishers
        self.mission_pub = self.create_publisher(MissionStatus, '/mission_1', 10)
        self.mission_msg = MissionStatus()
        self.mission_msg.status = MissionStatus.READY 

        self.start_sub = self.create_subscription(
            Int32, "/mission_start_cmd", self.mission_start_callback, 10
        )

        self.debug_pub = self.create_publisher(YawControlDebug, '/yaw_control_debug', 10)
        self.debug_msg = YawControlDebug()

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self._current_pose: Dict[str, float] = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.TF_SOURCE_FRAME = 'odom'
        self.TF_TARGET_FRAME = 'auv_link'
        
        # Internal Variables
        self.current_point_idx = 0        
        self.current_yaw_idx = 0            
        
        self.pid = PIDController(self.VISION_YAW_KP, self.VISION_YAW_KI, self.VISION_YAW_KD, output_limit=0.5)
        self.align_start_time = None
        self.is_aligning_stable = False
        self.last_pid_time = self.get_clock().now()
        
        self.recorded_buoys = [] 
        self.recorded_buoy_yaws = [] 
        
        # [Path Variables]
        self.final_buoy_positions = [] 
        self.generated_path = []       
        self.path_idx = 0              
        self.path_start_time = None  # ★ [추가] 경로 추종 시작 시간

        # 시각화 퍼블리셔
        self.buoy_pose_pub = self.create_publisher(
            PoseStamped, '/buoy_obstacle_pose', 10
        )
        # latch-like QoS for path
        path_qos = QoSProfile(depth=1)
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_viz_pub = self.create_publisher(
            Path, '/buoy_planned_path', path_qos
        )

        # Smart Setpoint Variables
        self.last_sent_setpoint = None
        self.last_sent_time = rclpy.time.Time()
        self.action_start_time = None
        self._warned_start = False

        self.get_logger().info("[BuoyMission] Ready. Waiting for trigger (/mission_start_cmd = 1).")
        
        # [Timer 설정] 수동 루프 대신 타이머 사용 (20Hz)
        self.timer = self.create_timer(1.0 / control_frequency, self.mission_loop_callback)

    def mission_start_callback(self, msg):
        if msg.data == 1:  # 1번 미션 (부표) 시작
            if self.state == MissionState.NONE or self.state == MissionState.MISSION_COMPLETE:
                self.state = MissionState.BUOY_INIT
                self.get_logger().info("[BuoyMission] Start Triggered!")

    def get_current_pose(self) -> Dict[str, float]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.TF_SOURCE_FRAME,
                self.TF_TARGET_FRAME,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
            trans = transform.transform.translation
            rot = transform.transform.rotation
            (_, _, yaw) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            self._current_pose['x'] = trans.x
            self._current_pose['y'] = trans.y
            self._current_pose['yaw'] = yaw
            return self._current_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return self._current_pose

    def _send_smart_setpoint(self, x, y, z, yaw):
        current_target = (x, y, z, yaw)
        now = self.get_clock().now()
        should_send = False
        if self.last_sent_setpoint != current_target:
            should_send = True
        elif (now - self.last_sent_time).nanoseconds / 1e9 > 1.0:
            should_send = True
        if should_send:
            self.comm.send_setpoint(x, y, z, yaw, is_relative=False)
            self.last_sent_setpoint = current_target
            self.last_sent_time = now

    def _publish_debug(self, target_x, target_yaw, current_pose, error_distance=0.0):
        self.debug_msg.target_yaw = target_yaw
        self.debug_msg.current_yaw = current_pose['yaw']
        self.debug_msg.error_yaw = self.wrap_angle(target_yaw - current_pose['yaw'])
        self.debug_msg.target_x = target_x
        self.debug_msg.current_x = current_pose['x']
        self.debug_msg.error_distance = error_distance
        self.debug_pub.publish(self.debug_msg)

    def _wait_and_stabilize(self, x, y, yaw, duration=0.5):
        start_time = self.get_clock().now()
        while (self.get_clock().now() - start_time).nanoseconds / 1e9 < duration:
            if not rclpy.ok():
                break
            self.comm.send_setpoint(x, y, self.TARGET_DEPTH, yaw, is_relative=False)
            self.mission_pub.publish(self.mission_msg) 
            cp = self.get_current_pose()
            dist = math.sqrt((cp['x']-x)**2 + (cp['y']-y)**2)
            self._publish_debug(x, yaw, cp, dist)
            self.rate.sleep()

    def _is_duplicate_buoy(self, current_yaw):
        for recorded_yaw in self.recorded_buoy_yaws:
            diff = abs(self.wrap_angle(current_yaw - recorded_yaw))
            if diff < self.IGNORE_DUPLICATE_ANGLE_RAD:
                return True 
        return False 

    # --- State Handlers ---
    def _handle_yaw_scan(self, current_pose: Dict[str, float]):
        if self.action_start_time is None:
            self.action_start_time = self._now()

        buoy_list = self.vision.find_objects_by_name('Buoy')
        if buoy_list:
            target_buoy = max(buoy_list, key=lambda b: b.confidence)
            if not self._is_duplicate_buoy(current_pose['yaw']):
                self.get_logger().info("[VISION] New Buoy Found! -> ALIGN_VISION")
                self.pid.reset() 
                self.state = MissionState.BUOY_ALIGN_VISION 
                self.last_pid_time = self._now() - Duration(seconds=self.VISION_PID_PERIOD)
                self.action_start_time = None 
                return

        if self.current_yaw_idx >= len(self.YAW_SCAN_RADIANS):
            self.get_logger().info(f"[SCAN] P{self.current_point_idx + 1} Scan Complete.")
            if self.current_point_idx + 1 >= len(self.OBSERVATION_POINTS):
                self.get_logger().info("[COMPLETE] All points scanned! Calculating Path...")
                self.state = MissionState.BUOY_CALCULATE_PATH 
            else:
                self.get_logger().info("[SCAN] Moving to next point.")
                self.current_point_idx += 1 
                self.state = MissionState.BUOY_MOVE_TO_POINT
            self.current_yaw_idx = 0
            self.action_start_time = None
            return

        target_yaw = self.YAW_SCAN_RADIANS[self.current_yaw_idx]
        self._send_smart_setpoint(current_pose['x'], current_pose['y'], self.TARGET_DEPTH, target_yaw)
        self._publish_debug(current_pose['x'], target_yaw, current_pose, 0.0)
        
        yaw_error = self.wrap_angle(target_yaw - current_pose['yaw'])
        elapsed_time = self._elapsed(self.action_start_time)

        if abs(yaw_error) < self.YAW_TOLERANCE_RAD:
            self._wait_and_stabilize(current_pose['x'], current_pose['y'], target_yaw, duration=0.5)
            self.current_yaw_idx += 1
            self.action_start_time = None 
        elif elapsed_time > 5.0: 
            self.get_logger().warn("[SCAN] Timeout! Skipping Yaw.")
            self.current_yaw_idx += 1
            self.action_start_time = None

    def _handle_align_vision(self, current_pose: Dict[str, float]):
        buoy_list = self.vision.find_objects_by_name('Buoy')
        if not buoy_list:
            self.get_logger().warn("[VISION] Buoy lost. Returning to SCAN.")
            self.state = MissionState.BUOY_YAW_SCAN
            return

        target_buoy = max(buoy_list, key=lambda b: b.confidence)
        screen_center_x = self.vision._latest_data.get('screen_width', 640) / 2.0
        error_pixel = screen_center_x - target_buoy.center_x 
        
        current_time = self._now()
        dt = (current_time - self.last_pid_time).nanoseconds / 1e9

        if dt >= self.VISION_PID_PERIOD:
            yaw_control_output = self.pid.compute(error_pixel)
            new_target_yaw = current_pose['yaw'] + yaw_control_output
            self.comm.send_setpoint(x=current_pose['x'], y=current_pose['y'], z=self.TARGET_DEPTH, yaw=new_target_yaw, is_relative=False)
            self._publish_debug(current_pose['x'], new_target_yaw, current_pose, 0.0)
            self.last_pid_time = current_time

        if abs(error_pixel) < self.VISION_ALIGN_THRESHOLD_PIXEL:
            if self.align_start_time is None:
                self.align_start_time = self._now()
                self.get_logger().info("[VISION] Aligned. Stabilizing...")
            elapsed = self._elapsed(self.align_start_time)
            if elapsed > self.VISION_ALIGN_STABLE_TIME:
                self._record_buoy_data(current_pose)
                self.get_logger().info("[VISION] Recorded. Returning to SCAN.")
                self.state = MissionState.BUOY_YAW_SCAN
                self.align_start_time = None 
        else:
            self.align_start_time = None

    def _record_buoy_data(self, current_pose):
        dist = 2.0
        buoy_x = current_pose['x'] + dist * math.cos(current_pose['yaw'])
        buoy_y = current_pose['y'] + dist * math.sin(current_pose['yaw'])
        data_entry = {'buoy_x': buoy_x, 'buoy_y': buoy_y, 'yaw': current_pose['yaw']}
        self.recorded_buoys.append(data_entry)
        self.recorded_buoy_yaws.append(current_pose['yaw'])
        self.get_logger().info(f"   [RECORD] Estimated Buoy Pos: ({buoy_x:.2f}, {buoy_y:.2f})")

    def _handle_move_to_point(self, current_pose: Dict[str, float]):
        if self.action_start_time is None:
            self.action_start_time = self._now()

        target_xy = self.OBSERVATION_POINTS[self.current_point_idx]
        target_yaw = self.MOVE_YAW_TARGET 
        self._send_smart_setpoint(target_xy[0], target_xy[1], self.TARGET_DEPTH, target_yaw)
        
        distance = math.sqrt((current_pose['x'] - target_xy[0])**2 + (current_pose['y'] - target_xy[1])**2)
        self._publish_debug(target_xy[0], target_yaw, current_pose, distance)
        
        elapsed_time = self._elapsed(self.action_start_time)

        if distance < 0.2:
            self.get_logger().info(f"[MOVE] P{self.current_point_idx + 1} Reached.")
            self._wait_and_stabilize(target_xy[0], target_xy[1], target_yaw, duration=0.5)
            self.state = MissionState.BUOY_YAW_SCAN
            self.action_start_time = None
        elif elapsed_time > 15.0: 
            self.get_logger().warn(f"[MOVE] Timeout! Forcing next state. Dist: {distance:.2f}m")
            self.state = MissionState.BUOY_YAW_SCAN
            self.action_start_time = None

    # --- [수정] 가상 부표 로직 제거 (순수 3개 부표로 경로 생성) ---
    def _handle_calculate_path(self, current_pose: Dict[str, float]):
        self.get_logger().info("--- [PATH] Calculating Final Buoy Positions ---")
        self.final_buoy_positions = []
        
        # 1. 실제/하드코딩 부표 병합 (3개)
        for i in range(3):
            if i < len(self.recorded_buoys):
                b_data = self.recorded_buoys[i]
                pos = (b_data['buoy_x'], b_data['buoy_y'])
                self.get_logger().info(f"Buoy {i+1}: Detected at {pos}")
            else:
                pos = self.HARDCODED_BUOY_POSITIONS[i]
                self.get_logger().warn(f"Buoy {i+1}: Not Detected -> Using Hardcoded {pos}")
            self.final_buoy_positions.append(pos)
            
        # [제거됨] 4번째 가상 부표(Dummy) 생성 로직 제거
        
        current_xy = (current_pose['x'], current_pose['y'])
        
        path = self.traj_calc.generate_path(
            current_pos=current_xy,
            buoy_positions=self.final_buoy_positions,
            num_path_points=25 # 점 개수 약간 늘림
        )
        
        if path:
            self.generated_path = path
            self.path_idx = 0
            self.get_logger().info(f"Path Generation Success! Waypoints: {len(path)}")
            self._publish_generated_path(path)
            self._publish_final_buoy_poses()
            self.state = MissionState.BUOY_PATH_FOLLOWING
        else:
            self.get_logger().error("Path Generation Failed! Mission Abort.")
            self.state = MissionState.MISSION_COMPLETE

    def _publish_generated_path(self, path_points):
        path_msg = Path()
        path_msg.header.frame_id = self.TF_SOURCE_FRAME 
        path_msg.header.stamp = self._now().to_msg()
        for (x, y) in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = self.TARGET_DEPTH
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_viz_pub.publish(path_msg)
        self.get_logger().info(
            f"[PATH] Published full trajectory to /buoy_planned_path (Length: {len(path_msg.poses)})"
        )

    def _publish_final_buoy_poses(self):
        if not self.final_buoy_positions: return
        for idx, (bx, by) in enumerate(self.final_buoy_positions):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self._now().to_msg()
            pose_msg.header.frame_id = self.TF_SOURCE_FRAME
            pose_msg.pose.position.x = bx
            pose_msg.pose.position.y = by
            pose_msg.pose.position.z = self.TARGET_DEPTH
            pose_msg.pose.orientation.w = 1.0
            self.buoy_pose_pub.publish(pose_msg)

    # --- [수정] 경로 추종 로직 (전체 타임아웃 추가) ---
    def _handle_path_following(self, current_pose: Dict[str, float]):
        # ★ [추가] 전체 경로 추종 타임아웃 체크용 타이머 시작
        if self.path_start_time is None:
            self.path_start_time = self._now()

        # 종료 조건: 모든 점 도달
        if self.path_idx >= len(self.generated_path):
            self.get_logger().info("[PATH] End of Path Reached.")
            self.state = MissionState.MISSION_COMPLETE
            return

        # ★ [추가] 타임아웃 체크 (40초 경과 시 강제 종료)
        elapsed_total = self._elapsed(self.path_start_time)
        if elapsed_total > self.TIMEOUT_PATH_TOTAL:
            self.get_logger().warn(
                f"[PATH] Total Timeout ({self.TIMEOUT_PATH_TOTAL}s) Reached! Forcing Mission Complete."
            )
            self.state = MissionState.MISSION_COMPLETE
            return

        target_xy = self.generated_path[self.path_idx]
        
        if self.path_idx < len(self.generated_path) - 1:
            next_xy = self.generated_path[self.path_idx + 1]
            dx = next_xy[0] - current_pose['x']
            dy = next_xy[1] - current_pose['y']
            target_yaw = math.atan2(dy, dx)
        elif self.path_idx > 0:
            prev_xy = self.generated_path[self.path_idx - 1]
            dx = target_xy[0] - prev_xy[0]
            dy = target_xy[1] - prev_xy[1]
            target_yaw = math.atan2(dy, dx)
        else:
            target_yaw = 0.0

        self.comm.send_setpoint(
            x=target_xy[0], y=target_xy[1], 
            z=self.TARGET_DEPTH, 
            yaw=target_yaw, is_relative=False
        )
        
        distance = math.sqrt((current_pose['x'] - target_xy[0])**2 + (current_pose['y'] - target_xy[1])**2)
        if distance < self.PATH_FOLLOW_TOLERANCE:
            self.path_idx += 1

    def mission_loop_callback(self):
        # 상태 퍼블리싱
        self.mission_pub.publish(self.mission_msg)
        self.debug_pub.publish(self.debug_msg)

        # 미션 시작 전 대기 상태
        if self.mission_msg.status == MissionStatus.READY:
            # 여기서 미션 시작 트리거를 기다리거나, 바로 시작할 수 있습니다.
            # 예: 특정 토픽 수신 시 self.state = MissionState.BUOY_INIT 로 변경
            # 현재는 바로 시작하는 로직으로 가정합니다.
            self.get_logger().info("[BuoyMission] Started! (/mission_1 = 1)")
            self.state = MissionState.BUOY_INIT
            self.mission_msg.status = MissionStatus.RUNNING
            # time.sleep(1.0) # 타이머 콜백에서는 sleep 사용 지양

        if self.mission_msg.status == MissionStatus.RUNNING:
            current_pose = self.get_current_pose()
            self.mission_pub.publish(self.mission_msg)

            if self.state == MissionState.NONE:
                self._send_smart_setpoint(0.0, 0.0, self.TARGET_DEPTH, 0.0)
            elif self.state == MissionState.BUOY_INIT:
                self.current_point_idx = 0
                self.current_yaw_idx = 0
                self.state = MissionState.BUOY_MOVE_TO_POINT
                self.get_logger().info("[INIT] Moving to P1.")
            elif self.state == MissionState.BUOY_MOVE_TO_POINT:
                self._handle_move_to_point(current_pose)
            elif self.state == MissionState.BUOY_YAW_SCAN:
                self._handle_yaw_scan(current_pose)
            elif self.state == MissionState.BUOY_ALIGN_VISION:
                self._handle_align_vision(current_pose)
            elif self.state == MissionState.BUOY_CALCULATE_PATH: 
                self._handle_calculate_path(current_pose)
            elif self.state == MissionState.BUOY_PATH_FOLLOWING: 
                self._handle_path_following(current_pose)
            
            elif self.state == MissionState.MISSION_COMPLETE:
                if self.mission_msg.status != MissionStatus.COMPLETED:
                    self.mission_msg.status = MissionStatus.COMPLETED
                    self.get_logger().info("[BuoyMission] Mission Complete! (/mission_1 = 2)")
                    self.get_logger().info("[INFO] Stopping /auv_setpoint publishing to release control.")
                
                # [수정] 미션 완료 시 더 이상 setpoint를 보내지 않음.
                # self._send_smart_setpoint(...) <--- 제거됨
                
            self.rate.sleep()

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def _now(self):
        return self.get_clock().now()

    def _elapsed(self, start_time):
        return (self.get_clock().now() - start_time).nanoseconds / 1e9

def main():
    rclpy.init()
    node = BuoyMissionNode(control_frequency=20.0)
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
