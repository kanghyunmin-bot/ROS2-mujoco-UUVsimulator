#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# mission_5.py (Return to Home and Disarm with Launch-File Configurable Start)

import rclpy
from rclpy.executors import ExternalShutdownException
import math
import sys
from enum import Enum, auto
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

import tf2_ros
from tf_transformations import euler_from_quaternion

from hit25_auv.auv_setpoint_sender import AUVSetpointSender

# --- Message and Service Imports ---
from hit25_auv.msg import MissionStatus
from mavros_msgs.srv import CommandBool
from geometry_msgs.msg import PoseStamped

class MissionState(Enum):
    WAIT_FOR_TRIGGER, MOVE_TO_HOME, DISARMING, MISSION_COMPLETE = (auto() for i in range(4))

class Mission5Node(Node):
    # --- Mission Parameters & Constants ---
    HOME_POINT = (1.5, 0.0)
    TARGET_DEPTH = -0.5
    DIST_TOLERANCE = 0.3
    XY_TIMEOUT = 20.0

    def __init__(self, control_frequency: float = 20.0):
        super().__init__('mission_5_node')
        
        # ★★★ 핵심 수정: 'manual_start' 파라미터를 읽어와서 시작 모드를 결정합니다. ★★★
        # rosparam get /mission_5_node/manual_start 로 확인 가능
        # launch 파일에서 <param name="manual_start" type="bool" value="true"/> 로 설정
        self.is_manual_start = self.declare_parameter("manual_start", False).value

        if self.is_manual_start:
            self.get_logger().info("[Mission 5] MANUAL START mode activated.")
        else:
            self.get_logger().info("[Mission 5] AUTO START mode activated. Waiting for Mission 4.")

        self.rate = self.create_rate(control_frequency)
        self.state = MissionState.WAIT_FOR_TRIGGER

        # --- ROS Connections ---
        self.comm = AUVSetpointSender(self, '/auv_setpoint')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.mission_pub = self.create_publisher(MissionStatus, '/mission_5', 10)
        marker_qos = QoSProfile(depth=1)
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.marker_pub = self.create_publisher(PoseStamped, '/mission_5/home_marker', marker_qos)

        # ★★★ 핵심 수정: 수동 시작 모드가 아닐 때만 Mission 4를 구독합니다. ★★★
        if not self.is_manual_start:
            self.mission_4_sub = self.create_subscription(
                MissionStatus, '/mission_4', self._mission_4_callback, 10
            )

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.get_logger().info("[Mission 5] Waiting for /mavros/cmd/arming service...")
        if not self.arming_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("[Mission 5] Failed to connect to MAVROS arming service.")
            self.arming_client = None
        else:
            self.get_logger().info("[Mission 5] MAVROS arming service connected.")

        # --- Internal State Variables ---
        self.mission_msg = MissionStatus()
        self.mission_msg.status = MissionStatus.READY
        self._current_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.TF_SOURCE_FRAME = 'odom'
        self.TF_TARGET_FRAME = 'auv_link'
        self.mission_start_trigger = False
        self.move_start_time = None
        
        self.get_logger().info("[Mission 5] Ready.")

    def _mission_4_callback(self, msg: MissionStatus):
        if msg.status == MissionStatus.COMPLETED and not self.mission_start_trigger:
            self.get_logger().info("[Mission 5] Trigger Received: Mission 4 Completed.")
            self.mission_start_trigger = True
            
    def get_current_pose(self):
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
            self._current_pose.update({'x': trans.x, 'y': trans.y, 'yaw': yaw})
        except: pass
        return self._current_pose
    def _publish_home_marker(self):
        """HOME_POINT 위치에 RViz 마커를 발행합니다."""
        marker_msg = PoseStamped()
        marker_msg.header.frame_id = self.TF_SOURCE_FRAME
        marker_msg.header.stamp = self._now().to_msg()
        marker_msg.pose.position.x = self.HOME_POINT[0]
        marker_msg.pose.position.y = self.HOME_POINT[1]
        marker_msg.pose.position.z = self.TARGET_DEPTH
        marker_msg.pose.orientation.w = 1.0
        self.marker_pub.publish(marker_msg)
        self.get_logger().info(f"[Mission 5] Home marker published at {self.HOME_POINT}")

    # ( _handle_move_to_home, _handle_disarming 함수는 이전과 동일 )
    def _handle_move_to_home(self, pose):
        tx, ty = self.HOME_POINT
        self.comm.send_setpoint(tx, ty, self.TARGET_DEPTH, 0.0, is_relative=False)
        if self.move_start_time is None: self.move_start_time = self._now()
        dist_to_home = math.hypot(pose['x'] - tx, pose['y'] - ty)
        elapsed_time = self._elapsed(self.move_start_time)
        if dist_to_home < self.DIST_TOLERANCE:
            self.get_logger().info(f"[Mission 5] Reached Home Point ({tx}, {ty}).")
            self.state = MissionState.DISARMING
        elif elapsed_time > self.XY_TIMEOUT:
            self.get_logger().warn("[Mission 5] Timeout while moving to home. Proceeding with disarm.")
            self.state = MissionState.DISARMING

    def _handle_disarming(self):
        self.get_logger().info("[Mission 5] Attempting to disarm vehicle...")
        if self.arming_client is None:
            self.get_logger().error("[Mission 5] Arming service is not available. Cannot disarm.")
            self.state = MissionState.MISSION_COMPLETE
            return
        try:
            req = CommandBool.Request()
            req.value = False
            future = self.arming_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            if future.result() is not None and future.result().success:
                self.get_logger().info("Vehicle successfully disarmed.")
            else:
                self.get_logger().error("Failed to disarm vehicle, but completing mission anyway.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
        self.state = MissionState.MISSION_COMPLETE

    def run(self):
        """메인 루프: 미션의 전체 흐름을 제어합니다."""
        
        # =========================================================
        # [Option 1] Mission 4 완료 시 자동 시작 (활성화됨)
        # =========================================================
        self.get_logger().info("[Mission 5] Waiting for Mission 4 completion trigger...")
        while rclpy.ok() and not self.mission_start_trigger:
            self.mission_pub.publish(self.mission_msg) # READY 상태 계속 발행
            self.rate.sleep()
        # =========================================================

        # =========================================================
        # [Option 2] Keyboard Input Trigger (Enter) - 주석 처리됨
        # =========================================================
        # self.get_logger().info("[Mission 5] Waiting for Enter key to start manually...")
        # while rclpy.ok():
        #    self.mission_pub.publish(self.mission_msg) # READY 상태 계속 발행
        #    self.get_logger().warn("!!! [Mission 5] Press Enter to START Return-to-Home !!!")
        #    try:
        #        if sys.version_info[0] < 3: raw_input()
        #        else: input()
        #        break # Enter가 입력되면 루프 탈출
        #    except (NameError, EOFError):
        #        if not rclpy.ok(): return
        #        break
        # =========================================================
        
        if not rclpy.ok():
            return

        # --- 2. 미션 시작 ---
        self.get_logger().info("[Mission 5] Mission Started! Returning to home.")
        
        self._publish_home_marker()
        
        self.mission_msg.status = MissionStatus.RUNNING
        self.state = MissionState.MOVE_TO_HOME        

        # --- 3. 메인 상태 머신 루프 ---
        while rclpy.ok() and self.state != MissionState.MISSION_COMPLETE:
            self.mission_pub.publish(self.mission_msg) # RUNNING
            pose = self.get_current_pose()
            if self.state == MissionState.MOVE_TO_HOME: self._handle_move_to_home(pose)
            elif self.state == MissionState.DISARMING: self._handle_disarming()
            self.rate.sleep()

        # --- 4. 임무 완료 후 무한 대기 ---
        if self.mission_msg.status != MissionStatus.COMPLETED:
            self.get_logger().info("[Mission 5] Mission Complete.")
            self.mission_msg.status = MissionStatus.COMPLETED
        self.get_logger().info("[Mission 5] Node will now remain active, publishing final status.")
        while rclpy.ok():
            self.mission_pub.publish(self.mission_msg)
            self.rate.sleep()

    def _now(self):
        return self.get_clock().now()

    def _elapsed(self, start_time):
        return (self.get_clock().now() - start_time).nanoseconds / 1e9


def main():
    rclpy.init()
    node = Mission5Node()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
