#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
import tf2_ros
# tf_conversions 호환성
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped, TransformStamped
from visualization_msgs.msg import Marker

# [중요] Controller와 동일한 메시지 임포트
try:
    from waterlinked_a50_ros_driver.msg import DvlIntegratedData
except ImportError:
    # 테스트용 더미 클래스
    class DvlIntegratedData:
        pass

try:
    from hit25_auv.msg import AuvSetpoint
except ImportError:
    class AuvSetpoint:
        x=0.0; y=0.0; z=0.0; yaw=0.0; mode=0

class RefPointMarkerNode(object):
    def __init__(self):
        rospy.init_node("ref_point_marker_node")

        # --- 파라미터 ---
        self.parent_frame = rospy.get_param("~parent_frame", "odom")
        self.auv_frame    = rospy.get_param("~auv_frame", "auv_link")
        self.marker_topic = rospy.get_param("~marker_topic", "/ref_point_marker")
        
        # [변경] Controller가 구독하는 DVL 토픽과 동일하게 설정
        self.dvl_topic = rospy.get_param("~dvl_topic", "/dvl/integrated_data")

        # 경로 시각화용 설정
        self.max_path_points = rospy.get_param("~max_path_points", 2000)
        self.path_points = []
        self.last_path_pos = None
        self.min_dist_sq = 0.05 * 0.05  # 5cm 이동 시 점 찍기

        # ID 카운터
        self.buoy_id_counter = 0
        self.gate_id_counter = 0
        self.qr_id_counter = 0 
        self.hydro_id_counter = 0 # [추가] 하이드로폰 ID 카운터

        # TF 리스너 (상대 좌표 계산용)
        self.tf_buffer  = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 마커 퍼블리셔
        self.marker_pub = rospy.Publisher(self.marker_topic, Marker, queue_size=10)

        # 1. 목표 지점 (Reference Point) 구독 -> 빨간 화살표
        self.setpoint_sub = rospy.Subscriber(
            "/auv_setpoint",
            AuvSetpoint,
            self.setpoint_callback,
            queue_size=10
        )

        # 2. [핵심 변경] 실제 위치 (DVL) 구독 -> 파란색 경로(History)
        self.dvl_sub = rospy.Subscriber(
            self.dvl_topic,
            DvlIntegratedData,
            self.dvl_callback,
            queue_size=10
        )

        # 3. 장애물 구독
        self.buoy_sub = rospy.Subscriber("/buoy_obstacle_pose", PoseStamped, self.buoy_pose_callback, queue_size=10)
        self.gate_sub = rospy.Subscriber("/gate_obstacle_pose", PoseStamped, self.gate_pose_callback, queue_size=10)
        self.qr_sub = rospy.Subscriber("/qr_obstacle_pose", PoseStamped, self.qr_pose_callback, queue_size=10)
        
        # [추가] 하이드로폰 구독
        self.hydro_sub = rospy.Subscriber("/hydrophone_obstacle_pose", PoseStamped, self.hydro_pose_callback, queue_size=10)

        rospy.loginfo("[Marker] Initialized. Path Source: %s (Matching Controller Logic)", self.dvl_topic)

    # --- 유틸 ---
    @staticmethod
    def yaw_to_quat(yaw):
        q = quaternion_from_euler(0.0, 0.0, yaw)
        return Quaternion(*q)

    @staticmethod
    def quat_to_yaw(q):
        q_list = [q.x, q.y, q.z, q.w]
        _, _, yaw = euler_from_quaternion(q_list)
        return yaw

    @staticmethod
    def wrap_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    # ======================================================================
    # [핵심] DVL Callback: 실제 로봇 경로 (Blue Line)
    # ======================================================================
    def dvl_callback(self, msg):
        """
        controller.py의 로직과 동일하게 좌표를 변환하여 궤적을 그립니다.
        Controller Logic: self.curr_pos = [msg.x, -msg.y, -msg.z]
        """
        # [중요] 컨트롤러와 동일한 좌표 변환 적용
        curr_x = msg.x
        curr_y = -msg.y  # 컨트롤러에 맞춰 반전
        curr_z = -msg.z  # 컨트롤러에 맞춰 반전

        # 최적화: 너무 적게 움직였으면 그리지 않음
        if self.last_path_pos is not None:
            dx = curr_x - self.last_path_pos[0]
            dy = curr_y - self.last_path_pos[1]
            dz = curr_z - self.last_path_pos[2]
            dist_sq = dx*dx + dy*dy + dz*dz
            
            if dist_sq < self.min_dist_sq:
                return

        self.last_path_pos = (curr_x, curr_y, curr_z)
        
        # 파란색 선 업데이트
        self.update_and_publish_path(curr_x, curr_y, curr_z)

    # ======================================================================
    # Setpoint Callback: 목표 지점 (Red Arrow)
    # ======================================================================
    def setpoint_callback(self, msg):
        try:
            p_x = 0.0; p_y = 0.0; p_z = 0.0; yaw_global = 0.0

            if msg.mode == 0:
                # 절대 좌표 모드
                p_x = msg.x
                p_y = msg.y
                p_z = msg.z
                yaw_global = self.wrap_angle(msg.yaw)

            elif msg.mode == 1:
                # 상대 좌표 모드: TF를 이용해 현재 auv_link 기준 변환
                # (컨트롤러가 TF를 퍼블리시하고 있으므로 TF 조회 가능)
                transform = self.tf_buffer.lookup_transform(
                    self.parent_frame,
                    self.auv_frame,
                    rospy.Time(0),
                    rospy.Duration(0.1)
                )

                t = transform.transform.translation
                q = transform.transform.rotation

                auv_x, auv_y, auv_z = t.x, t.y, t.z
                yaw_auv = self.quat_to_yaw(q)

                cos_yaw = math.cos(yaw_auv)
                sin_yaw = math.sin(yaw_auv)

                # 회전 변환 적용
                p_x = auv_x + (cos_yaw * msg.x - sin_yaw * msg.y)
                p_y = auv_y + (sin_yaw * msg.x + cos_yaw * msg.y)
                p_z = auv_z + msg.z

                yaw_global = self.wrap_angle(yaw_auv + msg.yaw)
            else:
                return

            # 빨간 화살표 발행 (목표 지점)
            self.publish_ref_arrow_marker(p_x, p_y, p_z, yaw_global)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

    # ------------------------------------------------------------------
    # 마커 생성 함수들
    # ------------------------------------------------------------------
    def update_and_publish_path(self, x, y, z):
        # 큐처럼 동작 (오래된 점 삭제)
        self.path_points.append((x, y, z))
        if len(self.path_points) > self.max_path_points:
            self.path_points.pop(0)

        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.parent_frame # odom
        marker.ns = "auv_actual_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05 # 선 두께 약간 키움
        
        # 파란색 (Actual Path)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.8 

        marker.points = [Point(px, py, pz) for (px, py, pz) in self.path_points]
        marker.lifetime = rospy.Duration(0) # 영구 지속

        self.marker_pub.publish(marker)

    def publish_ref_arrow_marker(self, x, y, z, yaw):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = self.parent_frame
        marker.ns = "ref_setpoint"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position = Point(x, y, z)
        marker.pose.orientation = self.yaw_to_quat(yaw)
        
        # 화살표 크기
        marker.scale.x = 0.6; marker.scale.y = 0.08; marker.scale.z = 0.08
        
        # 빨간색 (Target Setpoint)
        marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; marker.color.a = 1.0
        
        marker.lifetime = rospy.Duration(0.5) # 0.5초 뒤 사라짐 (갱신 없으면)
        self.marker_pub.publish(marker)

    def buoy_pose_callback(self, msg):
        self.publish_obstacle_marker(msg, "buoy", Marker.SPHERE, 0.3, 0.3, 0.3, (1.0, 1.0, 0.0), self.buoy_id_counter)
        self.buoy_id_counter += 1

    def gate_pose_callback(self, msg):
        self.publish_obstacle_marker(msg, "gate", Marker.CYLINDER, 0.2, 0.2, 1.0, (0.0, 1.0, 0.0), self.gate_id_counter)
        self.gate_id_counter += 1

    # [추가] QR 콜백
    def qr_pose_callback(self, msg):
        # x(두께)=0.1, y=0.2, z=0.2, 흰색 (1,1,1) 육면체
        self.publish_obstacle_marker(msg, "qr", Marker.CUBE, 0.1, 0.2, 0.2, (1.0, 1.0, 1.0), self.qr_id_counter)
        self.qr_id_counter += 1

    # [추가] 하이드로폰 콜백 - 빨간색 구, 직경 0.1m
    def hydro_pose_callback(self, msg):
        self.publish_obstacle_marker(msg, "hydrophone", Marker.SPHERE, 0.1, 0.1, 0.1, (1.0, 0.0, 0.0), self.hydro_id_counter)
        self.hydro_id_counter += 1

    def publish_obstacle_marker(self, msg, ns, shape, sx, sy, sz, rgb, counter):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = msg.header.frame_id if msg.header.frame_id else self.parent_frame
        marker.ns = ns
        marker.id = counter % 20 # 개수 제한
        marker.type = shape
        marker.action = Marker.ADD
        marker.pose = msg.pose
        marker.scale.x = sx; marker.scale.y = sy; marker.scale.z = sz
        marker.color.r = rgb[0]; marker.color.g = rgb[1]; marker.color.b = rgb[2]; marker.color.a = 0.8
        marker.lifetime = rospy.Duration(0)
        self.marker_pub.publish(marker)

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    try:
        node = RefPointMarkerNode()
        node.spin()
    except rospy.ROSInterruptException:
        pass