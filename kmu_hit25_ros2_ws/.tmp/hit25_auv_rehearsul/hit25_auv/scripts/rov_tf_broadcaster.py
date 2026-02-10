#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os
import rospkg
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

# --- 1. 모듈 경로 설정 및 임포트 (main_node.py 방식 적용) ---
# map_odom_broadcaster.py를 클래스로 불러오기 위한 경로 설정
try:
    package_dir = rospkg.RosPack().get_path("hit25_auv")
    scripts_dir = os.path.join(package_dir, "scripts")
    if scripts_dir not in sys.path:
        sys.path.insert(0, scripts_dir)
    
    # [핵심] 런치 파일 실행이 아닌, 클래스 직접 임포트
    from map_odom_broadcaster import MapOdomBroadcaster

except ImportError as e:
    rospy.logfatal(f"필수 모듈 임포트 실패: {e}")
    sys.exit(1)

# DVL 메시지 임포트
try:
    from waterlinked_a50_ros_driver.msg import DvlIntegratedData
except ImportError:
    rospy.logwarn("DVL driver not found. Using placeholder.")
    from std_msgs.msg import Header
    class DvlIntegratedData:
        header = Header()
        x=0.0; y=0.0; z=0.0; yaw_rad=0.0

class RovTfBroadcaster(object):
    """
    ROV의 모든 TF를 총괄하는 노드입니다.
    1. MapOdomBroadcaster 클래스를 내장하여 map -> odom TF 방송
    2. DVL 데이터를 받아 odom -> base_link -> dvl TF 방송
    """
    def __init__(self):
        rospy.init_node("rov_tf_broadcaster")
        rospy.loginfo("[RovTf] Initializing ROV TF Manager...")

        # --- 2. Map -> Odom TF 초기화 (클래스 사용) ---
        # MapOdomBroadcaster는 초기화 시 자신의 파라미터(~map_frame 등)를 로드하고
        # 내부 타이머를 통해 주기적으로 TF를 방송합니다. (main_node.py와 동일)
        try:
            self.map_odom = MapOdomBroadcaster()
            rospy.loginfo("[RovTf] MapOdomBroadcaster module loaded successfully.")
        except Exception as e:
            rospy.logerr(f"[RovTf] Failed to load MapOdomBroadcaster: {e}")

        # --- 3. Odom -> Base_link -> DVL TF 설정 ---
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "auv_link")
        self.dvl_frame  = rospy.get_param("~dvl_frame", "dvl")

        # DVL 센서 오프셋
        self.dvl_x = rospy.get_param("~dvl_x", 0.0)
        self.dvl_y = rospy.get_param("~dvl_y", 0.0)
        self.dvl_z = rospy.get_param("~dvl_z", 0.0)
        self.dvl_r = rospy.get_param("~dvl_roll", 0.0)
        self.dvl_p = rospy.get_param("~dvl_pitch", 0.0)
        self.dvl_yaw_offset = rospy.get_param("~dvl_yaw", 0.0)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # DVL 구독
        self.dvl_sub = rospy.Subscriber(
            "/dvl/integrated_data", DvlIntegratedData, self.dvl_callback, queue_size=10
        )

        rospy.loginfo(f"[RovTf] Ready. Managing Full TF Tree.")

    def dvl_callback(self, msg):
        # 메시지 시간 사용
        if msg.header.stamp.to_sec() > 0:
            current_time = msg.header.stamp
        else:
            current_time = rospy.Time.now()

        # --- A. odom -> base_link (Dynamic) ---
        # controller.py 로직 준수 (Y, Z 반전)
        pos_x = msg.x
        pos_y = -msg.y
        pos_z = -msg.z
        yaw   = -msg.yaw_rad

        t_base = TransformStamped()
        t_base.header.stamp = current_time
        t_base.header.frame_id = self.odom_frame
        t_base.child_frame_id = self.base_frame

        t_base.transform.translation.x = pos_x
        t_base.transform.translation.y = pos_y
        t_base.transform.translation.z = pos_z

        q = tft.quaternion_from_euler(0, 0, yaw)
        t_base.transform.rotation.x = q[0]
        t_base.transform.rotation.y = q[1]
        t_base.transform.rotation.z = q[2]
        t_base.transform.rotation.w = q[3]

        # --- B. base_link -> dvl (Static) ---
        t_dvl = TransformStamped()
        t_dvl.header.stamp = current_time
        t_dvl.header.frame_id = self.base_frame
        t_dvl.child_frame_id = self.dvl_frame

        t_dvl.transform.translation.x = self.dvl_x
        t_dvl.transform.translation.y = self.dvl_y
        t_dvl.transform.translation.z = self.dvl_z

        q_dvl = tft.quaternion_from_euler(self.dvl_r, self.dvl_p, self.dvl_yaw_offset)
        t_dvl.transform.rotation.x = q_dvl[0]
        t_dvl.transform.rotation.y = q_dvl[1]
        t_dvl.transform.rotation.z = q_dvl[2]
        t_dvl.transform.rotation.w = q_dvl[3]

        # TF 발행
        self.tf_broadcaster.sendTransform([t_base, t_dvl])

if __name__ == "__main__":
    try:
        RovTfBroadcaster()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass