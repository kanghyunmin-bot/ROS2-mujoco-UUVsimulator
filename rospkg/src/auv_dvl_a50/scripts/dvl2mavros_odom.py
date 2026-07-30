#!/usr/bin/env python3
"""
DVL(Water Linked) 데이터를 수신하여 /mavros/odometry/out 토픽으로 발행하는
Odometry 전용 ROS 노드입니다. (POSHOLD 구현 목표)

작동 방식:
1. EKF 원점 초기화: MAVROS가 준비되면 /mavros/global_position/set_gp_origin 발행 (1회)
2. DVL 데이터 수신: 'position_local'과 'velocity'를 비동기(병렬)로 수신하여 내부 변수(_last) 갱신
3. Odometry 발행: 고정된 주기(pub_rate_hz)로 _last의 데이터를 Odometry 메시지로 발행
"""

import socket
import json
import threading
import time
import rospy
import tf.transformations as tft
from geometry_msgs.msg import Point, Quaternion, PoseWithCovariance, TwistWithCovariance
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPointStamped

TCP_PORT_DEF = 16171

# DVL에서 수신한 마지막 데이터 (스레드 간 공유)
_last = {
    "t_pos": None,    # 'position_local' 수신 시각 (ROS)
    "t_vel": None,    # 'velocity' 수신 시각 (ROS)
    "x": 0.0, "y": 0.0, "z": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,  # deg
    "vx": 0.0, "vy": 0.0, "vz": 0.0
}
_lock = threading.Lock() # _last 변수 접근 제어를 위한 잠금

# EKF가 DVL 데이터를 신뢰하도록 정적 공분산 행렬 정의
# (DVL 프로토콜의 covariance 배열은 0으로 채워져 있어 사용 불가)

# 위치 공분산: DVL의 적분된 위치는 드리프트가 있으므로 속도보다 불확실성을 높게 설정
# (x, y, z, roll, pitch, yaw)
# Z(수심)는 어차피 Baro/수심계를 사용하므로 매우 높은 값(100.0)을 부여
POS_COVARIANCE = [
    0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 100.0, 0, 0, 0,
    0, 0, 0, 0.01, 0, 0,
    0, 0, 0, 0, 0.01, 0,
    0, 0, 0, 0, 0, 0.01
]
# 속도 공분산: DVL의 핵심 측정값이므로 신뢰도를 높게(불확실성을 낮게) 설정
# (vx, vy, vz, vroll, vpitch, vyaw)
VEL_COVARIANCE = [
    0.01, 0, 0, 0, 0, 0,
    0, 0.01, 0, 0, 0, 0,
    0, 0, 0.01, 0, 0, 0,
    0, 0, 0, 0.001, 0, 0,
    0, 0, 0, 0, 0.001, 0,
    0, 0, 0, 0, 0, 0.001
]

def _rad(deg):
    """도를 라디안으로 변환"""
    return deg * 3.141592653589793 / 180.0

def recv_thread(ip, port):
    """
    DVL에서 'position_local'과 'velocity' 메시지를 비동기(병렬)로 수신하여
    _last 딕셔너리를 갱신하는 스레드
    """
    global _last
    while not rospy.is_shutdown():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((ip, port))
            f = s.makefile('r') # makefile을 사용하여 라인 단위로 효율적 파싱
            rospy.loginfo("DVL connected to %s:%d", ip, port)
            
            while not rospy.is_shutdown():
                line = f.readline()
                if not line:
                    raise socket.timeout("DVL connection closed")
                
                data = json.loads(line)
                now = rospy.Time.now() # JSON 수신 시각

                data_type = data.get("type")

                # --- 1. 위치 데이터 처리 (약 5Hz) ---
                if data_type == "position_local":
                    x, y, z = data.get("x", 0.0), data.get("y", 0.0), data.get("z", 0.0)
                    roll, pitch, yaw = data.get("roll", 0.0), data.get("pitch", 0.0), data.get("yaw", 0.0)
                    
                    with _lock:
                        _last["t_pos"] = now # 위치 타임스탬프 갱신
                        _last["x"] = x
                        _last["y"] = y
                        _last["z"] = z
                        _last["roll"] = roll
                        _last["pitch"] = pitch
                        _last["yaw"] = yaw
                
                # --- 2. 속도 데이터 처리 (약 12Hz) ---
                elif data_type == "velocity":
                    with _lock:
                        _last["t_vel"] = now # 속도 타임스탬프 갱신
                        
                        if data.get("velocity_valid", False):
                            _last["vx"] = data.get("vx", 0.0) # DVL (FRD)
                            _last["vy"] = data.get("vy", 0.0) # DVL (FRD)
                            _last["vz"] = data.get("vz", 0.0) # DVL (FRD)
                        else:
                            # 속도가 유효하지 않으면 0으로 설정
                            _last["vx"] = 0.0
                            _last["vy"] = 0.0
                            _last["vz"] = 0.0
                
        except Exception as e:
            rospy.logwarn_throttle(2.0, "DVL recv thread error: %s. Reconnecting...", e)
            if 'f' in locals():
                f.close()
            if 's' in locals():
                s.close()
            time.sleep(1.0)  # 1초 후 재연결

def set_ekf_origin_thread(lat, lon, alt, event):
    """
    MAVROS를 통해 EKF 글로벌 원점을 1회 설정하는 스레드.
    완료되면 'event'를 set하여 Odometry 발행 스레드를 시작시킴.
    """
    pub = rospy.Publisher('/mavros/global_position/set_gp_origin', GeoPointStamped, queue_size=1, latch=True)
    rospy.loginfo("[OriginSetter] EKF Origin Setter Thread started, waiting for MAVROS...")
    
    # MAVROS가 /set_gp_origin 토픽을 구독할 때까지 (즉, MAVROS가 준비될 때까지) 대기
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("[OriginSetter] Waiting for MAVROS subscriber...")
        rospy.sleep(1.0) 
    
    if rospy.is_shutdown():
        rospy.loginfo("[OriginSetter] Shutdown requested.")
        return

    # MAVROS가 준비되었으므로 원점 설정 메시지 발행
    msg = GeoPointStamped()
    msg.header.stamp = rospy.Time.now()
    msg.position.latitude = lat
    msg.position.longitude = lon
    msg.position.altitude = alt
    
    pub.publish(msg)
    rospy.loginfo_once("[OriginSetter] Published EKF Global Origin set to: Lat=%f, Lon=%f, Alt=%f", lat, lon, alt)
    
    rospy.sleep(1.0) # 발행 보장을 위한 잠시 대기
    pub.unregister() 
    rospy.loginfo("[OriginSetter] EKF Origin Setter Thread finished.")
    
    # Odometry 발행 스레드를 시작하도록 이벤트 신호 전송
    event.set()

def publish_odometry_thread(map_frame_id, body_frame_id, pub_rate_hz, event):
    """
    [POSHOLD 권장] EKF 원점 설정(event)을 기다린 후,
    /mavros/odometry/out 토픽으로 Odometry 메시지를 발행하는 메인 루프
    """
    # EKF 원점이 설정될 때까지 대기
    rospy.loginfo("[OdomPublisher] Waiting for EKF origin to be set...")
    event.wait() # set_ekf_origin_thread가 event.set()을 호출할 때까지 대기
    rospy.loginfo("[OdomPublisher] EKF origin is set. Starting Odometry publisher loop.")
    
    # [수정] MAVROS가 구독 중인 /mavros/odometry/out 토픽으로 발행합니다.
    pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=10)
    rospy.logwarn("[OdomPublisher] Publishing to /mavros/odometry/out (This is the correct INPUT topic for your MAVROS setup)")
    
    rate = rospy.Rate(pub_rate_hz)
    
    rospy.loginfo("[OdomPublisher] == POSHOLD REQUIRES ArduSub Params ==")
    rospy.loginfo("[OdomPublisher] VISO_TYPE: 0 (Odometry)")
    rospy.loginfo("[OdomPublisher] EK3_SRC1_VELXY: 7 (Odometry)")
    rospy.loginfo("[OdomPublisher] EK3_SRC1_POSXY: 0 (None) or 7 (Odometry)")
    rospy.loginfo("[OdomPublisher] ========================================")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        with _lock:
            # DVL로부터 수신한 최신 데이터 복사
            stamp_pos = _last["t_pos"]
            stamp_vel = _last["t_vel"]
            x, y, z = _last["x"], _last["y"], _last["z"]
            roll, pitch, yaw = _last["roll"], _last["pitch"], _last["yaw"]
            vx_dvl = _last["vx"]
            vy_dvl = _last["vy"]
            vz_dvl = _last["vz"] 

        # 위치와 속도 데이터가 최소 1회 이상 수신되었는지 확인
        if stamp_pos is None or stamp_vel is None:
            rospy.logwarn_throttle(5.0, "[OdomPublisher] Waiting for first 'position_local' and 'velocity' data...")
            rate.sleep(); 
            continue
        
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = map_frame_id   # EKF 원점 기준 좌표계 (예: "map" 또는 "odom")
        msg.child_frame_id = body_frame_id # 기체 기준 좌표계 (예: "base_link")
        
        # --- 1. Pose (위치/자세) 정보 ---
        # DVL의 (x, y, z)와 (roll, pitch, yaw)를 변환
        q = tft.quaternion_from_euler(_rad(roll), _rad(pitch), _rad(yaw))
        msg.pose.pose.position = Point(x, y, z)
        msg.pose.pose.orientation = Quaternion(*q)
        msg.pose.covariance = POS_COVARIANCE # 정적으로 정의된 위치 공분산
        
        # --- 2. Twist (속도) 정보 ---
        # DVL 속도(FRD)를 MAVROS(FLU) 좌표계로 변환하여 '기체 기준'으로 발행
        msg.twist.twist.linear.x = vx_dvl   # DVL Fwd(x) -> ROS Fwd(x)
        msg.twist.twist.linear.y = -vy_dvl  # DVL Right(y) -> ROS Left(y)
        msg.twist.twist.linear.z = -vz_dvl  # DVL Down(z) -> ROS Up(z)
        # DVL은 각속도를 제공하지 않으므로 0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        msg.twist.covariance = VEL_COVARIANCE # 정적으로 정의된 속도 공분산
        
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("dvl_odom_publisher", anonymous=False)

    # 파라미터 읽기
    ip = rospy.get_param("~ip", "192.168.194.95")
    port = int(rospy.get_param("~port", TCP_PORT_DEF))
    pub_hz = float(rospy.get_param("~pub_rate_hz", 20.0)) # DVL 속도(12Hz)보다 약간 높게 설정
    map_frame = rospy.get_param("~map_frame_id", "odom_ned")
    body_frame = rospy.get_param("~body_frame_id", "base_link")

    # 스레드간 동기화를 위한 Event (원점 설정 -> 발행 시작)
    ekf_origin_set_event = threading.Event()

    # --- 1. DVL 수신 스레드 시작 (데몬) ---
    threading.Thread(target=recv_thread, args=(ip, port), daemon=True).start()

    # --- 2. EKF 원점 초기화 스레드 시작 (데몬) ---
    threading.Thread(target=set_ekf_origin_thread, 
                     args=(0.0, 0.0, 0.0, ekf_origin_set_event), 
                     daemon=True).start()

    # --- 3. Odometry 발행 스레드 (메인 스레드에서 실행) ---
    try:
        # [수정] 다른 모드(pose, speed)는 제거하고 Odometry 발행만 남김
        publish_odometry_thread(map_frame, body_frame, pub_hz, ekf_origin_set_event)
            
    except rospy.ROSInterruptException:
        pass
    rospy.loginfo("dvl_odom_publisher node finished.")

