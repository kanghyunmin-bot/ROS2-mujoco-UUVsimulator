#!/usr/bin/env python3
import socket, json, threading, time
import rospy
from geometry_msgs.msg import TwistStamped  # <-- PoseStamped 대신 TwistStamped 사용

TCP_PORT_DEF = 16171

# 공유 상태 (속도 정보만 저장)
_last_velocity = {
    "t": None,      # 수신 시각(ROS)
    "vx": 0.0, "vy": 0.0, "vz": 0.0,
    "valid": False  # DVL이 속도가 유효하다고 보고했는지
}
_lock = threading.Lock()

def recv_thread(ip, port):
    """
    DVL에서 TCP로 JSON 데이터를 수신하는 스레드
    'velocity' 타입 메시지만 파싱하여 공유 변수(_last_velocity)를 업데이트합니다.
    """
    global _last_velocity
    while not rospy.is_shutdown():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((ip, port))
            f = s.makefile('r')  # 라인 단위 고속 파싱
            rospy.loginfo("DVL connected to %s:%d for velocity", ip, port)

            while not rospy.is_shutdown():
                line = f.readline()
                if not line:
                    raise socket.timeout("empty line / connection closed")
                
                data = json.loads(line)

                # --- 로직 변경: 'velocity' 타입만 파싱 ---
                # publisher.py 참고
                if data.get("type") != "velocity":
                    continue

                now = rospy.Time.now()
                vx = data.get("vx", 0.0)
                vy = data.get("vy", 0.0)
                vz = data.get("vz", 0.0)
                valid = data.get("velocity_valid", False) # <-- 유효성 플래그
                
                with _lock:
                    _last_velocity.update({
                        "t": now,
                        "vx": vx, "vy": vy, "vz": vz,
                        "valid": valid
                    })
                # --- 로직 변경 완료 ---

        except Exception as e:
            rospy.logwarn_throttle(2.0, "DVL recv reconnecting: %s", e)
            time.sleep(0.5)  # 짧은 백오프 후 재연결

def publish_thread(frame_id, pub_rate_hz, data_timeout_s):
    """
    공유 변수(_last_velocity)의 값을 읽어
    /mavros/vision_speed/speed 토픽으로 TwistStamped 메시지를 발행하는 스레드
    """
    
    # --- 로직 변경: 토픽 이름 및 메시지 타입 ---
    pub = rospy.Publisher('/mavros/vision_speed/speed', TwistStamped, queue_size=10)
    rate = rospy.Rate(pub_rate_hz)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        with _lock:
            stamp = _last_velocity["t"]
            vx = _last_velocity["vx"]
            vy = _last_velocity["vy"]
            vz = _last_velocity["vz"]
            valid = _last_velocity["valid"]
        # --- 로직 변경 완료 ---

        # 1. 수신된 데이터가 없으면 skip
        if stamp is None:
            rate.sleep(); continue

        # 2. DVL이 데이터가 유효하지 않다고 하면 skip
        if not valid:
            rospy.logwarn_throttle(5.0, "DVL velocity data is invalid, not publishing.")
            rate.sleep(); continue

        # 3. 데이터가 너무 오래됐으면(연결 끊김) skip
        dt = (now - stamp).to_sec()
        if dt > data_timeout_s:
            rospy.logwarn_throttle(2.0, "DVL velocity data is stale (%.2f s old), not publishing.", dt)
            rate.sleep(); continue

        # --- 로직 변경: TwistStamped 메시지 생성 ---
        # 외삽(extrapolation) 로직 불필요 (속도 자체를 발행)
        
        msg = TwistStamped()
        msg.header.stamp = now                # MAVROS는 현재 시간을 선호
        msg.header.frame_id = frame_id        # DVL의 body-fixed frame ("dvl_link" 등)
        
        # DVL에서 받은 선형 속도
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        
        # DVL은 각속도를 제공하지 않으므로 0으로 설정
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        # --- 로직 변경 완료 ---

        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("dvl2mavros_speed", anonymous=False) # <-- 노드 이름 변경
    
    # ROS 파라미터 읽기
    ip   = rospy.get_param("~ip",   "192.168.194.95")
    port = int(rospy.get_param("~port", TCP_PORT_DEF))
    
    # frame_id: DVL의 속도 기준 프레임 (일반적으로 "dvl_link" 또는 "base_link")
    frame_id = rospy.get_param("~frame_id", "dvl_link")
    
    # 발행 주기 (DVL 수신 주기보다 높게 설정 가능)
    pub_hz = float(rospy.get_param("~pub_rate_hz", 20.0))
    
    # 데이터 타임아웃 (이 시간(초)보다 오래된 데이터는 발행 안 함)
    data_timeout_s = float(rospy.get_param("~data_timeout_s", 1.0))

    # --- 외삽(extrapolation) 파라미터 제거됨 ---

    # 수신 및 발행 스레드 시작
    threading.Thread(target=recv_thread, args=(ip, port), daemon=True).start()
    publish_thread(frame_id, pub_hz, data_timeout_s)