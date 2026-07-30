#!/usr/bin/env python3
import socket, json, threading, time
import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations as tft

TCP_PORT_DEF = 16171

# 공유 상태
_last = {
    "t": None,      # 수신 시각(ROS)
    "x": 0.0, "y": 0.0, "z": 0.0,
    "roll": 0.0, "pitch": 0.0, "yaw": 0.0,  # deg
    # 속도 추정(간단한 미분용)
    "vx": 0.0, "vy": 0.0, "vz": 0.0
}
_lock = threading.Lock()

def _rad(deg): return deg * 3.141592653589793 / 180.0

def recv_thread(ip, port):
    global _last
    while not rospy.is_shutdown():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((ip, port))
            f = s.makefile('r')  # 라인 단위 고속 파싱
            rospy.loginfo("DVL connected to %s:%d", ip, port)
            prev = None
            while not rospy.is_shutdown():
                line = f.readline()
                if not line:
                    raise socket.timeout("empty line / connection closed")
                data = json.loads(line)
                if data.get("type") != "position_local":
                    continue
                now = rospy.Time.now()
                x, y, z = data.get("x", 0.0), data.get("y", 0.0), data.get("z", 0.0)
                roll, pitch, yaw = data.get("roll", 0.0), data.get("pitch", 0.0), data.get("yaw", 0.0)

                # 간단 속도 추정 (샘플 간 차분)
                if prev is not None:
                    dt = (now - prev["t"]).to_sec()
                    if 0.001 < dt < 1.0:
                        vx = (x - prev["x"]) / dt
                        vy = (y - prev["y"]) / dt
                        vz = (z - prev["z"]) / dt
                    else:
                        vx = vy = vz = 0.0
                else:
                    vx = vy = vz = 0.0

                with _lock:
                    _last.update({"t": now, "x": x, "y": y, "z": z,
                                  "roll": roll, "pitch": pitch, "yaw": yaw,
                                  "vx": vx, "vy": vy, "vz": vz})
                prev = {"t": now, "x": x, "y": y, "z": z}
        except Exception as e:
            rospy.logwarn_throttle(2.0, "DVL recv reconnecting: %s", e)
            time.sleep(0.5)  # 짧은 백오프 후 재연결

def publish_thread(frame_id, pub_rate_hz, max_extrapolate_s=0.2, use_extrapolation=True):
    pub = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=30)
    rate = rospy.Rate(pub_rate_hz)
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        with _lock:
            stamp = _last["t"]
            x, y, z = _last["x"], _last["y"], _last["z"]
            roll, pitch, yaw = _last["roll"], _last["pitch"], _last["yaw"]
            vx, vy, vz = _last["vx"], _last["vy"], _last["vz"]

        # 신호 없으면 skip
        if stamp is None:
            rate.sleep(); continue

        # 필요 시 짧게 외삽(상수속도)
        dt = (now - stamp).to_sec()
        if use_extrapolation and 0.0 < dt <= max_extrapolate_s:
            x = x + vx * dt
            y = y + vy * dt
            # Z는 Depth 센서로 쓰는 게 안전 -> 기본 홀드
            # z = z + vz * dt   # 수조 환경이면 권장 비활성
        # dt가 너무 크면(연결 끊김) 외삽 금지: 마지막 값 그대로

        qx, qy, qz, qw = tft.quaternion_from_euler(_rad(roll), _rad(pitch), _rad(yaw))

        msg = PoseStamped()
        msg.header.stamp = now                # 항상 현재시간으로 퍼블리시
        msg.header.frame_id = frame_id        # "map" 권장
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        pub.publish(msg)
        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("dvl2mavros_pub_upsampled", anonymous=False)
    ip   = rospy.get_param("~ip",   "192.168.194.95")
    port = int(rospy.get_param("~port", TCP_PORT_DEF))
    frame_id = rospy.get_param("~frame_id", "dvl_link")  # ← "dvl_link" 대신 "map" 권장
    pub_hz = float(rospy.get_param("~pub_rate_hz", 15.0))
    max_extrapolate_s = float(rospy.get_param("~max_extrapolate_s", 0.2))
    use_extrapolation = bool(rospy.get_param("~use_extrapolation", True))

    threading.Thread(target=recv_thread, args=(ip, port), daemon=True).start()
    publish_thread(frame_id, pub_hz, max_extrapolate_s, use_extrapolation)
