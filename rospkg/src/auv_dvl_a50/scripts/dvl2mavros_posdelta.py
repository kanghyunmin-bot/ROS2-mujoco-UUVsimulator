#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import json
import threading
import time
import math

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3

# 1. 통합 메시지 import
from waterlinked_a50_ros_driver.msg import DvlIntegratedData

TCP_PORT_DEF = 16171

# ===== 공유 상태 =====
_lock = threading.Lock()
_latest_velocity_data = {}
_latest_position_data = {}
_last_vel_stamp = None
_last_pose_stamp = None

# 유효기간(aging) 가드: 필요시 파라미터로 조정 가능
VEL_MAX_AGE = rospy.Duration(0.15)   # 150 ms 이내(속도는 필수)
POSE_MAX_AGE = rospy.Duration(0.50)  # 500 ms 이내(자세/위치는 선택)

# ===== 유틸 =====
UINT32_MAX = 0xFFFFFFFF

def _rad(deg):
    try:
        return float(deg) * 3.141592653589793 / 180.0
    except Exception:
        return 0.0

def _to_uint32(x) -> int:
    try:
        v = float(x)
    except Exception:
        return 0
    if not math.isfinite(v):
        return 0
    if v < 0:
        v = 0
    v = int(v)  # 소수 → 버림(필요시 round 교체)
    if v > UINT32_MAX:
        v = UINT32_MAX
    return v

def _to_float(x) -> float:
    try:
        v = float(x)
        return v if math.isfinite(v) else 0.0
    except Exception:
        return 0.0

def _to_bool(x) -> bool:
    if isinstance(x, bool):
        return x
    if isinstance(x, str):
        return x.strip().lower() in ("1", "true", "t", "yes", "y", "on")
    try:
        return bool(int(x))
    except Exception:
        return False

# ===== 퍼블리시 =====
def publish_integrated_data(publisher, frame_id, reason=""):
    """최신 속도/자세 캐시를 사용하여 통합 메시지 발행"""
    global _last_vel_stamp, _last_pose_stamp

    now = rospy.Time.now()
    try:
        with _lock:
            vel_data = _latest_velocity_data.copy()
            pos_data = _latest_position_data.copy()
            t_vel = _last_vel_stamp
            t_pos = _last_pose_stamp

        # velocity는 필수 + 유효기간 체크
        if not vel_data or t_vel is None or (now - t_vel) > VEL_MAX_AGE:
            return

        msg = DvlIntegratedData()
        msg.header.stamp = now
        msg.header.frame_id = frame_id

        # Velocity (필수)
        msg.velocity.x = _to_float(vel_data.get("vx", 0.0))
        msg.velocity.y = _to_float(vel_data.get("vy", 0.0))
        msg.velocity.z = _to_float(vel_data.get("vz", 0.0))
        msg.time_ms    = _to_uint32(vel_data.get("time", 0))
        msg.fom        = _to_float(vel_data.get("fom", 99.0))
        msg.velocity_valid = _to_bool(vel_data.get("velocity_valid", False))
        msg.altitude   = _to_float(vel_data.get("altitude", 0.0))

        # Pose (선택: 유효기간 내면 포함, 아니면 0으로 채움)
        pose_fresh = (pos_data and t_pos is not None and (now - t_pos) <= POSE_MAX_AGE)
        if pose_fresh:
            msg.roll_rad  = _to_float(pos_data.get("roll_rad", 0.0))
            msg.pitch_rad = _to_float(pos_data.get("pitch_rad", 0.0))
            msg.yaw_rad   = _to_float(pos_data.get("yaw_rad", 0.0))
            msg.x = _to_float(pos_data.get("x", 0.0))
            msg.y = _to_float(pos_data.get("y", 0.0))
            msg.z = _to_float(pos_data.get("z", 0.0))
        else:
            msg.roll_rad = msg.pitch_rad = msg.yaw_rad = 0.0
            msg.x = msg.y = msg.z = 0.0
            # 과다 로그 방지
            rospy.logwarn_throttle(5.0, "[DVL] pose is stale, publishing velocity-only")

        publisher.publish(msg)
        rospy.loginfo_throttle(
            2.0,
            f"[DVL] published integrated_data ({reason}) "
            f"vel_ok=1 pose_ok={int(pose_fresh)} "
            f"vx={msg.velocity.x:.3f} vy={msg.velocity.y:.3f} vz={msg.velocity.z:.3f}"
        )
    except Exception as e:
        # 퍼블리시 중 예외는 연결을 끊지 말고 흡수
        rospy.logwarn_throttle(2.0, f"[DVL] publish_integrated_data error: {e}")

# ===== 수신 스레드 =====
def recv_thread(ip, port, publisher, frame_id):
    """
    DVL에서 TCP JSON 라인 스트림을 수신 → 캐시 갱신 → 발행 트리거
    라인 파서 보강: \r\n, 조각 JSON 처리
    """
    global _latest_velocity_data, _latest_position_data, _last_vel_stamp, _last_pose_stamp

    tcp_buffer = ""

    while not rospy.is_shutdown():
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3.0)
            s.connect((ip, port))
            rospy.loginfo("DVL connected to %s:%d", ip, port)

            while not rospy.is_shutdown():
                raw = s.recv(1024)
                if not raw:
                    raise socket.timeout("Empty recv / connection closed")

                # 안전 디코딩
                try:
                    chunk = raw.decode('utf-8', errors='ignore')
                except Exception as de:
                    rospy.logwarn_throttle(2.0, f"DVL decode error: {de}")
                    continue

                tcp_buffer += chunk

                # \n 단위로 라인 분리, \r 제거
                while '\n' in tcp_buffer:
                    line, tcp_buffer = tcp_buffer.split('\n', 1)
                    line = line.strip('\r')
                    if not line:
                        continue

                    # JSON 파싱 (조각 방어)
                    try:
                        data = json.loads(line)
                    except json.JSONDecodeError:
                        # 불완전 조각 가능: 다음 recv와 결합
                        tcp_buffer = line + "\n" + tcp_buffer
                        break
                    except Exception as je:
                        rospy.logwarn_throttle(2.0, f"DVL json error: {je}")
                        continue

                    data_type = data.get("type")

                    if data_type == "position_local":
                        with _lock:
                            _latest_position_data = {
                                "roll_rad": _rad(data.get("roll", 0.0)),
                                "pitch_rad": _rad(data.get("pitch", 0.0)),
                                "yaw_rad": _rad(data.get("yaw", 0.0)),
                                "x": _to_float(data.get("x", 0.0)),
                                "y": _to_float(data.get("y", 0.0)),
                                "z": _to_float(data.get("z", 0.0)),
                            }
                            _last_pose_stamp = rospy.Time.now()
                        publish_integrated_data(publisher, frame_id, reason="pos")

                    elif data_type == "velocity":
                        with _lock:
                            _latest_velocity_data = {
                                "vx": _to_float(data.get("vx", 0.0)),
                                "vy": _to_float(data.get("vy", 0.0)),
                                "vz": _to_float(data.get("vz", 0.0)),
                                "time": data.get("time", 0),  # 원값 보존(캐스팅은 publish 시점)
                                "fom": _to_float(data.get("fom", 99.0)),
                                "velocity_valid": _to_bool(data.get("velocity_valid", False)),
                                "altitude": _to_float(data.get("altitude", 0.0)),
                            }
                            _last_vel_stamp = rospy.Time.now()
                        publish_integrated_data(publisher, frame_id, reason="vel")

                    else:
                        # 다른 타입은 무시(필요시 로그 하향)
                        rospy.logdebug_throttle(5.0, f"DVL unknown type: {data_type}")

        except Exception as e:
            rospy.logwarn_throttle(2.0, f"DVL recv thread error: {e}. Reconnecting...")
            time.sleep(1.0)
        finally:
            try:
                if s:
                    s.close()
            except Exception:
                pass

# ===== 메인 =====
if __name__ == "__main__":
    rospy.init_node("dvl_parser_node", anonymous=False)

    ip = rospy.get_param("~ip", "192.168.194.95")
    port = int(rospy.get_param("~port", TCP_PORT_DEF))
    frame_id = rospy.get_param("~frame_id", "dvl_link")

    # 유효기간 파라미터를 노드 파라미터로 덮어쓸 수 있게 지원(옵션)
    try:
        vel_age_ms = rospy.get_param("~vel_max_age_ms", 150)
        pose_age_ms = rospy.get_param("~pose_max_age_ms", 500)
        VEL_MAX_AGE = rospy.Duration(vel_age_ms / 1000.0)
        POSE_MAX_AGE = rospy.Duration(pose_age_ms / 1000.0)
    except Exception:
        pass

    # 통합 메시지 퍼블리셔
    pub_integrated = rospy.Publisher('/dvl/integrated_data', DvlIntegratedData, queue_size=10)

    # 수신 스레드 시작
    thread = threading.Thread(
        target=recv_thread,
        args=(ip, port, pub_integrated, frame_id),
        daemon=True
    )
    thread.start()

    rospy.spin()
