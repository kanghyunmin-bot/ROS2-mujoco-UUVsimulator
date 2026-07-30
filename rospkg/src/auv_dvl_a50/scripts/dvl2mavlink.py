#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DVL JSON → MAVLink VISION_POSITION_DELTA(UDP 주입)
- BlueOS의 POSITION_DELTA 파이프라인과 동일 개념
- dialect/버전 불일치로 인한 AttributeError를 방지하기 위해
  ardupilotmega v2.0 다이얼렉트를 강제하고, 메서드 존재 여부를 사전 검사한다.
- TX 카운터 로그(50회마다) 추가
- (옵션) HEARTBEAT_TEST: 동일 포트로 하트비트도 주기 전송해 경로 점검 가능
"""

import os
# [중요] 다이얼렉트/프로토콜 강제
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
os.environ['MAVLINK20'] = '1'  # MAVLink2 사용 권장

import socket
import json
import threading
import time
import math
from typing import Optional

# --- (선택) ROS 의존성: 있으면 종료 플래그로 활용 ---
try:
    import rospy
    HAVE_ROS = True
except Exception:
    HAVE_ROS = False

def ros_running() -> bool:
    return (not HAVE_ROS) or (HAVE_ROS and not rospy.is_shutdown())

# --- pymavlink ---
from pymavlink import mavutil
try:
    from pymavlink.dialects.v20 import ardupilotmega as mavlink_protocol
    print("[DEBUG] dialect import OK: ardupilotmega v2.0")
except Exception as e:
    print(f"[WARN] dialect import failed: {e}")
    mavlink_protocol = None

# ----------------------------
# 설정
# ----------------------------
DVL_IP   = "192.168.194.95"
DVL_PORT = 16171

TARGET_UDP_PORT       = 14445
TARGET_CONNECTION_STR = f"udpout:127.0.0.1:{TARGET_UDP_PORT}"

SOURCE_SYSID  = 255
SOURCE_COMPID = 190

# (옵션) 하트비트 경로 점검용 테스트 발사기
HEARTBEAT_TEST = False          # True로 켜면 하트비트도 함께 전송
HEARTBEAT_RATE_HZ = 2.0

# ----------------------------
# 유틸
# ----------------------------
def _wrap_pi(a: float) -> float:
    """[-pi, pi] 래핑"""
    while a >  math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a

def _clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

# ----------------------------
# 메인 클래스
# ----------------------------
class DVLMavlinkInjector:
    def __init__(self, dvl_ip: str, dvl_port: int, target_conn: str):
        self.dvl_ip   = dvl_ip
        self.dvl_port = dvl_port
        self.target_conn = target_conn

        self.master: Optional[mavutil.mavfile] = None
        self.lock = threading.Lock()

        # 상태 버퍼
        self.last = {
            "t_pos": None, "t_vel": None, "t_vel_prev": None,
            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "prev_roll": 0.0, "prev_pitch": 0.0, "prev_yaw": 0.0,
            "vx": 0.0, "vy": 0.0, "vz": 0.0,
            "fom": 0.4, "valid": False
        }
        self.first_pose = False
        self.first_vel  = False
        self.origin_set = False

        # 송신 카운터/레이트 측정
        self.tx_count = 0
        self.tx_t0 = time.time()

    # ----------------------------
    # MAVLink 연결/검증
    # ----------------------------
    def connect_target(self) -> bool:
        try:
            print(f"[INFO] Connecting MAVLink: {self.target_conn} (sys={SOURCE_SYSID}, comp={SOURCE_COMPID})")
            self.master = mavutil.mavlink_connection(
                self.target_conn,
                source_system=SOURCE_SYSID,
                source_component=SOURCE_COMPID,
                dialect='ardupilotmega'
            )
            if not self.master:
                print("[ERROR] mavlink_connection returned None")
                return False

            if not hasattr(self.master.mav, 'vision_position_delta_send'):
                raise RuntimeError(
                    "현재 pymavlink/다이얼렉트에 'vision_position_delta_send'가 없습니다. "
                    "pip로 pymavlink를 업데이트하고, MAVLINK_DIALECT=ardupilotmega 환경을 보장하세요."
                )

            print("[INFO] MAVLink connection established & DELTA method available")
            return True
        except ConnectionRefusedError:
            print(f"[ERROR] Connection refused: Is something listening on UDP {TARGET_UDP_PORT}?")
            return False
        except Exception as e:
            print(f"[ERROR] MAVLink connection error: {e}")
            return False

    def set_ekf_origin(self) -> bool:
        """EKF Global Origin(0,0,0) 설정 시도 — 필요 시만 사용"""
        if not self.master:
            return False
        try:
            # target_system=1 가정(필요시 변경)
            self.master.mav.set_gps_global_origin_send(
                1, 0, 0, 0, 0
            )
            self.origin_set = True
            print("[INFO] SET_GPS_GLOBAL_ORIGIN sent (0,0,0)")
            return True
        except Exception as e:
            print(f"[WARN] set_ekf_origin failed: {e}")
            return False

    # ----------------------------
    # (옵션) HEARTBEAT 테스트 송신
    # ----------------------------
    def heartbeat_loop(self):
        if not self.master:
            return
        period = 1.0 / max(0.1, HEARTBEAT_RATE_HZ)
        while ros_running():
            try:
                # MAV_TYPE = 6(MAV_TYPE_GCS) 등 아무거나 무방
                self.master.mav.heartbeat_send(6, 0, 0, 0, 3)
            except Exception as e:
                print(f"[WARN] heartbeat_send failed: {e}")
            time.sleep(period)

    # ----------------------------
    # DELTA 송신
    # ----------------------------
    def send_vision_position_delta(self):
        """VISION_POSITION_DELTA 전송(vel·att 차분 기반)"""
        if not self.master or not self.origin_set:
            return

        with self.lock:
            if not self.last["valid"] or self.last["t_vel"] is None or self.last["t_vel_prev"] is None:
                return

            t_vel      = self.last["t_vel"]
            t_vel_prev = self.last["t_vel_prev"]
            vx, vy, vz = float(self.last["vx"]), float(self.last["vy"]), float(self.last["vz"])

            roll, pitch, yaw = float(self.last["roll"]), float(self.last["pitch"]), float(self.last["yaw"])
            prev_roll, prev_pitch, prev_yaw = float(self.last["prev_roll"]), float(self.last["prev_pitch"]), float(self.last["prev_yaw"])
            fom = float(self.last["fom"])

        time_delta_usec = int((t_vel - t_vel_prev) * 1e6)
        # 1ms < dt < 1s 범위만 인정
        if time_delta_usec <= 1000 or time_delta_usec > 1_000_000:
            return

        dt = time_delta_usec / 1e6

        # 위치 델타(필요시 ENU→NED 변환/부호 보정)
        position_delta = [vx * dt, vy * dt, vz * dt]

        # 각도 델타(라디안), yaw는 래핑
        dRoll  = math.radians(roll  - prev_roll)
        dPitch = math.radians(pitch - prev_pitch)
        dyaw_deg = yaw - prev_yaw
        if   dyaw_deg >  180.0: dyaw_deg -= 360.0
        elif dyaw_deg <= -180.0: dyaw_deg += 360.0
        dYaw = math.radians(dyaw_deg)
        attitude_delta = [dRoll, dPitch, dYaw]

        # 신뢰도(0~100) — fom(0~0.4) 작을수록 신뢰 높음
        confidence = 100.0 * (1.0 - _clamp(fom, 0.0, 0.4) / 0.4)

        try:
            usec = int(time.time() * 1e6)  # 현재 UTC usec
            self.master.mav.vision_position_delta_send(
                int(usec),                        # time_usec (uint64)
                int(time_delta_usec),             # time_delta_us (uint32)
                list(map(float, attitude_delta)), # angle_delta[3] (float[3])
                list(map(float, position_delta)), # position_delta[3] (float[3])
                float(confidence)                 # confidence (0..100)
            )
            # --- TX 카운터/레이트 로그 ---
            self.tx_count += 1
            if self.tx_count % 50 == 0:
                t_now = time.time()
                rate = self.tx_count / max(1e-3, (t_now - self.tx_t0))
                print(f"[TX] DELTA sent={self.tx_count}, avg_rate={rate:.1f} Hz")
        except Exception as e:
            print(f"[ERROR] send VISION_POSITION_DELTA failed: {e}")

    # ----------------------------
    # DVL 수신 스레드
    # ----------------------------
    def dvl_recv_loop(self):
        print("[INFO] DVL recv loop starting...")
        while ros_running():
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(3.0)
                s.connect((self.dvl_ip, self.dvl_port))
                f = s.makefile('r')
                print(f"[INFO] DVL connected: {self.dvl_ip}:{self.dvl_port}")

                self.first_pose = False
                self.first_vel  = False

                while ros_running():
                    line = f.readline()
                    if not line:
                        raise socket.timeout("DVL connection closed")
                    data = json.loads(line)
                    now  = time.time()
                    dtyp = data.get("type")

                    if dtyp == "position_local":
                        with self.lock:
                            if self.last["t_pos"] is not None:
                                self.last["prev_roll"]  = self.last["roll"]
                                self.last["prev_pitch"] = self.last["pitch"]
                                self.last["prev_yaw"]   = self.last["yaw"]
                            self.last["t_pos"] = now
                            self.last["roll"]  = data.get("roll",  0.0)
                            self.last["pitch"] = data.get("pitch", 0.0)
                            self.last["yaw"]   = data.get("yaw",   0.0)
                            if not self.first_pose:
                                print("[INFO] First 'position_local' received (attitude)")
                                self.first_pose = True

                    elif dtyp == "velocity":
                        with self.lock:
                            valid = bool(data.get("velocity_valid", False))
                            if valid and self.last["t_vel"] is not None:
                                self.last["t_vel_prev"] = self.last["t_vel"]
                            self.last["t_vel"]  = now
                            self.last["valid"]  = valid
                            if valid:
                                self.last["vx"]  = data.get("vx",  0.0)
                                self.last["vy"]  = data.get("vy",  0.0)
                                self.last["vz"]  = data.get("vz",  0.0)
                                self.last["fom"] = data.get("fom", 0.4)
                                if not self.first_vel:
                                    print("[INFO] First 'velocity' received")
                                    self.first_vel = True
                            else:
                                self.last["vx"] = self.last["vy"] = self.last["vz"] = 0.0
                                self.last["fom"] = 0.4

            except Exception as e:
                print(f"[ERROR] DVL recv error: {e} — reconnecting in 1s")
                try:
                    if 'f' in locals() and f: f.close()
                    if 's' in locals() and s: s.close()
                except Exception:
                    pass
                time.sleep(1.0)

    # ----------------------------
    # 실행
    # ----------------------------
    def start(self):
        print(f"[INFO] TARGET: {self.target_conn}")
        if not self.connect_target():
            print("[ERROR] MAVLink connection failed. Abort.")
            return

        # DVL 수신 스레드
        threading.Thread(target=self.dvl_recv_loop, daemon=True).start()

        # (옵션) 하트비트 테스트
        if HEARTBEAT_TEST:
            threading.Thread(target=self.heartbeat_loop, daemon=True).start()
            print(f"[INFO] HEARTBEAT TEST ON ({HEARTBEAT_RATE_HZ} Hz)")

        # (선택) EKF Origin 세팅 — 필요 없으면 건너뛰어도 됨
        print("[INFO] Waiting 3s before origin set...")
        time.sleep(3)
        attempts = 0
        while attempts < 3 and not self.origin_set and ros_running():
            attempts += 1
            print(f"[INFO] set_ekf_origin attempt {attempts}/3")
            if self.set_ekf_origin():
                break
            time.sleep(2)

        if not self.origin_set:
            print("[WARN] EKF origin not set; continuing (DELTA는 동작 가능)")

        # DELTA 전송 루프
        rate_hz = 15.0
        print("[INFO] DELTA mode publishing started")
        try:
            while ros_running():
                self.send_vision_position_delta()
                time.sleep(1.0 / rate_hz)
        except KeyboardInterrupt:
            print("\n[INFO] KeyboardInterrupt — stopping")
        except Exception as e:
            print(f"[ERROR] main loop error: {e}")

        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
        print("[INFO] Injector finished.")

# ----------------------------
# Main
# ----------------------------
if __name__ == "__main__":
    print("[INFO] Starting DVL MAVLink Injector (DELTA Mode)")
    print("[INFO] == POSHOLD REQUIRES ArduSub Params ==")
    print("[INFO] VISO_TYPE: 1 (Vision)")
    print("[INFO] EK3_SRC1_POSXY: 6 (ExternalNav)")
    print("[INFO] EK3_SRC1_VELXY: 6 (ExternalNav)")
    print("[INFO] EK3_SRC1_YAW : 6 (ExternalNav) or MAG)")
    print("[INFO] EK3_SRC1_POSZ: 1 (Depth/Baro only)")

    if HAVE_ROS and not rospy.core.is_initialized():
        rospy.init_node("dvl_delta_injector", anonymous=True, disable_signals=True)

    injector = DVLMavlinkInjector(
        dvl_ip=DVL_IP,
        dvl_port=DVL_PORT,
        target_conn=TARGET_CONNECTION_STR
    )
    injector.start()
