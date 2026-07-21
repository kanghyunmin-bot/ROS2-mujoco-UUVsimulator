#!/usr/bin/python3
"""
DroneCAN → ROS2 Battery Bridge (v2)

ROS1 hit25_auv/scripts/dronecan2mavros_battery.py를 ROS2로 포팅한 버전.
Powermodule → CANable → NUC 구성에서 DroneCAN BatteryInfo를 ROS2 토픽으로 변환.

기능 요약:
  1. CAN 인터페이스 존재/상태 확인 및 자동 bring-up
  2. DroneCAN 노드로 BatteryInfo 수신
  3. sensor_msgs/BatteryState → /battery
  4. mavros_msgs/BatteryStatus → /mavros/battery (MAVROS 호환)
  5. 동적 Node ID 서버 (선택, CentralizedServer는 spin 전용 스레드에서만 생성·실행)
  6. 소프트 종단저항 설정 (선택)
  7. 링크 감시 및 TransferError 복구

동적 ID 할당기는 SQLite를 쓰므로, make_node/NodeMonitor/CentralizedServer/add_handler는
메인이 아니라 dronecan_spin 스레드에서만 초기화한다 (스레드 안전).
"""

import math
import os
import re
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

try:
    from mavros_msgs.msg import BatteryStatus as MavrosBatteryStatus
    HAVE_MAVROS = True
except Exception:
    HAVE_MAVROS = False

import dronecan
from dronecan.app.node_monitor import NodeMonitor
from dronecan.app.dynamic_node_id import CentralizedServer


# ─────────────────────────────────────────────────────────────────────────────
# 유틸리티 함수 (CAN 인터페이스 관리)
# ─────────────────────────────────────────────────────────────────────────────

def _run(cmd):
    """subprocess로 명령 실행. stdout/stderr 캡처."""
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def _iface_exists(name: str) -> bool:   #장치 연결 확인
    """
    CAN 인터페이스(can0 등)가 시스템에 존재하는지 확인.
    ip -br link 출력에서 이름 검색.
    """
    out = _run(["/sbin/ip", "-br", "link"]).stdout
    return bool(re.search(rf"\b{name}\b", out))


def _iface_state(name: str) -> str:   #인터페이스 상태 확인
    """
    인터페이스 상태(UP/DOWN 등) 반환.
    ip -d -s link show <name> 출력에서 state 파싱.
    """
    out = _run(["/sbin/ip", "-d", "-s", "link", "show", name]).stdout
    m = re.search(r"\bstate\s+(\S+)", out)
    return m.group(1) if m else "UNKNOWN"


def _bringup_can(name: str, bitrate: int | None = None, use_sudo: bool = True, logger=None) -> bool:    #CAN 인터페이스 상태 활성화
    """
    CAN 인터페이스를 UP 상태로 전환.
    bitrate가 주어지면 down → type can bitrate 설정 → up 순서로 실행.
    권한 없으면 sudo -n으로 재시도 (NOPASSWD 설정 시).
    """
    ok = False
    cmds = []
    if bitrate:
        cmds += [
            ["/sbin/ip", "link", "set", name, "down"],
            [
                "/sbin/ip", "link", "set", name, "type", "can",
                "bitrate", str(bitrate), "berr-reporting", "on", "restart-ms", "100",
            ],
        ]
    cmds += [["/sbin/ip", "link", "set", name, "up"]]

    for c in cmds:
        r = _run(c)
        if r.returncode != 0 and use_sudo:
            r = _run(["sudo", "-n", *c])
        if r.returncode != 0:
            if logger:
                logger.warn(f"bringup cmd failed: {' '.join(c)} -> {r.stderr.strip()}")
        else:
            ok = True

    if _iface_exists(name):
        st = _iface_state(name)
        if logger:
            logger.info(f"CAN iface {name} state={st}")
        ok = ok or (st in ("UP", "UNKNOWN", "DORMANT", "LOWERLAYERDOWN", "ERROR-ACTIVE"))
    return ok


# ─────────────────────────────────────────────────────────────────────────────
# Bridge 클래스 (ROS2 Node)
# ─────────────────────────────────────────────────────────────────────────────

class Bridge(Node):
    """
    DroneCAN BatteryInfo → ROS2 토픽 브리지.
    ROS1 rospy 기반 코드를 rclpy Node로 변환.
    """

    def __init__(self):
        super().__init__("dronecan_to_mavros_battery")

        # ── ROS2 파라미터 (ROS1과 동일 기본값) ─────────────────────────────────
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("local_node_id", 127)
        self.declare_parameter("target_node_id", 125)  # Powermodule 기본 Node ID

        self.declare_parameter("auto_bringup", True)
        self.declare_parameter("bitrate", 0)
        self.declare_parameter("bringup_with_sudo", True)

        # True: NodeMonitor + CentralizedServer (동적 ID 할당). DroneCAN 스택은 spin 전용 스레드에서만 생성.
        self.declare_parameter("enable_dynamic_id_server", True)
        self.declare_parameter("dynamic_id_db", "~/.dronecan_node_id_table.db")

        self.declare_parameter("enable_termination", True)
        self.declare_parameter("termination_param_name", "")
        self.declare_parameter("save_params", True)

        # 파라미터 로드
        self.iface = str(self.get_parameter("can_interface").value)
        self.local_nid = int(self.get_parameter("local_node_id").value)

        target_param = self.get_parameter("target_node_id").value
        target_str = str(target_param).strip() if target_param is not None else ""
        self.target_nid = int(target_str) if target_str else None

        self.auto_bringup = bool(self.get_parameter("auto_bringup").value)
        bitrate_val = int(self.get_parameter("bitrate").value)
        self.bitrate = bitrate_val if bitrate_val > 0 else None
        self.bringup_sudo = bool(self.get_parameter("bringup_with_sudo").value)

        self.enable_id_alloc = bool(self.get_parameter("enable_dynamic_id_server").value)
        self.id_alloc_db = os.path.expanduser(str(self.get_parameter("dynamic_id_db").value))

        self.enable_term = bool(self.get_parameter("enable_termination").value)
        self.term_param_name = str(self.get_parameter("termination_param_name").value)
        self.save_params = bool(self.get_parameter("save_params").value)

        # DroneCAN 객체는 메인 스레드에서 만들지 않음 (spin 전용 스레드에서 생성 → SQLite/CentralizedServer 스레드 안전)
        self.dronecan_node = None
        self.dc_mon = None
        self.dc_alloc = None

        # ── CAN bring-up ─────────────────────────────────────────────────────
        if self.auto_bringup:
            if not _iface_exists(self.iface):
                self.get_logger().warn(f"Interface {self.iface} not found. 계속 진행합니다.")
            else:
                if not _bringup_can(self.iface, self.bitrate, self.bringup_sudo, self.get_logger()):
                    self.get_logger().warn(
                        f"Interface {self.iface} bring-up 실패. "
                        "권한(cap_net_admin) 또는 sudo 설정 확인 필요."
                    )

        # ── 핸들러/퍼블리셔 (DroneCAN 연결은 spin 스레드에서) ─────────────────
        self.pub_sensor = self.create_publisher(BatteryState, "/battery", 10)
        self.pub_mavros = (
            self.create_publisher(MavrosBatteryStatus, "/mavros/battery", 10)
            if HAVE_MAVROS
            else None
        )

        # 에러 로깅/복구 헬퍼
        self._transfer_err_count = 0
        self._last_transfer_err_log_t = 0.0
        self._transfer_err_log_interval = 5.0
        self._transfer_err_recover_threshold = 10

        # spin(DroneCAN 전용 스레드) + 링크 감시
        threading.Thread(target=self.spin, daemon=True, name="dronecan_spin").start()
        threading.Thread(target=self.link_watchdog, daemon=True).start()

        self.get_logger().info(
            f"DroneCAN→ROS2 Battery bridge on {self.iface} "
            f"(local_nid={self.local_nid}, target={self.target_nid})"
        )

    def _init_dronecan_stack(self) -> None:
        """
        DroneCAN 노드·동적 ID 할당기·핸들러를 이 스레드(spin 전용)에서만 생성.
        CentralizedServer의 SQLite는 생성 스레드에서만 사용해야 하므로 여기서 한 번에 구성.
        """
        devs = [f"socketcan:{self.iface}", self.iface]
        last_err = None
        for dev in devs:
            try:
                self.dronecan_node = dronecan.make_node(dev, node_id=self.local_nid)
                self.get_logger().info(f"Using CAN iface: {dev}")
                break
            except Exception as e:
                last_err = e
        else:
            raise RuntimeError(f"CAN open failed for {self.iface}: {last_err}")

        if self.enable_id_alloc:
            self.dc_mon = NodeMonitor(self.dronecan_node)
            self.dc_alloc = CentralizedServer(
                self.dronecan_node, self.dc_mon, database_storage=self.id_alloc_db
            )
            self.get_logger().info(f"Dynamic Node ID server enabled (db={self.id_alloc_db})")

        self.dronecan_node.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.on_batt)

        if self.enable_term and self.target_nid is not None:
            try:
                pname = self._ensure_termination_on(self.target_nid, self.term_param_name)
                if pname:
                    self.get_logger().info(
                        f"Termination param '{pname}' set to ON on nid={self.target_nid}"
                    )
                else:
                    self.get_logger().warn(f"Termination parameter not found on nid={self.target_nid}")
            except Exception as e:
                self.get_logger().warn(f"Failed to set termination on nid={self.target_nid}: {e!r}")

    # ── 링크 감시: 전원 사이클 후 DOWN이면 자동 bring-up ──────────────────────
    def link_watchdog(self):
        """1초마다 CAN 인터페이스 상태 확인. DOWN이면 bring-up 재시도."""
        while rclpy.ok():
            try:
                if self.auto_bringup and _iface_exists(self.iface):
                    st = _iface_state(self.iface)
                    if st not in ("UP", "UNKNOWN", "DORMANT", "LOWERLAYERDOWN", "ERROR-ACTIVE"):
                        self.get_logger().warn(f"CAN iface {self.iface} state={st} → bring-up 시도")
                        _bringup_can(self.iface, self.bitrate, self.bringup_sudo, self.get_logger())
            except Exception as e:
                self.get_logger().warn(f"link_watchdog error: {e!r}")
            time.sleep(1.0)

    # ── DroneCAN param helpers ───────────────────────────────────────────────
    def _getset(self, nid, name=None, index=None, set_value=None, timeout=1.0):
        """DroneCAN param.GetSet 요청/응답."""
        if self.dronecan_node is None:
            return None
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        if name:
            req.name = str(name)
        elif index is not None:
            req.index = int(index)
        if set_value is not None:
            v = dronecan.uavcan.protocol.param.Value()
            try:
                v.integer_value = int(set_value)
            except Exception:
                v.boolean_value = bool(set_value)
            req.value = v
        return self.dronecan_node.request(req, nid, timeout=timeout)

    def _save_params(self, nid, timeout=2.0):
        """DroneCAN param.ExecuteOpcode(SAVE) 요청."""
        if self.dronecan_node is None:
            return
        try:
            req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
            req.opcode = getattr(
                dronecan.uavcan.protocol.param.ExecuteOpcode.Request, "OPCODE_SAVE", 0
            )
            req.argument = 0
            self.dronecan_node.request(req, nid, timeout=timeout)
        except Exception:
            pass

    def _ensure_termination_on(self, nid, preferred_name=""):
        """
        대상 노드에서 종단저항 관련 파라미터를 찾아 1로 설정.
        후보: CAN_TERMINATE, CAN_TERM, TERM_ENABLE 등.
        못 찾으면 인덱스로 순회하며 'term' 포함 파라미터 탐색.
        """
        candidates = [preferred_name] if preferred_name else []
        candidates += [
            "CAN_TERMINATE", "CAN_TERM", "TERM_ENABLE", "TERMINATION",
            "CAN_TERM_EN", "TERM", "CAN_120R", "CAN_ENABLE_TERM",
        ]
        for nm in candidates:
            if not nm:
                continue
            try:
                r = self._getset(nid, name=nm, set_value=1)
                if r and getattr(r, "name", ""):
                    if self.save_params:
                        self._save_params(nid)
                    return r.name
            except Exception:
                continue
        for idx in range(0, 256):
            try:
                r = self._getset(nid, index=idx)
            except Exception:
                break
            if not r or not getattr(r, "name", ""):
                break
            nm = r.name.strip()
            if "term" in nm.lower():
                try:
                    self._getset(nid, name=nm, set_value=1)
                    if self.save_params:
                        self._save_params(nid)
                    return nm
                except Exception:
                    pass
        return ""

    # ── DroneCAN spin 루프 (이 스레드에서만 DroneCAN 노드/할당기 생성·사용) ───
    def spin(self):
        """
        DroneCAN 전용 스레드: _init_dronecan_stack() 후 node.spin 반복.
        TransferError는 디버그 로그만, 10회 이상 연속 시 bring-up 재시도.
        """
        while rclpy.ok():
            if self.dronecan_node is None:
                try:
                    self._init_dronecan_stack()
                except Exception as e:
                    self.get_logger().warn(f"DroneCAN init failed (retrying): {e!r}")
                    time.sleep(1.0)
                    continue

            try:
                self.dronecan_node.spin(0.01)
                self._transfer_err_count = 0
            except Exception as e:
                transfer_err_cls = None
                try:
                    transfer_err_cls = getattr(dronecan.transport, "TransferError", None)
                except Exception:
                    pass

                if transfer_err_cls is not None and isinstance(e, transfer_err_cls):
                    self._transfer_err_count += 1
                    now = time.time()
                    if now - self._last_transfer_err_log_t > self._transfer_err_log_interval:
                        self.get_logger().debug(
                            f"TransferError (ignored): {e!r} (count={self._transfer_err_count})"
                        )
                        self._last_transfer_err_log_t = now
                    if (
                        self._transfer_err_count >= self._transfer_err_recover_threshold
                        and self.auto_bringup
                    ):
                        self.get_logger().warn(
                            f"Repeated TransferError (x{self._transfer_err_count}) → bring-up {self.iface}"
                        )
                        try:
                            _bringup_can(self.iface, self.bitrate, self.bringup_sudo, self.get_logger())
                        except Exception as ex:
                            self.get_logger().warn(f"bring-up 시도 실패: {ex!r}")
                        self._transfer_err_count = 0
                    time.sleep(0.01)
                    continue

                err_text = str(e).lower()
                if "no such device" in err_text or "network is down" in err_text:
                    self.get_logger().warn(f"CAN link lost ({e!r}); will re-init DroneCAN stack")
                    self.dronecan_node = None
                    self.dc_mon = None
                    self.dc_alloc = None
                    time.sleep(0.5)
                    continue

                self.get_logger().warn(f"node.spin error: {e!r}")
                time.sleep(0.5)

    # ── BatteryInfo 콜백 ─────────────────────────────────────────────────────
    def on_batt(self, e):
        """
        DroneCAN BatteryInfo 수신 시 ROS2 메시지로 변환하여 퍼블리시.
        target_nid가 설정되면 해당 노드에서 온 메시지만 처리.
        Node ID 0(미할당)은 Powermodule 부팅 직후 흔하므로 함께 허용.
        """
        src = e.transfer.source_node_id
        if self.target_nid is not None and src != self.target_nid and src != 0:
            return

        m = e.message
        voltage = float(getattr(m, "voltage", float("nan")))
        current = float(getattr(m, "current", float("nan")))
        tK = getattr(m, "temperature", float("nan"))
        try:
            temp_c = (tK - 273.15) if not math.isnan(tK) else float("nan")
        except Exception:
            temp_c = float("nan")
        soc_pct = getattr(m, "state_of_charge_pct", float("nan"))
        soc_01 = (
            (float(soc_pct) / 100.0)
            if (isinstance(soc_pct, (int, float)) and not math.isnan(float(soc_pct)))
            else float("nan")
        )

        # sensor_msgs/BatteryState → /battery
        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage = voltage
        bs.current = current  # 방전=+, 충전=-
        bs.temperature = 0.0 if math.isnan(temp_c) else temp_c
        bs.present = True
        bs.percentage = -1.0 if math.isnan(soc_01) else soc_01
        self.pub_sensor.publish(bs)

        # mavros_msgs/BatteryStatus → /mavros/battery
        if self.pub_mavros is not None:
            ms = MavrosBatteryStatus()
            if hasattr(ms, "voltage"):
                ms.voltage = float(voltage)
            if hasattr(ms, "current"):
                ms.current = float(current)
            if hasattr(ms, "remaining"):
                ms.remaining = float(-1.0 if math.isnan(soc_01) else soc_01)
            if hasattr(ms, "temperature"):
                ms.temperature = float(0.0 if math.isnan(temp_c) else temp_c)
            self.pub_mavros.publish(ms)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
