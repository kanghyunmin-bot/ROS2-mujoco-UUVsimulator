#!/usr/bin/env python3
import math
import os
import re
import subprocess
import sys
import threading
import time

import logging
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
try:
    from mavros_msgs.msg import BatteryStatus as MavrosBatteryStatus
    HAVE_MAVROS = True
except Exception:
    HAVE_MAVROS = False

try:
    import dronecan
    from dronecan.app.node_monitor import NodeMonitor
    from dronecan.app.dynamic_node_id import CentralizedServer
    HAVE_DRONECAN = True
except Exception:
    dronecan = None
    NodeMonitor = None
    CentralizedServer = None
    HAVE_DRONECAN = False

def _run(cmd):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

def _iface_exists(name: str) -> bool:
    out = _run(["/sbin/ip","-br","link"]).stdout
    return bool(re.search(rf"\b{name}\b", out))

def _iface_state(name: str) -> str:
    out = _run(["/sbin/ip","-d","-s","link","show",name]).stdout
    m = re.search(r"\bstate\s+(\S+)", out)
    return m.group(1) if m else "UNKNOWN"

def _bringup_can(name: str, bitrate: int=None, use_sudo: bool=True) -> bool:
    """canX를 UP(필요시 타입/bitrate 재설정). 권한 없으면 best-effort."""
    ok = False
    cmds = []
    if bitrate:
        cmds += [
            ["/sbin/ip","link","set",name,"down"],
            ["/sbin/ip","link","set",name,"type","can", "bitrate", str(bitrate), "berr-reporting","on","restart-ms","100"],
        ]
    cmds += [["/sbin/ip","link","set",name,"up"]]

    for c in cmds:
        r = _run(c)
        if r.returncode != 0 and use_sudo:
            r = _run(["sudo","-n",*c])  # 무암호 sudo 가능 시
        if r.returncode != 0:
            logging.getLogger("dronecan_bridge").warning(
                "bringup cmd failed: %s -> %s", " ".join(c), r.stderr.strip()
            )
            # 계속 시도하되 마지막 결과를 따름
        else:
            ok = True
    # 최종 상태 점검
    if _iface_exists(name):
        st = _iface_state(name)
        logging.getLogger("dronecan_bridge").info("CAN iface %s state=%s", name, st)
        ok = ok or (st in ("UP","UNKNOWN","DORMANT","LOWERLAYERDOWN","ERROR-ACTIVE"))
    return ok

class Bridge(Node):
    def __init__(self):
        super().__init__("dronecan2mavros_battery")
        if not HAVE_DRONECAN:
            self.get_logger().warn("dronecan module not available; DroneCAN battery bridge disabled.")
            return
        # ── ROS params ───────────────────────────────────────────────
        self.iface          = self.declare_parameter("can_interface", "can0").value
        self.local_nid      = int(self.declare_parameter("local_node_id", 127).value)
        target_param        = self.declare_parameter("target_node_id", 125).value   # 기본 125
        self.target_nid     = int(target_param) if str(target_param).strip() else None

        # CAN 자동 bring-up 옵션
        self.auto_bringup   = self.declare_parameter("auto_bringup", True).value
        self.bitrate        = int(self.declare_parameter("bitrate", 0).value) or None  # 0이면 건드리지 않음
        self.bringup_sudo   = self.declare_parameter("bringup_with_sudo", True).value

        # 동적 Node ID 서버
        self.enable_id_alloc= self.declare_parameter("enable_dynamic_id_server", True).value
        self.id_alloc_db    = os.path.expanduser(self.declare_parameter("dynamic_id_db", "~/.dronecan_node_id_table.db").value)

        # 종단저항(소프트) 옵션
        self.enable_term    = self.declare_parameter("enable_termination", True).value
        self.term_param_name= self.declare_parameter("termination_param_name", "").value
        self.save_params    = self.declare_parameter("save_params", True).value

        # ── CAN bring-up ─────────────────────────────────────────────
        if self.auto_bringup:
            if not _iface_exists(self.iface):
                self.get_logger().warn(f"Interface {self.iface} not found in ip link. 계속 진행합니다.")
            else:
                if not _bringup_can(self.iface, self.bitrate, self.bringup_sudo):
                    self.get_logger().warn(
                        f"Interface {self.iface} bring-up 실패. 권한(cap_net_admin) 또는 sudo 설정 확인 필요."
                    )

        # ── DroneCAN node open (socketcan 접두사/무접두사 모두 시도) ──
        devs = [f"socketcan:{self.iface}", self.iface]
        last_err = None
        for dev in devs:
            try:
                self.node = dronecan.make_node(dev, node_id=self.local_nid)
                self.get_logger().info(f"Using CAN iface: {dev}")
                break
            except Exception as e:
                last_err = e
        else:
            raise RuntimeError(f"CAN open failed for {self.iface}: {last_err}")

        # ── 동적 Node ID 서버 ────────────────────────────────────────
        if self.enable_id_alloc:
            self.mon   = NodeMonitor(self.node)
            self.alloc = CentralizedServer(self.node, self.mon, database_storage=self.id_alloc_db)
            self.get_logger().info(f"Dynamic Node ID server enabled (db={self.id_alloc_db})")

        # ── 종단저항 on (선택) ───────────────────────────────────────
        if self.enable_term and self.target_nid is not None:
            try:
                pname = self._ensure_termination_on(self.target_nid, self.term_param_name)
                if pname:
                    self.get_logger().info(f"Termination param '{pname}' set to ON on nid={self.target_nid}")
                else:
                    self.get_logger().warn(f"Termination parameter not found on nid={self.target_nid}")
            except Exception as e:
                self.get_logger().warn(f"Failed to set termination on nid={self.target_nid}: {e!r}")

        # ── 핸들러/퍼블리셔 ─────────────────────────────────────────
        self.node.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.on_batt)
        self.pub_sensor = self.create_publisher(BatteryState, "battery", 10)
        self.pub_mavros = self.create_publisher(MavrosBatteryStatus, "mavros/battery", 10) if HAVE_MAVROS else None

        # 에러 로깅/복구 헬퍼
        self._transfer_err_count = 0
        self._last_transfer_err_log_t = 0.0
        self._transfer_err_log_interval = 5.0  # seconds
        self._transfer_err_recover_threshold = 10

        # spin + 링크 감시 스레드
        threading.Thread(target=self.spin, daemon=True).start()
        threading.Thread(target=self.link_watchdog, daemon=True).start()

        self.get_logger().info(
            f"DroneCAN→ROS Battery bridge on {self.iface} (local_nid={self.local_nid}, target={self.target_nid})"
        )

    # ── 링크 감시: 전원 사이클 후 DOWN이면 자동 bring-up ──────────────
    def link_watchdog(self):
        while rclpy.ok():
            try:
                if self.auto_bringup and _iface_exists(self.iface):
                    st = _iface_state(self.iface)
                    if st not in ("UP","UNKNOWN","DORMANT","LOWERLAYERDOWN","ERROR-ACTIVE"):
                        self.get_logger().warn(f"CAN iface {self.iface} state={st} → bring-up 시도")
                        _bringup_can(self.iface, self.bitrate, self.bringup_sudo)
            except Exception as e:
                self.get_logger().warn(f"link_watchdog error: {e!r}")
            time.sleep(1.0)

    # ── DroneCAN param helpers ───────────────────────────────────────
    def _getset(self, nid, name=None, index=None, set_value=None, timeout=1.0):
        req = dronecan.uavcan.protocol.param.GetSet.Request()
        if name: req.name = str(name)
        elif index is not None: req.index = int(index)
        if set_value is not None:
            v = dronecan.uavcan.protocol.param.Value()
            try:
                v.integer_value = int(set_value)
            except Exception:
                v.boolean_value = bool(set_value)
            req.value = v
        return self.node.request(req, nid, timeout=timeout)

    def _save_params(self, nid, timeout=2.0):
        try:
            req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
            req.opcode = getattr(dronecan.uavcan.protocol.param.ExecuteOpcode.Request, 'OPCODE_SAVE', 0)
            req.argument = 0
            self.node.request(req, nid, timeout=timeout)
        except Exception:
            pass

    def _ensure_termination_on(self, nid, preferred_name=""):
        candidates = [preferred_name] if preferred_name else []
        candidates += ["CAN_TERMINATE","CAN_TERM","TERM_ENABLE","TERMINATION","CAN_TERM_EN","TERM","CAN_120R","CAN_ENABLE_TERM"]
        for nm in candidates:
            if not nm: continue
            try:
                r = self._getset(nid, name=nm, set_value=1)
                if r and getattr(r, 'name', ''):
                    if self.save_params: self._save_params(nid)
                    return r.name
            except Exception:
                continue
        for idx in range(0, 256):
            try:
                r = self._getset(nid, index=idx)
            except Exception:
                break
            if not r or not getattr(r, 'name', ''): break
            nm = r.name.strip()
            if 'term' in nm.lower():
                try:
                    self._getset(nid, name=nm, set_value=1)
                    if self.save_params: self._save_params(nid)
                    return nm
                except Exception:
                    pass
        return ""

    # ── 루프/핸들러 ────────────────────────────────────────────────
    def spin(self):
        r = self.create_rate(100)
        while rclpy.ok():
            try:
                self.node.spin(0.01)
                # 성공 시 전이성 에러 카운터 초기화
                self._transfer_err_count = 0
            except Exception as e:
                # dronecan.transport.TransferError (토글 비트 등)만 디버그로 처리하고 계속함
                transfer_err_cls = None
                try:
                    transfer_err_cls = getattr(dronecan.transport, "TransferError", None)
                except Exception:
                    transfer_err_cls = None

                if transfer_err_cls is not None and isinstance(e, transfer_err_cls):
                    self._transfer_err_count += 1
                    now = time.time()
                    if now - self._last_transfer_err_log_t > self._transfer_err_log_interval:
                        self.get_logger().debug(
                            "TransferError (ignored): %r (count=%d)", e, self._transfer_err_count
                        )
                        self._last_transfer_err_log_t = now
                    # 반복적 에러면 인터페이스 재설정 시도
                    if self._transfer_err_count >= self._transfer_err_recover_threshold and self.auto_bringup:
                        self.get_logger().warn(
                            "Repeated TransferError (x%d) → bring-up %s", self._transfer_err_count, self.iface
                        )
                        try:
                            _bringup_can(self.iface, self.bitrate, self.bringup_sudo)
                        except Exception as ex:
                            self.get_logger().warn(f"bring-up 시도 실패: {ex!r}")
                        self._transfer_err_count = 0
                    time.sleep(0.01)
                    continue

                # 기타 예외는 기존처럼 경고 후 대기
                self.get_logger().warn(f"node.spin error: {e!r}")
                time.sleep(0.5)

    def on_batt(self, e):
        if self.target_nid is not None and e.transfer.source_node_id != self.target_nid:
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
        soc_01  = (float(soc_pct)/100.0) if (isinstance(soc_pct,(int,float)) and not math.isnan(float(soc_pct))) else float("nan")

        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage      = voltage
        bs.current      = current           # 방전=+, 충전=-
        bs.temperature  = 0.0 if math.isnan(temp_c) else temp_c
        bs.present      = True
        bs.percentage   = -1.0 if math.isnan(soc_01) else soc_01
        self.pub_sensor.publish(bs)

        if self.pub_mavros is not None:
            ms = MavrosBatteryStatus()
            if hasattr(ms, "voltage"):     ms.voltage = float(voltage)
            if hasattr(ms, "current"):     ms.current = float(current)
            if hasattr(ms, "remaining"):   ms.remaining = float(-1.0 if math.isnan(soc_01) else soc_01)
            if hasattr(ms, "temperature"): ms.temperature = float(0.0 if math.isnan(temp_c) else temp_c)
            self.pub_mavros.publish(ms)

def main():
    rclpy.init()
    node = Bridge()
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
