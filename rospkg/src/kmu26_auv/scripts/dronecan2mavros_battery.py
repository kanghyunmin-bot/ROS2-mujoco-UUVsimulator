#!/usr/bin/python3
import math
import os
import re
import shutil
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

import dronecan
from dronecan.app.node_monitor import NodeMonitor
from dronecan.app.dynamic_node_id import CentralizedServer


def _run(cmd):
    return subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def _resolve_ip_cmd() -> str | None:
    candidate = shutil.which("ip")
    if candidate:
        return candidate
    for path in ("/usr/sbin/ip", "/sbin/ip", "/usr/bin/ip", "/bin/ip"):
        if os.path.exists(path) and os.access(path, os.X_OK):
            return path
    return None


_IP_CMD = _resolve_ip_cmd()


def _iface_exists(name: str) -> bool:
    if os.path.exists(f"/sys/class/net/{name}"):
        return True
    if _IP_CMD is None:
        return False
    out = _run([_IP_CMD, "-br", "link"]).stdout
    return bool(re.search(rf"\b{name}\b", out))


def _iface_state(name: str) -> str:
    state_file = f"/sys/class/net/{name}/operstate"
    if os.path.exists(state_file):
        try:
            return open(state_file, "r", encoding="utf-8").read().strip().upper()
        except OSError:
            pass
    if _IP_CMD is None:
        return "UNKNOWN"
    out = _run([_IP_CMD, "-d", "-s", "link", "show", name]).stdout
    m = re.search(r"\bstate\s+(\S+)", out)
    return m.group(1) if m else "UNKNOWN"


def _bringup_can(name: str, bitrate: int | None = None, use_sudo: bool = True, logger=None) -> bool:
    """Bring up canX. Reset type/bitrate if requested."""
    if _IP_CMD is None:
        if logger is not None:
            logger.warn("ip command not found; skipping CAN auto bring-up")
        return False

    ok = False
    cmds = []
    if bitrate:
        cmds.append([_IP_CMD, "link", "set", name, "down"])

        # Some adapters/drivers (e.g. certain gs_usb firmwares) don't support
        # berr-reporting/restart-ms. Try full options first, then fallback.
        type_cmds = [
            [
                _IP_CMD,
                "link",
                "set",
                name,
                "type",
                "can",
                "bitrate",
                str(bitrate),
                "berr-reporting",
                "on",
                "restart-ms",
                "100",
            ],
            [_IP_CMD, "link", "set", name, "type", "can", "bitrate", str(bitrate)],
        ]
        type_ok = False
        for tc in type_cmds:
            r = _run(tc)
            if r.returncode != 0 and use_sudo:
                r = _run(["sudo", "-n", *tc])
            if r.returncode == 0:
                type_ok = True
                break
            if logger is not None:
                logger.warn(f"type setup failed: {' '.join(tc)} -> {r.stderr.strip()}")
        ok = ok or type_ok
    cmds += [[_IP_CMD, "link", "set", name, "up"]]

    for c in cmds:
        r = _run(c)
        if r.returncode != 0 and use_sudo:
            r = _run(["sudo", "-n", *c])  # best effort if NOPASSWD sudo is configured
        if r.returncode != 0:
            if logger is not None:
                logger.warn(f"bringup cmd failed: {' '.join(c)} -> {r.stderr.strip()}")
        else:
            ok = True

    if _iface_exists(name):
        st = _iface_state(name)
        if logger is not None:
            logger.info(f"CAN iface {name} state={st}")
        ok = ok or st in ("UP", "UNKNOWN", "DORMANT", "LOWERLAYERDOWN", "ERROR-ACTIVE")
    return ok


class Bridge(Node):
    def __init__(self):
        super().__init__("dronecan_to_mavros_battery")

        # ROS 2 params
        self.declare_parameter("can_interface", "can0")
        self.declare_parameter("local_node_id", 127)
        # Empty/0/-1 disables source filtering and accepts BatteryInfo from any node ID.
        self.declare_parameter("target_node_id", "")

        self.declare_parameter("auto_bringup", True)
        self.declare_parameter("bitrate", 0)
        self.declare_parameter("bringup_with_sudo", True)

        self.declare_parameter("enable_dynamic_id_server", True)
        self.declare_parameter("dynamic_id_db", "~/.dronecan_node_id_table.db")

        self.declare_parameter("enable_termination", True)
        self.declare_parameter("termination_param_name", "")
        self.declare_parameter("save_params", True)

        self.iface = str(self.get_parameter("can_interface").value)
        self.local_nid = int(self.get_parameter("local_node_id").value)

        target_param = self.get_parameter("target_node_id").value
        target_text = str(target_param).strip() if target_param is not None else ""
        self.target_nid = self._parse_target_node_id(target_text)

        self.auto_bringup = bool(self.get_parameter("auto_bringup").value)
        bitrate_val = int(self.get_parameter("bitrate").value)
        self.bitrate = bitrate_val if bitrate_val > 0 else None
        self.bringup_sudo = bool(self.get_parameter("bringup_with_sudo").value)

        self.enable_id_alloc = bool(self.get_parameter("enable_dynamic_id_server").value)
        self.id_alloc_db = os.path.expanduser(str(self.get_parameter("dynamic_id_db").value))

        self.enable_term = bool(self.get_parameter("enable_termination").value)
        self.term_param_name = str(self.get_parameter("termination_param_name").value)
        self.save_params = bool(self.get_parameter("save_params").value)

        # Publishers are created even when CAN is not ready yet.
        self.pub_sensor = self.create_publisher(BatteryState, "/battery", 10)
        self.pub_mavros = self.create_publisher(BatteryState, "/mavros/battery", 10)

        self.dronecan_node = None
        self.mon = None
        self.alloc = None
        self._termination_configured = False

        # CAN bring-up
        if self.auto_bringup:
            if not _iface_exists(self.iface):
                self.get_logger().warn(f"Interface {self.iface} not found in ip link. Continue anyway.")
            else:
                if not _bringup_can(self.iface, self.bitrate, self.bringup_sudo, self.get_logger()):
                    self.get_logger().warn(
                        f"Interface {self.iface} bring-up failed. "
                        "Check CAP_NET_ADMIN or sudo configuration."
                    )

        # Try once at startup; if unavailable, watchdog keeps retrying.
        self._try_open_dronecan(log_on_fail=True)

        # Error/recovery helpers
        self._transfer_err_count = 0
        self._last_transfer_err_log_t = 0.0
        self._transfer_err_log_interval = 5.0
        self._transfer_err_recover_threshold = 10
        self._rx_count = 0
        self._last_rx_monotonic = time.monotonic()
        self.create_timer(5.0, self._log_rx_health)

        # spin + watchdog background threads
        threading.Thread(target=self.spin_dronecan, daemon=True).start()
        threading.Thread(target=self.link_watchdog, daemon=True).start()

        self.get_logger().info(
            f"DroneCAN->ROS2 Battery bridge on {self.iface} "
            f"(local_nid={self.local_nid}, target={self.target_nid})"
        )
        self.get_logger().info(
            f"Publishing BatteryState to {self.pub_sensor.topic_name} and {self.pub_mavros.topic_name}"
        )
        if self.target_nid is None:
            self.get_logger().info(
                "Battery source filter disabled. Accepting BatteryInfo from any DroneCAN node ID."
            )
        else:
            self.get_logger().info(
                f"Battery source filter enabled. target_node_id={self.target_nid}"
            )

    @staticmethod
    def _parse_target_node_id(text: str) -> int | None:
        if not text:
            return None
        try:
            nid = int(text)
        except (TypeError, ValueError):
            return None
        # Treat 0/-1 as wildcard for convenience.
        if nid <= 0:
            return None
        return nid

    def _log_rx_health(self):
        elapsed = time.monotonic() - self._last_rx_monotonic
        if self._rx_count == 0:
            self.get_logger().warn(
                "No DroneCAN BatteryInfo received yet. "
                "If topics exist but stay empty, verify battery node is publishing "
                "uavcan.equipment.power.BatteryInfo on this CAN bus."
            )
            return
        if elapsed > 10.0:
            self.get_logger().warn(
                f"BatteryInfo stream stalled for {elapsed:.1f}s "
                f"(received {self._rx_count} total frames)."
            )

    def _try_open_dronecan(self, log_on_fail: bool = False) -> bool:
        if self.dronecan_node is not None:
            return True

        devs = [f"socketcan:{self.iface}", self.iface]
        last_err = None
        for dev in devs:
            try:
                self.dronecan_node = dronecan.make_node(dev, node_id=self.local_nid)
                self.dronecan_node.add_handler(dronecan.uavcan.equipment.power.BatteryInfo, self.on_batt)
                self.get_logger().info(f"Using CAN iface: {dev}")

                if self.enable_id_alloc:
                    self.mon = NodeMonitor(self.dronecan_node)
                    self.alloc = CentralizedServer(
                        self.dronecan_node, self.mon, database_storage=self.id_alloc_db
                    )
                    self.get_logger().info(f"Dynamic Node ID server enabled (db={self.id_alloc_db})")

                if self.enable_term and self.target_nid is not None and not self._termination_configured:
                    try:
                        pname = self._ensure_termination_on(self.target_nid, self.term_param_name)
                        if pname:
                            self.get_logger().info(
                                f"Termination param '{pname}' set to ON on nid={self.target_nid}"
                            )
                        else:
                            self.get_logger().warn(
                                f"Termination parameter not found on nid={self.target_nid}"
                            )
                    except Exception as e:
                        self.get_logger().warn(
                            f"Failed to set termination on nid={self.target_nid}: {e!r}"
                        )
                    self._termination_configured = True

                return True
            except Exception as e:
                last_err = e

        if log_on_fail:
            self.get_logger().warn(f"CAN open failed for {self.iface}: {last_err}")
        return False

    def link_watchdog(self):
        while rclpy.ok():
            try:
                if self.auto_bringup and _iface_exists(self.iface):
                    st = _iface_state(self.iface)
                    if st not in ("UP", "UNKNOWN", "DORMANT", "LOWERLAYERDOWN", "ERROR-ACTIVE"):
                        self.get_logger().warn(f"CAN iface {self.iface} state={st} -> trying bring-up")
                        _bringup_can(self.iface, self.bitrate, self.bringup_sudo, self.get_logger())

                if self.dronecan_node is None and _iface_exists(self.iface):
                    self._try_open_dronecan(log_on_fail=False)
            except Exception as e:
                self.get_logger().warn(f"link_watchdog error: {e!r}")
            time.sleep(1.0)

    def _getset(self, nid, name=None, index=None, set_value=None, timeout=1.0):
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
        if self.dronecan_node is None:
            return
        try:
            req = dronecan.uavcan.protocol.param.ExecuteOpcode.Request()
            req.opcode = getattr(
                dronecan.uavcan.protocol.param.ExecuteOpcode.Request,
                "OPCODE_SAVE",
                0,
            )
            req.argument = 0
            self.dronecan_node.request(req, nid, timeout=timeout)
        except Exception:
            pass

    def _ensure_termination_on(self, nid, preferred_name=""):
        candidates = [preferred_name] if preferred_name else []
        candidates += [
            "CAN_TERMINATE",
            "CAN_TERM",
            "TERM_ENABLE",
            "TERMINATION",
            "CAN_TERM_EN",
            "TERM",
            "CAN_120R",
            "CAN_ENABLE_TERM",
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

    def spin_dronecan(self):
        while rclpy.ok():
            if self.dronecan_node is None:
                time.sleep(0.2)
                continue

            try:
                self.dronecan_node.spin(0.01)
                self._transfer_err_count = 0
            except Exception as e:
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
                            f"TransferError (ignored): {e!r} (count={self._transfer_err_count})"
                        )
                        self._last_transfer_err_log_t = now
                    if self._transfer_err_count >= self._transfer_err_recover_threshold and self.auto_bringup:
                        self.get_logger().warn(
                            f"Repeated TransferError (x{self._transfer_err_count}) -> bring-up {self.iface}"
                        )
                        try:
                            _bringup_can(self.iface, self.bitrate, self.bringup_sudo, self.get_logger())
                        except Exception as ex:
                            self.get_logger().warn(f"bring-up retry failed: {ex!r}")
                        self._transfer_err_count = 0
                    time.sleep(0.01)
                    continue

                err_text = str(e).lower()
                if "no such device" in err_text or "network is down" in err_text:
                    self.get_logger().warn(f"CAN link lost ({e!r}), waiting for reconnect")
                    self.dronecan_node = None
                    self.mon = None
                    self.alloc = None
                    time.sleep(0.5)
                    continue

                self.get_logger().warn(f"dronecan_node.spin error: {e!r}")
                time.sleep(0.5)

    def _to_float(self, value, default_nan=True):
        try:
            return float(value)
        except Exception:
            return float("nan") if default_nan else 0.0

    def on_batt(self, e):
        if self.target_nid is not None and e.transfer.source_node_id != self.target_nid:
            return

        self._rx_count += 1
        self._last_rx_monotonic = time.monotonic()

        m = e.message
        voltage = self._to_float(getattr(m, "voltage", float("nan")))
        current = self._to_float(getattr(m, "current", float("nan")))
        temp_k = self._to_float(getattr(m, "temperature", float("nan")))
        soc_pct = self._to_float(getattr(m, "state_of_charge_pct", float("nan")))

        temp_c = (temp_k - 273.15) if not math.isnan(temp_k) else float("nan")
        soc_01 = soc_pct / 100.0 if not math.isnan(soc_pct) else float("nan")

        bs = BatteryState()
        bs.header.stamp = self.get_clock().now().to_msg()
        bs.voltage = voltage
        bs.current = current  # discharge=+, charge=-
        bs.temperature = 0.0 if math.isnan(temp_c) else temp_c
        bs.present = True
        bs.percentage = -1.0 if math.isnan(soc_01) else soc_01

        self.pub_sensor.publish(bs)
        self.pub_mavros.publish(bs)


def main():
    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
