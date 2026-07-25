"""Small two-way MAVLink relay from the active SITL link to QGC."""

from __future__ import annotations

import os
import socket
import time

from bridge.sitl_env import env_flag, env_to_int


def initialize_qgc_mavlink_relay(transport: object) -> None:
    transport._qgc_mavlink_relay_sock = None
    transport._qgc_mavlink_relay_target = None
    transport._qgc_mavlink_relay_last_warn_wall = -1.0
    transport._qgc_mavlink_relay_last_log_wall = -1.0
    transport._qgc_mavlink_relay_rx_packets = 0
    transport._qgc_mavlink_relay_tx_packets = 0
    if not env_flag("ROS2_UUV_QGC_MAVLINK_RELAY_ENABLE", False):
        return

    host = os.getenv("ROS2_UUV_QGC_MAVLINK_RELAY_HOST", "127.0.0.1").strip() or "127.0.0.1"
    port = env_to_int("ROS2_UUV_QGC_MAVLINK_RELAY_PORT", 14550)
    bind_host = os.getenv("ROS2_UUV_QGC_MAVLINK_RELAY_BIND_HOST", "0.0.0.0").strip() or "0.0.0.0"
    bind_port = env_to_int("ROS2_UUV_QGC_MAVLINK_RELAY_BIND_PORT", 0)
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((bind_host, bind_port))
        sock.setblocking(False)
    except OSError as exc:
        print(f"[sitl_transport] QGC MAVLink relay unavailable: {exc}", flush=True)
        return

    transport._qgc_mavlink_relay_sock = sock
    transport._qgc_mavlink_relay_target = (host, int(port))
    print(
        "[sitl_transport] QGC MAVLink relay enabled: "
        f"local={sock.getsockname()[0]}:{sock.getsockname()[1]} target={host}:{port}",
        flush=True,
    )


def close_qgc_mavlink_relay(transport: object) -> None:
    sock = getattr(transport, "_qgc_mavlink_relay_sock", None)
    transport._qgc_mavlink_relay_sock = None
    if sock is None:
        return
    try:
        sock.close()
    except OSError:
        pass


def relay_ap_message_to_qgc(transport: object, msg: object) -> None:
    sock = getattr(transport, "_qgc_mavlink_relay_sock", None)
    target = getattr(transport, "_qgc_mavlink_relay_target", None)
    if sock is None or target is None:
        return
    get_msgbuf = getattr(msg, "get_msgbuf", None)
    if not callable(get_msgbuf):
        return
    try:
        payload = bytes(get_msgbuf())
        if not payload:
            return
        sock.sendto(payload, target)
        if transport._qgc_mavlink_relay_tx_packets == 0:
            print("[sitl_transport] QGC MAVLink relay first ArduSub packet sent", flush=True)
        transport._qgc_mavlink_relay_tx_packets += 1
    except OSError as exc:
        _warn_qgc_relay(transport, f"send failed: {exc}")


def drain_qgc_mavlink_relay(transport: object, *, budget: int = 64) -> None:
    sock = getattr(transport, "_qgc_mavlink_relay_sock", None)
    mav = getattr(transport, "_sitl_mav", None)
    if sock is None or mav is None:
        return
    for _ in range(max(1, int(budget))):
        try:
            payload, _addr = sock.recvfrom(4096)
        except BlockingIOError:
            return
        except OSError as exc:
            _warn_qgc_relay(transport, f"receive failed: {exc}")
            return
        if not payload:
            continue
        try:
            mav.write(payload)
            if transport._qgc_mavlink_relay_rx_packets == 0:
                print("[sitl_transport] QGC MAVLink relay first QGC packet forwarded", flush=True)
            transport._qgc_mavlink_relay_rx_packets += 1
        except Exception as exc:
            _warn_qgc_relay(transport, f"forward to SITL failed: {exc}")
            return


def _warn_qgc_relay(transport: object, message: str) -> None:
    now = time.monotonic()
    last = float(getattr(transport, "_qgc_mavlink_relay_last_warn_wall", -1.0))
    if now - last < 2.0:
        return
    transport._qgc_mavlink_relay_last_warn_wall = now
    print(f"[sitl_transport] QGC MAVLink relay {message}", flush=True)


__all__ = [
    "close_qgc_mavlink_relay",
    "drain_qgc_mavlink_relay",
    "initialize_qgc_mavlink_relay",
    "relay_ap_message_to_qgc",
]
