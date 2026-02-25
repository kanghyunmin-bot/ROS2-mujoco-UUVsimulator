#!/usr/bin/env python3
"""Check whether QGC manual input reaches ArduSub SITL.

Usage:
  python3 scripts/check_sitl_manual_input.py --mode tcp --host 127.0.0.1 --port 5760
  python3 scripts/check_sitl_manual_input.py --mode udp --listen 14550

What to look for:
  - `armed=True` and RC channel values (roll/pitch/throttle/yaw) move away from 1500
  - If channels stay 1500 while moving QGC virtual joystick, QGC input is not flowing
"""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor SITL manual input flow")
    parser.add_argument("--mode", choices=("tcp", "udp"), default="tcp", help="MAVLink transport mode")
    parser.add_argument("--host", default="127.0.0.1", help="TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5760, help="TCP port (default: 5760)")
    parser.add_argument("--listen", type=int, default=14550, help="UDP listen port for --mode udp")
    parser.add_argument("--hb-timeout", type=float, default=20.0, help="Heartbeat wait timeout seconds")
    parser.add_argument("--timeout", type=float, default=60.0, help="Monitoring timeout seconds")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero when arm/input movement is not detected",
    )
    args = parser.parse_args()

    if args.mode == "tcp":
        endpoint = f"tcp:{args.host}:{int(args.port)}"
    else:
        endpoint = f"udpin:0.0.0.0:{int(args.listen)}"

    try:
        conn = mavutil.mavlink_connection(endpoint, source_system=255, source_component=190)
    except Exception as exc:
        print(f"[check] FAIL: cannot open MAVLink endpoint {endpoint}: {exc}", flush=True)
        return 4
    print(f"[check] waiting heartbeat on {endpoint} ...", flush=True)
    hb = conn.wait_heartbeat(timeout=float(args.hb_timeout))
    if hb is None:
        print(
            "[check] FAIL: heartbeat not received. "
            "ArduSub may be down or endpoint/link is wrong.",
            flush=True,
        )
        return 4
    print(
        f"[check] connected vehicle sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}",
        flush=True,
    )
    try:
        conn.mav.request_data_stream_send(
            hb.get_srcSystem(),
            hb.get_srcComponent(),
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
            10,
            1,
        )
        conn.mav.request_data_stream_send(
            hb.get_srcSystem(),
            hb.get_srcComponent(),
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            5,
            1,
        )
        for mid in (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW):
            conn.mav.command_long_send(
                hb.get_srcSystem(),
                hb.get_srcComponent(),
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                float(mid),
                100000.0,  # 10 Hz
                0, 0, 0, 0, 0,
            )
    except Exception:
        pass

    start = time.time()
    last_print = 0.0
    last_rc = None
    armed_seen = False
    rc_active_seen = False
    servo_active_seen = False
    while time.time() - start < float(args.timeout):
        msg = conn.recv_match(blocking=True, timeout=1.0)
        if msg is None:
            continue

        mtype = msg.get_type()
        now = time.time()
        if mtype == "HEARTBEAT":
            d = msg.to_dict()
            base_mode = int(d.get("base_mode", 0))
            armed = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                armed_seen = True
            if now - last_print > 0.8:
                print(
                    f"[hb] mode={int(d.get('custom_mode', 0))} base_mode={base_mode} armed={armed}",
                    flush=True,
                )
                last_print = now
        elif mtype == "RC_CHANNELS":
            d = msg.to_dict()
            rc = (
                int(d.get("chan1_raw", 0)),
                int(d.get("chan2_raw", 0)),
                int(d.get("chan3_raw", 0)),
                int(d.get("chan4_raw", 0)),
            )
            if any(abs(v - 1500) > 15 for v in rc):
                rc_active_seen = True
            if rc != last_rc:
                print(f"[rc] ch1..4={rc}", flush=True)
                last_rc = rc
        elif mtype == "SERVO_OUTPUT_RAW":
            d = msg.to_dict()
            sv = (
                int(d.get("servo1_raw", 0)),
                int(d.get("servo2_raw", 0)),
                int(d.get("servo3_raw", 0)),
                int(d.get("servo4_raw", 0)),
            )
            if any(abs(v - 1500) > 15 for v in sv):
                servo_active_seen = True
        elif mtype == "STATUSTEXT":
            d = msg.to_dict()
            text = str(d.get("text", "")).strip()
            if text:
                print(f"[status] {text}", flush=True)

    print(
        f"[check] summary armed_seen={armed_seen} rc_active_seen={rc_active_seen} servo_active_seen={servo_active_seen}",
        flush=True,
    )
    if args.strict:
        if not armed_seen:
            print(
                "[check] FAIL: vehicle never armed. Arm in QGC first (top bar must show Armed).",
                flush=True,
            )
            return 2
        if not rc_active_seen and not servo_active_seen:
            print(
                "[check] FAIL: manual input did not change RC/SERVO. "
                "Verify QGC Virtual Joystick enabled, MANUAL mode, and Comm Link is active.",
                flush=True,
            )
            return 3
        if servo_active_seen and not rc_active_seen:
            print(
                "[check] note: SERVO active but RC channels stayed 1500 "
                "(normal when QGC Virtual Joystick uses MANUAL_CONTROL).",
                flush=True,
            )
    print("[check] done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
