#!/usr/bin/env python3
"""Check whether QGC manual input reaches ArduSub SITL.

Usage:
  python3 scripts/check_sitl_manual_input.py --listen 14550

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
    parser.add_argument("--listen", type=int, default=14550, help="UDP listen port (default: 14550)")
    parser.add_argument("--timeout", type=float, default=60.0, help="Monitoring timeout seconds")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero when arm/input movement is not detected",
    )
    args = parser.parse_args()

    conn = mavutil.mavlink_connection(
        f"udpin:0.0.0.0:{int(args.listen)}",
        source_system=255,
        source_component=190,
    )
    print(f"[check] waiting heartbeat on udp:{args.listen} ...", flush=True)
    hb = conn.wait_heartbeat(timeout=20)
    print(
        f"[check] connected vehicle sysid={hb.get_srcSystem()} compid={hb.get_srcComponent()}",
        flush=True,
    )

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
                "Verify QGC Virtual Joystick enabled and mode is MANUAL.",
                flush=True,
            )
            return 3
    print("[check] done", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
