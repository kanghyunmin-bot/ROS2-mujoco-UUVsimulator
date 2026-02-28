#!/usr/bin/env python3
"""Check whether QGC manual input reaches ArduSub SITL.

Usage:
  python3 scripts/check_sitl_manual_input.py --mode udp --listen 14550
  python3 scripts/check_sitl_manual_input.py --mode tcp --host 127.0.0.1 --port 5760

What to look for:
  - `armed=True` and any RC/SERVO channel values move away from 1500
  - If all channels are neutral while moving QGC virtual joystick, QGC input path is not flowing
"""

from __future__ import annotations

import argparse
import time
from typing import Sequence

from pymavlink import mavutil


RC_CHANNEL_FIELDS = tuple(f"chan{i}_raw" for i in range(1, 9))
SERVO_FIELDS = tuple(f"servo{i}_raw" for i in range(1, 9))


def _collect_values(msg: object, names: Sequence[str]) -> tuple[int, ...]:
    d = msg.to_dict() if hasattr(msg, "to_dict") else {}
    values: list[int] = []
    for name in names:
        values.append(int(d.get(name, 0) or 0))
    return tuple(values)


def main() -> int:
    parser = argparse.ArgumentParser(description="Monitor SITL manual input flow")
    parser.add_argument("--mode", choices=("tcp", "udp"), default="udp", help="MAVLink transport mode")
    parser.add_argument("--host", default="127.0.0.1", help="TCP host (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5760, help="TCP port (default: 5760)")
    parser.add_argument("--listen", type=int, default=14550, help="UDP listen port for --mode udp")
    parser.add_argument("--target-sysid", type=int, default=0, help="Filter HEARTBEAT by this sysid (0 for first heartbeat)")
    parser.add_argument("--target-compid", type=int, default=0, help="Filter HEARTBEAT by this compid (0 for any)")
    parser.add_argument("--source-sysid", type=int, default=255, help="MAVLink source system id for this checker")
    parser.add_argument("--source-compid", type=int, default=190, help="MAVLink source component id for this checker")
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
        conn = mavutil.mavlink_connection(
            endpoint,
            source_system=int(args.source_sysid),
            source_component=int(args.source_compid),
        )
    except Exception as exc:
        print(f"[check] FAIL: cannot open MAVLink endpoint {endpoint}: {exc}", flush=True)
        return 4
    print(f"[check] waiting heartbeat on {endpoint} ...", flush=True)
    target_sysid = int(args.target_sysid)
    target_compid = int(args.target_compid)
    hb = None
    hb_deadline = time.time() + float(args.hb_timeout)
    while time.time() < hb_deadline:
        msg = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
        if msg is None:
            continue
        if target_sysid and msg.get_srcSystem() != target_sysid:
            continue
        if target_compid and msg.get_srcComponent() != target_compid:
            continue
        hb = msg
        break
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
    last_servo = None
    last_manual = None
    armed_seen = False
    rc_active_seen = False
    servo_active_seen = False
    rc_active_channels: set[int] = set()
    servo_active_channels: set[int] = set()
    manual_control_seen = False
    manual_control_nonzero_seen = False
    manual_mode_hint_seen = False
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
            manual_mode = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)
            if armed:
                armed_seen = True
            if manual_mode:
                manual_mode_hint_seen = True
            if now - last_print > 0.8:
                print(
                    f"[hb] mode={int(d.get('custom_mode', 0))} base_mode={base_mode} "
                    f"armed={armed} manual_mode={manual_mode}",
                    flush=True,
                )
                last_print = now
        elif mtype == "MANUAL_CONTROL":
            d = msg.to_dict()
            mc = (
                int(d.get("x", 0)),
                int(d.get("y", 0)),
                int(d.get("z", 0)),
                int(d.get("r", 0)),
            )
            if mc != last_manual:
                print(f"[manual] x,y,z,r={mc}", flush=True)
                last_manual = mc
            manual_control_seen = True
            if any(v != 0 for v in mc):
                manual_control_nonzero_seen = True
        elif mtype in {"RC_CHANNELS", "RC_CHANNELS_RAW"}:
            rc = _collect_values(msg, RC_CHANNEL_FIELDS)
            if rc != last_rc:
                print(
                    "[rc] ch1..8="
                    + ",".join(f"{v:4d}" for v in rc),
                    flush=True,
                )
                last_rc = rc
            for idx, value in enumerate(rc, start=1):
                if abs(value - 1500) > 15:
                    rc_active_seen = True
                    rc_active_channels.add(idx)
        elif mtype == "SERVO_OUTPUT_RAW":
            servo = _collect_values(msg, SERVO_FIELDS)
            if servo != last_servo:
                print(
                    "[servo] s1..s8="
                    + ",".join(f"{v:4d}" for v in servo),
                    flush=True,
                )
                last_servo = servo
            for idx, value in enumerate(servo, start=1):
                if abs(value - 1500) > 15:
                    servo_active_seen = True
                    servo_active_channels.add(idx)
        elif mtype == "STATUSTEXT":
            d = msg.to_dict()
            text = str(d.get("text", "")).strip()
            if text:
                print(f"[status] {text}", flush=True)

    print(
        f"[check] summary "
        f"armed_seen={armed_seen} "
        f"manual_mode_hint={manual_mode_hint_seen} "
        f"rc_active_seen={rc_active_seen} "
        f"rc_active_channels={sorted(rc_active_channels)} "
        f"manual_control_seen={manual_control_seen} "
        f"manual_control_nonzero={manual_control_nonzero_seen} "
        f"servo_active_seen={servo_active_seen} "
        f"servo_active_channels={sorted(servo_active_channels)}",
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
                "Verify QGC Manual control is active and joystick link is connected.",
                flush=True,
            )
            return 3
        if not manual_control_seen and not rc_active_seen and not servo_active_seen:
            print(
                "[check] NOTE: no MANUAL_CONTROL messages observed. "
                "This may happen when QGC does not stream this message type.",
                flush=True,
            )
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
