#!/usr/bin/env python3
"""Wait for a fresh MAVLink HEARTBEAT on a UDP endpoint without pymavlink."""

from __future__ import annotations

import argparse
import socket
import time


def _heartbeat_matches(
    payload: bytes,
    *,
    target_sysid: int,
    target_compid: int,
    target_autopilot: int,
) -> bool:
    index = 0
    size = len(payload)
    while index < size:
        magic = payload[index]
        if magic == 0xFE and index + 8 <= size:
            frame_size = int(payload[index + 1]) + 8
            if index + frame_size <= size and payload[index + 5] == 0:
                sysid = int(payload[index + 3])
                compid = int(payload[index + 4])
                heartbeat_payload = index + 6
                autopilot = int(payload[heartbeat_payload + 5]) if payload[index + 1] >= 6 else -1
                if (
                    sysid == target_sysid
                    and compid == target_compid
                    and autopilot == target_autopilot
                ):
                    return True
            index += max(frame_size, 1)
            continue
        if magic == 0xFD and index + 12 <= size:
            signed = bool(payload[index + 2] & 0x01)
            frame_size = int(payload[index + 1]) + 12 + (13 if signed else 0)
            message_id = int.from_bytes(payload[index + 7:index + 10], "little")
            if index + frame_size <= size and message_id == 0:
                sysid = int(payload[index + 5])
                compid = int(payload[index + 6])
                heartbeat_payload = index + 10
                autopilot = int(payload[heartbeat_payload + 5]) if payload[index + 1] >= 6 else -1
                if (
                    sysid == target_sysid
                    and compid == target_compid
                    and autopilot == target_autopilot
                ):
                    return True
            index += max(frame_size, 1)
            continue
        index += 1
    return False


def _contains_heartbeat(payload: bytes) -> bool:
    """Compatibility helper: accept only the expected ArduPilot FCU heartbeat."""
    return _heartbeat_matches(payload, target_sysid=1, target_compid=1, target_autopilot=3)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=14551)
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--target-sysid", type=int, default=1)
    parser.add_argument("--target-compid", type=int, default=1)
    parser.add_argument("--target-autopilot", type=int, default=3)
    args = parser.parse_args()

    deadline = time.monotonic() + max(args.timeout, 0.1)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((args.bind_host, args.port))
        sock.settimeout(0.5)
        while time.monotonic() < deadline:
            try:
                payload, peer = sock.recvfrom(65535)
            except socket.timeout:
                continue
            if _heartbeat_matches(
                payload,
                target_sysid=args.target_sysid,
                target_compid=args.target_compid,
                target_autopilot=args.target_autopilot,
            ):
                print(
                    "[mavlink-readiness] heartbeat=fresh "
                    f"fcu={args.target_sysid}.{args.target_compid} "
                    f"autopilot={args.target_autopilot} "
                    f"endpoint=udp://{args.bind_host}:{args.port} "
                    f"peer={peer[0]}:{peer[1]}",
                    flush=True,
                )
                return 0
    print(
        f"[mavlink-readiness] heartbeat=missing endpoint=udp://{args.bind_host}:{args.port} timeout={args.timeout:.1f}s",
        flush=True,
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
