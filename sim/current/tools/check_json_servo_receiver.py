#!/usr/bin/env python3
"""Smoke tests for the JSON-SITL servo receiver facade."""

from __future__ import annotations

from json_servo_receiver_smoke_cases import check_receive_decode_and_send_bookkeeping


def main() -> int:
    check_receive_decode_and_send_bookkeeping()
    print("json_servo_receiver=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
