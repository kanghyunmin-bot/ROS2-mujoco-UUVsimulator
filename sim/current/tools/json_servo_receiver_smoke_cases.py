"""Smoke-test cases for the JSON servo receiver facade."""

from __future__ import annotations

from json_servo_receiver_smoke_assertions import (
    assert_default_target_priority,
    assert_received_packet,
    assert_send_bookkeeping,
)
from json_servo_receiver_smoke_fixture import make_receiver, make_servo_packet, udp_socket


def check_receive_decode_and_send_bookkeeping() -> None:
    receiver = make_receiver()
    client = udp_socket()
    sink = udp_socket()
    try:
        sock = receiver.connect()
        pwm = [1500 + i for i in range(16)]
        client.sendto(b"bad", sock.getsockname())
        client.sendto(make_servo_packet(pwm), sock.getsockname())
        assert_received_packet(receiver, pwm)
        assert_default_target_priority(receiver)
        assert_send_bookkeeping(receiver, sink)
    finally:
        client.close()
        sink.close()
        receiver.close()
    if receiver.socket is not None:
        raise AssertionError("receiver socket was not cleared by close")


__all__ = ["check_receive_decode_and_send_bookkeeping"]
