"""Assertions for JSON servo receiver smoke tests."""

from __future__ import annotations

from json_servo_receiver_smoke_fixture import receive_until


def assert_received_packet(receiver, pwm: list[int]) -> None:
    packets = receive_until(receiver, 1)
    if len(packets) != 1:
        raise AssertionError(f"expected one decoded servo packet, got {len(packets)}")
    packet, addr = packets[0]
    if packet.pwm_values != pwm or packet.frame_count != 7 or packet.frame_rate_hz != 400:
        raise AssertionError(f"decoded servo packet mismatch: {packet}")
    if addr[0] != "127.0.0.1":
        raise AssertionError(f"unexpected client address: {addr}")


def assert_default_target_priority(receiver) -> None:
    receiver.client_addr = ("127.0.0.1", 54321)
    if receiver.default_send_target() != ("127.0.0.1", 54321):
        raise AssertionError("client address must win over sensor target")


def assert_send_bookkeeping(receiver, sink) -> None:
    sink.bind(("127.0.0.1", 0))
    sink.settimeout(1.0)
    sent, target = receiver.send_bytes(b"{}", target=sink.getsockname())
    if sent != 2 or target != sink.getsockname() or receiver.send_counter != 1:
        raise AssertionError("send telemetry did not update")
    payload, _sender = sink.recvfrom(16)
    if payload != b"{}":
        raise AssertionError(f"unexpected sensor payload: {payload!r}")


__all__ = ["assert_default_target_priority", "assert_received_packet", "assert_send_bookkeeping"]
