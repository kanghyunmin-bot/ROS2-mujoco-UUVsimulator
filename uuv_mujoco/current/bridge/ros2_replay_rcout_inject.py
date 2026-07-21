"""Injection ownership helpers for replay RCOUT plant input."""

from __future__ import annotations


def _transport_injector(bridge: object):
    transport = getattr(bridge, "_sitl_transport", None)
    return getattr(transport, "inject_servo_pwm_values", None) if transport is not None else None


def inject_replay_rcout_channels(bridge: object, channels: list[int], *, source: str) -> bool:
    injector = _transport_injector(bridge)
    if callable(injector):
        with bridge._sitl_transport_lock:
            injector(channels, hold_s=1.0, source=source)
        return True
    handler = getattr(bridge, "_replay_rcout_handler", None)
    if callable(handler):
        handler(channels)
        return True
    return False


__all__ = ["inject_replay_rcout_channels"]
