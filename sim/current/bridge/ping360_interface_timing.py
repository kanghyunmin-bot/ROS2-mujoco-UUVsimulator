"""Ping360 interface timing helpers."""

from __future__ import annotations

import numpy as np

from .ping360_types import PING360_GRADS_PER_REV


INTERFACE_FULL_SCAN_S = {
    "usb": ((1.0, 4.17), (50.0, 33.0)),
    "ethernet": ((1.0, 3.42), (50.0, 33.0)),
    "rs485": ((1.0, 4.26), (50.0, 33.0)),
}


def profile_period_s(effective_range_m: float, num_steps: int, interface_mode: str) -> float:
    one_step_full = full_scan_period_one_grad_s(effective_range_m, interface_mode)
    motor_full_s = 2.4
    listen_full_s = max(0.0, one_step_full - motor_full_s)
    motor_per_grad_s = motor_full_s / PING360_GRADS_PER_REV
    listen_per_ping_s = listen_full_s / PING360_GRADS_PER_REV
    return max(0.001, motor_per_grad_s * num_steps + listen_per_ping_s)


def full_scan_period_one_grad_s(effective_range_m: float, interface_mode: str) -> float:
    mode = normalize_interface(interface_mode)
    (r0, t0), (r1, t1) = INTERFACE_FULL_SCAN_S[mode]
    r = float(np.clip(effective_range_m, r0, r1))
    if r1 <= r0:
        return t1
    alpha = (r - r0) / (r1 - r0)
    return (1.0 - alpha) * t0 + alpha * t1


def normalize_interface(interface_mode: str) -> str:
    mode = str(interface_mode or "ethernet").strip().lower()
    return mode if mode in INTERFACE_FULL_SCAN_S else "ethernet"


__all__ = [
    "INTERFACE_FULL_SCAN_S",
    "full_scan_period_one_grad_s",
    "normalize_interface",
    "profile_period_s",
]
