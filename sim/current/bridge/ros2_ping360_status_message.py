"""Ping360 status payload and ROS2 String message builders."""

from __future__ import annotations

import json
from typing import Any

from .ping360_types import Ping360Config, Ping360Sample


def build_ping360_status_msg(string_type: type, stamp: Any, payload: dict[str, Any]) -> Any:
    msg = string_type()
    payload = dict(payload)
    payload["stamp"] = {
        "sec": int(getattr(stamp, "sec", 0)),
        "nanosec": int(getattr(stamp, "nanosec", 0)),
    }
    msg.data = json.dumps(payload, sort_keys=True)
    return msg


def build_ping360_status_payload(
    *,
    sim_t: float,
    config: Ping360Config,
    ping360: Any | None,
    sample: Ping360Sample | None,
    site_present: bool,
) -> dict[str, Any]:
    if ping360 is None or not getattr(ping360, "active", False):
        payload = {
            "sim_time_s": float(sim_t),
            "active": False,
            "enabled": bool(config.enabled),
            "site_present": bool(site_present),
            "updated": False,
            "settings": {
                "requested_range_m": float(config.requested_range_m),
                "num_steps": int(config.num_steps),
                "gain_setting": int(config.gain_setting),
                "interface_mode": str(config.interface_mode),
                "transmit_frequency_khz": int(config.transmit_frequency_khz),
                "start_angle_grad": int(config.start_angle_grad),
                "stop_angle_grad": int(config.stop_angle_grad),
            },
        }
    elif sample is not None:
        payload = sample.status_dict()
        payload["updated"] = bool(sample.updated)
    else:
        payload = ping360.status_dict(sim_t)
    payload["enabled"] = bool(config.enabled)
    return payload


__all__ = ["build_ping360_status_msg", "build_ping360_status_payload"]
