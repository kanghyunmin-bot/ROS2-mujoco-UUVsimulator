"""Ping360 config publishers for UuvGuiNode."""

from __future__ import annotations

import json

from .runtime import String


def publish_ping360_config(
    self,
    *,
    range_m: float,
    num_steps: int,
    gain: int,
    interface_mode: str,
    frequency_khz: int,
    start_angle_grad: int,
    stop_angle_grad: int,
) -> None:
    payload = {
        "requested_range_m": float(range_m),
        "num_steps": int(num_steps),
        "gain_setting": int(gain),
        "interface_mode": str(interface_mode),
        "transmit_frequency_khz": int(frequency_khz),
        "start_angle_grad": int(start_angle_grad),
        "stop_angle_grad": int(stop_angle_grad),
    }
    msg = String()
    msg.data = json.dumps(payload, sort_keys=True)
    self._ping360_config_pub.publish(msg)
    self._push_event(
        "ping360 config -> "
        f"range={range_m:g}m step={num_steps} gain={gain} "
        f"{interface_mode} {frequency_khz}kHz sector={start_angle_grad}..{stop_angle_grad}grad"
    )


def publish_ping360_enabled(self, enabled: bool) -> None:
    msg = String()
    msg.data = json.dumps({"enabled": bool(enabled)}, sort_keys=True)
    self._ping360_config_pub.publish(msg)
    self._push_event(f"ping360 sonar -> {'on' if enabled else 'off'}")


__all__ = ["publish_ping360_config", "publish_ping360_enabled"]
