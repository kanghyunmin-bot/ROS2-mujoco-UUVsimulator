"""RC override frame construction for the roll stability probe."""

from __future__ import annotations

from mavros_msgs.msg import OverrideRCIn


def clamp_norm(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def rc_pwm_from_norm(value: float) -> int:
    return int(round(1500 + 300.0 * clamp_norm(value)))


def build_roll_stability_rc_frame(
    *,
    roll: float = 0.0,
    pitch: float = 0.0,
    forward: float = 0.0,
    sway: float = 0.0,
    yaw: float = 0.0,
    heave: float = 0.0,
) -> OverrideRCIn:
    msg = OverrideRCIn()
    for idx in range(len(msg.channels)):
        msg.channels[idx] = 0
    for idx in range(min(8, len(msg.channels))):
        msg.channels[idx] = 1500
    if len(msg.channels) >= 6:
        # ArduSub defaults: RC1=pitch, RC2=roll, RC3=heave, RC4=yaw,
        # RC5=forward, RC6=lateral.
        msg.channels[0] = rc_pwm_from_norm(pitch)
        msg.channels[1] = rc_pwm_from_norm(roll)
        msg.channels[2] = rc_pwm_from_norm(heave)
        msg.channels[3] = rc_pwm_from_norm(yaw)
        msg.channels[4] = rc_pwm_from_norm(forward)
        msg.channels[5] = rc_pwm_from_norm(sway)
    return msg


__all__ = ["build_roll_stability_rc_frame", "clamp_norm", "rc_pwm_from_norm"]
