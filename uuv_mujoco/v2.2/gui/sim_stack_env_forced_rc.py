"""Forced GUI-start RC override environment contract."""

from __future__ import annotations


def rc_override_contract() -> dict[str, str]:
    return {
        # GUI-started SITL must keep pilot input on the same ArduSub
        # RC_CHANNELS_OVERRIDE path that the dist stack used successfully.
        # Direct MuJoCo fallback is only an explicit debug override.
        "ROS2_UUV_MAVROS_RC_OVERRIDE_LOCAL_FALLBACK": "0",
        "ROS2_UUV_MAVROS_RC_PWM_SPAN": "300",
        "UUV_GUI_RC_PWM_SPAN": "300",
        "UUV_GUI_PILOT_CONTROL_MODE": "rc_override",
        "ROS2_UUV_MAVROS_RC_OVERRIDE_BACKEND": "rc_channels_override",
        # Real/QGC/rosbag RC values stay on the 1500-centered convention.
        # ArduSub 4.1.2 with RC3_MIN/MAX=1100/1900 interprets RC3=1500 as neutral
        # in ALT_HOLD; remapping it corrupts the vertical target.
        "ROS2_UUV_SITL_NEUTRAL_RC_KEEPALIVE": "0",
    }


__all__ = ["rc_override_contract"]
