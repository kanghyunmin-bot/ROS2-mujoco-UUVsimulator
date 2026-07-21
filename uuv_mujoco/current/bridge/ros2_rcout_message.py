"""MAVROS-compatible RCOut message construction."""

from __future__ import annotations

from .ros2_rcout_stamp import sensor_replay_real_stamp


def build_sitl_servo_rcout_message(bridge, pwm_values: list[int]):
    rc_out = bridge.RCOut()
    header = getattr(rc_out, "header", None)
    if header is not None and bridge.node is not None:
        stamp_msg = sensor_replay_real_stamp(bridge, type(header.stamp))
        header.stamp = stamp_msg if stamp_msg is not None else bridge.node.get_clock().now().to_msg()
        if hasattr(header, "frame_id"):
            header.frame_id = "sensor_replay_real" if stamp_msg is not None else "fcu"

    channels = [int(v) for v in list(pwm_values)[:18]]
    if hasattr(rc_out, "channels"):
        rc_out.channels = channels
    return rc_out


__all__ = ["build_sitl_servo_rcout_message", "sensor_replay_real_stamp"]
