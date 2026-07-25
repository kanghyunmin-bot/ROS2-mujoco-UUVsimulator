"""ROS2 /cmd_vel command callback for Ros2Bridge."""

from __future__ import annotations

import numpy as np


def _on_cmd_vel_stamped(self, msg) -> None:
    twist = getattr(msg, "twist", msg)
    fwd = float(np.clip(getattr(twist.linear, "x", 0.0), -1.0, 1.0))
    left = float(np.clip(getattr(twist.linear, "y", 0.0), -1.0, 1.0))
    up = float(np.clip(getattr(twist.linear, "z", 0.0), -1.0, 1.0))
    yaw = float(np.clip(getattr(twist.angular, "z", 0.0), -1.0, 1.0))
    if self._sitl_transport is not None:
        if not self._sitl_cmd_vel_setpoint_enabled:
            if not self._sitl_cmd_vel_blocked_warned and self.node is not None:
                self._sitl_cmd_vel_blocked_warned = True
                self.node.get_logger().warn(
                    "/cmd_vel ignored in SITL closed-loop mode. "
                    "Use /mavros/rc/override for ALT_HOLD validation, or set "
                    "ROS2_UUV_SITL_CMD_VEL_SETPOINT_ENABLE=1 for guided-setpoint smoke tests."
                )
            return
        if not self._sitl_cmd_vel_warned and self.node is not None:
            self._sitl_cmd_vel_warned = True
            self.node.get_logger().warn(
                "/cmd_vel is forwarded as MAVLink body-velocity setpoint; "
                "ArduSub only accepts it in guided setpoint modes. "
                "Use /mavros/rc/override for ALT_HOLD replay."
            )
        with self._sitl_transport_lock:
            self._sitl_transport.send_body_velocity_setpoint(
                forward_mps=fwd,
                left_mps=left,
                up_mps=up,
                yaw_rate_rad_s=yaw,
            )
        return
    self._handle_normalized_cmd(fwd, left, -yaw, -up)


__all__ = ["_on_cmd_vel_stamped"]
