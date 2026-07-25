"""Subscription callbacks for the roll stability probe."""

from __future__ import annotations

import time
from typing import Any

from roll_stability_metrics import quat_to_rpy_deg


class RollStabilityCallbacksMixin:
    def elapsed(self) -> float:
        return time.monotonic() - self.t0

    def _on_state(self, msg) -> None:
        self.state = msg

    def _on_imu(self, msg) -> None:
        self.latest_gyro = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
        )

    def _on_depth(self, msg) -> None:
        self.latest_depth = float(msg.data)
        if self.sample_enabled:
            self.depth_samples.append((self.elapsed(), self.latest_depth))

    def _on_rc_out(self, msg) -> None:
        if self.sample_enabled:
            self.rc_samples.append((self.elapsed(), [int(v) for v in msg.channels]))

    def _on_pose(self, msg) -> None:
        if not self.sample_enabled:
            return
        q = msg.pose.orientation
        roll, pitch, yaw = quat_to_rpy_deg(float(q.w), float(q.x), float(q.y), float(q.z))
        gx, gy, gz = self.latest_gyro
        self.samples.append(
            {
                "t": self.elapsed(),
                "roll_deg": roll,
                "pitch_deg": pitch,
                "yaw_deg": yaw,
                "z_m": float(msg.pose.position.z),
                "depth_m": self.latest_depth,
                "gyro_x": gx,
                "gyro_y": gy,
                "gyro_z": gz,
            }
        )


__all__ = ["RollStabilityCallbacksMixin"]
