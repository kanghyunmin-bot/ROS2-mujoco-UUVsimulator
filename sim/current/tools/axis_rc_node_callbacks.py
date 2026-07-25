"""ROS subscription callbacks and sampling for axis RC checks."""

from __future__ import annotations

import time

from axis_rc_sampling import build_axis_sample


class AxisRcNodeCallbacks:
    def _on_state(self, msg) -> None:
        self.state = msg

    def _on_rc_in(self, msg) -> None:
        self.rc_in = msg

    def _on_rc_out(self, msg) -> None:
        self.rc_out = msg

    def _on_imu(self, msg) -> None:
        self.imu = msg

    def _on_dvl_twist(self, msg) -> None:
        self.dvl_twist = msg

    def _on_depth(self, msg) -> None:
        self.depth = msg

    def _on_local_odom(self, msg) -> None:
        self.local_odom = msg

    def _sample(self) -> None:
        now = time.monotonic()
        if self._last_sample_wall > 0.0 and now - self._last_sample_wall < self.sample_dt * 0.9:
            return
        self._last_sample_wall = now
        if not self.recording:
            return
        self.samples.append(
            build_axis_sample(
                elapsed_s=self.elapsed(),
                wall_mono_s=now,
                phase=self.current_phase,
                state=self.state,
                imu=self.imu,
                dvl_twist=self.dvl_twist,
                depth=self.depth,
                local_odom=self.local_odom,
                rc_in=self.rc_in,
                rc_out=self.rc_out,
                command_axis=self.current_axis,
                command_value=self.current_command,
                command_publish_t=self.last_command_publish_t,
                command_sequence=self.last_command_sequence,
                command_mode=self.last_command_mode,
            )
        )


__all__ = ["AxisRcNodeCallbacks"]
