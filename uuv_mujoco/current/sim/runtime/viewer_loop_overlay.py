"""Viewer overlay text helpers for the MuJoCo runner."""

from __future__ import annotations

from typing import Any

from sim.runtime.viewer_overlay_text import command_overlay_text, sensor_overlay_text


class ViewerLoopOverlayMixin:
    def update_overlay(self, viewer: Any, *, forward: float, sway: float, yaw: float, heave: float) -> None:
        overlay_line = self.viewer_controls.base_overlay_line()
        if self.viewer_controls.show_sensor_overlay:
            sensor_overlay = self.sensor_overlay_line()
            if sensor_overlay:
                overlay_line = sensor_overlay
        viewer.set_texts(
            [
                (
                    None,
                    None,
                    command_overlay_text(
                        forward=forward,
                        sway=sway,
                        yaw=yaw,
                        heave=heave,
                        camera_mode=self.viewer_controls.camera_mode,
                    ),
                    overlay_line,
                )
            ]
        )

    def sensor_overlay_line(self) -> str:
        imu_g = self.sensor_value("imu_gyro")
        imu_a = self.sensor_value("imu_acc")
        dvl_v = self.sensor_value("dvl_vel_body")
        dvl_alt = self.sensor_value("dvl_altitude")
        depth_pos = self.sensor_value("depth_pos")
        return sensor_overlay_text(
            imu_g=imu_g,
            imu_a=imu_a,
            dvl_v=dvl_v,
            dvl_alt=dvl_alt,
            depth_pos=depth_pos,
        )


__all__ = ["ViewerLoopOverlayMixin"]
