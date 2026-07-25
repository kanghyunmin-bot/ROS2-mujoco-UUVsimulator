"""Ping360 ROS topic scheduling."""

from __future__ import annotations


def schedule_ping360_ros_jobs(self, jobs, add_rate_limited, *, builders: dict[str, object]) -> None:
    publish_hz = float(getattr(self, "_ping360_publish_hz", 10.0))
    if self._ping360_config.publish_image:
        add_rate_limited(self.pub_ping360_image, "/ping360/image", builders["ping360_image"], publish_hz, on_demand=True)
        add_rate_limited(
            self.pub_ping360_scan_image,
            "/ping360/scan_image",
            builders["ping360_image"],
            publish_hz,
            on_demand=True,
        )
    if self._ping360_config.publish_scan:
        add_rate_limited(self.pub_ping360_scan, "/ping360/scan", builders["ping360_scan"], publish_hz, on_demand=True)
    if self._ping360_config.publish_echo and self.pub_ping360_echo is not None:
        add_rate_limited(self.pub_ping360_echo, "/ping360/scan_echo", builders["ping360_echo"], publish_hz, on_demand=True)
        add_rate_limited(self.pub_ping360_echo_alias, "/ping360/echo", builders["ping360_echo"], publish_hz, on_demand=True)
    if self._ping360_config.publish_status:
        jobs.add(self.pub_ping360_status, "/ping360/status", builders["ping360_status"], on_demand=True)


__all__ = ["schedule_ping360_ros_jobs"]
