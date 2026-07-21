"""Publish-rate gate for Ros2Bridge sensor output."""

from __future__ import annotations


def publish_time_due(self, sim_t: float) -> bool:
    if sim_t <= self.last_pub_t:
        return False
    if self.last_pub_t >= 0.0 and sim_t + 1.0e-9 < (self.last_pub_t + self.sensor_dt):
        return False
    self.last_pub_t = sim_t
    return True


__all__ = ["publish_time_due"]
