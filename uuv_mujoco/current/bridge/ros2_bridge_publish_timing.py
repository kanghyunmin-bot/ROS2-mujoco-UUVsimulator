"""Publish-rate gate for Ros2Bridge sensor output."""

from __future__ import annotations


def publish_time_due(self, sim_t: float) -> bool:
    if sim_t + 1.0e-9 < self.last_pub_t:
        # MuJoCo reset and bag/clock rewinds must be observable by every
        # stateful sensor runtime.  Reopen the outer publish gate so those
        # runtimes can reset their schedules and seeded streams instead of
        # remaining permanently blocked behind the pre-reset timestamp.
        self.last_pub_t = -1.0
        rate_schedules = getattr(self, "_ros_sensor_rate_next_t", None)
        if rate_schedules is not None:
            rate_schedules.clear()
    if sim_t <= self.last_pub_t:
        return False
    if self.last_pub_t >= 0.0 and sim_t + 1.0e-9 < (self.last_pub_t + self.sensor_dt):
        return False
    self.last_pub_t = sim_t
    return True


__all__ = ["publish_time_due"]
