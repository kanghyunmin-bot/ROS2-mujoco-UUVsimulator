"""Velocity callbacks for UuvGuiNode."""

from __future__ import annotations


def _on_velocity(self, msg: TwistStamped, source: str) -> None:
    with self._lock:
        self._snapshot.velocity_xyz = (
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        )
        self._snapshot.velocity_source = source


def _on_velocity_body(self, msg: TwistStamped) -> None:
    self._on_velocity(msg, self._topic("local_position/velocity_body"))


def _on_velocity_local(self, msg: TwistStamped) -> None:
    self._on_velocity(msg, self._topic("local_position/velocity_local"))


def _on_dvl_velocity(self, msg: TwistStamped) -> None:
    self._on_velocity(msg, "/dvl/velocity")


__all__ = ["_on_dvl_velocity", "_on_velocity", "_on_velocity_body", "_on_velocity_local"]
