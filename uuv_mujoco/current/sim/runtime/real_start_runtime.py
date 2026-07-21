"""Runtime wrapper for publishing real-start state status."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .real_start import (
    EnvFloat,
    RealStartStatusPublisher,
    build_real_start_status_payload,
)


@dataclass
class RealStartRuntimeStatus:
    """Build and publish real-start status from live MuJoCo state callbacks."""

    publisher: RealStartStatusPublisher
    env_float: EnvFloat
    required: bool
    hold_active_fn: Callable[[], bool]
    base_depth_m_fn: Callable[[], float]
    bar30_depth_m_fn: Callable[[], float]
    base_xy_m_fn: Callable[[], Any]
    current_rpy_rad_fn: Callable[[], tuple[float, float, float]]
    release_linear_velocity_body_fn: Callable[[], Any]
    release_angular_velocity_body_fn: Callable[[], Any]
    model_density_fn: Callable[[], float]
    depth_tolerance_m: float
    attitude_tolerance_rad: float
    velocity_tolerance_mps: float
    latched_payload: dict[str, object] | None = None

    @classmethod
    def create(cls, *, ros_bridge, **kwargs) -> "RealStartRuntimeStatus":
        """Create a status publisher bound to the supplied ROS bridge."""
        return cls(
            publisher=RealStartStatusPublisher.create(ros_bridge),
            **kwargs,
        )

    def payload(self) -> dict[str, object]:
        """Return the current real-start status payload."""
        payload, updated_latch = build_real_start_status_payload(
            env_float=self.env_float,
            required=self.required,
            hold_active=bool(self.hold_active_fn()),
            latched_payload=self.latched_payload,
            base_depth_m=float(self.base_depth_m_fn()),
            bar30_depth_m=float(self.bar30_depth_m_fn()),
            base_xy_m=self.base_xy_m_fn(),
            current_rpy_rad=self.current_rpy_rad_fn(),
            release_linear_velocity_body=self.release_linear_velocity_body_fn(),
            release_angular_velocity_body=self.release_angular_velocity_body_fn(),
            model_density=float(self.model_density_fn()),
            depth_tolerance_m=self.depth_tolerance_m,
            attitude_tolerance_rad=self.attitude_tolerance_rad,
            velocity_tolerance_mps=self.velocity_tolerance_mps,
        )
        if updated_latch is not None:
            self.latched_payload = dict(updated_latch)
        return payload

    def publish(self) -> None:
        """Publish the current real-start status payload."""
        self.publisher.publish(self.payload())


__all__ = ["RealStartRuntimeStatus"]
