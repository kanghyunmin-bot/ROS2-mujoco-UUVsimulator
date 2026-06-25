"""Runtime-readiness assembly for GUI command-readiness checks."""

from __future__ import annotations

from .models import TelemetrySnapshot
from .node_readiness_freshness import depth_fresh, imu_fresh, rcout_fresh, state_fresh
from sim.runtime.readiness import RuntimeReadiness


def build_runtime_readiness(*, snap: TelemetrySnapshot, command_alive: bool, extnav_ready: bool) -> RuntimeReadiness:
    fresh_state = state_fresh(snap)
    return RuntimeReadiness(
        mujoco_alive=fresh_state,
        json_sensor_transport_alive=bool(depth_fresh(snap) and imu_fresh(snap)),
        json_servo_receiver_alive=True,
        mavlink_command_endpoint_alive=command_alive,
        external_nav_alive=extnav_ready,
        vehicle_state_known=fresh_state,
        arm_state_known=fresh_state,
        mode_state_known=fresh_state,
        plant_servo_rows_available=rcout_fresh(snap),
    )


__all__ = ["build_runtime_readiness"]
