"""Bridge lifecycle adapter for the Water Linked A50 TCP emulator.

Strict real-package compatibility requires the device boundary: the bridge
exposes newline-delimited JSON on TCP and the physical ``auv_dvl_a50`` driver
owns ``/dvl/data`` and ``/dvl/position``. Non-strict runs keep the direct ROS
publisher path unless the emulator is explicitly requested.
"""

from __future__ import annotations

import os
from typing import Any

from .dvl_a50_tcp_emulator import A50TcpJsonEmulator, DEFAULT_HOST, DEFAULT_PORT
from .ros2_dvl_sensor_runtime import calibrate_dvl_gyro, reset_dvl_dead_reckoning
from .sitl_env import env_flag, env_to_int


ENABLE_ENV = "ROS2_UUV_DVL_DEVICE_EMULATOR_ENABLE"
HOST_ENV = "ROS2_UUV_DVL_DEVICE_EMULATOR_HOST"
PORT_ENV = "ROS2_UUV_DVL_DEVICE_EMULATOR_PORT"
QUEUE_SIZE_ENV = "ROS2_UUV_DVL_DEVICE_EMULATOR_QUEUE_SIZE"


def configure_dvl_device_emulator_runtime(bridge: Any) -> None:
    """Configure and optionally start the A50 device-protocol boundary."""

    strict_real_package_mode = bool(getattr(bridge, "_real_pkg_compat", False))
    enabled = env_flag(
        ENABLE_ENV,
        strict_real_package_mode,
    )
    if strict_real_package_mode and not enabled:
        raise ValueError(
            f"{ENABLE_ENV}=0 is incompatible with strict real-package "
            "compatibility; the physical A50 driver must be the sole owner of "
            "/dvl/data and /dvl/position"
        )
    bridge._dvl_device_emulator_enabled = enabled
    bridge._dvl_device_emulator = None
    bridge._dvl_device_emulator_seen_reset_count = 0
    bridge._dvl_device_emulator_seen_gyro_calibration_count = 0
    if not enabled:
        return
    if not bool(getattr(bridge, "_dvl_sensor_model_enabled", False)):
        raise ValueError(
            f"{ENABLE_ENV}=1 requires the A50 sensor model to be enabled"
        )

    host = os.environ.get(HOST_ENV, DEFAULT_HOST).strip() or DEFAULT_HOST
    port = env_to_int(PORT_ENV, DEFAULT_PORT, log_prefix="[dvl_a50_emulator]")
    queue_size = env_to_int(
        QUEUE_SIZE_ENV,
        128,
        log_prefix="[dvl_a50_emulator]",
    )
    if port > 65_535:
        raise ValueError(f"{PORT_ENV} must be in [0, 65535]")
    if queue_size < 1:
        raise ValueError(f"{QUEUE_SIZE_ENV} must be positive")

    emulator = A50TcpJsonEmulator(
        host=host,
        port=port,
        queue_size=queue_size,
    ).start()
    bridge._dvl_device_emulator = emulator
    bridge._dvl_device_emulator_seen_reset_count = (
        emulator.dead_reckoning_reset_count
    )
    bridge._dvl_device_emulator_seen_gyro_calibration_count = (
        emulator.gyro_calibration_count
    )
    print(
        "[dvl_a50_emulator] "
        f"listening={emulator.address[0]}:{emulator.address[1]} "
        "ros_owner=auv_dvl_a50",
        flush=True,
    )


def synchronize_dvl_device_commands(
    bridge: Any,
    *,
    sim_t: float | None = None,
    rot_world_body: Any | None = None,
) -> None:
    """Apply device commands received by the emulator to modeled state."""

    emulator = getattr(bridge, "_dvl_device_emulator", None)
    if emulator is None:
        return

    reset_count = emulator.dead_reckoning_reset_count
    if reset_count != bridge._dvl_device_emulator_seen_reset_count:
        bridge._dvl_device_emulator_seen_reset_count = reset_count
        reset_dvl_dead_reckoning(
            bridge,
            reset_time_s=sim_t,
            rot_world_body=rot_world_body,
        )
        reset_dvl_device_emulator_report_schedule(bridge)

    gyro_count = emulator.gyro_calibration_count
    if gyro_count != bridge._dvl_device_emulator_seen_gyro_calibration_count:
        bridge._dvl_device_emulator_seen_gyro_calibration_count = gyro_count
        calibrate_dvl_gyro(bridge)


def forward_dvl_delivery_to_device_emulator(
    bridge: Any,
    delivery: Any | None,
) -> None:
    """Forward one arrived sensor delivery through the physical A50 protocol."""

    emulator = getattr(bridge, "_dvl_device_emulator", None)
    if emulator is None or delivery is None:
        return

    sensor_rate_hz = float(bridge._dvl_sensor_timing_config.schedule.rate_hz)
    emulator.publish_velocity(delivery.sample, 1.0 / sensor_rate_hz)


def forward_dvl_position_to_device_emulator(
    bridge: Any,
    delivery: Any | None,
) -> None:
    """Forward one independent 5 Hz DR report through the A50 protocol."""

    emulator = getattr(bridge, "_dvl_device_emulator", None)
    if emulator is None or delivery is None:
        return
    emulator.publish_position(
        delivery,
        expected_reset_count=bridge._dvl_device_emulator_seen_reset_count,
    )


def reset_dvl_device_emulator_report_schedule(bridge: Any) -> None:
    """Preserve the independent 5 Hz report phase across a DR reset."""

    # Water Linked reset changes the DR origin, not the device report clock.
    # Kept as a compatibility hook for older callers.
    del bridge


def reset_dvl_device_emulator_dead_reckoning(bridge: Any) -> None:
    """Reset the emulated device generation and purge pre-reset wire reports."""

    emulator = getattr(bridge, "_dvl_device_emulator", None)
    if emulator is None:
        return
    bridge._dvl_device_emulator_seen_reset_count = (
        emulator.reset_dead_reckoning()
    )


def close_dvl_device_emulator(bridge: Any) -> None:
    """Stop the A50 TCP server if it was enabled."""

    emulator = getattr(bridge, "_dvl_device_emulator", None)
    if emulator is None:
        return
    emulator.stop()
    bridge._dvl_device_emulator = None


__all__ = [
    "ENABLE_ENV",
    "close_dvl_device_emulator",
    "configure_dvl_device_emulator_runtime",
    "forward_dvl_delivery_to_device_emulator",
    "forward_dvl_position_to_device_emulator",
    "reset_dvl_device_emulator_dead_reckoning",
    "reset_dvl_device_emulator_report_schedule",
    "synchronize_dvl_device_commands",
]
