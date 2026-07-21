"""ROS static-pressure output source configuration."""

from __future__ import annotations

import os

from bridge.sitl_env import env_to_float


VALID_STATIC_PRESSURE_SOURCES = {"internal", "external"}


def configure_static_pressure_output_contract(bridge: object) -> None:
    bridge._static_pressure_source = str(
        os.environ.get("ROS2_UUV_STATIC_PRESSURE_SOURCE", "external")
    ).strip().lower()
    if bridge._static_pressure_source not in VALID_STATIC_PRESSURE_SOURCES:
        bridge._static_pressure_source = "external"
    bridge._internal_pressure_pa = float(
        env_to_float("ROS2_UUV_INTERNAL_PRESSURE_PA", bridge._bar30_surface_pressure_pa)
    )


__all__ = ["VALID_STATIC_PRESSURE_SOURCES", "configure_static_pressure_output_contract"]
