"""Depth and pressure callbacks for UuvGuiNode."""

from __future__ import annotations

import math


def _on_depth(self, msg: Float32) -> None:
    self._touch("depth")
    with self._lock:
        self._snapshot.depth_m = float(msg.data)
        self._snapshot.depth_source = "/depth"


def _on_bar30_pressure(self, msg: Float32) -> None:
    self._on_pressure_value(float(msg.data), "/bar30/pressure_pa")


def _on_atm_pressure(self, msg: FluidPressure) -> None:
    self._on_pressure_value(float(msg.fluid_pressure), self._topic("imu/atm_pressure"))


def _on_static_pressure(self, msg: FluidPressure) -> None:
    self._on_pressure_value(float(msg.fluid_pressure), self._topic("imu/static_pressure"))


def _on_pressure_value(self, pressure_pa: float, source: str) -> None:
    self._touch("depth")
    rho = 997.0
    g = 9.80665
    approx_depth = max(0.0, (pressure_pa - 101325.0) / (rho * g))
    with self._lock:
        self._snapshot.pressure_pa = pressure_pa
        if not math.isfinite(self._snapshot.depth_m):
            self._snapshot.depth_m = approx_depth
            self._snapshot.depth_source = f"{source} (approx)"


__all__ = [
    "_on_atm_pressure",
    "_on_bar30_pressure",
    "_on_depth",
    "_on_pressure_value",
    "_on_static_pressure",
]
