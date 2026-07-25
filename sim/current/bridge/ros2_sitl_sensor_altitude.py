"""DVL altitude source selection for SITL sensor snapshots."""

from __future__ import annotations

from typing import Any


def dvl_altitude_from_sensor_or_model(owner: Any, data: Any) -> float | None:
    dvl_altitude_sensor = owner._sensor_slice(owner.model, owner.sensor_ids, "dvl_altitude", data)
    dvl_altitude_m = (
        float(dvl_altitude_sensor[0])
        if dvl_altitude_sensor is not None and dvl_altitude_sensor.size > 0
        else None
    )
    if owner._sitl_transport is not None and dvl_altitude_m is None:
        try:
            dvl_altitude_m = owner._sitl_transport.sitl_rangefinder_from_model(data)
        except Exception:
            dvl_altitude_m = None
    return dvl_altitude_m


__all__ = ["dvl_altitude_from_sensor_or_model"]
