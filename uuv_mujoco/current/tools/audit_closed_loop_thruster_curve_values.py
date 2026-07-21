"""Value conversion helpers for thruster curve audits."""

from __future__ import annotations

from typing import Any


def finite_float_or_none(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if result == result else None


def curve_voltage_from_mapping_key(key: Any, curve: Any) -> float | None:
    voltage = finite_float_or_none(key)
    if voltage is not None:
        return voltage
    if not isinstance(curve, dict):
        return None
    return finite_float_or_none(curve.get("voltage", "nan"))


def curve_voltage_from_row(curve: Any) -> float | None:
    if not isinstance(curve, dict):
        return None
    return finite_float_or_none(curve.get("voltage_v", curve.get("voltage")))


def append_force_value(force_values: list[float], value: Any) -> None:
    force = finite_float_or_none(value)
    if force is not None:
        force_values.append(force)


__all__ = [
    "append_force_value",
    "curve_voltage_from_mapping_key",
    "curve_voltage_from_row",
    "finite_float_or_none",
]
