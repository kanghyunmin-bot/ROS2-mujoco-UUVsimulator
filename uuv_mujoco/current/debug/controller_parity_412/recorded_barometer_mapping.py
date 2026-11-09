"""Map recorded water-barometer identity to the virtual sensor identity."""

from __future__ import annotations

from collections.abc import Mapping
import math


def barometer_parameters_from_recorded_primary(
    parameters: Mapping[str, str], field: Mapping[str, float]
) -> tuple[dict[str, str], dict[str, float | int | str]]:
    """Map recorded ground pressure [Pa] to ArduSub SITL's first water sensor."""
    try:
        raw_index = float(field["BARO_PRIMARY"])
        if not math.isfinite(raw_index) or not raw_index.is_integer() or not 0 <= raw_index <= 2:
            raise ValueError("Invalid recorded primary barometer index")
        source_index = int(raw_index)
        source_key = f"BARO{source_index + 1}_GND_PRESS"
        ground_pressure_pa = float(field[source_key])
    except (KeyError, TypeError) as error:
        raise ValueError("Recorded primary barometer and its ground pressure are required") from error
    if not math.isfinite(ground_pressure_pa) or ground_pressure_pa <= 0:
        raise ValueError("Recorded ground pressure must be finite and positive")
    result = dict(parameters)
    # AP_Baro_SITL marks its sensors WATER; Sub selects the first water sensor.
    # This is a hardware identity adapter, not an altitude/output offset fit.
    result["BARO_PRIMARY"] = "0"
    result["BARO1_GND_PRESS"] = repr(ground_pressure_pa)
    return result, {
        "recorded_sensor_index": source_index, "recorded_ground_parameter": source_key,
        "ground_pressure_pa": ground_pressure_pa, "virtual_sensor_index": 0,
        "virtual_ground_parameter": "BARO1_GND_PRESS",
        "provenance": "Recorded parameter; not estimated from the replay residual",
    }
