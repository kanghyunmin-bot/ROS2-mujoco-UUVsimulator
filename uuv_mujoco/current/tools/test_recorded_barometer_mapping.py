"""Physical barometer instance numbers differ from virtual water-sensor order."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "debug/controller_parity_412"))
from recorded_barometer_mapping import barometer_parameters_from_recorded_primary


def test_real_secondary_water_sensor_maps_to_first_virtual_water_sensor():
    old = {"BARO_PRIMARY": "1", "BARO1_GND_PRESS": "101473.796875", "BARO2_GND_PRESS": "102230",
           "ATC_RAT_RLL_P": ".135"}
    field = {"BARO_PRIMARY": 1., "BARO2_GND_PRESS": 102230.}
    result, receipt = barometer_parameters_from_recorded_primary(old, field)
    assert float(result["BARO_PRIMARY"]) == 0
    assert float(result["BARO1_GND_PRESS"]) == 102230.
    assert result["ATC_RAT_RLL_P"] == old["ATC_RAT_RLL_P"]
    assert old["BARO1_GND_PRESS"] == "101473.796875"
    assert receipt["recorded_sensor_index"] == 1


def test_absolute_pressure_is_unchanged_and_frontend_altitude_matches():
    result, _ = barometer_parameters_from_recorded_primary(
        {"BARO1_GND_PRESS": "105141.6875"}, {"BARO_PRIMARY": 1., "BARO2_GND_PRESS": 102230.})
    for pressure in (102230., 104019.9951171875, 106129.99267578125):
        expected = (102230. - pressure) / 9800.
        assert (float(result["BARO1_GND_PRESS"]) - pressure) / 9800. == expected


@pytest.mark.parametrize("field", [
    {}, {"BARO_PRIMARY": 1.}, {"BARO_PRIMARY": .5, "BARO1_GND_PRESS": 102230.},
    {"BARO_PRIMARY": 1., "BARO2_GND_PRESS": 0.},
    {"BARO_PRIMARY": 1., "BARO2_GND_PRESS": float("nan")},
])
def test_missing_or_invalid_recorded_ground_datum_is_not_guessed(field):
    with pytest.raises(ValueError):
        barometer_parameters_from_recorded_primary({}, field)
