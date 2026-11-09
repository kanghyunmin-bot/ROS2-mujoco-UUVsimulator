"""Recorded connection and mode boundaries must not invent an initial mode."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "debug/controller_parity_412"))
from recorded_initial_mode import select_initial_mode_row

MODES = {"MANUAL": 19, "STABILIZE": 0, "ALT_HOLD": 2, "POSHOLD": 16}


def test_first_supported_connected_mode_is_unchanged():
    assert select_initial_mode_row([[1., True, False, "ALT_HOLD"]], MODES) == 0


def test_initial_blank_mode_preserves_first_observed_connected_manual_event():
    rows = [[.1, False, False, ""], [1.2, True, False, "MANUAL"], [2., True, True, "ALT_HOLD"]]
    assert select_initial_mode_row(rows, MODES) == 1


def test_disconnected_mode_is_not_an_initial_condition():
    rows = [[.1, False, False, "STABILIZE"], [1.2, True, False, "MANUAL"]]
    assert select_initial_mode_row(rows, MODES) == 1


def test_recorded_numeric_fallback_is_the_same_known_mode():
    rows = [[.1, False, False, ""], [1.2, True, False, "CMODE(19)"], [1.3, True, False, "MANUAL"]]
    assert select_initial_mode_row(rows, MODES) == 1


@pytest.mark.parametrize("rows", [[], [[.1, False, False, ""]], [[.1, True, False, "GUIDED"]]])
def test_missing_or_unsupported_mode_is_rejected(rows):
    with pytest.raises(ValueError):
        select_initial_mode_row(rows, MODES)
