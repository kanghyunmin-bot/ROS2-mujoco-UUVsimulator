#!/usr/bin/env python3
# Copyright (c) 2026, KMU Underwater Robot Team.
# SPDX-License-Identifier: MIT

"""Contract tests for focused A50 RViz visualization helpers."""

from __future__ import annotations

import importlib.util
import math
import sys
from pathlib import Path
from types import SimpleNamespace


SCRIPT = Path(__file__).parents[1] / "scripts" / "dvl_localization_visualizer.py"
SPEC = importlib.util.spec_from_file_location("dvl_localization_visualizer", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _beam(
    beam_id: int, *, valid: bool = True, distance: float = 1.5
) -> SimpleNamespace:
    return SimpleNamespace(id=beam_id, valid=valid, distance=distance)


def _message(*, valid_beams: int = 4, fom: float = 0.01) -> SimpleNamespace:
    return SimpleNamespace(
        velocity_valid=True,
        velocity=SimpleNamespace(x=0.1, y=-0.2, z=0.03),
        altitude=1.2,
        fom=fom,
        beams=[_beam(index, valid=index < valid_beams) for index in range(4)],
        covariance=[0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01],
    )


def test_beam_geometry_matches_convex_janus_contract() -> None:
    directions = [MODULE.beam_direction_frd(index) for index in range(4)]
    for direction in directions:
        assert math.isclose(sum(value * value for value in direction), 1.0)
        assert direction[2] > 0.0
    assert directions[0][0] > 0.0 and directions[0][1] > 0.0
    assert directions[2][0] < 0.0 and directions[2][1] < 0.0


def test_configurable_beam_direction_is_normalized() -> None:
    direction = MODULE.normalized_beam_direction_frd((1.0, 2.0, 2.0), 0)
    assert direction == (1.0 / 3.0, 2.0 / 3.0, 2.0 / 3.0)


def test_invalid_distance_is_not_a_bottom_hit() -> None:
    assert MODULE.beam_has_valid_range(_beam(0))
    assert not MODULE.beam_has_valid_range(_beam(0, valid=False))
    assert not MODULE.beam_has_valid_range(_beam(0, distance=-1.0))


def test_estimator_gate_matches_four_beam_runtime_defaults() -> None:
    assert MODULE.dvl_sample_passes_input_gate(_message())
    assert not MODULE.dvl_sample_passes_input_gate(_message(valid_beams=3))
    assert not MODULE.dvl_sample_passes_input_gate(_message(fom=0.06))


def test_path_resets_on_time_rollback_or_frame_change() -> None:
    assert not MODULE.path_state_must_reset(None, None, 10.0, "odom")
    assert not MODULE.path_state_must_reset(10.0, "odom", 10.1, "odom")
    assert MODULE.path_state_must_reset(10.0, "odom", 0.0, "odom")
    assert MODULE.path_state_must_reset(10.0, "odom", 10.1, "map")


if __name__ == "__main__":
    test_beam_geometry_matches_convex_janus_contract()
    test_configurable_beam_direction_is_normalized()
    test_invalid_distance_is_not_a_bottom_hit()
    test_estimator_gate_matches_four_beam_runtime_defaults()
    test_path_resets_on_time_rollback_or_frame_change()
    print("dvl_localization_visualizer=PASS")
