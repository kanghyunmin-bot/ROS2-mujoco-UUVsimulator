#!/usr/bin/env python3
# Copyright (c) 2026, KMU Underwater Robot Team.
# SPDX-License-Identifier: MIT

"""Contract tests for camera and sonar display image conversion."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np


SCRIPT = Path(__file__).parents[1] / "scripts" / "camera_sonar_viewer.py"
SPEC = importlib.util.spec_from_file_location("camera_sonar_viewer", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _message(
    data: bytes,
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
) -> SimpleNamespace:
    return SimpleNamespace(
        data=data,
        width=width,
        height=height,
        encoding=encoding,
        step=step,
    )


def test_rgb_and_bgr_conversion() -> None:
    rgb_message = _message(
        bytes([10, 20, 30, 40, 50, 60]),
        width=2,
        height=1,
        encoding="rgb8",
        step=6,
    )
    bgr_message = _message(
        bytes([30, 20, 10, 60, 50, 40]),
        width=2,
        height=1,
        encoding="bgr8",
        step=6,
    )
    expected = np.asarray([[[10, 20, 30], [40, 50, 60]]], dtype=np.uint8)
    np.testing.assert_array_equal(MODULE.image_message_to_rgb(rgb_message), expected)
    np.testing.assert_array_equal(MODULE.image_message_to_rgb(bgr_message), expected)


def test_row_padding_is_not_rendered() -> None:
    padded = _message(
        bytes([1, 2, 3, 4, 5, 6, 99, 99]),
        width=2,
        height=1,
        encoding="rgb8",
        step=8,
    )
    converted = MODULE.image_message_to_rgb(padded)
    assert converted.shape == (1, 2, 3)
    assert converted.reshape(-1).tolist() == [1, 2, 3, 4, 5, 6]


def test_sonar_palette_preserves_black_and_brightens_returns() -> None:
    sonar = _message(
        bytes([0, 64, 128, 255]),
        width=4,
        height=1,
        encoding="mono8",
        step=4,
    )
    converted = MODULE.image_message_to_rgb(sonar, sonar_palette=True)
    assert converted.shape == (1, 4, 3)
    assert converted[0, 0].tolist() == [0, 0, 0]
    assert int(converted[0, -1].sum()) > int(converted[0, 1].sum())
    assert int(converted[0, -1, 0]) >= int(converted[0, -1, 1])


if __name__ == "__main__":
    test_rgb_and_bgr_conversion()
    test_row_padding_is_not_rendered()
    test_sonar_palette_preserves_black_and_brightens_returns()
    print("camera_sonar_viewer=PASS")
