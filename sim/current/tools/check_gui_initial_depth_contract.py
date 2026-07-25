#!/usr/bin/env python3
"""Smoke checks for GUI initial-depth launch contracts."""

from __future__ import annotations

import pathlib
import sys


ROOT = pathlib.Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from gui.sim_stack_initial_depth_args import build_initial_depth_args  # noqa: E402
from gui.sim_stack_initial_depth_contract import (  # noqa: E402
    base_link_initial_depth_event,
    initial_depth_args_explicitly_set,
)


def _assert_equal(actual: object, expected: object, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def main() -> int:
    _assert_equal(initial_depth_args_explicitly_set(["--initial-depth-m", "0.2"]), True, "explicit base depth")
    _assert_equal(initial_depth_args_explicitly_set(["--initial-bar30-depth-m=auto"]), True, "explicit bar30 depth")
    _assert_equal(initial_depth_args_explicitly_set(["--foo"]), False, "no explicit depth")

    real_start = build_initial_depth_args({"UUV_REAL_START_STATE": "1"}, launch_extra_args=())
    _assert_equal(real_start.args, (), "real-start args")
    _assert_equal(real_start.events, ("sim initial state: real CSV contract handled by launcher",), "real-start event")

    bar30 = build_initial_depth_args({"UUV_GUI_INITIAL_BAR30_DEPTH_M": "0.42"}, launch_extra_args=())
    _assert_equal(bar30.args, ("--initial-bar30-depth-m", "0.42"), "bar30 args")
    _assert_equal(bar30.events, ("sim initial depth: bar30=0.42 m",), "bar30 event")

    base_link = build_initial_depth_args(
        {
            "UUV_GUI_INITIAL_BAR30_DEPTH_M": "off",
            "UUV_GUI_INITIAL_DEPTH_M": "-0.10",
            "UUV_GUI_HOLD_INITIAL_DEPTH_UNTIL_RELEASE": "true",
            "UUV_GUI_INITIAL_DEPTH_HOLD_TARGET_M": "0.20",
        },
        launch_extra_args=(),
    )
    _assert_equal(
        base_link.args,
        (
            "--initial-depth-m",
            "-0.10",
            "--initial-depth-hold-target-m",
            "0.20",
            "--hold-initial-depth-until-release",
        ),
        "base-link hold args",
    )
    _assert_equal(base_link.events, ("sim drop start: base_link=0.100 m above water",), "base-link event")
    _assert_equal(base_link_initial_depth_event("0.3"), "sim initial depth: base_link=0.3 m", "positive event")

    explicit_wins = build_initial_depth_args({"UUV_GUI_INITIAL_BAR30_DEPTH_M": "0.42"}, launch_extra_args=("--initial-depth-m", "0.1"))
    _assert_equal(explicit_wins.args, (), "explicit launch args win")

    print("gui_initial_depth_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
