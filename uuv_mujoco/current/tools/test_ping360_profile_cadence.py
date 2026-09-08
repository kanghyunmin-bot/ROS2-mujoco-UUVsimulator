#!/usr/bin/env python3
# Copyright (c) 2026, KMU Underwater Robot Team.
# SPDX-License-Identifier: MIT

"""Regression tests for Ping360 acquisition cadence catch-up."""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

try:
    import mujoco  # noqa: F401
except ModuleNotFoundError:
    # The cadence code only uses MuJoCo in postponed annotations. Keeping this
    # test ROS/MuJoCo-independent makes it runnable in the lightweight CI job.
    sys.modules["mujoco"] = types.ModuleType("mujoco")

from bridge.ping360_sim_update import (  # noqa: E402
    MAX_PROFILE_CATCH_UP,
    update_ping360_simulator,
)
from bridge.ping360_config import Ping360Config  # noqa: E402
from bridge.ping360_settings import build_effective_settings  # noqa: E402


class FakeSweep:
    """Minimal full-circle sweep used by the scheduling regression test."""

    def __init__(self) -> None:
        self.angle_grad = 0

    def advance(self, *, settings, config) -> None:
        del config
        self.angle_grad = (self.angle_grad + int(settings.num_steps)) % 400


def make_owner(profile_period_s: float = 0.01) -> SimpleNamespace:
    """Build the mutable fields required by ``update_ping360_simulator``."""

    return SimpleNamespace(
        model=object(),
        config=SimpleNamespace(enabled=True),
        settings=SimpleNamespace(profile_period_s=profile_period_s, num_steps=1),
        site_id=0,
        base_body_id=0,
        _runtime=SimpleNamespace(sweep=FakeSweep()),
        _latest=None,
        _next_profile_t=-1.0,
        _ping_number=0,
    )


class Ping360ProfileCadenceTest(unittest.TestCase):
    """Verify acquisition timing remains independent from ROS image cadence."""

    def setUp(self) -> None:
        self.calls: list[tuple[float, int, int]] = []

        def fake_scan_and_record(**kwargs):
            sim_t = float(kwargs["sim_t"])
            angle_grad = int(kwargs["angle_grad"])
            ping_number = int(kwargs["ping_number"])
            self.calls.append((sim_t, angle_grad, ping_number))
            return SimpleNamespace(
                sim_time_s=sim_t,
                angle_grad=angle_grad,
                profile=(ping_number,),
                image=((ping_number,),),
                ranges_m=(float("inf"),),
                intensities=(0.0,),
                settings=kwargs["settings"],
                ping_number=ping_number,
                updated=True,
            )

        self.patchers = [
            patch(
                "bridge.ping360_sim_update.refresh_ping360_simulator_runtime",
                return_value=None,
            ),
            patch(
                "bridge.ping360_sim_update.current_ping360_angle_grad",
                side_effect=lambda owner: owner._runtime.sweep.angle_grad,
            ),
            patch(
                "bridge.ping360_sim_update.scan_and_record_ping360_profile",
                side_effect=fake_scan_and_record,
            ),
        ]
        for patcher in self.patchers:
            patcher.start()

    def tearDown(self) -> None:
        for patcher in reversed(self.patchers):
            patcher.stop()

    def test_ten_hz_publish_catches_up_one_hundred_hz_profiles(self) -> None:
        owner = make_owner(profile_period_s=0.01)

        update_ping360_simulator(owner, object(), 0.0)
        latest = update_ping360_simulator(owner, object(), 0.1)

        self.assertEqual(len(self.calls), 11)
        self.assertEqual(
            [round(call[0], 6) for call in self.calls],
            [index / 100 for index in range(11)],
        )
        self.assertEqual([call[1] for call in self.calls], list(range(11)))
        self.assertEqual(latest.angle_grad, 10)
        self.assertEqual(latest.ping_number, 11)
        self.assertAlmostEqual(owner._next_profile_t, 0.11, places=12)

        held = update_ping360_simulator(owner, object(), 0.105)
        self.assertEqual(len(self.calls), 11)
        self.assertFalse(held.updated)
        self.assertAlmostEqual(held.sim_time_s, 0.105, places=12)

    def test_full_rotation_finishes_in_four_seconds(self) -> None:
        owner = make_owner(profile_period_s=0.01)

        for tick in range(41):
            update_ping360_simulator(owner, object(), tick / 10)

        self.assertEqual(len(self.calls), 401)
        self.assertAlmostEqual(self.calls[-1][0], 4.0, places=12)
        self.assertEqual(self.calls[-1][1:], (0, 401))
        self.assertEqual(owner._runtime.sweep.angle_grad, 1)
        self.assertAlmostEqual(owner._next_profile_t, 4.01, places=12)

    def test_repository_default_finishes_at_declared_scan_period(self) -> None:
        config = Ping360Config.from_file(CURRENT / "config" / "ping360.json")
        settings = build_effective_settings(config)
        owner = make_owner(profile_period_s=settings.profile_period_s)

        for tick in range(41):
            update_ping360_simulator(owner, object(), tick / 10)
        last_bearing_time = 399 * settings.profile_period_s
        update_ping360_simulator(owner, object(), last_bearing_time)

        self.assertAlmostEqual(settings.scan_period_s, 4.0116, places=6)
        self.assertEqual(len(self.calls), 400)
        self.assertEqual(self.calls[-1][1:], (399, 400))
        self.assertAlmostEqual(self.calls[-1][0], last_bearing_time, places=12)

        latest = update_ping360_simulator(owner, object(), settings.scan_period_s)
        self.assertEqual(len(self.calls), 401)
        self.assertEqual(latest.angle_grad, 0)
        self.assertEqual(latest.ping_number, 401)

    def test_large_time_jump_has_bounded_work_and_drops_backlog(self) -> None:
        owner = make_owner(profile_period_s=0.01)
        update_ping360_simulator(owner, object(), 0.0)

        latest = update_ping360_simulator(owner, object(), 100.0)

        self.assertEqual(len(self.calls), 1 + MAX_PROFILE_CATCH_UP)
        self.assertEqual(latest.ping_number, 1 + MAX_PROFILE_CATCH_UP)
        self.assertAlmostEqual(owner._next_profile_t, 100.01, places=12)

        update_ping360_simulator(owner, object(), 100.005)
        self.assertEqual(len(self.calls), 1 + MAX_PROFILE_CATCH_UP)
        update_ping360_simulator(owner, object(), 100.01)
        self.assertEqual(len(self.calls), 2 + MAX_PROFILE_CATCH_UP)


if __name__ == "__main__":
    unittest.main()
