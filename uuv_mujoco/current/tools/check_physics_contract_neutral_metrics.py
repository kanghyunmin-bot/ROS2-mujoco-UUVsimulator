#!/usr/bin/env python3
"""Focused smoke test for neutral-audit attitude and angular-rate metrics."""

from __future__ import annotations

import csv
import io
import math
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TOOLS = Path(__file__).resolve().parent
for item in (ROOT, TOOLS):
    if str(item) not in sys.path:
        sys.path.insert(0, str(item))

from physics_contract_neutral_loop import sample_neutral_state  # noqa: E402
from physics_contract_neutral_output import (  # noqa: E402
    NEUTRAL_SIM_CSV_HEADER,
    build_neutral_sim_summary,
)
from physics_contract_types import NeutralMotionSamples, NeutralSimSummary  # noqa: E402


def main() -> int:
    # The original positional constructor remains valid; added metrics default to zero.
    legacy = NeutralSimSummary("legacy", 1.0, 0.4, 0.4, 0.0, 0.0, 0.0, "legacy.csv")
    assert legacy.start_pitch_deg == 0.0
    assert legacy.max_angular_speed_rad_s == 0.0

    pitch_rad = math.radians(10.0)
    data = SimpleNamespace(
        time=0.25,
        xpos=np.array([[0.0, 0.0, -0.4]], dtype=np.float64),
        qpos=np.array(
            [0.0, 0.0, -0.4, math.cos(pitch_rad / 2.0), 0.0, math.sin(pitch_rad / 2.0), 0.0],
            dtype=np.float64,
        ),
        qvel=np.array([0.0, 0.0, -0.05, 0.1, -0.2, 0.3], dtype=np.float64),
    )
    stream = io.StringIO()
    writer = csv.writer(stream)
    writer.writerow(NEUTRAL_SIM_CSV_HEADER)
    motion = NeutralMotionSamples()
    depth, vz_down = sample_neutral_state(
        writer=writer,
        data=data,
        base_id=0,
        world_qpos_adr=0,
        world_qvel_adr=0,
        water_surface_z=0.1,
        vehicle_mass=10.0,
        gravity=9.81,
        buoyancy_z=98.1,
        weighted_submerged=1.0,
        motion_samples=motion,
    )
    assert abs(depth - 0.5) < 1.0e-12
    assert abs(vz_down - 0.05) < 1.0e-12
    assert abs(motion.roll_deg[0]) < 1.0e-12
    assert abs(motion.pitch_deg[0] - 10.0) < 1.0e-12
    assert abs(motion.yaw_deg[0]) < 1.0e-12
    assert abs(motion.angular_speed_rad_s[0] - math.sqrt(0.14)) < 1.0e-12

    rows = list(csv.DictReader(io.StringIO(stream.getvalue())))
    assert len(rows) == 1
    assert set(NEUTRAL_SIM_CSV_HEADER) == set(rows[0])
    assert abs(float(rows[0]["pitch_deg"]) - 10.0) < 1.0e-12
    assert abs(float(rows[0]["angular_rate_y_rad_s"]) + 0.2) < 1.0e-12

    motion.roll_deg.append(-3.0)
    motion.pitch_deg.append(-12.0)
    motion.yaw_deg.append(20.0)
    motion.angular_rate_x_rad_s.append(-0.4)
    motion.angular_rate_y_rad_s.append(0.5)
    motion.angular_rate_z_rad_s.append(-0.6)
    motion.angular_speed_rad_s.append(math.sqrt(0.77))
    summary = build_neutral_sim_summary(
        label="smoke",
        duration_s=1.0,
        base_depth_m=0.5,
        depths=[0.5, 0.51],
        vz_down_values=[0.05, -0.02],
        output_csv=Path("smoke.csv"),
        motion_samples=motion,
    )
    assert summary.start_pitch_deg == motion.pitch_deg[0]
    assert summary.end_pitch_deg == -12.0
    assert summary.max_abs_roll_deg == 3.0
    assert summary.max_abs_pitch_deg == 12.0
    assert summary.max_abs_angular_rate_z_rad_s == 0.6
    assert abs(summary.max_angular_speed_rad_s - math.sqrt(0.77)) < 1.0e-12
    expected_rms = math.sqrt((0.14 + 0.77) / 2.0)
    assert abs(summary.rms_angular_speed_rad_s - expected_rms) < 1.0e-12

    print("physics_contract_neutral_metrics=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
