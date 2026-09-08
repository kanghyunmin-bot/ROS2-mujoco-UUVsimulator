#!/usr/bin/env python3
"""MuJoCo integration tests for A50 beam rays and sensor-site velocity."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.dvl_a50_sensor_model import A50SensorConfig, A50SensorModel  # noqa: E402
from bridge.ros2_dvl_beam_geometry import (  # noqa: E402
    dvl_beam_geometry_from_mujoco,
    dvl_beam_geometry_if_capture_due,
)
from bridge.ros2_sitl_sensor_dvl_state import (  # noqa: E402
    dvl_velocity_from_snapshot,
)
from bridge.ros2_sitl_sensor_types import BaseKinematicState  # noqa: E402


def make_model(*, floor_euler: str = "0 0 0") -> tuple[mujoco.MjModel, mujoco.MjData]:
    xml = f"""
    <mujoco model="dvl-ray-test">
      <worldbody>
        <geom name="bottom" type="plane" pos="0 0 -2" euler="{floor_euler}"
              size="0 0 0.1"/>
        <geom name="visual_water" type="box" pos="0 0 -1" size="5 5 1"
              contype="0" conaffinity="0" group="5" rgba="0 0.5 1 0.1"/>
        <body name="base_link">
          <freejoint/>
          <geom type="sphere" size="0.1"/>
          <site name="dvl_site" pos="1 0 0" quat="0 1 0 0"/>
        </body>
      </worldbody>
      <sensor>
        <velocimeter name="dvl_vel_body" site="dvl_site"/>
      </sensor>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data


def ids(model: mujoco.MjModel) -> tuple[int, int]:
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "dvl_site")
    return int(base_id), int(site_id)


class DvlBeamRaycastTest(unittest.TestCase):
    def test_time_rewind_forces_first_capture_raycast(self) -> None:
        model, data = make_model()
        base_id, site_id = ids(model)
        bridge = SimpleNamespace(
            model=model,
            _base_id=base_id,
            _dvl_site_id=site_id,
            _dvl_sensor_model=A50SensorModel(A50SensorConfig()),
            _dvl_sensor_timing=SimpleNamespace(
                config=SimpleNamespace(
                    schedule=SimpleNamespace(epsilon_s=1.0e-9)
                ),
                next_capture_time_s=10.1,
            ),
            _dvl_sensor_last_advance_time_s=10.0,
            _dvl_sensor_needs_time_anchor=False,
        )

        ranges, incidences = dvl_beam_geometry_if_capture_due(bridge, data, 0.0)

        self.assertIsNotNone(ranges)
        self.assertIsNotNone(incidences)

    def test_flat_floor_ignores_visual_water_and_produces_slant_ranges(self) -> None:
        model, data = make_model()
        base_id, site_id = ids(model)
        bridge = SimpleNamespace(
            model=model,
            _base_id=base_id,
            _dvl_site_id=site_id,
            _dvl_sensor_model=A50SensorModel(A50SensorConfig()),
        )

        ranges, incidences = dvl_beam_geometry_from_mujoco(bridge, data)

        expected_range = 2.0 / math.cos(math.radians(22.5))
        np.testing.assert_allclose(ranges, expected_range, atol=1.0e-10)
        np.testing.assert_allclose(
            incidences,
            math.cos(math.radians(22.5)),
            atol=1.0e-10,
        )

    def test_sloped_floor_gives_beam_specific_ranges_and_incidence(self) -> None:
        model, data = make_model(floor_euler="0 20 0")
        base_id, site_id = ids(model)
        bridge = SimpleNamespace(
            model=model,
            _base_id=base_id,
            _dvl_site_id=site_id,
            _dvl_sensor_model=A50SensorModel(A50SensorConfig()),
        )

        ranges, incidences = dvl_beam_geometry_from_mujoco(bridge, data)

        self.assertEqual(len({round(value, 8) for value in ranges}), 2)
        self.assertEqual(len({round(value, 8) for value in incidences}), 2)
        self.assertTrue(all(value > 0.0 for value in ranges))


class DvlSiteVelocityTest(unittest.TestCase):
    def test_pure_yaw_includes_omega_cross_r_at_offset_sensor(self) -> None:
        model, data = make_model()
        base_id, site_id = ids(model)
        data.qvel[:] = 0.0
        data.qvel[5] = 1.0
        mujoco.mj_forward(model, data)
        base = BaseKinematicState(
            sim_t=0.0,
            base_pos_enu=np.zeros(3, dtype=np.float64),
            base_rot_enu=np.eye(3, dtype=np.float64),
            quat_base=np.array((1.0, 0.0, 0.0, 0.0), dtype=np.float64),
            base_vel_enu=np.zeros(3, dtype=np.float64),
            zero_vertical_reason=None,
        )
        owner = SimpleNamespace(
            model=model,
            _base_id=base_id,
            _dvl_site_id=site_id,
            _dvl_filter_alpha=1.0,
            _dvl_vel_body_filt=None,
            _sitl_initial_depth_hold_active=False,
        )
        velocity_body = dvl_velocity_from_snapshot(
            owner,
            data,
            base,
            np.asarray(data.sensordata, dtype=np.float64),
            np.array((0.0, 0.0, 1.0), dtype=np.float64),
        )

        np.testing.assert_allclose(velocity_body, (0.0, 1.0, 0.0), atol=1.0e-12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
