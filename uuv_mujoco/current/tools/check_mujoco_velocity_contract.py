#!/usr/bin/env python3
"""Check MuJoCo world/body velocity frame contracts used by the ROS bridge."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import mujoco  # noqa: E402

from bridge.ros2_state_body_velocity_read import (  # noqa: E402
    body_cvel_world_linear_velocity_enu_or_zero,
)


def _make_data() -> tuple[mujoco.MjModel, mujoco.MjData, int, int]:
    xml = """
    <mujoco>
      <worldbody>
        <body name="base_link" quat="0.70710678 0 0 0.70710678">
          <freejoint/>
          <geom type="box" size="0.1 0.1 0.1" mass="1"/>
        </body>
      </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    dof_adr = int(model.jnt_dofadr[0])
    return model, data, body_id, dof_adr


def main() -> int:
    _model, data, body_id, dof_adr = _make_data()
    data.qvel[dof_adr : dof_adr + 3] = [1.0, 0.0, 0.0]
    mujoco.mj_forward(data.model, data)
    body_rot_enu = data.xmat[body_id].reshape(3, 3).copy()
    observed = body_cvel_world_linear_velocity_enu_or_zero(
        data=data,
        body_id=body_id,
        body_rot_enu=body_rot_enu,
    )
    expected = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    if not np.allclose(observed, expected, atol=1.0e-9):
        raise SystemExit(
            "base velocity contract failed: "
            f"expected world velocity {expected.tolist()}, got {observed.tolist()}"
        )
    print("PASS mujoco_velocity_contract")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
