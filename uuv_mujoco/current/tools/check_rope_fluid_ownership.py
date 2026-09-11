"""Verify custom vehicle hydrodynamics do not remove environment rope drag."""

from pathlib import Path
import sys

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from sim.physics.fluid_contract import configure_fluid_model_contract
from sim.physics.fluid_geom_apply import apply_fluid_geom_runtime_scales


def main():
    m = mujoco.MjModel.from_xml_path(
        str(CURRENT / "scenes/research_pool_slam_scene.xml")
    )
    rope_ids = [i for i in range(m.ngeom) if "_rope_geom_" in (m.geom(i).name or "")]
    before_fluid = m.geom_fluid[rope_ids].copy()
    before_size = m.geom_size[rope_ids].copy()
    apply_fluid_geom_runtime_scales(
        m,
        mujoco,
        {"mujoco_fluidcoef_scale": [2, 2, 2, 2, 2]},
        to_float_array=lambda value: (
            None if value is None else np.asarray(value, dtype=float)
        ),
        env_get=lambda name, default="": (
            "1.2" if name == "UUV_MJ_FLUID_GEOM_SIZE_EXTRA_SCALE" else default
        ),
    )
    np.testing.assert_array_equal(m.geom_fluid[rope_ids], before_fluid)
    np.testing.assert_array_equal(m.geom_size[rope_ids], before_size)
    configure_fluid_model_contract(
        model=m,
        fluid_model="legacy",
        scene_path="research_pool",
        scene_fluid_density=1000,
        scene_fluid_viscosity=0.001,
    )
    if "--without_fix" in sys.argv:
        m.opt.density = m.opt.viscosity = 0
    d = mujoco.MjData(m)
    d.qvel[:6] = [1, 0, 0, 0.2, 0.3, 0.4]
    joint = m.joint("course_buoy_a_yellow_1_rope_joint_05").id
    dof = int(m.jnt_dofadr[joint])
    d.qvel[dof] = 10
    mujoco.mj_forward(m, d)
    wet = d.qfrc_passive.copy()
    m.opt.density = m.opt.viscosity = 0
    mujoco.mj_forward(m, d)
    fluid = wet - d.qfrc_passive
    np.testing.assert_allclose(fluid[:6], 0, atol=1e-10)
    assert np.linalg.norm(fluid[dof : dof + 3]) > 1e-7, "rope lost its water drag"
    print("PASS native rope drag remains active; no duplicate vehicle fluid wrench")


if __name__ == "__main__":
    main()
