"""Sweep the CAD vehicle through an attached mooring at the operating timestep."""

from pathlib import Path
import sys

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from sim.physics.fluid_contract import configure_fluid_model_contract
from tools.check_buoy_physics_contract import runtime_for
from sim.runtime.model_runtime_setup import _align_fcu_timestep, _apply_timestep_override


def main():
    for fluid in ("current", "legacy"):
        for side in (-1, 1):
            model = mujoco.MjModel.from_xml_path(
                str(CURRENT / "scenes/research_pool_slam_scene.xml")
            )
            _apply_timestep_override(
                model, mujoco_module=mujoco,
                env_float=lambda name, default: .0025 if name == "UUV_MUJOCO_TIMESTEP" else default,
                env_flag=lambda name, default: default,
            )
            _align_fcu_timestep(model, 400.)
            configure_fluid_model_contract(
                model=model,
                fluid_model=fluid,
                scene_path="research_pool",
                scene_fluid_density=1000,
                scene_fluid_viscosity=0.001,
            )
            data = mujoco.MjData(model)
            mujoco.mj_forward(model, data)
            runtime = runtime_for(mujoco, model, data)
            if "--without_fix" in sys.argv:
                runtime.contact_break_hold_s = 0.04
            # The research scene now contains one yellow target. Resolve its
            # identity rather than relying on the old three-buoy ordering.
            buoy = next(
                item for item in runtime.buoys
                if item.name == "course_buoy_a_yellow_1"
            )
            origin = data.qpos[:7].copy()
            origin[:3] = data.xpos[buoy.body_id] - [0.214, side * 0.237, 0.5]
            origin[3:] = [1, 0, 0, 0]
            contacts = 0
            rope = {
                i
                for i in range(model.ngeom)
                if (model.geom(i).name or "").startswith(buoy.name + "_rope_geom_")
            }
            for _ in range(round(1.5 / model.opt.timestep)):
                data.qpos[:7] = origin
                data.qpos[2] += 0.5 * min(data.time, 1.1)
                data.qvel[:6] = [0, 0, 0.5 if data.time < 1.1 else 0, 0, 0, 0]
                mujoco.mj_forward(model, data)
                runtime.apply(model.opt.timestep)
                mujoco.mj_step(model, data)
                contacts += sum(
                    (c.geom1 in rope and c.geom2 in runtime.vehicle_geom_ids)
                    or (c.geom2 in rope and c.geom1 in runtime.vehicle_geom_ids)
                    for c in data.contact
                )
                assert not any(w.number for w in data.warning), (
                    "rope instability",
                    fluid,
                    side,
                    data.time,
                )
                assert np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all()
            assert contacts and buoy.detached and not data.eq_active[buoy.eq_id]
            print(
                f"PASS {fluid} side {side:+}: rope contact, magnetic separation, no reset",
                flush=True,
            )


if __name__ == "__main__":
    main()
