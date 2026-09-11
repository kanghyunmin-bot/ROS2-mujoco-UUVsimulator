"""A released buoy's lower jig must catch under the CAD tines under load.

The guided fixture fixes lateral alignment and attitude, leaving vertical motion
free. It isolates penetration from legitimate escape through an open rake mouth.
"""

import sys
from pathlib import Path

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from tools.check_buoy_physics_contract import runtime_for
from sim.runtime.model_runtime_setup import _apply_timestep_override


def main():
    m = mujoco.MjModel.from_xml_path(
        str(CURRENT / "scenes/research_pool_slam_scene.xml")
    )
    _apply_timestep_override(
        m,
        mujoco_module=mujoco,
        env_float=lambda name, default: (
            0.005 if name == "UUV_MUJOCO_TIMESTEP" else default
        ),
        env_flag=lambda name, default: default,
    )
    assert m.opt.timestep <= 0.002
    hand = {
        i
        for i in range(m.ngeom)
        if (m.geom(i).name or "").startswith(("cad_collision_92_", "cad_collision_93_"))
    }
    if "--without_fix" in sys.argv:
        for i in hand:
            m.geom_priority[i] = 0
            m.geom_solref[i] = [0.02, 1]
            m.geom_solimp[i] = [0.9, 0.95, 0.001, 0.5, 2]
    peak = 0
    cases = 0
    for sign in (1, -1):
        for x in (0.34, 0.38, 0.40):
            for y in (0.095, 0.14, 0.185, 0.23):
                d = mujoco.MjData(m)
                mujoco.mj_forward(m, d)
                r = runtime_for(mujoco, m, d)
                b = r.buoys[0]
                r._detach(b, reason="retention_fixture", force_n=15)
                b.release_time_s = -1
                base = d.qpos[:7].copy()
                q, v = b.free_qposadr, b.free_dofadr
                d.qpos[q : q + 3] = base[:3] + [x, sign * y, 0.035]
                mujoco.mj_forward(m, d)
                jig_contact = False
                penetration = 0
                for _ in range(round(1 / m.opt.timestep)):
                    d.qpos[:7] = base
                    d.qvel[:6] = 0
                    d.qpos[q : q + 2] = base[:2] + [x, sign * y]
                    d.qpos[q + 3 : q + 7] = [1, 0, 0, 0]
                    d.qvel[v : v + 2] = 0
                    d.qvel[v + 3 : v + 6] = 0
                    r.apply(m.opt.timestep)
                    load = 15 if d.time > 0.3 else 0
                    d.xfrc_applied[b.body_id, 2] += load
                    mujoco.mj_step(m, d)
                    d.xfrc_applied[b.body_id, 2] -= load
                    for c in d.contact:
                        other = (
                            c.geom2
                            if c.geom1 in hand
                            else c.geom1
                            if c.geom2 in hand
                            else -1
                        )
                        if other in b.geom_ids:
                            penetration = max(penetration, -c.dist)
                            jig_contact |= (m.geom(other).name or "").endswith(
                                ("_moving_lower_jig", "_magnet_lower_plate")
                            )
                    assert d.qpos[q + 2] - base[2] < 0.07, (
                        "jig crossed tines",
                        sign,
                        x,
                        y,
                    )
                    assert penetration < 0.002, (
                        "excessive tine penetration",
                        sign,
                        x,
                        y,
                        penetration,
                    )
                assert jig_contact, ("no lower jig contact", sign, x, y)
                peak = max(peak, penetration)
                cases += 1
    print(
        f"PASS {cases} guided jig catches under 15 N extra load; peak penetration {peak * 1000:.3f} mm"
    )
    check_free_transport(m)


def check_free_transport(m):
    """Transport with all six buoy DOFs free and no attachment to the robot."""
    hand = {
        i
        for i in range(m.ngeom)
        if (m.geom(i).name or "").startswith(("cad_collision_92_", "cad_collision_93_"))
    }
    for sign in (1, -1):
        for y in (0.095, 0.14, 0.185, 0.23):
            d = mujoco.MjData(m)
            mujoco.mj_forward(m, d)
            r = runtime_for(mujoco, m, d)
            b = r.buoys[0]
            r._detach(b, reason="transport_fixture", force_n=15)
            b.release_time_s = -1
            base = d.qpos[:7].copy()
            q = b.free_qposadr
            d.qpos[q : q + 3] = base[:3] + [0.36, sign * y, 0.035]
            mujoco.mj_forward(m, d)
            hits = 0
            penetration = 0
            for _ in range(round(2 / m.opt.timestep)):
                robot = base.copy()
                robot[0] -= 0.04 * max(0, d.time - 1)
                robot[2] += 0.02 * max(0, d.time - 1)
                d.qpos[:7] = robot
                d.qvel[:6] = 0
                r.apply(m.opt.timestep)
                mujoco.mj_step(m, d)
                for c in d.contact:
                    if (c.geom1 in hand and c.geom2 in b.geom_ids) or (
                        c.geom2 in hand and c.geom1 in b.geom_ids
                    ):
                        hits += 1
                        penetration = max(penetration, -c.dist)
            relative = d.qpos[q : q + 3] - robot[:3]
            assert hits and penetration < 0.002, (
                "free transport penetration",
                sign,
                y,
                penetration,
            )
            assert 0.32 < relative[0] < 0.41 and abs(relative[1] - sign * y) < 0.015
            assert 0.05 < relative[2] < 0.07, (
                "free buoy lost mechanical catch",
                relative,
            )
            assert b.detached and not d.eq_active[b.eq_id]
    print(
        "PASS eight slots retain freely moving/rotating buoys during slow ROV transport"
    )


if __name__ == "__main__":
    main()
