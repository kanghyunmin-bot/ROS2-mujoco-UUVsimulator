"""Exercise measured magnetic loads, rope mechanics, and free buoy rise."""

import sys
from pathlib import Path

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from tools.check_buoy_physics_contract import runtime_for


def main():
    m = mujoco.MjModel.from_xml_path(
        str(CURRENT / "scenes/research_pool_slam_scene.xml")
    )
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    r = runtime_for(mujoco, m, d)
    b = r.buoys[0]
    assert r.magnet_force_release and r.break_force_n == 15
    origin = d.qpos[:7].copy()
    rope = [
        i
        for i in range(m.ngeom)
        if (m.geom(i).name or "").startswith(b.name + "_rope_geom_")
    ]
    tip = m.body(b.name + "_rope_magnet_tip").id
    q = b.free_qposadr

    def step(force=(0, 0, 0)):
        d.qpos[:7] = origin
        d.qvel[:6] = 0
        r.apply(m.opt.timestep)
        d.xfrc_applied[b.body_id, :3] += force
        mujoco.mj_step(m, d)
        d.xfrc_applied[b.body_id, :3] -= force
        assert np.isfinite(d.qpos).all() and np.max(np.abs(d.qvel)) < 500

    for _ in range(500):
        step()
    assert not any(x.detached for x in r.buoys), "idle detachment"
    idle = r._magnet_constraint_force_n(b)
    assert 0.5 < idle < 2, ("unexpected initial magnet load", idle)
    # Touch alone must not defeat the magnet. Legacy mode deliberately fails here.
    if "--without_fix" in sys.argv:
        r.magnet_force_release = False
    r._release_if_contact_or_break_force(b, vehicle_contact=True, contact_force_n=0.1)
    assert not b.detached, "light touch bypassed force threshold"
    for _ in range(500):
        step((0, 0, 10))
    assert not b.detached, "sub-threshold sustained pull released magnet"
    low = r._magnet_constraint_force_n(b)
    assert 9 < low < 13, low
    # Mechanical separation must precede the former 40 ms delay. A previous
    # brief-impact hold test masked the rope impulse that now has a regression.
    pull_start = d.time
    for _ in range(round(0.010 / m.opt.timestep)):
        step((0, 0, 20))
        if b.detached:
            break
    assert d.time - pull_start <= 0.010 + 1e-9
    assert b.detached and not d.eq_active[b.eq_id], (
        "20 N sustained pull failed to release"
    )
    assert sum(x.detached for x in r.buoys) == 1
    assert all(m.geom_contype[i] and m.geom_conaffinity[i] for i in rope)
    print(
        f"PASS idle {idle:.3f} N, low pull {low:.3f} N held; sustained overload released"
    )
    start_tip = d.xpos[tip].copy()
    # Small off-axis motion breaks the perfectly vertical mathematical symmetry.
    root_joint = m.joint(b.name + "_rope_joint_00").id
    d.qvel[m.jnt_dofadr[root_joint]] = 0.2
    for _ in range(round(20 / m.opt.timestep)):
        step()
    assert d.xpos[b.body_id, 2] > -0.25, ("buoy did not rise", d.xpos[b.body_id])
    assert d.xpos[tip, 2] < start_tip[2] - 0.3, "unloaded rope did not sag"
    assert all(m.geom_contype[i] and m.geom_conaffinity[i] for i in rope)
    # Move an actual CAD finger across a middle rope link and inspect contacts.
    middle = rope[len(rope) // 2]
    location = d.geom_xpos[middle].copy()
    d.qpos[:3] = location - [0.405, 0.074, -0.094]
    d.qvel[:6] = 0
    mujoco.mj_forward(m, d)
    assert any(
        (int(c.geom1) in rope and int(c.geom2) in r.vehicle_geom_ids)
        or (int(c.geom2) in rope and int(c.geom1) in r.vehicle_geom_ids)
        for c in d.contact
    ), "rope did not collide with robot"
    print(
        "PASS free buoy rises, anchored rope sags, released rope still collides with robot"
    )
    np.savez("/tmp/research_pool_magnet_rope_state.npz", qpos=d.qpos, qvel=d.qvel)
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)
    r = runtime_for(mujoco, m, d)
    b = r.buoys[0]
    start = d.xpos[b.body_id].copy()
    for _ in range(1000):
        step((0.2, 0, 0))
    assert not b.detached, "gentle lateral push released buoy"
    assert abs(d.xpos[b.body_id, 0] - start[0]) > 0.02, (
        "rope remained a rigid vertical rod"
    )
    # At low speed the existing impulse limiter does not mask projected-area drag.
    for axis in (0, 2):
        d.qvel[:] = 0
        d.qpos[b.free_qposadr + 3 : b.free_qposadr + 7] = [1, 0, 0, 0]
        d.qvel[b.free_dofadr + axis] = 0.05
        mujoco.mj_forward(m, d)
        drag = r._water_drag_wrench(b, vehicle_contact=False, dt=m.opt.timestep)
        if axis == 0:
            horizontal_drag = abs(drag[0])
        else:
            assert horizontal_drag > abs(drag[2]), "ellipsoid projected area ignored"
    print(
        "PASS attached rope bends under gentle push; ellipsoid orientation affects drag"
    )


if __name__ == "__main__":
    main()
