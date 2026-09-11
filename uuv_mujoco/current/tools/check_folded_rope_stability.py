"""Replay a perturbed folded failure pose, not the already diverged velocities."""

import argparse
import json
from pathlib import Path
import sys
import tempfile

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from tools.check_buoy_physics_contract import runtime_for

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("snapshot", type=Path)
p.add_argument("--seconds", type=float, default=20.0)
p.add_argument("--without_fix", action="store_true")
a = p.parse_args()
scene = CURRENT / "scenes/research_pool_slam_scene.xml"
xml = scene.read_text()
if a.without_fix:
    xml = xml.replace('armature="0.00001"', 'armature="0.000001"')
with tempfile.NamedTemporaryFile(mode="w", suffix=".xml", dir=scene.parent) as f:
    f.write(xml)
    f.flush()
    m = mujoco.MjModel.from_xml_path(f.name)
d = mujoco.MjData(m)
with np.load(a.snapshot) as z:
    m.opt.timestep = float(z["timestep"])
    d.qpos[:] = z["qpos"]
    d.qvel[:] = z["qvel"] * 1e-5
    d.eq_active[:] = z["eq_active"]
mujoco.mj_forward(m, d)
r = runtime_for(mujoco, m, d)
for b in r.buoys:
    b.detached = not bool(d.eq_active[b.eq_id])
peak = 0.0
for _ in range(round(a.seconds / m.opt.timestep)):
    r.apply(m.opt.timestep)
    mujoco.mj_step(m, d)
    peak = max(peak, float(np.max(np.abs(d.qvel))))
    assert np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
    assert not any(w.number for w in d.warning), "MuJoCo numerical warning"
    assert peak < 1e4, f"Folded rope diverged at {d.time}: speed={peak}"
print(json.dumps({"seconds": d.time, "peak_generalized_speed": peak, "warnings": 0}))
