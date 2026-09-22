"""Check region classification against actual MuJoCo rod transforms."""
import sys
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'uuv_mujoco/current'))
from tools.check_buoy_physics_contract import runtime_for
m=mujoco.MjModel.from_xml_path(str(ROOT/'uuv_mujoco/current/scenes/research_pool_slam_scene.xml'))
d=mujoco.MjData(m);mujoco.mj_forward(m,d);r=runtime_for(mujoco,m,d)
b=r.buoys[0];gid=m.geom(b.name+'_pvc_pipe').id
center=np.array([.335511938,-.091927344,-.104344226])
initial=d.qpos.copy()
for offset,expected in [(np.zeros(3),True),(np.array([0,.15,0]),False),(np.array([.15,0,0]),False)]:
 d.qpos[:]=initial;mujoco.mj_forward(m,d)
 rotation=d.xmat[r.vehicle_root_body_id].reshape(3,3)
 desired=d.xpos[r.vehicle_root_body_id]+rotation@(center+offset)
 d.qpos[b.free_qposadr:b.free_qposadr+3]+=desired-d.geom_xpos[gid]
 mujoco.mj_forward(m,d)
 assert r.right_fork_region_contains(b)==expected,(offset,expected)
print('PASS rod in slot / lateral miss / forward miss')
