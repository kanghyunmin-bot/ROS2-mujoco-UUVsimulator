"""Actual scene check of reward-only closest-rod slot telemetry."""
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'uuv_mujoco/current'))
from bridge.ros2_publish_course_buoys import _course_buoy_row
from tools.check_buoy_physics_contract import runtime_for

m=mujoco.MjModel.from_xml_path(str(ROOT/'uuv_mujoco/current/scenes/research_pool_slam_scene.xml'))
d=mujoco.MjData(m);mujoco.mj_forward(m,d);runtime=runtime_for(mujoco,m,d)
b=runtime.buoys[0];gid=m.geom(b.name+'_pvc_pipe').id
bridge=SimpleNamespace(_course_buoy_runtime=runtime)
os.environ['UUV_RL_REWARD_TELEMETRY']='1'
root=runtime.vehicle_root_body_id;rot=d.xmat[root].reshape(3,3)
fork=d.xpos[root]+rot@np.array([.3355119380367045,-.09192734377108369,-.10434422587321063])
axis=d.geom_xmat[gid].reshape(3,3)[:,2]
# The center is deliberately offset along the cylinder. Closest-line distance
# must be near zero while old geometric-center distance remains 5 cm.
d.qpos[b.free_qposadr:b.free_qposadr+3]+=fork+.05*axis-d.geom_xpos[gid]
mujoco.mj_forward(m,d)
row=_course_buoy_row(bridge,m,d,b.body_id)
assert np.linalg.norm(row['rl_slot_offset_m']) < 1e-6,row
assert row['rl_fork_stem_distance_m'] > .049
os.environ.pop('UUV_RL_REWARD_TELEMETRY')
assert 'rl_slot_offset_m' not in _course_buoy_row(bridge,m,d,b.body_id)
print('PASS closest-rod slot geometry and privileged telemetry gate')
