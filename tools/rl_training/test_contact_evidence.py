"""Check actual CAD right/left finger contact against the lower PVC stem."""
import sys
from pathlib import Path
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'uuv_mujoco/current'))
from tools.check_buoy_physics_contract import runtime_for
m=mujoco.MjModel.from_xml_path(str(ROOT/'uuv_mujoco/current/scenes/research_pool_slam_scene.xml'))
d=mujoco.MjData(m);mujoco.mj_forward(m,d);r=runtime_for(mujoco,m,d)
b=r.buoys[0];pipe=m.geom(b.name+'_pvc_pipe').id
initial=d.qpos.copy()
right=list(r._right_fork_geom_ids)
left=list(r._hand_geom_id_set-r._right_fork_geom_ids)
assert right and left
for label,ids,wanted in [('right',right,True),('left',left,False)]:
 found=False
 for gid in ids:
  d.qpos[:]=initial;mujoco.mj_forward(m,d)
  # Translating the root preserves each CAD finger's shape and orientation.
  mesh=int(m.geom_dataid[gid]);start=int(m.mesh_vertadr[mesh]);count=int(m.mesh_vertnum[mesh])
  center=m.mesh_vert[start:start+count].mean(axis=0)
  world=d.geom_xpos[gid]+d.geom_xmat[gid].reshape(3,3)@center
  d.qpos[:3]+=d.geom_xpos[pipe]-world
  mujoco.mj_forward(m,d);r._contact_snapshot()
  pairs=[(int(c.geom1),int(c.geom2)) for c in d.contact]
  if not any(set(pair)=={gid,pipe} for pair in pairs):continue
  hit=b.body_id in r._right_fork_stem_contacts
  if hit==wanted:found=True;break
 assert found,(label,'no suitable contact fixture')
 print('PASS actual geometry',label,'right-fork stem evidence',wanted)
