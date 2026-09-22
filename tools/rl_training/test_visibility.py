import sys
from pathlib import Path
import mujoco
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'uuv_mujoco/current'))
from bridge.ros2_publish_course_buoys import camera_points_visible
for group,expected in [(3,True),(0,False)]:
    m=mujoco.MjModel.from_xml_string(f'''<mujoco><worldbody><camera name="hand" pos="0 0 0"/><geom name="occluder" type="box" pos="0 0 -1" size=".3 .3 .1" group="{group}"/><body name="target" pos="0 0 -2"><geom type="sphere" size=".2"/></body></worldbody></mujoco>''')
    d=mujoco.MjData(m);mujoco.mj_forward(m,d)
    points=[d.xpos[m.body('target').id]]
    assert camera_points_visible(m,d,0,m.body('target').id,points,16/9)==expected
    assert not camera_points_visible(m,d,0,m.body('target').id,[np.array([0,0,2.])],16/9)
print('PASS hidden collision does not occlude; rendered obstacle does; behind camera rejected')
# Before-fix reproduction: all-geometry rays hit invisible collision geometry.
m=mujoco.MjModel.from_xml_string('<mujoco><worldbody><camera name="hand"/><geom type="box" pos="0 0 -1" size=".3 .3 .1" group="3"/><body name="target" pos="0 0 -2"><geom type="sphere" size=".2"/></body></worldbody></mujoco>')
d=mujoco.MjData(m);mujoco.mj_forward(m,d);hit=np.array([-1],dtype=np.int32)
mujoco.mj_ray(m,d,d.cam_xpos[0],np.array([0.,0.,-1.]),None,1,-1,hit)
assert int(m.geom_bodyid[hit[0]])!=m.body('target').id
assert camera_points_visible(m,d,0,m.body('target').id,[d.xpos[m.body('target').id]],16/9)
print('PASS reproduced old false negative and verified fix')

# The real scene has transparent optical glass immediately ahead of both
# cameras. It must not make every target invisible to reward telemetry.
root=Path(__file__).resolve().parents[2]
m=mujoco.MjModel.from_xml_path(str(root/'uuv_mujoco/current/scenes/research_pool_slam_scene.xml'))
d=mujoco.MjData(m);mujoco.mj_forward(m,d)
b=m.body('course_buoy_a_yellow_1_float').id;c=m.camera('stereo_right').id
adr=int(m.jnt_qposadr[m.body_jntadr[b]])
desired=d.cam_xpos[c]+d.cam_xmat[c].reshape(3,3)@np.array([-.25,0.,-.6])
d.qpos[adr:adr+3]+=desired-d.xpos[b];mujoco.mj_forward(m,d)
delta=d.xpos[b]-d.cam_xpos[c];hit=np.array([-1],dtype=np.int32)
mujoco.mj_ray(m,d,d.cam_xpos[c],delta/np.linalg.norm(delta),np.array([1,1,1,0,0,0],dtype=np.uint8),1,-1,hit)
assert hit[0]==m.geom('front_optical_window').id  # Old test falsely rejected this.
assert camera_points_visible(m,d,c,b,[d.xpos[b]],16/9), 'Real hand camera glass hides target'
window=m.geom('front_optical_window').id
m.geom_rgba[window,3]=1.
assert not camera_points_visible(m,d,c,b,[d.xpos[b]],16/9), 'Opaque window must occlude'
m.geom_rgba[window,3]=.06
desired=d.cam_xpos[c]+d.cam_xmat[c].reshape(3,3)@np.array([0.,0.,-.6])
d.qpos[adr:adr+3]+=desired-d.xpos[b];mujoco.mj_forward(m,d)
assert not camera_points_visible(m,d,c,b,[d.xpos[b]],16/9), 'Fork must still occlude the center ray'
print('PASS real hand camera: transparent window transmits, opaque window blocks')
