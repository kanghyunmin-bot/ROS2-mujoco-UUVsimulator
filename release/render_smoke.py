"""Render both recorded RGB cameras without starting or arming a vehicle."""
from pathlib import Path
import mujoco
import numpy as np
root=Path(__file__).resolve().parents[1]
model=mujoco.MjModel.from_xml_path(str(root/'uuv_mujoco/current/scenes/research_pool_slam_scene.xml'))
data=mujoco.MjData(model)
mujoco.mj_forward(model,data)
with mujoco.Renderer(model,height=360,width=640) as renderer:
    for camera in ('stereo_left','stereo_right'):
        renderer.update_scene(data,camera=camera)
        rgb=renderer.render()
        assert rgb.shape==(360,640,3) and np.isfinite(rgb).all() and rgb.std()>1
        print(camera,rgb.shape,'render OK')
