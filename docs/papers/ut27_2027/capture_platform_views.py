"""Capture one static MuJoCo state through existing water and sensor pipelines."""
import hashlib
import json
import os
from pathlib import Path
import sys
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import mujoco
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
CURRENT = ROOT / 'uuv_mujoco/current'
sys.path.insert(0, str(CURRENT))
from bridge.water_lighting import PoolWaterLighting
from bridge.ros2_stereo_image import configure_stereo_image_runtime, render_camera_frame, close_stereo_image_renderers
from sim.runtime.water_surface_visual import WaterSurfaceVisual

OUT = HERE / 'figures/platform_capture'
OUT.mkdir(exist_ok=True)
scene = CURRENT / 'scenes/research_pool_slam_scene.xml'
model = mujoco.MjModel.from_xml_path(str(scene))
model.vis.quality.shadowsize = 1024
model.vis.quality.offsamples = 1
model.vis.global_.offwidth = 960
model.vis.global_.offheight = 540
data = mujoco.MjData(model)
data.time = 1.2
mujoco.mj_forward(model, data)
water = WaterSurfaceVisual(mujoco, model)
water.update(data.time)
qpos = data.qpos.copy()
rgba = model.geom_rgba.copy()
base = model.body('base_link').id
option = mujoco.MjvOption()
option.sitegroup[:] = 0
option.geomgroup[3:] = 0
camera = mujoco.MjvCamera()
camera.type = mujoco.mjtCamera.mjCAMERA_FREE
lighting = PoolWaterLighting(960, 540, float(model.vis.global_.fovy))
views = [
    ('surface', np.array([0, 0, -0.2]), 9.5, 80, -70),
    ('robot', data.xpos[base].copy(), 1.65, 135, -20),
    ('buoy', data.xpos[model.body('course_buoy_a_yellow_1_float').id] + [0, 0, -0.12], 1.0, 135, -10),
]
with mujoco.Renderer(model, height=540, width=960) as renderer:
    for name,target,distance,azimuth,elevation in views:
        camera.lookat[:] = target
        camera.distance, camera.azimuth, camera.elevation = distance, azimuth, elevation
        renderer.update_scene(data, camera=camera, scene_option=option)
        rgb = renderer.render()
        renderer.enable_depth_rendering(); depth = renderer.render(); renderer.disable_depth_rendering()
        eye = renderer.scene.camera
        pos = (eye[0].pos + eye[1].pos)/2
        forward,up = eye[0].forward,eye[0].up
        rotation = np.column_stack((np.cross(forward,up),up,-forward))
        rgb = lighting.apply(rgb,depth,position=pos,rotation=rotation,time_s=data.time,surface_z=0,center_xy=np.zeros(2),half_size_xy=np.array([5,2.5]))
        cv2.imwrite(str(OUT / f'{name}.png'),cv2.cvtColor(rgb,cv2.COLOR_RGB2BGR))
profile = CURRENT / 'config/sensor_models/imx219_underwater_pool_lite.json'
assert np.array_equal(qpos,data.qpos) and np.array_equal(rgba,model.geom_rgba)
files = [scene,profile,CURRENT/'bridge/water_lighting.py',CURRENT/'bridge/ros2_stereo_image.py',CURRENT/'assets/body_2026_09/body_hand_lower_green.obj']
report = {'scope':'One model and static state, no physics stepping or SITL. Existing renderer effects; no image edits.',
 'time_s':float(data.time),'qpos':qpos.tolist(),'material_state_sha256':hashlib.sha256(rgba.tobytes()).hexdigest(),
 'green_geom_rgba':model.geom_rgba[model.geom('body_hand_lower_green').id].tolist(),
 'source_sha256':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files if p.exists()},
 'panels':{name:hashlib.sha256((OUT/f'{name}.png').read_bytes()).hexdigest() for name in ['surface','robot','buoy']},
 'backend':os.environ.get('MUJOCO_GL'),'mujoco_version':mujoco.__version__}
(OUT/'capture.json').write_text(json.dumps(report,indent=2)+'\n')
print(OUT)
