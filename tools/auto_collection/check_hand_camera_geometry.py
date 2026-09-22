"""Check the reference camera and right/depth image coupling without arming."""
import json
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
profile = json.loads((ROOT / 'uuv_mujoco/current/config/hand_camera_calibration.json').read_text())
model = mujoco.MjModel.from_xml_path(str(ROOT / 'uuv_mujoco/current/scenes/research_pool_slam_scene.xml'))
cam = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, 'stereo_right')
rotation = np.zeros(9)
mujoco.mju_quat2Mat(rotation, model.cam_quat[cam])
rotation = rotation.reshape(3, 3)
np.testing.assert_allclose(model.cam_pos[cam], profile['position_m'], atol=1e-8)
np.testing.assert_allclose(np.r_[rotation[:, 0], rotation[:, 1]], profile['xyaxes'], atol=1e-8)
assert model.cam_fovy[cam] == profile['fovy_deg']
fy = .5 / np.tan(np.deg2rad(model.cam_fovy[cam] / 2))
fx = fy * 360 / 640

def project(point):
    local = rotation.T @ (point-model.cam_pos[cam])
    return np.array([.5 + fx*local[0]/-local[2], .5 - fy*local[1]/-local[2]])

point = np.array(profile['target_body_m'])
goal = np.array(profile['hand_target_uv'])
np.testing.assert_allclose(project(point), goal, atol=1e-8)
inverse = np.array(profile['error_to_right_down_m'])
# A fixed world point shifts opposite to vehicle motion, in body FLU coordinates.
for offset in ([0,.01,0], [0,-.01,0], [0,0,.01], [0,0,-.01], [0,.01,.01]):
    observed = point + offset
    error = 2*(project(observed)-goal)
    right, down = inverse @ error
    corrected = observed + [0,right,down]
    before = np.linalg.norm(project(observed)-goal)
    after = np.linalg.norm(project(corrected)-goal)
    assert after < before*.2, (offset,before,after)
    print(f'offset={offset}: image error {before:.5f} -> {after:.5f}')
print('PASS camera profile, fork-gap projection, right/left and down/up correction directions')
print('Geometric check only: not a physical insertion or detector validation.')
