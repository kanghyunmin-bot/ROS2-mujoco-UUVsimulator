"""Check CAD-mounted sensors, camera TF orientation, and unobstructed DVL beams."""

import json
from pathlib import Path
import sys

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from bridge.ros2_static_tf_specs import build_static_tf_specs


def main():
    config = json.loads((CURRENT / "config/sensor_mounts_2026.json").read_text())
    for scene in ("research_pool_slam_scene.xml", "tank_current_scene.xml"):
        model = mujoco.MjModel.from_xml_path(str(CURRENT / "scenes" / scene))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        for name, pos in config["site_positions"].items():
            np.testing.assert_allclose(model.site(name).pos, pos, atol=1e-8)
        specs = build_static_tf_specs(
            model=model,
            ping360_frame_id="ping360_link",
            hydrophone_site_id=model.site("hydrophone_center_site").id,
            **{
                name + "_id": model.site(name).id
                for name in (
                    "imu_site",
                    "bar30_site",
                    "dvl_site",
                    "ping360_site",
                    "cam_left_site",
                    "cam_right_site",
                )
            },
        )
        poses = {"base_link": np.eye(4)}
        for parent, child, pos, quat in specs:
            if parent not in poses:
                continue
            transform = np.eye(4)
            transform[:3, 3] = pos
            rotation = np.zeros(9)
            # Internal TF specifications store quaternion wxyz.
            mujoco.mju_quat2Mat(rotation, quat)
            transform[:3, :3] = rotation.reshape(3, 3)
            poses[child] = poses[parent] @ transform
        for index, name in enumerate(("stereo_left", "stereo_right")):
            camera = model.camera(name)
            tf = poses[f"imx219_camera{index}_optical_frame"]
            np.testing.assert_allclose(tf[:3, 3], camera.pos, atol=1e-8)
            rotation = np.zeros(9)
            mujoco.mju_quat2Mat(rotation, camera.quat)
            np.testing.assert_allclose(
                tf[:3, 2], -rotation.reshape(3, 3)[:, 2], atol=1e-8
            )
            # Optical centres are behind the CAD panel's inner x face, within
            # the enclosure bore. The window remains physically solid.
            assert 0.21 < camera.pos[0] < 0.24918056
            assert np.linalg.norm(camera.pos[1:] - [0, 0.00999]) < 0.09
        pane = model.geom("cad_collision_339_0").id
        assert model.geom_contype[pane] and model.geom_conaffinity[pane]
        assert model.geom_rgba[model.geom("front_optical_window").id, 3] < 0.1
        site = model.site("dvl_site").id
        hit = np.zeros(1, dtype=np.int32)
        distances = []
        for azimuth in np.arange(4) * np.pi / 2:
            direction = np.array(
                [0.5 * np.cos(azimuth), 0.5 * np.sin(azimuth), np.sqrt(0.75)]
            )
            direction = data.site_xmat[site].reshape(3, 3) @ direction
            distance = mujoco.mj_ray(
                model,
                data,
                data.site_xpos[site],
                direction,
                np.array([1, 0, 0, 1, 0, 0], dtype=np.uint8),
                1,
                -1,
                hit,
            )
            assert distance < 0 or distance > 0.1, (
                "DVL beam hits its housing",
                scene,
                distance,
                model.geom(int(hit[0])).name,
            )
            distances.append(round(distance, 3))
        print(
            f"PASS {scene}: CAD mounts, optical TF, solid transparent window; DVL rays {distances}"
        )


if __name__ == "__main__":
    main()
