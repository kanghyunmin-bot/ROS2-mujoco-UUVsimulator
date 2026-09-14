#!/usr/bin/env python3
"""Render pool surface/robot/rope previews without starting or moving live SITL."""

import argparse
from pathlib import Path
import sys

import cv2
import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from bridge.water_lighting import PoolWaterLighting
from sim.runtime.water_surface_visual import WaterSurfaceVisual


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output_dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(
        str(CURRENT / "scenes/research_pool_slam_scene.xml")
    )
    model.vis.quality.shadowsize = 1024
    model.vis.quality.offsamples = 1
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    water = WaterSurfaceVisual(mujoco, model)
    width, height = 640, 360
    light = PoolWaterLighting(width, height, float(model.vis.global_.fovy))
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    option = mujoco.MjvOption()
    option.sitegroup[:] = 0
    option.geomgroup[3:] = 0
    rope_id = mujoco.mj_name2id(
        model, mujoco.mjtObj.mjOBJ_BODY, "course_buoy_a_yellow_1_rope_03"
    )
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    views = [
        ("surface", np.array([0.0, 0.0, -0.2]), 8.0, 80.0, -70.0),
        ("robot", data.xpos[base_id].copy(), 1.8, 135.0, -22.0),
        ("white_rope", data.xpos[rope_id] + [0, 0, 0.14], 0.09, 90.0, -5.0),
    ]
    pictures = []
    with mujoco.Renderer(model, height=height, width=width) as renderer:
        for name, target, distance, azimuth, elevation in views:
            camera.lookat[:] = target
            camera.distance, camera.azimuth, camera.elevation = (
                distance,
                azimuth,
                elevation,
            )
            writer = None
            if name == "surface":
                writer = cv2.VideoWriter(
                    str(args.output_dir / "surface.mp4"),
                    cv2.VideoWriter_fourcc(*"mp4v"),
                    15,
                    (width, height),
                )
            for index in range(45 if writer else 1):
                data.time = index / 15
                water.update(data.time)
                renderer.update_scene(data, camera=camera, scene_option=option)
                rgb = renderer.render()
                renderer.enable_depth_rendering()
                depth = renderer.render()
                renderer.disable_depth_rendering()
                eyes = renderer.scene.camera
                position = (eyes[0].pos + eyes[1].pos) / 2
                forward, up = eyes[0].forward, eyes[0].up
                rotation = np.column_stack((np.cross(forward, up), up, -forward))
                rgb = light.apply(
                    rgb,
                    depth,
                    position=position,
                    rotation=rotation,
                    time_s=data.time,
                    surface_z=0.0,
                    center_xy=np.zeros(2),
                    half_size_xy=np.array([5.0, 2.5]),
                )
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                if index == 0:
                    cv2.imwrite(str(args.output_dir / f"{name}.png"), bgr)
                    pictures.append(bgr)
                if writer:
                    writer.write(bgr)
            if writer:
                writer.release()
        cv2.imwrite(
            str(args.output_dir / "visuals.png"), np.concatenate(pictures, axis=0)
        )
        # Native viewer uses the same wave field as a visual-only height field.
        camera.lookat[:] = views[0][1]
        camera.distance, camera.azimuth, camera.elevation = views[0][2:]
        option.geomgroup[5] = 1
        renderer.update_scene(data, camera=camera, scene_option=option)
        mujoco.mjr_uploadHField(model, renderer._mjr_context, water.field_id)
        cv2.imwrite(
            str(args.output_dir / "native_surface.png"),
            cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR),
        )
    print(args.output_dir)


if __name__ == "__main__":
    main()
