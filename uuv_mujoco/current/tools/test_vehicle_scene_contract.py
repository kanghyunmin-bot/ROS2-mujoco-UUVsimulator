# Copyright (c) 2022-2026, The Isaac Lab Project Developers (https://github.com/isaac-sim/IsaacLab/blob/main/CONTRIBUTORS.md).
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
"""Regression checks for the shared Research pool vehicle across maps."""
import copy
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

CURRENT = Path(__file__).resolve().parents[1]
BLUEROV2_SCENES = ("bluerov2_dfki_scene.xml", "bluerov2_research_pool_scene.xml")
sys.path.insert(0, str(CURRENT))
from gui.test_tank_layout_model import default_course_layout_config, generate_test_tank_scene
from sim.vehicle_scene_contract import CANONICAL_SCENE, synchronize_vehicle, vehicle_body, vehicle_signature


class VehicleSceneContractTest(unittest.TestCase):
    def test_all_maps_and_compiled_vehicle(self):
        source = ET.parse(CANONICAL_SCENE).getroot()
        reference = mujoco.MjModel.from_xml_path(str(CANONICAL_SCENE))
        names = [node.get("name") for node in vehicle_body(source).iter("body")]
        for path in sorted((CURRENT / "scenes").glob("*.xml")):
            if path.name.startswith(".") or path.name in BLUEROV2_SCENES:
                continue
            with self.subTest(scene=path.name):
                root = ET.parse(path).getroot()
                self.assertEqual(vehicle_signature(root), vehicle_signature(source))
                model = mujoco.MjModel.from_xml_path(str(path))
                for name in names:
                    for field in ("mass", "inertia", "ipos", "iquat"):
                        np.testing.assert_allclose(getattr(model.body(name), field),
                                                   getattr(reference.body(name), field))
                for node in vehicle_body(source).iter("camera"):
                    name = node.get("name")
                    for field in ("pos", "quat", "fovy"):
                        np.testing.assert_allclose(getattr(model.camera(name), field),
                                                   getattr(reference.camera(name), field))
                data = mujoco.MjData(model)
                for _ in range(10):
                    mujoco.mj_step(model, data)
                self.assertTrue(np.isfinite(data.qpos).all())
                self.assertTrue(np.isfinite(data.qvel).all())

    def test_bluerov2_meshes_and_thrust_sites_in_both_scenes(self):
        # Source URDF propeller origins translated 72.5 mm into the MJCF frame.
        positions = {
            "yaw_rf": (.1355, -.1000, 0), "yaw_lf": (.1355, .1000, 0),
            "yaw_rr": (-.1475, -.1000, 0), "yaw_lr": (-.1475, .1000, 0),
            "ver_rf": (.0025, -.1105, .0675), "ver_lf": (.0025, .1105, .0675),
        }
        for scene in BLUEROV2_SCENES:
            with self.subTest(scene=scene):
                model = mujoco.MjModel.from_xml_path(str(CURRENT / "scenes" / scene))
                data = mujoco.MjData(model)
                self.assertEqual([model.actuator(i).name for i in range(model.nu)], list(positions))
                self.assertEqual(model.joint("world_joint").id, 0)
                self.assertEqual(model.geom("bluerov2_body_material_15").contype[0], 0)
                for name in ("imu_site", "bar30_site"):
                    self.assertGreaterEqual(model.site(name).id, 0)
                for index, (name, position) in enumerate(positions.items()):
                    site = model.site(f"thr_{name}")
                    prop = model.body(f"prop_{name}")
                    np.testing.assert_allclose(site.pos, position)
                    np.testing.assert_allclose(prop.pos, site.pos)
                    data.ctrl[:] = 0
                    data.ctrl[index] = 1
                    mujoco.mj_forward(model, data)
                    axis = data.xmat[prop.id].reshape(3, 3)[:, 0]
                    force = model.actuator(index).gear[:3]
                    np.testing.assert_allclose(force, axis if index < 4 else -axis, atol=1e-9)
                    joint_axis = data.xaxis[model.joint(f"prop_{name}_j").id]
                    np.testing.assert_allclose(force, joint_axis, atol=1e-9)
                    np.testing.assert_allclose(data.qfrc_actuator[:6],
                                               [*force, *np.cross(site.pos, force)], atol=1e-9)
                data.ctrl[:] = 0
                for _ in range(100):
                    mujoco.mj_step(model, data)
                self.assertTrue(np.isfinite(data.qpos).all())
                self.assertTrue(np.isfinite(data.qvel).all())

    def test_bluerov2_reuses_ut27_pool_and_buoy(self):
        def environment(scene):
            root = ET.parse(CURRENT / "scenes" / scene).getroot()
            world = [node for node in root.find("worldbody") if node.get("name") != "base_link"]
            return [ET.tostring(node).strip() for node in [*world, root.find("equality"), root.find("contact")]]

        self.assertEqual(environment("bluerov2_research_pool_scene.xml"),
                         environment("research_pool_slam_scene.xml"))

    def test_sync_preserves_environment_and_spawn(self):
        root = ET.parse(CURRENT / "scenes/tank_current_scene.xml").getroot()
        original = copy.deepcopy(root)
        vehicle_body(root).find("inertial").set("mass", "1")
        synchronize_vehicle(root)
        self.assertEqual(vehicle_body(root).attrib, vehicle_body(original).attrib)
        for tag in ("option", "equality"):
            self.assertEqual(ET.tostring(root.find(tag)), ET.tostring(original.find(tag)))
        environment = lambda r: [ET.tostring(n) for n in r.find("worldbody") if n.get("name") != "base_link"]
        self.assertEqual(environment(root), environment(original))
        before = ET.tostring(root)
        synchronize_vehicle(root)
        self.assertEqual(before, ET.tostring(root))

    def test_generator_repairs_stale_vehicle(self):
        with tempfile.TemporaryDirectory(dir=CURRENT) as folder:
            source_path = Path(folder) / "source.xml"
            output_path = Path(folder) / "generated.xml"
            tree = ET.parse(CURRENT / "scenes/tank_current_scene.xml")
            vehicle_body(tree.getroot()).find("inertial").set("mass", "1")
            tree.write(source_path)
            generate_test_tank_scene(base_scene_path=source_path, output_scene_path=output_path,
                                     config=default_course_layout_config())
            self.assertEqual(vehicle_signature(ET.parse(output_path).getroot()),
                             vehicle_signature(ET.parse(CANONICAL_SCENE).getroot()))
            model = mujoco.MjModel.from_xml_path(str(output_path))
            self.assertEqual(float(model.body("base_link").mass[0]), 15.0)


if __name__ == "__main__":
    unittest.main()
