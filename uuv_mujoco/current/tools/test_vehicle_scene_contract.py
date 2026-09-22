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
sys.path.insert(0, str(CURRENT))
from gui.test_tank_layout_model import default_course_layout_config, generate_test_tank_scene
from sim.vehicle_scene_contract import CANONICAL_SCENE, synchronize_vehicle, vehicle_body, vehicle_signature


class VehicleSceneContractTest(unittest.TestCase):
    def test_all_maps_and_compiled_vehicle(self):
        source = ET.parse(CANONICAL_SCENE).getroot()
        reference = mujoco.MjModel.from_xml_path(str(CANONICAL_SCENE))
        names = [node.get("name") for node in vehicle_body(source).iter("body")]
        for path in sorted((CURRENT / "scenes").glob("*.xml")):
            if path.name.startswith("."):
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
