"""Research-pool placement persistence and mooring geometry checks."""

from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from gui.research_pool_layout import load_layout, save_layout


class ResearchPoolLayoutTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "pool.xml"
        self.path.write_bytes(
            (CURRENT / "scenes/research_pool_slam_scene.xml").read_bytes()
        )

    def test_move_depth_updates_anchor_support_and_weld(self):
        item = load_layout(self.path)["items"][0]
        prefix = item["id"]
        save_layout(
            self.path, {prefix: {"x": 2, "y": 1, "z": -4}}, {"x": -2, "y": -1, "z": -2}
        )
        r = ET.parse(self.path)
        self.assertEqual(
            r.find(f'.//body[@name="{prefix}_magnet_base"]').get("pos"),
            "2.0000 1.0000 -5.0000",
        )
        self.assertEqual(
            r.find(f'.//weld[@name="{prefix}_magnet_weld"]').get("body1"),
            prefix + "_rope_magnet_tip",
        )
        self.assertEqual(
            r.find(f'.//weld[@name="{prefix}_magnet_weld"]').get("relpose"),
            "0 0 .215 1 0 0 0",
        )

        def rope_height(tree):
            link = tree.find(f'.//body[@name="{prefix}_rope_00"]')
            height = 0
            while link is not None:
                height += float(link.get("pos").split()[2])
                link = link.find("body")
            return height

        self.assertAlmostEqual(rope_height(r), 0.785, places=6)
        updated = load_layout(self.path)
        self.assertEqual(updated["items"][0]["z"], -4)
        self.assertEqual(updated["robot"]["z"], -2)
        # Repeated saves must not accumulate changes to the support length.
        save_layout(self.path, {prefix: {"z": -4}})
        self.assertAlmostEqual(rope_height(ET.parse(self.path)), 0.785, places=6)

    def test_invalid_batch_leaves_file_unchanged(self):
        initial = self.path.read_bytes()
        ids = [item["id"] for item in load_layout(self.path)["items"]]
        for invalid in ({"z": -5}, {"z": 0}, {"x": 5}, {"y": 2.5}, {"x": float("nan")}):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                save_layout(self.path, {ids[0]: {"x": 2}, ids[1]: invalid})
            self.assertEqual(self.path.read_bytes(), initial)
        with self.assertRaises(ValueError):
            save_layout(self.path, {"missing": {"x": 1}})
        with self.assertRaises(ValueError):
            save_layout(self.path, {}, {"z": -4.9})
        self.assertEqual(self.path.read_bytes(), initial)

    def test_edited_scene_compiles_and_moorings_remain_stable(self):
        import mujoco
        import numpy as np

        items = load_layout(self.path)["items"]
        save_layout(
            self.path,
            {item["id"]: {"z": -1.5 - index} for index, item in enumerate(items)},
        )
        root = ET.parse(self.path).getroot()
        compiler = root.find("compiler")
        compiler.set(
            "meshdir", str((CURRENT / "scenes" / compiler.get("meshdir")).resolve())
        )
        model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        ids = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, item["id"] + "_float")
            for item in items
        ]
        start = data.xpos[ids].copy()
        from tools.check_buoy_physics_contract import runtime_for

        runtime = runtime_for(mujoco, model, data)
        self.assertLessEqual(model.opt.timestep, 0.001)
        for _ in range(round(0.8 / model.opt.timestep)):
            runtime.apply(model.opt.timestep)
            mujoco.mj_step(model, data)
        self.assertFalse(any(b.detached for b in runtime.buoys))
        self.assertTrue(np.isfinite(data.qpos).all())
        np.testing.assert_allclose(data.xpos[ids], start, atol=0.02)


if __name__ == "__main__":
    unittest.main()
