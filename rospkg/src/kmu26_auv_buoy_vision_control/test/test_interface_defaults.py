"""Check the direct-node and launch interfaces against the shared robot contract."""

import ast
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]


def python_default(path, function, parameter):
    tree = ast.parse((ROOT / path).read_text())
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        name = getattr(node.func, "id", getattr(node.func, "attr", ""))
        if name != function or not isinstance(node.args[0], ast.Constant):
            continue
        if node.args[0].value == parameter:
            value = node.args[1] if len(node.args) > 1 else next(
                kw.value for kw in node.keywords if kw.arg == "default_value"
            )
            return ast.literal_eval(value)
    raise AssertionError(f"Missing {parameter} in {path}")


class InterfaceDefaults(unittest.TestCase):
    def test_camera_uses_physical_surface(self):
        for path, function in [
            ("scripts/yolo_buoy_detector.py", "declare_parameter"),
            ("launch/laptop_yolo_detection.launch.py", "DeclareLaunchArgument"),
        ]:
            with self.subTest(path=path):
                self.assertEqual(python_default(path, function, "image_topic"),
                                 "/imx219/camera0/image_raw/compressed")

    def test_controllers_use_physical_rc_output(self):
        for executable, source in [("bbox_controller_node", "vision"),
                                   ("mission_state_machine_node", "mission")]:
            text = (ROOT / "src" / f"{executable}.cpp").read_text()
            match = re.search(r'declare_parameter<std::string>\("rc_override_topic",\s*"([^"]+)"', text)
            self.assertIsNotNone(match)
            self.assertEqual(match.group(1), "/mavros/rc/override")

    def test_mission_launch_preserves_physical_rc_output(self):
        self.assertEqual(python_default("launch/auv_bbox_controller.launch.py",
                                       "DeclareLaunchArgument", "rc_override_topic"),
                         "/mavros/rc/override")


if __name__ == "__main__":
    unittest.main()
