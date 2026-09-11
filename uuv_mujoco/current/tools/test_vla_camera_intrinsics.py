"""The default optical prior must describe the actual front and hand renders."""

import os
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import sys
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.ros2_stereo_image import configure_stereo_image_runtime


class CameraIntrinsicsTest(unittest.TestCase):
    def test_default_profile_uses_each_rendered_fov(self):
        model = mujoco.MjModel.from_xml_string(
            '<mujoco><worldbody><camera name="stereo_left" fovy="70"/><camera name="stereo_right" fovy="82"/></worldbody></mujoco>'
        )
        bridge = SimpleNamespace(model=model)
        with patch.dict(
            os.environ, {"ROS2_UUV_CAMERA_SENSOR_MODEL_ENABLE": "1"}, clear=False
        ):
            configure_stereo_image_runtime(
                bridge,
                publish_images=True,
                image_width=320,
                image_height=240,
                image_hz=30,
            )
        for name, fov in [("stereo_left", 70), ("stereo_right", 82)]:
            c = bridge._camera_calibrations[name].scaled_to(320, 240)
            expected = 120 / np.tan(np.deg2rad(fov / 2))
            self.assertAlmostEqual(c.k[0], expected, places=5)
            self.assertAlmostEqual(c.k[4], expected, places=5)


if __name__ == "__main__":
    unittest.main()
