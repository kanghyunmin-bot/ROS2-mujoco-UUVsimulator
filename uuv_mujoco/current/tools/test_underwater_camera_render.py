"""Geometry checks for water path projection, independent of an OpenGL context."""

from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.underwater_camera_render import CameraOpticalPathProjector


class WaterPathTest(unittest.TestCase):
    def test_axial_depth_becomes_longer_off_axis_range(self):
        projector = CameraOpticalPathProjector(width=3, height=3, fovy_degrees=90)
        paths = projector.project(
            np.ones((3, 3), dtype=np.float32), camera_position_z_m=-3,
            camera_rotation=np.eye(3), water_surface_z_m=0, max_path_length_m=12,
        )
        self.assertAlmostEqual(float(paths[1, 1]), 1.0)
        self.assertAlmostEqual(float(paths[0, 0]), np.sqrt(1 + 2 * (2 / 3) ** 2), places=6)

    def test_surface_intersection_and_range_cap(self):
        projector = CameraOpticalPathProjector(width=1, height=1, fovy_degrees=70)
        for height, rotation, depth, expected in (
            (-1, np.eye(3), 4, 4),
            (1, np.eye(3), 4, 3),
            (1, np.eye(3), 0.5, 0),
            (-1, np.diag([1, -1, -1]), 4, 1),
            (1, np.diag([1, -1, -1]), 4, 0),
            (-1, np.eye(3), 1000, 12),
        ):
            with self.subTest(height=height, depth=depth, expected=expected):
                result = projector.project(
                    np.array([[depth]], dtype=np.float32), camera_position_z_m=height,
                    camera_rotation=rotation, water_surface_z_m=0, max_path_length_m=12,
                )
                self.assertAlmostEqual(float(result[0, 0]), expected)

    def test_horizontal_camera_top_rays_exit_water_and_bottom_rays_stay_submerged(self):
        projector = CameraOpticalPathProjector(width=3, height=3, fovy_degrees=90)
        # Forward +X, up +Z, right -Y in MuJoCo world coordinates.
        rotation = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        paths = projector.project(
            np.full((3, 3), 4.0, dtype=np.float32), camera_position_z_m=-1,
            camera_rotation=rotation, water_surface_z_m=0, max_path_length_m=12,
        )
        self.assertAlmostEqual(float(paths[0, 1]), 1.5 * np.sqrt(1 + (2 / 3) ** 2), places=6)
        self.assertAlmostEqual(float(paths[2, 1]), 4 * np.sqrt(1 + (2 / 3) ** 2), places=6)


if __name__ == "__main__":
    unittest.main()
