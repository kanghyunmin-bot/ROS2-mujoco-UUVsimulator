"""Check offline replay input integrity and isolation from the live controller."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from replay_recorded_pwm_plant import profile_file_paths, runtime_arguments, validate_pwm_rows


class RecordedPwmPlantTests(unittest.TestCase):
    def test_execution_receipt_hashes_the_selected_identified_profile(self):
        actual = profile_file_paths([
            "--profile", "identified", "--profile-file", "/trial/identified.json", "--headless",
        ])
        self.assertEqual(actual, [Path("/trial/identified.json")])

    def test_execution_receipt_retains_all_distinct_profile_layers(self):
        actual = profile_file_paths([
            "--profile-file", "/base.json", "--profile-file", "/fit.json",
            "--profile-file", "/fit.json",
        ])
        self.assertEqual(actual, [Path("/base.json"), Path("/fit.json")])

    def test_native_commands_and_disabled_outputs_are_preserved(self):
        rows = np.array([[0, *([1500] * 8)], [.51, 0, 65535, *([1700] * 6)]], float)
        previous = rows.copy()
        validate_pwm_rows(rows)
        np.testing.assert_array_equal(rows, previous)

    def test_duplicate_receipts_require_explicit_resolution(self):
        with self.assertRaises(ValueError):
            validate_pwm_rows(np.array([[1, *([1500] * 8)], [1, *([1600] * 8)]]))

    def test_reversed_receipts_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_pwm_rows(np.array([[1, *([1500] * 8)], [0, *([1600] * 8)]]))

    def test_missing_channels_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_pwm_rows(np.ones((2, 8)))

    def test_nan_does_not_become_neutral(self):
        rows = np.array([[0, *([1500] * 8)], [1, *([1500] * 8)]], float)
        rows[1, 1] = np.nan
        with self.assertRaises(ValueError):
            validate_pwm_rows(rows)

    def test_out_of_range_is_rejected_instead_of_clipped(self):
        rows = np.array([[0, *([1500] * 8)], [1, *([1500] * 8)]], float)
        rows[1, 8] = 2500
        with self.assertRaises(ValueError):
            validate_pwm_rows(rows)

    def test_fractional_pwm_is_not_silently_rounded(self):
        rows = np.array([[0, *([1500] * 8)], [1, *([1500] * 8)]], float)
        rows[1, 8] = 1600.5
        with self.assertRaises(ValueError):
            validate_pwm_rows(rows)

    def test_controller_is_removed_but_physical_arguments_are_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            reference, output = root / "reference", root / "output"
            reference.mkdir()
            output.mkdir()
            original = '<mujoco><worldbody><body pos="1 2 3"/></worldbody></mujoco>'
            (reference / "bag_scene.xml").write_text(original)
            (reference / "commands.json").write_text(json.dumps({"mujoco": [
                "python", "run_uuv_mujoco.py", "--scene", "/trial/old/bag_scene.xml",
                "--profile", "bag0402_effective", "--fluid-model", "distributed",
                "--sitl", "--sitl-mavlink-endpoint", "udpin:0.0.0.0:14660",
                "--ros2", "--ros2-real-pkg-compat", "--initial-depth-m", "0.255",
            ]}))
            result = runtime_arguments(reference, output)
            self.assertNotIn("--sitl", result)
            self.assertNotIn("--ros2", result)
            self.assertNotIn("--sitl-mavlink-endpoint", result)
            self.assertEqual(result[result.index("--initial-depth-m") + 1], "0.255")
            self.assertEqual(result[result.index("--fluid-model") + 1], "distributed")
            self.assertEqual((output / "scene.xml").read_text(), original)
            self.assertEqual((reference / "bag_scene.xml").read_text(), original)


if __name__ == "__main__":
    unittest.main()
