"""Verify the research pool selects calibrated physics without replacing its scene."""

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gui.sim_launch_preset import (
    RESEARCH_POOL_DISTRIBUTED_PRESET_ID,
    build_sim_launch_preset_args,
    resolve_sim_launch_preset,
    sim_launch_preset_environment,
    validate_sim_launch_preset,
)
from gui.sim_stack_env_contract import build_gui_sim_stack_env
from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles
from sim.physics.thruster_direct_overrides import apply_thruster_direct_gain_overrides


class TestResearchPoolReal2Sim(unittest.TestCase):
    def test_gui_environment_preserves_calibration_at_runtime(self):
        preset = resolve_sim_launch_preset(RESEARCH_POOL_DISTRIBUTED_PRESET_ID)
        profiles, _ = load_sim_profiles(preset.profile_path)
        plant = build_sim_profile(profiles, preset.profile)
        for explicit, expected in [
            ({}, 0.12293316274117821),
            ({"UUV_HORIZONTAL_DIRECT_GAIN_SCALE": "0.2"}, 0.2),
        ]:
            env = build_gui_sim_stack_env(
                explicit, backend="native", sim_stack_dir=ROOT
            )
            env.update(sim_launch_preset_environment(preset))
            scales = {
                name: 1.0 for name in ["yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr", "ver_lf"]
            }
            apply_thruster_direct_gain_overrides(
                plant,
                scales,
                vertical_thrusters=["ver_lf"],
                horizontal_thrusters=["yaw_lf", "yaw_lr", "yaw_rf", "yaw_rr"],
                env_get=env.get,
                log=lambda _: None,
            )
            self.assertAlmostEqual(scales["yaw_lf"], expected)
            self.assertEqual(scales["ver_lf"], 1.0)

    def test_pool_scene_uses_calibrated_plant_and_controller(self):
        preset = resolve_sim_launch_preset(RESEARCH_POOL_DISTRIBUTED_PRESET_ID)
        validate_sim_launch_preset(preset)
        args = build_sim_launch_preset_args(preset)
        self.assertEqual(preset.scene_path.name, "research_pool_slam_scene.xml")
        self.assertEqual(preset.viewer_camera_mode, "follow")
        self.assertFalse(preset.uses_active_course_scene)
        self.assertIn("--profile-file", args)
        profiles, warning = load_sim_profiles(
            Path(args[args.index("--profile-file") + 1])
        )
        self.assertIsNone(warning)
        plant = build_sim_profile(profiles, preset.profile)
        self.assertAlmostEqual(
            plant["thruster_direct_gain_scales"]["yaw_lf"], 0.12293316274117821
        )
        self.assertEqual(plant["thruster_direct_gain_scales"].get("ver_lf", 1.0), 1.0)
        self.assertEqual(plant["buoyancy_scale"], 1.01)
        self.assertEqual(plant["thruster_voltage"], 20.0)
        self.assertTrue(plant["distributed_hydrodynamics"]["active"])
        self.assertFalse(plant["calibration_provenance"]["independent_real_validation"])
        self.assertEqual(
            sim_launch_preset_environment(preset)["SITL_REAL2SIM_BAG0402"], "1"
        )


if __name__ == "__main__":
    unittest.main()
