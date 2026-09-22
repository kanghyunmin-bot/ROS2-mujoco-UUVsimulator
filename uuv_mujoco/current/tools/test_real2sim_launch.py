"""Check that GUI selection reaches the frozen plant and SITL controller."""

from pathlib import Path
import os
import shlex
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from gui.sim_launch_preset import (  # noqa: E402
    RESEARCH_POOL_DISTRIBUTED_PRESET_ID,
    RESEARCH_POOL_YAW_STABLE_PRESET_ID,
    COURSE_CURRENT_PRESET_ID,
    COURSE_REAL2SIM_PRESET_ID,
    COURSE_REAL2SIM_YAW_PRESET_ID,
    build_sim_launch_preset_args,
    default_sim_launch_preset_id,
    merge_sim_launch_preset_args,
    resolve_sim_launch_preset,
    sim_launch_preset_environment,
    validate_sim_launch_preset,
)
from physics.sim_profile_helpers import build_sim_profile, load_sim_profiles  # noqa: E402


class TestReal2SimLaunch(unittest.TestCase):
    def test_yaw_candidate_preserves_default_and_controller_overlay(self):
        self.assertEqual(default_sim_launch_preset_id({}), RESEARCH_POOL_YAW_STABLE_PRESET_ID)
        preset = resolve_sim_launch_preset(COURSE_REAL2SIM_YAW_PRESET_ID)
        validate_sim_launch_preset(preset)
        args = build_sim_launch_preset_args(preset)
        profiles, warning = load_sim_profiles(
            Path(args[args.index("--profile-file") + 1])
        )
        self.assertIsNone(warning)
        plant = build_sim_profile(profiles, preset.profile)
        self.assertEqual(plant["body_inertia_scale_xyz"][2], 0.8)
        self.assertEqual(
            plant["thruster_response_time_constants"]["horizontal"],
            {"tau_up_s": 0.01, "tau_down_s": 0.015},
        )
        self.assertEqual(
            sim_launch_preset_environment(preset)["SITL_REAL2SIM_BAG0402"], "1"
        )

    def test_default_selects_stable_plant_and_overlay(self):
        preset = resolve_sim_launch_preset(default_sim_launch_preset_id({}))
        self.assertEqual(preset.preset_id, RESEARCH_POOL_YAW_STABLE_PRESET_ID)
        validate_sim_launch_preset(preset)
        args = build_sim_launch_preset_args(preset)
        profiles, warning = load_sim_profiles(
            Path(args[args.index("--profile-file") + 1])
        )
        self.assertIsNone(warning)
        plant = build_sim_profile(profiles, args[args.index("--profile") + 1])
        self.assertEqual(args[args.index("--fluid-model") + 1], "distributed")
        self.assertAlmostEqual(
            plant["thruster_direct_gain_scales"]["yaw_lf"], 0.12293316274117821
        )
        self.assertEqual(plant["buoyancy_scale"], 1.01)
        self.assertEqual(plant["thruster_voltage"], 20.0)
        self.assertFalse(plant["calibration_provenance"]["independent_real_validation"])
        self.assertEqual(
            sim_launch_preset_environment(preset)["SITL_REAL2SIM_BAG0402"], "1"
        )

    def test_stable_preset_is_opt_in_and_records_separate_plant(self):
        stable = resolve_sim_launch_preset(RESEARCH_POOL_YAW_STABLE_PRESET_ID)
        validate_sim_launch_preset(stable)
        env = sim_launch_preset_environment(stable)
        self.assertEqual(env["SITL_YAW_STABLE"], "1")
        self.assertEqual(env["UUV_SITL_YAW_BRAKE"], "1")
        legacy = resolve_sim_launch_preset(RESEARCH_POOL_DISTRIBUTED_PRESET_ID)
        env.update(sim_launch_preset_environment(legacy))
        self.assertEqual(env["SITL_YAW_STABLE"], "0")
        self.assertEqual(env["UUV_SITL_YAW_BRAKE"], "0")
        self.assertNotEqual(stable.profile_path, legacy.profile_path)

    def test_nominal_selection_clears_controller_overlay(self):
        preset = resolve_sim_launch_preset(COURSE_CURRENT_PRESET_ID)
        env = {"SITL_REAL2SIM_BAG0402": "1"}
        env.update(sim_launch_preset_environment(preset))
        self.assertEqual(env["SITL_REAL2SIM_BAG0402"], "0")
        self.assertNotIn("--profile-file", build_sim_launch_preset_args(preset))
        self.assertEqual(
            default_sim_launch_preset_id(
                {"UUV_GUI_SIM_PRESET": COURSE_CURRENT_PRESET_ID}
            ),
            COURSE_CURRENT_PRESET_ID,
        )

    def test_profile_file_cannot_override_selected_plant(self):
        preset = resolve_sim_launch_preset(COURSE_REAL2SIM_PRESET_ID)
        for extra in [
            ["--profile-file", "/tmp/different.json"],
            ["--profile-file=/tmp/different.json"],
        ]:
            with self.assertRaises(ValueError):
                merge_sim_launch_preset_args(
                    build_sim_launch_preset_args(preset), extra
                )

    def test_actual_sitl_parameter_builder_uses_reviewed_values(self):
        self._check_parameter_builder(stable=False)

    def test_stable_sitl_parameter_builder_overrides_hardware_values(self):
        self._check_parameter_builder(stable=True)

    def _check_parameter_builder(self, stable):
        source = (ROOT / "start_ardusub_sitl_mj311.sh").read_text()
        functions = source[
            source.index("real_param_file_has() {") : source.index(
                "# Core frame/output layout"
            )
        ]
        overlay = {}
        for line in (
            (ROOT / "config/ardusub_bag0402_replay_overlay.param")
            .read_text()
            .splitlines()
        ):
            if line.strip() and not line.startswith("#"):
                key, value = line.split()
                overlay[key] = float(value)
        if stable:
            for line in (ROOT / "config/ardusub_yaw_stable.param").read_text().splitlines():
                if line.strip() and not line.startswith("#"):
                    key, value = line.split()
                    overlay[key] = float(value)
        with tempfile.TemporaryDirectory() as tmp:
            script = "\n".join(
                [
                    "set -euo pipefail",
                    "SCRIPT_DIR=" + shlex.quote(str(ROOT)),
                    "USE_REAL_PARAM_FILE=1",
                    "REAL_PARAM_FILE="
                    + shlex.quote(
                        str(ROOT / "config/ardusub_realrobot_contract.param")
                    ),
                    "SIM_ARGS=(); USER_ARGS=()",
                    "param_supported_by_firmware() { return 0; }",
                    functions,
                    *[
                        "append_param_if_not_overridden " + key + " 99"
                        for key in overlay
                    ],
                    'printf "%s\\n" "${EXTRA_PARAM_LINES[@]}"',
                ]
            )
            result = subprocess.run(
                ["bash", "-c", script],
                check=True,
                text=True,
                capture_output=True,
                env={**os.environ, "TMPDIR": tmp, "SITL_REAL2SIM_BAG0402": "1", "SITL_YAW_STABLE": "1" if stable else "0"},
            )
        actual = {}
        for line in result.stdout.splitlines():
            fields = line.split()
            if len(fields) == 2 and fields[0] in overlay:
                self.assertNotIn(fields[0], actual)
                actual[fields[0]] = float(fields[1])
        self.assertEqual(actual, overlay)


if __name__ == "__main__":
    unittest.main()
