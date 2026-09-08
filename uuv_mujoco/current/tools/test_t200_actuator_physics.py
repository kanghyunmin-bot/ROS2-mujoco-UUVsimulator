"""Behavioral regressions for measured T200/Basic ESC force and dynamics."""

from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
import sys
import json

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from physics.thruster_curve_helpers import shape_thruster_command
from sim.physics.thruster_voltage import measured_voltage_surface, force_curve_at_voltage, load_voltage_trace, update_supply_voltage
from sim.physics.thruster_dynamics import thruster_response
from sim.physics.thruster_force_model import force_from_shaped_command
from sim.physics.thruster_force_performance import pwm_to_force_from_performance
from sim.physics.thruster_param_defaults import default_thruster_global_params
from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.runtime.thruster_actuator_command import update_thruster_state, shaped_thruster_command
from sim.runtime.thruster_actuator_params import global_thruster_update_params


class T200ActuatorTests(unittest.TestCase):
    def setUp(self):
        self.payload = json.loads((ROOT / "config/thruster_performance.json").read_text())
        self.config = measured_voltage_surface(self.payload)
        self.config.update(active=True, direct=True, selected_voltage=16.0)
        self.config["force"] = force_curve_at_voltage(self.config, 16.0)

    def test_every_measured_pwm_voltage_knot_is_preserved(self):
        for raw in self.payload["curves"]:
            if raw["voltage_v"] > 20:
                continue
            force = force_curve_at_voltage(self.config, raw["voltage_v"])
            np.testing.assert_allclose(np.interp(raw["pwm_us"], self.config["pwm"], force), raw["force_n"], atol=1e-12)

    def test_voltage_interpolation_is_continuous_and_bounded(self):
        a = force_curve_at_voltage(self.config, 14)
        b = force_curve_at_voltage(self.config, 16)
        np.testing.assert_allclose(force_curve_at_voltage(self.config, 15), (a + b) / 2)
        for voltage in [16 - 1e-7, 16 + 1e-7]:
            np.testing.assert_allclose(force_curve_at_voltage(self.config, voltage), b, atol=1e-6)

    def test_synthetic_and_unmeasured_voltages_are_not_used(self):
        self.assertEqual(self.config["voltage_grid"][-1], 20)
        for voltage in [0, 9.9, 22.2, float("nan"), float("inf")]:
            with self.assertRaises(ValueError):
                force_curve_at_voltage(self.config, voltage)

    def test_standalone_cli_defaults_to_the_same_measured_pwm_path(self):
        import argparse
        from sim.runtime.cli_profile import add_profile_args
        parser = argparse.ArgumentParser()
        add_profile_args(parser, scenes_dir=ROOT/"scenes", profile_path=ROOT/"config/sim_profiles.json", thruster_perf_path=ROOT/"config/thruster_performance.json")
        args = parser.parse_args([])
        self.assertTrue(args.thruster_perf_direct)
        self.assertFalse(args.disable_thruster_perf)
        self.assertTrue(parser.parse_args(["--disable-thruster-perf"]).disable_thruster_perf)

    def test_basic_esc_deadband_and_pwm_saturation(self):
        for pwm in range(1475, 1526):
            self.assertEqual(pwm_to_force_from_performance((pwm - 1500) / 400, self.config), 0)
        self.assertAlmostEqual(pwm_to_force_from_performance(2, self.config), 51.43622732069733)
        self.assertAlmostEqual(pwm_to_force_from_performance(-2, self.config), -39.90792903824067)

    def test_legacy_gain_and_reverse_asymmetry_cannot_modify_measured_curve(self):
        for command in [-1, -.5, .5, 1]:
            actual = force_from_shaped_command(name="yaw_lf", command_shaped=command,
                gain=3.3, perf_cfg=self.config, thruster_direct_scale={"yaw_lf": 1},
                thruster_global=default_thruster_global_params(), thruster_force_max=21,
                thruster_reverse_asymmetry={"yaw_lf": .1})
            self.assertEqual(actual, pwm_to_force_from_performance(command, self.config))

    def test_command_limit_never_renormalizes_to_full_drive(self):
        self.assertLess(shape_thruster_command(1, .0625, .5), .5)
        runtime = SimpleNamespace(target={"t": 1}, state={"t": 0}, perf_cfg=self.config,
            thruster_global={**default_thruster_global_params(), "command_limit": .5},
            thruster_tau_up={"t": None}, thruster_tau_down={"t": None})
        params = global_thruster_update_params(runtime)
        state = update_thruster_state(runtime, "t", 10, params)
        self.assertAlmostEqual(shaped_thruster_command(runtime, state, params), .5)

    def test_neutral_pwm_brakes_existing_drive_and_stays_stopped(self):
        runtime = SimpleNamespace(target={"t": .05}, state={"t": 1}, perf_cfg=self.config,
            thruster_global=default_thruster_global_params(),
            thruster_tau_up={"t": None}, thruster_tau_down={"t": None})
        params = global_thruster_update_params(runtime)
        first = update_thruster_state(runtime, "t", .01, params)
        self.assertTrue(0 < first < 1)
        update_thruster_state(runtime, "t", 2, params)
        self.assertEqual(pwm_to_force_from_performance(runtime.state["t"], self.config), 0)

    def test_reversal_uses_deceleration_and_is_timestep_consistent(self):
        import math
        self.assertAlmostEqual(thruster_response(1, -1, .01, .04, .06), -1 + 2 * math.exp(-.01 / .06))
        coarse = thruster_response(1, -1, .1, .04, .06)
        fine = 1
        for _ in range(100):
            fine = thruster_response(fine, -1, .001, .04, .06)
        self.assertAlmostEqual(coarse, fine, places=12)
        self.assertAlmostEqual(thruster_response(-1, 1, .1, .04, .06), -coarse)

    def test_voltage_trace_changes_runtime_force_and_holds_endpoints(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "voltage.csv"
            path.write_text("sim_time,voltage_v\n0,16\n2,12\n")
            self.config["voltage_trace"] = load_voltage_trace(path, self.config)
            update_supply_voltage(self.config, 1)
            self.assertEqual(self.config["selected_voltage"], 14)
            np.testing.assert_allclose(self.config["force"], force_curve_at_voltage(self.config, 14))
            update_supply_voltage(self.config, 10)
            self.assertEqual(self.config["selected_voltage"], 12)

    def test_invalid_voltage_trace_and_missing_curve_fail_explicitly(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "bad.csv"
            for content in ["sim_time,voltage_v\n0,16\n0,14\n", "sim_time,voltage_v\n0,16\n1,22.2\n"]:
                path.write_text(content)
                with self.assertRaises(ValueError):
                    load_voltage_trace(path, self.config)
            with self.assertRaises(ValueError):
                load_thruster_performance_config(Path(temp) / "missing.json", requested_voltage=16, direct=True)


class T200MujocoIntegrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import contextlib
        import io
        import mujoco
        from check_research_pool_physics import _build_runtime
        cls.mujoco = mujoco
        with contextlib.redirect_stdout(io.StringIO()):
            cls.runtime = _build_runtime(mujoco, profile_name="research_pool_distributed", fluid_model="legacy", use_custom_hydrodynamics=True)
            cls.runtime.thruster_actuator.perf_cfg = load_thruster_performance_config(ROOT / "config/thruster_performance.json", requested_voltage=16, direct=True)

    def test_site_forces_reach_mujoco_once_with_geometric_moments(self):
        runtime = self.runtime
        model, data, actuator = runtime.model, runtime.data, runtime.thruster_actuator
        base_id, q, v = int(runtime.state.base_id), int(runtime.state.world_qpos_adr), int(runtime.state.world_qvel_adr)
        data.qpos[q+2] = -1.5
        data.qpos[q+3:q+7] = [1, 0, 0, 0]
        data.qvel[:] = 0
        self.mujoco.mj_forward(model, data)
        for name in actuator.all_thruster_names:
            for command in [-.5, .5]:
                actuator.target.update({n: 0 for n in actuator.all_thruster_names})
                actuator.state.update({n: 0 for n in actuator.all_thruster_names})
                actuator.target[name] = command
                actuator.update_forces(10, base_id=base_id)
                expected_force = pwm_to_force_from_performance(command, actuator.perf_cfg)
                self.assertAlmostEqual(actuator.force_cmd[name], expected_force)
                self.mujoco.mj_forward(model, data)
                np.testing.assert_allclose(data.qfrc_actuator[v:v+3], actuator.last_force_body, atol=1e-6)
                # Free-joint angular generalized force is about the frame origin,
                # while the runtime diagnostic reports moment about the CoM.
                torque_origin = actuator.last_torque_body + np.cross(model.body_ipos[base_id], actuator.last_force_body)
                np.testing.assert_allclose(data.qfrc_actuator[v+3:v+6], torque_origin, atol=1e-6)

    def test_debug_header_and_real_runtime_row_remain_aligned(self):
        import io
        import csv
        from sim.physics.thruster_debug import build_thruster_debug_header
        from sim.runtime.thruster_debug_emit import emit_thruster_debug_row
        runtime = self.runtime
        actuator = runtime.thruster_actuator
        stream = io.StringIO()
        header = build_thruster_debug_header(actuator.all_thruster_names)
        emit_thruster_debug_row(stream, thruster_names=actuator.all_thruster_names,
            mujoco_module=self.mujoco, model=runtime.model, data=runtime.data,
            base_id=int(runtime.state.base_id), world_qvel_adr=int(runtime.state.world_qvel_adr),
            water_surface_z=0, vehicle_mass=15, gravity=9.81,
            base_origin_world=lambda: np.array([0, 0, -1.5]),
            body_velocity_local=lambda: (np.zeros(3), np.zeros(3)),
            last_buoy_force=np.array([0, 0, 147.15]), initial_depth_hold_active=False,
            last_thruster_force_body=actuator.last_force_body, last_thruster_torque_body=actuator.last_torque_body,
            sitl_servo_pwm_values=[1500]*8, sitl_servo_cmd_norm={},
            thr_target=actuator.target, thr_state=actuator.state, thruster_force_cmd=actuator.force_cmd,
            thruster_direct_scale=actuator.thruster_direct_scale,
            thruster_diagnostics={"supply_voltage_v": 16})
        row = next(csv.reader(io.StringIO(stream.getvalue())))
        self.assertEqual(len(header), len(row))
        self.assertEqual(float(row[header.index("thruster_supply_voltage_v")]), 16)


if __name__ == "__main__":
    unittest.main()
