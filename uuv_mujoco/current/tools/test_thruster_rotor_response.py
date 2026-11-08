"""Regression tests for the measured T200 rotor response option."""

from pathlib import Path
import sys
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from sim.physics.thruster_force_performance import native_pwm_direction, pwm_to_force_from_performance
from sim.physics.thruster_performance_selector import select_thruster_performance_config
from sim.physics.thruster_voltage import select_supply_voltage
from sim.runtime.thruster_release_seed import seed_thruster_state_from_current_targets
from tools.check_research_pool_physics import _build_runtime
from tools.test_demo_reset import fixture as reset_fixture


CURVE = ROOT / "config/thruster_performance.json"


@pytest.fixture(scope="module")
def plant():
    runtime = _build_runtime(
        mujoco, profile_name="research_pool_distributed", fluid_model="legacy",
        use_custom_hydrodynamics=True,
    )
    runtime.data.qpos[int(runtime.state.world_qpos_adr) + 2] = -1.5
    mujoco.mj_forward(runtime.model, runtime.data)
    return runtime


def test_rotor_reversal_coast_and_mujoco_wrench(plant):
    actuator = plant.thruster_actuator
    actuator.perf_cfg = select_thruster_performance_config(
        args=SimpleNamespace(disable_thruster_perf=False, thruster_perf_direct=True),
        active_thruster_voltage=16.0, path=CURVE, plant_replay_direct_rcout=True,
        response_model="rpm",
    )
    # Pick a reversed scene channel so the asymmetric native curve is exercised.
    name = next(name for name in actuator.all_thruster_names if native_pwm_direction(name) < 0)
    base = int(plant.state.base_id)
    actuator.target[name] = -0.5
    actuator.update_forces(2.0, base_id=base)
    expected = -pwm_to_force_from_performance(0.5, actuator.perf_cfg)
    assert actuator.force_cmd[name] == pytest.approx(expected, abs=1e-10)
    assert actuator.rotor_rpm[name] < 0.0
    mujoco.mj_forward(plant.model, plant.data)
    free_dof = int(plant.state.world_qvel_adr)
    np.testing.assert_allclose(
        plant.data.qfrc_actuator[free_dof:free_dof + 3], actuator.last_force_body, atol=1e-6
    )

    actuator.target[name] = 0.5
    actuator.update_forces(0.001, base_id=base)
    assert actuator.rotor_rpm[name] < 0.0 and actuator.force_cmd[name] < 0.0
    actuator.update_forces(2.0, base_id=base)
    assert actuator.force_cmd[name] == pytest.approx(
        -pwm_to_force_from_performance(-0.5, actuator.perf_cfg), abs=1e-10
    )
    rpm_before = actuator.rotor_rpm[name]
    select_supply_voltage(actuator.perf_cfg, 12.0)
    actuator.update_forces(0.0, base_id=base)
    assert actuator.rotor_rpm[name] == rpm_before
    actuator.target[name] = 0.0
    actuator.update_forces(0.01, base_id=base)
    assert 0.0 < actuator.rotor_rpm[name] < rpm_before
    actuator.update_forces(10.0, base_id=base)
    assert abs(actuator.force_cmd[name]) < 1e-10
    actuator.target[name] = 0.5
    seed_thruster_state_from_current_targets(thruster_actuator_runtime=actuator, base_id=base)
    assert actuator.force_cmd[name] == pytest.approx(
        -pwm_to_force_from_performance(-0.5, actuator.perf_cfg), abs=1e-10
    )


def test_demo_reset_clears_rotor_memory():
    reset = reset_fixture()
    actuator = reset.physics.thruster_actuator_runtime
    actuator.rotor_rpm = {"thruster": 1400.0}
    reset.restore()
    assert actuator.rotor_rpm == {}
    assert not reset.step.data.ctrl.any()
