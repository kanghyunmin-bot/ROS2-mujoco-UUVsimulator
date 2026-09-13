"""Startup keeps physics time alive but rejects early arming and pose release."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
import sys

import mujoco
import numpy as np
import pytest

CURRENT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CURRENT))
from sim.startup_alignment import estimator_aligned, startup_wait_reason
from sim.runtime.initial_state_policy import create_initial_depth_hold_state
from sim.runtime.initial_depth_runtime import InitialDepthHoldRuntime
from sim.runtime.simulation_step_hold_release import maybe_release_initial_depth_hold
from sim.transport.mavlink_telemetry_handlers import handle_target_mavlink_telemetry
from bridge.sitl_arm_mode_queue_arm import queue_arm_command_impl
from gui.sim_stack_env_contract import build_gui_sim_stack_env


@pytest.mark.parametrize("flags,age,expected", [(0, 0, False), (1, 0, True), (5, 1, True), (1, 6, False), (1, float('nan'), False)])
def test_ready_requires_ekf_attitude_and_fresh_report(flags, age, expected):
    assert estimator_aligned({'ekf_flags': flags, 'ekf_age_s': age}) is expected


def test_elapsed_time_or_statustext_alone_cannot_arm(monkeypatch):
    monkeypatch.setenv('UUV_STARTUP_ALIGNMENT_HOLD', '1')
    transport = SimpleNamespace(mavlink_telemetry_status=lambda: {'statustext_text': 'ArduPilot Ready'})
    assert queue_arm_command_impl(transport, True) is False
    assert not hasattr(transport, '_sitl_pending_arm_target'), 'early ARM must not be queued'
    assert startup_wait_reason({'startup_alignment_required': True, 'ekf_flags': 0, 'ekf_age_s': 0})


def test_reboot_invalidates_previous_health():
    status = {'att_time_boot_ms': 30000, 'ekf_flags': 1, 'ekf_wall_s': 10}
    handle_target_mavlink_telemetry(status, 'ATTITUDE', {'time_boot_ms': 100}, 11)
    assert 'ekf_flags' not in status and 'ekf_wall_s' not in status


def make_hold(monkeypatch):
    monkeypatch.setenv('UUV_STARTUP_ALIGNMENT_HOLD', '1')
    args = SimpleNamespace(sitl=True, hold_initial_depth_until_release=False, initial_depth_m=None,
                           release_linear_velocity_body=None, release_angular_velocity_body=None)
    state = create_initial_depth_hold_state(args=args, initial_bar30=SimpleNamespace(value_m=None))
    model = mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".001"/>
        <worldbody><body pos="0 0 -1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>''')
    data = mujoco.MjData(model)
    state.capture_pose(data=data, world_qpos_adr=0, water_surface_z=0)
    runtime = InitialDepthHoldRuntime(state, data, mujoco, model, 1, 0, 0, Mock(), Mock())
    return state, model, data, runtime


def test_pose_stays_fixed_while_time_advances_without_depth_argument(monkeypatch):
    state, model, data, runtime = make_hold(monkeypatch)
    assert state.active and state.startup_alignment_required
    initial = data.qpos.copy()
    for _ in range(1000):
        data.xfrc_applied[1] = [10, 5, -20, 2, 3, 4]
        mujoco.mj_step(model, data)
        runtime.apply_hold()
    np.testing.assert_array_equal(data.qpos, initial)
    np.testing.assert_array_equal(data.qvel, np.zeros(6))
    assert data.time == pytest.approx(1.)


def test_release_requires_both_alignment_and_explicit_arm(monkeypatch):
    state, model, data, runtime = make_hold(monkeypatch)
    status = {'ekf_flags': 0, 'ekf_age_s': 0}
    bridge = SimpleNamespace(_sitl_transport=SimpleNamespace(mavlink_telemetry_status=lambda: status),
                             sitl_vehicle_armed=lambda: False, sitl_vehicle_mode=lambda: 'STABILIZE', publish=Mock())
    release = Mock(side_effect=lambda reason: runtime.release(reason, ros_bridge=bridge))
    def tick():
        maybe_release_initial_depth_hold(initial_depth_hold=state, auto_release=True, ros_bridge=bridge,
                                         sitl_servo_pwm_values=[1500]*8, release_initial_depth_hold=release)
    assert runtime.release('early service', ros_bridge=bridge) is False
    tick()
    status['ekf_flags'] = 1
    tick()
    assert state.active and not release.called, 'alignment alone must not release or arm'
    bridge.sitl_vehicle_armed = lambda: True
    tick()
    assert not state.active and release.call_count == 1
    tick()
    assert release.call_count == 1
    mujoco.mj_step(model, data)
    assert np.linalg.norm(data.qvel) > 0, 'vehicle must be free after operator ARM'


def test_gui_launch_enables_startup_hold():
    env = build_gui_sim_stack_env({}, backend='docker', sim_stack_dir=CURRENT)
    assert env['UUV_STARTUP_ALIGNMENT_HOLD'] == '1'
