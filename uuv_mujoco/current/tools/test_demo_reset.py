"""Exercise scene reset against actual MuJoCo constraints and runtime state."""
import sys
import time
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as NS

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sim.runtime.demo_reset import DemoReset
from tools.check_buoy_physics_contract import runtime_for


def fixture():
    model = mujoco.MjModel.from_xml_path(str(ROOT / 'scenes/research_pool_slam_scene.xml'))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    buoys = runtime_for(mujoco, model, data)
    actuator = NS(**{k: {'thruster': 0.8} for k in ('state', 'target', 'force_cmd', 'prop_phase')},
                  **{k: np.ones(3) for k in ('last_reaction_torque_world', 'last_reaction_torque_body', 'last_force_body', 'last_torque_body')})
    physics = NS(thruster_actuator_runtime=actuator,
                 underwater_wrench_runtime=NS(prev_rel_nu_body=np.ones(6), prev_rel_nu_valid=True))
    bridge = NS(_course_buoy_runtime=buoys, _mavros_armed=True, _mavros_mode='STABILIZE', _sitl_transport=None)
    step = NS(model=model, data=data, mujoco=mujoco, publish_ros_once=lambda: None, publish_qgc_video_once=lambda: None)
    return DemoReset(step, physics, bridge)


def test_restore_constraints_velocity_and_time():
    reset = fixture()
    data, model = reset.step.data, reset.step.model
    original = data.qpos.copy()
    buoy = reset.buoy_runtime.buoys[0]
    buoy.detached = buoy.netted = True
    buoy.last_runtime_wrench[:] = 5
    data.eq_active[buoy.eq_id] = False
    data.qpos[buoy.free_qposadr] += 2
    data.qvel[:] = 3
    data.xfrc_applied[:] = 5
    model.geom_contype[:] = 0
    data.time = 123.5
    reset.restore()
    np.testing.assert_array_equal(data.qpos, original)
    assert data.time == 123.5 and reset.bridge._mavros_armed
    assert not reset.buoy_runtime.buoys[0].detached
    assert not reset.buoy_runtime.buoys[0].netted
    assert data.eq_active[buoy.eq_id]
    assert not data.qvel.any() and not data.xfrc_applied.any()
    assert not reset.physics.underwater_wrench_runtime.prev_rel_nu_valid
    assert reset.physics.thruster_actuator_runtime.state['thruster'] == 0
    np.testing.assert_array_equal(model.geom_contype, reset.model_arrays['geom_contype'])
    # Reattached rope/magnet remains stable under the production buoy forces.
    for _ in range(50):
        data.qpos[:7] = original[:7]
        data.qvel[:6] = 0
        reset.buoy_runtime.apply(model.opt.timestep)
        mujoco.mj_step(model, data)
    assert np.isfinite(data.qpos).all()
    assert not reset.buoy_runtime.buoys[0].detached


def test_request_completes_after_monotonic_settle_and_repeats():
    reset = fixture()
    for _ in range(2):
        future = Future()
        reset.requests.put((time.monotonic(), future, NS()))
        start = reset.step.data.time
        for _ in range(int(2.0 / reset.step.model.opt.timestep) + 2):
            if not reset.tick(is_paused=False, publish_ros=True):
                break
        assert future.result(timeout=0).success
        assert reset.step.data.time >= start + 2
        assert reset.bridge._mavros_armed
        assert not reset.step.data.qvel.any()


@pytest.mark.parametrize('mode,paused,age', [('ALT_HOLD', False, 0), ('STABILIZE', True, 0), ('STABILIZE', False, 4)])
def test_reject_without_moving_scene(mode, paused, age):
    reset = fixture()
    reset.bridge._mavros_mode = mode
    reset.step.data.qpos[0] += 1
    before = reset.step.data.qpos.copy()
    future = Future()
    reset.requests.put((time.monotonic() - age, future, NS()))
    assert not reset.tick(is_paused=paused, publish_ros=False)
    assert not future.result(timeout=0).success
    np.testing.assert_array_equal(reset.step.data.qpos, before)


def test_ros_request_acknowledges_physics_completion():
    """Run only with a dedicated ROS domain, never against the operator's stack."""
    import os
    if os.environ.get('UUV_ISOLATED_RESET_TEST') != '1':
        pytest.skip('requires isolated ROS domain')
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from std_srvs.srv import Trigger
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node('demo_reset_isolated_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    reset = fixture()
    reset.bridge.node = node
    reset.install()
    client = node.create_client(Trigger, '/uuv_mujoco/demo_reset')
    try:
        assert client.wait_for_service(timeout_sec=3)
        response = client.call_async(Trigger.Request())
        deadline = time.monotonic() + 8
        while not response.done() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.001)
            reset.tick(is_paused=False, publish_ros=False)
        assert response.done()
        assert response.result().success
        assert reset.step.data.time >= 2.0
    finally:
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown()
        context.shutdown()


def test_sitl_mode_cycle_waits_for_ack_without_arming():
    import threading
    reset = fixture()
    commands = []
    transport = NS(vehicle_armed=True, vehicle_mode='STABILIZE', mavlink_connected=True,
                   send_set_mode=lambda mode: commands.append(mode) or True)
    reset.bridge._sitl_transport = transport
    reset.bridge._sitl_transport_lock = threading.Lock()
    future = Future()
    reset.requests.put((time.monotonic(), future, NS()))
    reset.tick(is_paused=False, publish_ros=False)
    assert commands == ['MANUAL'] and not future.done()
    reset.step.data.time = reset.until
    reset.tick(is_paused=False, publish_ros=False)
    assert commands == ['MANUAL'] and not future.done()
    transport.vehicle_mode = 'MANUAL'
    reset.tick(is_paused=False, publish_ros=False)
    assert commands == ['MANUAL', 'STABILIZE'] and not future.done()
    reset.step.data.time = reset.until
    reset.tick(is_paused=False, publish_ros=False)
    assert not future.done()
    transport.vehicle_mode = 'STABILIZE'
    reset.tick(is_paused=False, publish_ros=False)
    assert future.result(timeout=0).success and transport.vehicle_armed


def test_reset_timeout_does_not_report_success():
    reset = fixture()
    future = Future()
    reset.requests.put((time.monotonic(), future, NS()))
    reset.tick(is_paused=False, publish_ros=False)
    reset.deadline = time.monotonic() - 1
    reset.tick(is_paused=False, publish_ros=False)
    assert not future.result(timeout=0).success


@pytest.mark.parametrize('sitl_enabled', [True, False])
def test_reset_keeps_sitl_sensor_feed_between_ros_ticks(sitl_enabled):
    """A held pose still needs FCU sensor packets on non-telemetry ticks."""
    from unittest.mock import Mock

    reset = fixture()
    reset.step.sitl_enabled = sitl_enabled
    reset.step.publish_ros_once = Mock()
    reset.step.publish_qgc_video_once = Mock()
    future = Future()
    reset.requests.put((time.monotonic(), future, NS()))
    for _ in range(8):
        assert reset.tick(is_paused=False, publish_ros=False)
    assert reset.step.publish_ros_once.call_count == (8 if sitl_enabled else 0)
    reset.step.publish_qgc_video_once.assert_not_called()
