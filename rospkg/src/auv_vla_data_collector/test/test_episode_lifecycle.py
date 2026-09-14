"""Clock and vehicle transitions cannot join unrelated demonstrations."""

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from mavros_msgs.msg import State
from std_srvs.srv import Trigger
from kmu26_auv_vla_data_collector.collector import Latest, VlaDataCollector


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"dataset_root:={tmp_path}"])
    n = VlaDataCollector()
    n._now = lambda: 10.0
    n._task_description = "Check connection."
    for key in ("ego", "release"):
        setattr(n, "_" + key, Latest(np.zeros((8, 8, 3), np.uint8), 10, 10, key))
    n._imu = Latest(
        (np.zeros(3), np.zeros(3), np.array([1, 0, 0, 0])), 10, 10, "base_link"
    )
    n._depth = Latest(1.0, 10, 10, "odom")
    n._rc_tracker.update([1500] * 18, 10)
    n._control = Latest(np.full(4, 1500), 10, 10, "")
    yield n
    n.destroy_node()
    rclpy.shutdown()


def test_clock_rewind_clears_cached_inputs(node):
    node._watch_clock()
    node._now = lambda: 0.0
    node._watch_clock()
    assert node._ego is None
    assert not node._rc_tracker.fresh(10, 0.5)


@pytest.mark.parametrize("clock_state", ["zero", "unobserved", "stalled", "rewound"])
def test_simulation_start_and_status_share_clock_gate(node, clock_state):
    import json
    import time
    from rclpy.parameter import Parameter

    node.set_parameters([Parameter("use_sim_time", value=True)])
    node._clock_last = 10.0
    if clock_state == "zero":
        node._now = lambda: 0.0
        node._clock_last = 0.0
        for name in ("ego", "release", "imu", "depth", "control"):
            getattr(node, "_" + name).source_time = 0.0
            getattr(node, "_" + name).received_time = 0.0
        node._rc_tracker.update([1500] * 18, 0.0)
    elif clock_state == "unobserved":
        node._clock_last = None
    elif clock_state == "stalled":
        node._clock_wall = time.monotonic() - 1.1
    else:
        node._clock_last = 11.0

    status = json.loads(node._on_get_status(Trigger.Request(), Trigger.Response()).message)
    start = node._on_start_episode(Trigger.Request(), Trigger.Response())
    assert not status["ready"]
    assert "simulation clock" in status["missing"]
    assert not start.success
    assert "simulation clock" in start.message
    assert not node._active
    assert not list(node._dataset_root.glob(".recording_episode_*"))


def test_policy_rejects_stalled_simulation_before_watchdog_runs(node):
    import time
    from rclpy.parameter import Parameter

    node.set_parameters([Parameter("use_sim_time", value=True)])
    node._clock_last = 10.0
    node._clock_wall = time.monotonic() - 1.1
    with pytest.raises(ValueError, match="clock"):
        node.policy_observation(np.zeros(4))


def test_initialized_simulation_clock_allows_collection_and_policy(node):
    from rclpy.parameter import Parameter

    node.set_parameters([Parameter("use_sim_time", value=True)])
    node._watch_clock()
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    assert len(node._states) == 1
    assert node.policy_observation(np.zeros(4))["state.depth"][0, 0] == 1.0


def test_wall_clock_collection_does_not_require_simulation_clock(node):
    node._clock_last = None
    node._clock_wall = -float("inf")
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    assert len(node._states) == 1
    assert node.policy_observation(np.zeros(4))["state.depth"][0, 0] == 1.0


def test_clock_rewind_requires_new_vehicle_telemetry(node):
    node._on_vehicle_state(State(connected=True, armed=True, mode="STABILIZE"))
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    node._watch_clock()
    node._now = lambda: 0.0
    node._watch_clock()
    assert node._vehicle is None
    assert node._vehicle_at == -float("inf")
    assert node._vehicle_ros_at == -float("inf")
    assert node._episode_vehicle is None
    assert not node._active
    assert node._last_result["termination_reason"] == "clock_reset"


@pytest.mark.parametrize(
    "sim_time,wall_elapsed,ros_elapsed,clock_progress,expected",
    [
        (True, 4.0, 0.5, True, True),
        (True, 0.5, 2.1, True, False),
        (True, 1.2, 0.0, False, False),
        (True, 0.2, -0.5, False, False),
        (False, 1.9, 4.0, True, True),
        (False, 2.1, 0.5, True, False),
    ],
)
def test_vehicle_freshness_uses_collection_clock(
    node, monkeypatch, sim_time, wall_elapsed, ros_elapsed, clock_progress, expected
):
    from rclpy.parameter import Parameter

    wall = [100.0]
    monkeypatch.setattr("kmu26_auv_vla_data_collector.collector.time.monotonic", lambda: wall[0])
    node.set_parameters([Parameter("use_sim_time", value=sim_time)])
    node._data_source = "simulation" if sim_time else "real"
    node._session_id = "clock_freshness_test"
    node._provenance_context = {"test": True}
    node.count_publishers = lambda topic: 1
    node._clock_last = 10.0
    node._clock_wall = wall[0]
    node._on_vehicle_state(State(connected=True, armed=True, mode="STABILIZE"))
    wall[0] += wall_elapsed
    node._now = lambda: 10.0 + ros_elapsed
    if clock_progress:
        node._watch_clock()
    assert node._demonstration_ready() is expected


def test_active_recording_rejects_stalled_clock_before_watchdog_runs(node):
    import time
    from rclpy.parameter import Parameter

    node.set_parameters([Parameter("use_sim_time", value=True)])
    node._clock_last = 10.0
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    node._now = lambda: 10.1
    node._clock_wall = time.monotonic() - 1.1
    node._record_sample()
    assert not node._active
    assert node._last_result["termination_reason"] == "clock_stalled"
    assert len(node._states) == 1


def test_mode_change_ends_episode(node):
    node._on_vehicle_state(State(connected=True, armed=True, mode="STABILIZE"))
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    node._on_vehicle_state(State(connected=True, armed=True, mode="MANUAL"))
    assert not node._active


def test_stale_camera_stops_demonstration(node):
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    node._collection_kind = "task_demonstration"
    node._demonstration_ready = lambda: True
    node._now = lambda: 10.1
    node._record_sample()  # Same camera stamp despite next sample.
    assert not node._active


def test_driver_held_altitude_is_invalid_on_bottom_lock_loss(node):
    node._dvl_data = Latest((2.0, False), 10, 10, "dvl_link")
    obs = node.policy_observation(np.zeros(4))
    assert obs["state.validity"][0, 3] == 0
    assert obs["state.altitude"][0, 0] == 0


def test_rejected_new_dvl_packet_cannot_revalidate_old_twist(node):
    node._dvl_data = Latest((2.0, True), 10, 10, "dvl_link")
    node._dvl_twist = Latest(np.array([1, 2, 3]), 9.9, 9.9, "dvl_link")
    assert node.policy_observation(np.zeros(4))["state.validity"][0, 2] == 0


def test_clock_pause_ends_episode(node, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(
        "kmu26_auv_vla_data_collector.collector.time.monotonic", lambda: clock[0]
    )
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    node._watch_clock()
    clock[0] += 1.1
    node._watch_clock()
    assert not node._active
    assert node._ego is None


def test_gap_ends_episode_without_compressing_time(node):
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    node._now = lambda: 10.3
    node._record_sample()
    assert not node._active
    assert len(node._states) == 1


def test_recorder_status_tracks_save_and_discard(node):
    import json
    from std_srvs.srv import SetBool
    def status():
        return json.loads(node._on_get_status(Trigger.Request(), Trigger.Response()).message)
    assert status()['ready'] and not status()['active']
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    assert status()['active'] and status()['frames'] == 1
    assert node._on_stop_episode(SetBool.Request(data=True), SetBool.Response()).success
    assert status()['last_result']['success'] is True
    assert status()['last_result']['termination_reason'] == 'operator_stop'
    node._on_start_episode(Trigger.Request(), Trigger.Response())
    node._on_discard_episode(Trigger.Request(), Trigger.Response())
    assert status()['last_result']['termination_reason'] == 'discarded'


def test_recorder_status_exposes_automatic_interruption(node):
    import json
    node._on_start_episode(Trigger.Request(), Trigger.Response())
    node._record_sample()
    node._interrupt_episode('clock_stalled')
    status = json.loads(node._on_get_status(Trigger.Request(), Trigger.Response()).message)
    assert not status['active']
    assert status['last_result']['success'] is False
    assert status['last_result']['termination_reason'] == 'clock_stalled'


def test_web_recorder_real_ros_services(node):
    import sys
    import time
    from pathlib import Path
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'uuv_mujoco/current'))
    from gui.web_recorder import WebRecorder
    for timer in node.timers:
        timer.cancel()
    gui = Node('recorder_test_gui')
    manager = WebRecorder(gui, None)
    manager.session = node._session_id = 'test_session'
    executor = SingleThreadedExecutor()
    executor.add_node(gui)
    executor.add_node(node)
    def wait_for(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate(), manager.payload()
    try:
        wait_for(lambda: manager.payload()['ready'])
        manager.command('start')
        wait_for(lambda: manager.payload().get('active') and manager.payload()['online'])
        # The fixture is deliberately a wiring check, not a task demonstration.
        node._record_sample()
        manager.command('success')
        wait_for(lambda: manager.payload().get('last_result', {}).get('success') is True)
        assert manager.payload()['active'] is False
        with pytest.raises(ValueError):
            manager.command('success')
    finally:
        manager.shutdown()
        executor.remove_node(gui)
        executor.remove_node(node)
        gui.destroy_node()
        executor.shutdown()
