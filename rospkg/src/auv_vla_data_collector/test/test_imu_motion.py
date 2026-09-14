"""Independent raw motion must not borrow the AHRS capture timestamp."""

import json

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from test_episode_lifecycle import node  # noqa: F401


def enable_motion(n):
    n.set_parameters([Parameter("use_sim_time", value=True)])
    n._clock_last = 10.0
    n._imu_motion_topic = "/mavros/imu/data_raw"
    n._imu_motion_frame = "fcu_link"
    n._imu_motion_convention = "FLU"


def motion(stamp=10.0, frame="fcu_link"):
    msg = Imu()
    msg.header.stamp.sec = int(stamp)
    msg.header.stamp.nanosec = round((stamp - int(stamp)) * 1e9)
    msg.header.frame_id = frame
    msg.orientation_covariance[0] = -1.0
    msg.angular_velocity.x = 0.25
    msg.linear_acceleration.z = 9.8
    return msg


def test_missing_raw_motion_blocks_start_status_and_policy(node):
    enable_motion(node)
    status = json.loads(node._on_get_status(Trigger.Request(), Trigger.Response()).message)
    assert "IMU motion" in status["missing"]
    assert not node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    with pytest.raises(ValueError, match="imu_motion"):
        node.policy_observation(np.zeros(4))


@pytest.mark.parametrize("stamp,frame,bad", [(9.7, "fcu_link", False), (10, "wrong", False), (10, "fcu_link", True)])
def test_stale_wrong_frame_or_nonfinite_motion_is_rejected(node, stamp, frame, bad):
    enable_motion(node)
    msg = motion(stamp, frame)
    if bad:
        msg.linear_acceleration.z = float("nan")
    node._on_imu_motion(msg)
    assert not node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    with pytest.raises(ValueError, match="imu_motion"):
        node.policy_observation(np.zeros(4))


def test_raw_motion_drives_saved_state_policy_and_independent_audit(node):
    enable_motion(node)
    node._on_imu_motion(motion(9.95))
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    assert len(node._states) == 1
    assert node._states[0][7] == pytest.approx(0.25)
    assert node._states[0][12] == pytest.approx(9.8)
    obs = node.policy_observation(np.zeros(4))
    assert obs["state.linear_acceleration"][0, 2] == pytest.approx(9.8)
    saved = node._finish_episode(False, "operator_stop")
    manifest = json.loads((saved / "manifest.json").read_text())
    assert manifest["provenance"]["imu_motion"]["topic"] == "/mavros/imu/data_raw"
    row = json.loads((saved / "vehicle_state.jsonl").read_text())
    assert row["imu_motion"]["source_time"] == pytest.approx(9.95)
    assert row["imu_motion"]["receipt_time"] == 10.0
    with np.load(saved / "samples.npz") as data:
        assert data["observation_state"].shape == (1, 23)
        assert data["source_timestamp"].shape == (1, 7)
        assert data["source_timestamp"][0, 2] == 10.0


def test_active_sampling_rejects_stale_motion_and_rewind_clears_cache(node):
    enable_motion(node)
    node._on_imu_motion(motion())
    assert node._on_start_episode(Trigger.Request(), Trigger.Response()).success
    node._record_sample()
    node._imu_motion.source_time = 9.0
    node._now = lambda: 10.1
    node._record_sample()
    assert len(node._states) == 1
    node._now = lambda: 0.0
    node._watch_clock()
    assert node._imu_motion is None


def test_legacy_motion_still_comes_from_ahrs(node):
    node._imu.value[1][2] = 4.0
    assert node.policy_observation(np.zeros(4))["state.linear_acceleration"][0, 2] == 4.0
