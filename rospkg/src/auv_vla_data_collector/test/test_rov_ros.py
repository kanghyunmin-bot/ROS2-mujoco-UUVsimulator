import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("auv_dvl_a50_msg.msg")
from auv_dvl_a50_msg.msg import DVL
from geometry_msgs.msg import TwistWithCovarianceStamped
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu

from kmu26_auv_vla_data_collector.collector import Latest, VlaDataCollector


@pytest.fixture
def node(tmp_path):
    rclpy.init(args=["--ros-args", "-p", f"dataset_root:={tmp_path}"])
    instance = VlaDataCollector()
    instance._now = lambda: 10.0
    yield instance
    instance.destroy_node()
    rclpy.shutdown()


def test_actual_dvl_message_and_frame_reach_policy_observation(node):
    dvl = DVL()
    dvl.header.stamp.sec = 10
    dvl.altitude = 2.0
    dvl.velocity_valid = True
    node._on_dvl_data(dvl)
    twist = TwistWithCovarianceStamped()
    twist.header.stamp.sec = 10
    twist.header.frame_id = "dvl_link"
    twist.twist.twist.linear.x = 1.0
    twist.twist.twist.linear.y = 2.0
    twist.twist.twist.linear.z = 3.0
    node._on_dvl_twist(twist)
    imu = Imu()
    imu.header.stamp.sec = 10
    imu.header.frame_id = "base_link"
    imu.orientation.w = 1.0
    node._on_imu(imu)
    image = np.zeros((2, 2, 3), np.uint8)
    image[:, :, 0] = 255
    node._ego = Latest(image, 10, 10, "camera0")
    node._release = Latest(image, 10, 10, "camera1")
    node._depth = Latest(2.0, 10, 10, "odom")
    node._task_description = "Hold position."
    obs = node.policy_observation(np.zeros(4))
    np.testing.assert_equal(obs["state.dvl_velocity"], [[1, -2, -3]])
    np.testing.assert_equal(obs["state.validity"], [[1, 1, 1, 1]])
    np.testing.assert_equal(obs["video.ego"][0, 0, 0], [0, 0, 255])
    node._ego.source_time = 1.0
    with pytest.raises(ValueError, match="stale ego"):
        node.policy_observation(np.zeros(4))


def test_release_is_not_a_fresh_training_action(node):
    message = OverrideRCIn()
    message.channels = [1800] * 18
    node._on_rc_override(message)
    assert "RC override" not in node._missing_start_inputs(10.0)
    message.channels = [0] * 18
    node._on_rc_override(message)
    assert "RC override" in node._missing_start_inputs(10.0)


def test_unknown_imu_frame_is_rejected(node):
    message = Imu()
    message.header.frame_id = "sensor_mount"
    message.orientation.w = 1.0
    node._on_imu(message)
    assert node._imu is None


def test_capture_age_is_checked_on_the_ros_recorder(node):
    assert not node._is_fresh(Latest(None, 1.0, 10.0, ""), 10.0, 0.25)


def test_dvl_callback_changes_native_frd_axes(node):
    message = TwistWithCovarianceStamped()
    message.header.frame_id = "dvl_link"
    message.header.stamp.sec = 10
    message.twist.twist.linear.y = 2.0
    message.twist.twist.linear.z = 3.0
    node._on_dvl_twist(message)
    np.testing.assert_equal(node._dvl_twist.value, [0, -2, -3])
