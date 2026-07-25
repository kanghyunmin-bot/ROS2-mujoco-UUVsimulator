import time

import pytest
import rclpy
from dvl_msgs.msg import CommandResponse
from rclpy.qos import ReliabilityPolicy

from kmu26_auv_web_gui.ros_interface import LocalizationRosNode


class _FakeDvlPublisher:
    def __init__(self, subscriber_count: int = 1):
        self.subscriber_count = subscriber_count
        self.messages = []

    def get_subscription_count(self) -> int:
        return self.subscriber_count

    def publish(self, message) -> None:
        self.messages.append(message)


def _new_node() -> tuple[LocalizationRosNode, bool]:
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init(args=[])
    return LocalizationRosNode(), owned_context


def _destroy_node(node: LocalizationRosNode, owned_context: bool) -> None:
    node.destroy_node()
    if owned_context and rclpy.ok():
        rclpy.shutdown()


def _response(command: str, success: bool, error_message: str = "") -> CommandResponse:
    message = CommandResponse()
    message.response_to = command
    message.success = success
    message.error_message = error_message
    message.result = 0
    message.format = "json_v3.1"
    message.type = "response"
    return message


def test_calibration_completes_only_after_matching_success_ack() -> None:
    node, owned_context = _new_node()
    fake_publisher = _FakeDvlPublisher()
    node._dvl_config_pub = fake_publisher
    try:
        started = node.start_dvl_gyro_calibration(timeout_s=1.0)

        assert started["state"] == "calibrating"
        assert len(fake_publisher.messages) == 1
        assert fake_publisher.messages[0].command == "calibrate_gyro"
        assert node.snapshot()["dvl_events"][-1]["success"] is None
        with pytest.raises(RuntimeError, match="calibration is in progress"):
            node.publish_dvl_command("reset_dead_reckoning")
        assert len(fake_publisher.messages) == 1

        node._on_dvl_response(_response("reset_dead_reckoning", True))
        assert node.dvl_calibration_status()["state"] == "calibrating"

        node._on_dvl_response(_response("calibrate_gyro", True))
        completed = node.dvl_calibration_status()
        assert completed["state"] == "completed"
        assert completed["success"] is True
        assert "success ACK" in completed["message"]
    finally:
        _destroy_node(node, owned_context)


def test_calibration_reports_dvl_rejection_and_timeout() -> None:
    node, owned_context = _new_node()
    fake_publisher = _FakeDvlPublisher()
    node._dvl_config_pub = fake_publisher
    try:
        node.start_dvl_gyro_calibration(timeout_s=1.0)
        node._on_dvl_response(_response("calibrate_gyro", False, "vehicle moving"))
        failed = node.dvl_calibration_status()
        assert failed["state"] == "failed"
        assert failed["success"] is False
        assert failed["error_message"] == "vehicle moving"

        node.start_dvl_gyro_calibration(timeout_s=0.01)
        time.sleep(0.02)
        timed_out = node.dvl_calibration_status()
        assert timed_out["state"] == "timeout"
        assert timed_out["success"] is False
        assert "ACK" in timed_out["message"]
        node._on_dvl_response(_response("calibrate_gyro", True))
        assert node.dvl_calibration_status()["state"] == "timeout"
    finally:
        _destroy_node(node, owned_context)


def test_calibration_does_not_publish_without_dvl_subscriber() -> None:
    node, owned_context = _new_node()
    fake_publisher = _FakeDvlPublisher(subscriber_count=0)
    node._dvl_config_pub = fake_publisher
    try:
        with pytest.raises(RuntimeError, match="subscriber is not available"):
            node.start_dvl_gyro_calibration()
        assert fake_publisher.messages == []
        assert node.dvl_calibration_status()["state"] == "failed"
    finally:
        _destroy_node(node, owned_context)


def test_dvl_control_topics_use_reliable_qos() -> None:
    node, owned_context = _new_node()
    try:
        command_publisher = next(
            publisher
            for publisher in node.publishers
            if publisher.topic_name == "/dvl/config/command"
        )
        response_subscription = next(
            subscription
            for subscription in node.subscriptions
            if subscription.topic_name == "/dvl/command/response"
        )
        assert command_publisher.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        assert response_subscription.qos_profile.reliability == ReliabilityPolicy.RELIABLE
    finally:
        _destroy_node(node, owned_context)
