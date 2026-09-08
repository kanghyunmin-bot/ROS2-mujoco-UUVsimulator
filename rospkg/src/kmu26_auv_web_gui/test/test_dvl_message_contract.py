"""Contract tests for the physical Water Linked A50 GUI boundary."""

import rclpy
from auv_dvl_a50_msg.msg import DVL
from auv_dvl_a50_msg.msg import DVLBeam
import pytest

from kmu26_auv_web_gui.ros_interface import LocalizationRosNode


def test_dvl_data_uses_physical_a50_type_and_preserves_quality_state() -> None:
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init(args=[])
    node = LocalizationRosNode()
    try:
        subscriptions = [
            item for item in node.subscriptions if item.topic_name == "/dvl/data"
        ]
        assert len(subscriptions) == 1
        assert subscriptions[0].msg_type is DVL

        message = DVL()
        message.velocity.x = 0.3
        message.velocity.y = 0.4
        message.velocity.z = 0.0
        message.velocity_valid = True
        message.fom = 0.01
        message.altitude = 1.25
        for beam_id in range(1, 5):
            beam = DVLBeam()
            beam.id = beam_id
            beam.valid = True
            message.beams.append(beam)

        node._on_dvl_data(message)
        quality = node.snapshot()["dvl_quality"]
        assert quality["good"]
        assert quality["reason"] == "DVL good"
        assert quality["velocity_valid"]
        assert quality["fom"] == pytest.approx(0.01)
        assert quality["altitude"] == pytest.approx(1.25)
        assert quality["valid_beams"] == 4
        assert quality["speed"] == pytest.approx(0.5)
    finally:
        node.destroy_node()
        if owned_context and rclpy.ok():
            rclpy.shutdown()
