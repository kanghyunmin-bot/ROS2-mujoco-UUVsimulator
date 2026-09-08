#!/usr/bin/env python3
"""Ensure strict compatibility does not construct forbidden ROS endpoints."""

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from bridge.ros2_endpoint_core_publishers import create_core_sensor_publishers
from bridge.ros2_endpoint_mavros_publishers import create_mavros_publishers
from bridge.ros2_endpoint_misc_publishers import create_dvl_compat_publishers, create_tf_publishers
from bridge.ros2_endpoint_subscriptions import create_core_subscriptions, create_mavros_subscriptions
from bridge.ros2_hydrophone_sim import create_hydrophone_publishers
from bridge.ros2_publish_schedule_dvl import schedule_real_dvl_compat_jobs


class FakeNode:
    def __init__(self) -> None:
        self.publishers = []
        self.subscriptions = []

    def create_publisher(self, _msg_type, topic, _qos):
        self.publishers.append(topic)
        return topic

    def create_subscription(self, _msg_type, topic, _callback, _qos):
        self.subscriptions.append(topic)
        return topic


def main() -> int:
    node = FakeNode()
    bridge = SimpleNamespace(
        node=node,
        _real_pkg_compat=True,
        _dvl_device_emulator_enabled=True,
        _dvl_sensor_model_enabled=True,
        _mavros_surface_enabled=False,
        _strict_sitl_sensor_transport=True,
        DVLMsg=object,
        DVLDRMsg=object,
        CollectorState=object,
        RCOut=None,
        _allow_rcout_plant_override=False,
        _mavros_setpoint_enabled=False,
        _on_cmd_vel_stamped=lambda _msg: None,
        _on_ping360_config=lambda _msg: None,
        _on_sitl_command_override=lambda _msg: None,
    )
    for attr in (
        "Imu", "Float32", "PoseWithCovarianceStamped", "TwistStamped",
        "TwistWithCovarianceStamped", "Range", "Odometry", "BatteryState", "PoseStamped",
        "String", "Clock", "MavrosState", "FluidPressure", "VfrHud", "RCIn", "TFMessage",
        "OverrideRCIn", "ManualControl", "PositionTarget",
    ):
        setattr(bridge, attr, object)

    create_core_sensor_publishers(bridge, q10=10)
    sensor_qos = object()
    create_mavros_publishers(
        bridge,
        q10=10,
        latched_qos=1,
        sensor_qos=sensor_qos,
    )
    create_dvl_compat_publishers(bridge, dvl_sensor_qos=object())
    create_tf_publishers(bridge, tf_qos=10, latched_qos=1)
    create_core_subscriptions(bridge, q10=10)
    create_mavros_subscriptions(bridge, q10=10)
    bridge.Vector3Stamped = object
    bridge.AudioData = object
    bridge.AudioInfo = object
    bridge._hydrophone_config = SimpleNamespace(enabled=True, audio_enabled=True)
    create_hydrophone_publishers(bridge, q10=10)

    forbidden_publishers = {
        "/mavros/vfr_hud", "/dvl/twist", "/depth/pose", "/odometry/filtered", "/tf"
    }
    found = forbidden_publishers.intersection(node.publishers)
    assert not found, f"strict mode constructed forbidden publishers: {sorted(found)}"
    strict_sensor_topics = {
        "/mavros/imu/data_raw",
        "/mavros/imu/static_pressure",
    }
    assert {
        topic for topic in node.publishers if topic.startswith("/mavros/")
    } == strict_sensor_topics, node.publishers
    assert "/dvl/data" not in node.publishers
    assert "/dvl/position" not in node.publishers
    assert "/battery" in node.publishers
    assert "/clock" in node.publishers
    assert "/uuv_mujoco/clock" in node.publishers
    assert "/collector/state" in node.publishers
    assert "/mujoco/hydrophone/direction" in node.publishers
    assert "/audio" in node.publishers
    assert "/homing/direction" not in node.publishers
    assert "/uuv_mujoco/sitl/command_override" not in node.subscriptions
    assert "/cmd_vel" not in node.subscriptions

    sim_mavros_launch = (
        ROOT.parents[1]
        / "rospkg"
        / "src"
        / "kmu26_auv"
        / "launch"
        / "mavros_apm_sim.launch.py"
    ).read_text(encoding="utf-8")
    for suffix in ("data_raw", "static_pressure"):
        assert f'"/mavros/imu/{suffix}"' in sim_mavros_launch
        assert (
            f'"/uuv_mujoco/mavros_fcu_passthrough/imu/{suffix}"'
            in sim_mavros_launch
        )
    assert '"/uuv_mujoco/mavros_fcu_passthrough/imu/data"' not in sim_mavros_launch

    scheduled = []
    bridge._ros_rate_dvl_twist_hz = 20.0
    bridge._ros_rate_dvl_position_hz = 20.0
    schedule_real_dvl_compat_jobs(
        bridge,
        SimpleNamespace(add=None),
        lambda publisher, topic, builder, rate_hz: scheduled.append(
            (publisher, topic, builder, rate_hz)
        ),
        builders={
            "dvl_data_batch": lambda: (),
            "dvl_position_batch": lambda: (),
        },
    )
    assert not scheduled, "device mode must leave canonical DVL ROS topics to auv_dvl_a50"
    print("strict_real_pkg_surface=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
