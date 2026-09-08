#!/usr/bin/env python3
"""Exercise A50 TCP emulator through the unmodified physical ROS driver."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from auv_dvl_a50_msg.msg import DVL, DVLDR
from geometry_msgs.msg import TwistWithCovarianceStamped
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.dvl_a50_sensor_model import A50SensorConfig, A50SensorModel  # noqa: E402
from bridge.dvl_a50_tcp_emulator import A50TcpJsonEmulator  # noqa: E402


SIM_CLOCK_SEC = 4_242


def start_process(command: list[str]) -> subprocess.Popen[str]:
    return subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )


def stop_process(process: subprocess.Popen[str]) -> str:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            output, _ = process.communicate(timeout=3.0)
            return output
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
    output, _ = process.communicate(timeout=3.0)
    return output


def spin_until(node, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)
        if predicate():
            return True
    return False


def stamp_nanoseconds(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def main() -> int:
    emulator = A50TcpJsonEmulator(host="127.0.0.1", port=16171).start()
    driver = None
    converter = None
    rclpy.init()
    node = rclpy.create_node("dvl_real_driver_e2e_probe")
    data_messages: list[DVL] = []
    position_messages: list[DVLDR] = []
    twist_messages: list[TwistWithCovarianceStamped] = []
    clock_publisher = node.create_publisher(Clock, "/clock", 10)
    node.create_subscription(DVL, "/dvl/data", data_messages.append, qos_profile_sensor_data)
    node.create_subscription(
        DVLDR,
        "/dvl/position",
        position_messages.append,
        qos_profile_sensor_data,
    )
    node.create_subscription(
        TwistWithCovarianceStamped,
        "/dvl/twist",
        twist_messages.append,
        10,
    )

    try:
        driver = start_process(
            [
                "ros2",
                "launch",
                "hit25_auv_ros2",
                "dvl_a50_driver.launch.py",
                "ip_address:=127.0.0.1",
                "velocity_frame_id:=dvl_link",
                "position_frame_id:=dvl_link",
                "configure_acoustic_on_startup:=true",
                "request_config_on_startup:=true",
                "use_sim_time:=true",
            ]
        )
        converter = start_process(
            [
                "ros2",
                "run",
                "hit25_auv_ros2",
                "dvl_to_twist_bridge",
                "--ros-args",
                "-p",
                "output_frame_id:=dvl_link",
                "-p",
                "reacquire_good_samples:=1",
                "-p",
                "use_sim_time:=true",
            ]
        )
        clock_message = Clock()
        clock_message.clock.sec = SIM_CLOCK_SEC
        clock_message.clock.nanosec = 50_000_000
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            clock_publisher.publish(clock_message)
            rclpy.spin_once(node, timeout_sec=0.02)
            if emulator.client_connected and clock_publisher.get_subscription_count() >= 2:
                break
            time.sleep(0.01)
        else:
            raise AssertionError(
                "physical driver did not connect or simulated-clock subscribers "
                f"were missing: connected={emulator.client_connected} "
                f"clock_subscribers={clock_publisher.get_subscription_count()}"
            )
        for _ in range(5):
            clock_publisher.publish(clock_message)
            time.sleep(0.02)

        model = A50SensorModel(
            A50SensorConfig(
                seed=55,
                white_noise_std_mps=0.0,
                velocity_noise_per_meter_mps=0.0,
                range_noise_std_m=0.0,
                range_noise_fraction=0.0,
                min_valid_beams=4,
            )
        )
        for index in range(8):
            clock_message.clock.sec = SIM_CLOCK_SEC + (1 if index >= 4 else 0)
            clock_message.clock.nanosec = 100_000_000 + index * 10_000_000
            clock_publisher.publish(clock_message)
            time.sleep(0.02)
            validity_us = (index + 1) * 100_000
            sample = model.sample(
                (0.2, 0.1, -0.05),
                2.0,
                time_of_validity_us=validity_us,
                time_of_transmission_us=validity_us + 2_000,
            )
            delivery = type(
                "Delivery",
                (),
                {
                    "sample": sample,
                    "capture_time_s": validity_us * 1.0e-6,
                    "position_local_frd_m": (0.02 * index, 0.01 * index, 0.0),
                    "position_std_m": 0.01,
                    "attitude_rpy_deg": (0.0, 0.0, 1.0 * index),
                },
            )()
            emulator.publish_velocity(sample, report_period_s=0.1)
            if index % 2 == 0:
                emulator.publish_position(delivery)
            rclpy.spin_once(node, timeout_sec=0.06)
            if index == 3 and not spin_until(
                node,
                lambda: len(data_messages) >= 2
                and len(position_messages) >= 1
                and len(twist_messages) >= 1,
                3.0,
            ):
                raise AssertionError(
                    "first simulated-clock epoch was not observed before advancing /clock"
                )

        if not spin_until(
            node,
            lambda: len(data_messages) >= 4
            and len(position_messages) >= 2
            and len(twist_messages) >= 1
            and all(
                any(
                    message.header.stamp.sec == SIM_CLOCK_SEC + 1
                    for message in messages
                )
                for messages in (data_messages, position_messages, twist_messages)
            ),
            5.0,
        ):
            raise AssertionError(
                "driver E2E topics incomplete: "
                f"data={len(data_messages)} position={len(position_messages)} "
                f"twist={len(twist_messages)}"
            )

        data_message = data_messages[-1]
        twist_message = twist_messages[-1]
        position_message = position_messages[-1]
        assert data_message.header.frame_id == "dvl_link"
        assert position_message.header.frame_id == "dvl_link"
        assert twist_message.header.frame_id == "dvl_link"
        assert data_message.time_of_validity > 0
        assert data_message.time_of_transmission >= data_message.time_of_validity
        for messages in (data_messages, position_messages, twist_messages):
            header_times = [stamp_nanoseconds(message) for message in messages]
            assert header_times == sorted(header_times)
            assert any(message.header.stamp.sec == SIM_CLOCK_SEC for message in messages)
            assert any(message.header.stamp.sec == SIM_CLOCK_SEC + 1 for message in messages)
        assert len(data_message.beams) == 4
        assert all(beam.valid for beam in data_message.beams)
        assert abs(twist_message.twist.twist.linear.y - data_message.velocity.y) < 1.0e-9
        assert abs(twist_message.twist.twist.linear.z - data_message.velocity.z) < 1.0e-9
        assert position_message.type == "position_local"
        assert position_message.format == "json_v3"
        assert emulator.dead_reckoning_reset_count == 0
        print(
            "dvl_real_driver_e2e=PASS "
            f"data={len(data_messages)} position={len(position_messages)} "
            f"twist={len(twist_messages)} "
            f"clock_sec={SIM_CLOCK_SEC}->{SIM_CLOCK_SEC + 1}"
        )
        return 0
    finally:
        converter_output = stop_process(converter) if converter is not None else ""
        driver_output = stop_process(driver) if driver is not None else ""
        node.destroy_node()
        rclpy.shutdown()
        emulator.stop()
        if sys.exc_info()[0] is not None:
            if driver_output:
                print("driver output:\n" + driver_output[-4000:], file=sys.stderr)
            if converter_output:
                print("converter output:\n" + converter_output[-4000:], file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
