"""Check the shared sensor-rate and launch entry points without contacting a vehicle."""

import importlib.util
from pathlib import Path
import unittest

import rclpy
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sensor_rates", ROOT / "scripts" / "mavros_imu_rate_config.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BasicRovInterfaceTest(unittest.TestCase):
    def test_rate_configuration_launches_for_simulation_without_changing_real_path(self):
        launch_spec = importlib.util.spec_from_file_location(
            "rov_start_rates", ROOT / "launch" / "rov_start.launch.py"
        )
        launch_module = importlib.util.module_from_spec(launch_spec)
        launch_spec.loader.exec_module(launch_module)
        description = launch_module.generate_launch_description()
        context = LaunchContext()
        for action in description.entities:
            if isinstance(action, DeclareLaunchArgument):
                action.execute(context)
        rate_node = next(action for action in description.entities
                         if isinstance(action, Node) and action.node_executable == "mavros_imu_rate_config.py")
        self.assertTrue(rate_node.condition.evaluate(context))
        context.launch_configurations["use_sim_time"] = "true"
        self.assertTrue(rate_node.condition.evaluate(context))
        context.launch_configurations["configure_mavros_imu_rate"] = "false"
        self.assertFalse(rate_node.condition.evaluate(context))

    def test_simulation_requests_only_bounded_ahrs_attitude(self):
        rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
        node = module.MavrosImuRateConfig()
        try:
            self.assertEqual(node.requests, [("ATTITUDE", 30, 20.0)])
        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_default_sensor_requests_include_position_and_pressure(self):
        rclpy.init()
        node = module.MavrosImuRateConfig()
        try:
            requests = {name: (message_id, rate) for name, message_id, rate in node.requests}
            self.assertEqual(requests["SCALED_PRESSURE2"], (137, 10.0))
            self.assertEqual(requests["LOCAL_POSITION_NED"], (32, 20.0))
            self.assertEqual(requests["ATTITUDE"], (30, 50.0))
            self.assertEqual(requests["RAW_IMU"], (27, 50.0))
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
