"""Check the shared sensor-rate and launch entry points without contacting a vehicle."""

import importlib.util
from pathlib import Path
import unittest

import rclpy

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location(
    "sensor_rates", ROOT / "scripts" / "mavros_imu_rate_config.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BasicRovInterfaceTest(unittest.TestCase):
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
