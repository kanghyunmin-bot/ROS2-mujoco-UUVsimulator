"""Exercise heartbeat recording through the actual transport binding surface."""

from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from bridge.sitl_transport import SitlTransport
from bridge.sitl_vehicle_state_extract import record_vehicle_state_from_heartbeat


class HeartbeatRecordingTest(unittest.TestCase):
    def test_transport_updates_state_and_change_tracking(self):
        owner = SitlTransport.__new__(SitlTransport)
        owner._sitl_mavutil = SimpleNamespace(
            mavlink=SimpleNamespace(MAV_MODE_FLAG_SAFETY_ARMED=128),
            mode_string_v10=lambda msg: "ALT_HOLD",
        )
        owner._sitl_vehicle_armed = False
        owner._sitl_vehicle_mode = ""
        owner._sitl_last_vehicle_armed = None
        owner._sitl_last_vehicle_mode = None
        record_vehicle_state_from_heartbeat(
            owner, SimpleNamespace(base_mode=128), command_link=False
        )
        self.assertTrue(owner._sitl_vehicle_armed)
        self.assertEqual(owner._sitl_vehicle_mode, "ALT_HOLD")
        self.assertTrue(owner._sitl_last_vehicle_armed)
        self.assertEqual(owner._sitl_last_vehicle_mode, "ALT_HOLD")


if __name__ == "__main__":
    unittest.main()
