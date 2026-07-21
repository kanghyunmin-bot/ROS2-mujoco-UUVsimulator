"""Case runner for SITL command override smoke checks."""

from __future__ import annotations

from types import SimpleNamespace

from bridge.ros2_sitl_command_override import _handle_direct_cmd, _on_sitl_command_override
from bridge.ros2_sitl_command_override_replay import _replay_rcout_channels_from_payload
from bridge.ros2_sitl_command_override_sensor import _payload_marks_sensor_replay

from ros2_sitl_command_override_smoke_fixture import FakeBridge


EXPECTED_TOPIC = "/uuv_mujoco/sitl/command_override"


def run_ros2_sitl_command_override_smoke() -> None:
    _check_payload_helpers()
    bridge = FakeBridge()
    _on_sitl_command_override(bridge, SimpleNamespace(data="ALT_HOLD"))
    _check_bridge_calls(bridge)
    bridge._sitl_transport.vehicle_armed = True
    _handle_direct_cmd(bridge, {"direct_cmd": {"forward": 0.7, "sway": -0.2, "heave": 0.1, "yaw": -0.3}})
    if bridge.direct_calls != [(0.7, -0.2, -0.3, 0.1)]:
        raise AssertionError(f"armed debug direct command mismatch: {bridge.direct_calls}")


def _check_payload_helpers() -> None:
    if not _payload_marks_sensor_replay({"sensor_replay_rc_seen": True}):
        raise AssertionError("sensor replay marker not detected")
    if _payload_marks_sensor_replay({"mode": "MANUAL"}):
        raise AssertionError("mode-only payload should not mark sensor replay")
    if _replay_rcout_channels_from_payload({"rcout": ["1", "2", "bad"]}) != []:
        raise AssertionError("invalid replay RCOU payload should become empty channel list")
    if _replay_rcout_channels_from_payload({"replay_rcout": range(10)}) != list(range(8)):
        raise AssertionError("replay RCOU should truncate to 8 channels")


def _check_bridge_calls(bridge: FakeBridge) -> None:
    if bridge._sitl_transport.marked != [EXPECTED_TOPIC]:
        raise AssertionError(f"sensor replay mark mismatch: {bridge._sitl_transport.marked}")
    if bridge.replay_calls != [(list(range(1500, 1508)), "command_override_replay_rcout")]:
        raise AssertionError(f"replay call mismatch: {bridge.replay_calls}")
    if bridge.direct_calls:
        raise AssertionError(f"disarmed direct command must be blocked: {bridge.direct_calls}")
    if bridge.arm_calls != [(True, EXPECTED_TOPIC)]:
        raise AssertionError(f"arm call mismatch: {bridge.arm_calls}")
    if bridge.mode_calls != [("ALT_HOLD", EXPECTED_TOPIC)]:
        raise AssertionError(f"mode call mismatch: {bridge.mode_calls}")


__all__ = ["run_ros2_sitl_command_override_smoke"]
