"""Opt-in final-PWM telemetry must survive rate configuration unchanged."""

import importlib.util
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from bridge.sitl_transport_mavlink_telemetry_config import (
    initialize_mavlink_telemetry_state,
)


def test_sitl_observer_accepts_high_rate_without_silent_20hz_cap(monkeypatch):
    monkeypatch.setenv("ROS2_UUV_RCOU_TELEMETRY_HZ", "100")
    transport = SimpleNamespace(
        _sitl_mavlink_target_sysid=1,
        _sitl_mavlink_target_compid=1,
        _sitl_mavlink_endpoint="none",
    )
    initialize_mavlink_telemetry_state(transport)
    assert transport._sitl_rcout_telemetry_hz == 100
    monkeypatch.delenv("ROS2_UUV_RCOU_TELEMETRY_HZ")
    initialize_mavlink_telemetry_state(transport)
    assert transport._sitl_rcout_telemetry_hz == 2


@pytest.fixture
def rate_node(monkeypatch):
    # Load the real request-building method without connecting to ROS or an FCU.
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "rclpy.node", SimpleNamespace(Node=object))
    monkeypatch.setitem(sys.modules, "mavros_msgs", SimpleNamespace())
    monkeypatch.setitem(sys.modules, "mavros_msgs.msg", SimpleNamespace(State=object))
    monkeypatch.setitem(
        sys.modules, "mavros_msgs.srv", SimpleNamespace(MessageInterval=object)
    )
    path = ROOT.parents[1] / "rospkg/src/kmu26_auv/scripts/mavros_imu_rate_config.py"
    spec = importlib.util.spec_from_file_location("propulsion_rate_fixture", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.MavrosImuRateConfig


def requests(rate_node, *, sim=False, rate=-1):
    parameters = {"use_sim_time": sim, "rcout_rate_hz": rate}
    owner = SimpleNamespace(
        MESSAGE_IDS=rate_node.MESSAGE_IDS,
        get_parameter=lambda name: SimpleNamespace(value=parameters[name]),
        declare_parameter=lambda name, default: SimpleNamespace(
            value=parameters.get(name, default)
        ),
    )
    return rate_node._build_requests(owner)


@pytest.mark.parametrize("sim", [True, False])
def test_real_and_sim_explicitly_request_final_pwm(rate_node, sim):
    assert ("SERVO_OUTPUT_RAW", 36, 100.0) in requests(rate_node, sim=sim, rate=100)
    assert all(item[1] != 36 for item in requests(rate_node, sim=sim))


@pytest.mark.parametrize("rate", [0, 101, float("nan"), float("inf")])
def test_invalid_pwm_rates_are_rejected(rate_node, rate):
    with pytest.raises(ValueError):
        requests(rate_node, rate=rate)
