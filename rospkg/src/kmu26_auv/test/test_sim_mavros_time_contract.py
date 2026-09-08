#!/usr/bin/env python3
"""Static gate for the MAVROS/MuJoCo clock launch contract."""

from __future__ import annotations

import ast
from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def _parse_python(relative_path: str) -> tuple[ast.Module, str]:
    source = (PACKAGE_ROOT / relative_path).read_text(encoding="utf-8")
    return ast.parse(source), source


def _declared_launch_arguments(tree: ast.Module) -> set[str]:
    names: set[str] = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        if not isinstance(node.func, ast.Name) or node.func.id != "DeclareLaunchArgument":
            continue
        if isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str):
            names.add(node.args[0].value)
    return names


def main() -> int:
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "mavros_sim_time.yaml").read_text(
            encoding="utf-8"))
    assert config["/**"]["ros__parameters"]["use_sim_time"] is True
    time_parameters = config["/**/time"]["ros__parameters"]
    assert time_parameters["timesync_mode"] == "NONE"
    assert float(time_parameters["timesync_rate"]) == 0.0
    assert float(time_parameters["system_time_rate"]) == 0.0

    plugin_config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "mavros_sim_pluginlists.yaml").read_text(
            encoding="utf-8"))
    allowlist = plugin_config["/**"]["ros__parameters"]["plugin_allowlist"]
    assert set(allowlist) == {
        "command",
        "imu",
        "local_position",
        "manual_control",
        "param",
        "rc_io",
        "setpoint_raw",
        "vision_pose",
        "vision_position_delta",
        "sys_status",
        "sys_time",
        "vfr_hud",
    }

    wrapper_tree, wrapper_source = _parse_python(
        "launch/mavros_apm_sim.launch.py")
    parameter_names = []
    for node in ast.walk(wrapper_tree):
        if not isinstance(node, ast.keyword) or node.arg != "parameters":
            continue
        if not isinstance(node.value, ast.List):
            continue
        parameter_names = [
            element.id for element in node.value.elts if isinstance(element, ast.Name)
        ]
        break
    assert parameter_names[-3:] == [
        "pluginlists_yaml", "apm_config_yaml", "sim_time_yaml"]
    assert '"mavros_sim_pluginlists.yaml"' in wrapper_source

    rov_tree, rov_source = _parse_python("launch/rov_start.launch.py")
    rov_arguments = _declared_launch_arguments(rov_tree)
    assert {"use_sim_time", "mavros_sim_launch_file"} <= rov_arguments
    assert "condition=mavros_real_enabled" in rov_source
    assert "condition=mavros_sim_enabled" in rov_source
    assert 'DeclareLaunchArgument("configure_mavros_imu_rate"' in rov_source
    assert "condition=mavros_imu_rate_config_enabled" in rov_source
    assert 'DeclareLaunchArgument("use_dvl_position_odom"' in rov_source
    assert 'executable="dvl_position_to_odom_bridge"' in rov_source
    assert 'executable="buoy_position_control"' in rov_source

    localization_tree, localization_source = _parse_python(
        "launch/localization_test.launch.py")
    localization_arguments = _declared_launch_arguments(localization_tree)
    forwarded_arguments = (
        "fcu_url",
        "gcs_url",
        "use_dvl",
        "use_joy2mavros",
        "use_battery_bridge",
        "use_odom2mavros",
        "use_web_gui",
        "use_rviz",
        "use_mission_rviz_visualizer",
    )
    assert set(forwarded_arguments) <= localization_arguments
    for argument in forwarded_arguments:
        assert f'"{argument}": {argument}' in localization_source

    print("sim_mavros_time_contract=PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
