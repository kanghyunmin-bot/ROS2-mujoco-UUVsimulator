"""Sensor input/output source checks for the active runtime contract audit."""

from __future__ import annotations

from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Check, Evidence, OFFICIAL_REFS


def _sensor_io_paths(runtime_paths: dict[str, Path]) -> dict[str, Path]:
    keys = (
        "ros2_sitl_sensor_feed_py",
        "ros2_sitl_sensor_transport_py",
        "sitl_json_sensor_runtime_py",
        "ros2_publish_state_py",
        "ros2_publish_core_factories_py",
        "ros2_publish_dvl_factories_py",
        "ros2_publish_mavros_cache_factories_py",
        "ros2_publish_mavros_cache_imu_py",
        "ros2_publish_mavros_cache_status_py",
    )
    return {key: runtime_paths[key] for key in keys}


def _snapshot_and_sitl_ok(paths: dict[str, Path]) -> bool:
    return all(
        (
            contains_all(
                paths["ros2_sitl_sensor_feed_py"],
                [
                    "build_imu_dvl_state(self, data, base)",
                    "build_bar30_vertical_state(self, data, base)",
                    "send_sitl_sensor_state(self, base, imu_dvl, vertical)",
                    "Ros2SensorSnapshot(",
                ],
            ),
            contains_all(
                paths["ros2_sitl_sensor_transport_py"],
                [
                    "gyro_frd = self._bmj_to_frd @ imu_dvl.gyro_bmj",
                    "acc_frd = self._bmj_to_frd @ imu_dvl.acc_bmj",
                    "pressure_pa=vertical.bar30_pressure_pa",
                ],
            ),
        )
    )


def _json_payload_ok(paths: dict[str, Path]) -> bool:
    return contains_all(
        paths["sitl_json_sensor_runtime_py"],
        [
            '"timestamp": float(sensor_time_s)',
            '"imu": {',
            '"position": [float(x) for x in json_position]',
            '"velocity": [float(x) for x in json_velocity]',
            '"attitude": [float(roll), float(pitch), float(yaw)]',
            '"quaternion": [float(x) for x in quat]',
        ],
    )


def _ros_publish_ok(paths: dict[str, Path]) -> bool:
    return all(
        (
            contains_all(
                paths["ros2_publish_state_py"],
                [
                    "bar30_pressure_pa = snapshot.bar30_pressure_pa",
                    "ros_depth_m = snapshot.ros_depth_m",
                    'if self._static_pressure_source == "external":',
                    "static_pressure_pa = bar30_pressure_pa",
                ],
            ),
            contains_all(
                paths["ros2_publish_core_factories_py"],
                [
                    '("imu", build_core_imu_msg)',
                    '("depth", build_core_depth_msg)',
                    '("depth_pose", build_core_depth_pose_msg)',
                    '("baro", build_core_baro_msg)',
                ],
            ),
            contains_all(
                paths["ros2_publish_mavros_cache_factories_py"],
                [
                    '("mavros_imu", build_mavros_imu_msg)',
                    '("mavros_imu_raw", build_imu_raw_msg)',
                    '("mavros_static_pressure", build_static_pressure_msg)',
                ],
            ),
            contains_all(
                paths["ros2_publish_mavros_cache_imu_py"],
                ["state.quat_ros", "state.gyro_ros", "state.acc_ros_surface", 'frame_id="fcu_link"'],
            ),
            contains_all(
                paths["ros2_publish_mavros_cache_status_py"],
                ["state.static_pressure_pa", "bridge._mavros_atm_pressure_value"],
            ),
            contains_all(
                paths["ros2_publish_dvl_factories_py"],
                [
                    "state.dvl_sensor_delivery",
                    "velocity_dvl_frd = _dvl_velocity_frd(bridge, state)",
                    "state.dvl_vel_dvl_frd",
                    "state.dvl_altitude_m",
                ],
            ),
        )
    )


def _sensor_io_evidence(paths: dict[str, Path]) -> list[Evidence]:
    return [
        evidence(paths["ros2_sitl_sensor_feed_py"], "Ros2SensorSnapshot("),
        evidence(paths["ros2_sitl_sensor_transport_py"], "pressure_pa=vertical.bar30_pressure_pa"),
        evidence(paths["sitl_json_sensor_runtime_py"], '"imu": {'),
        evidence(paths["ros2_publish_state_py"], "static_pressure_pa = bar30_pressure_pa"),
        evidence(paths["ros2_publish_core_factories_py"], '("depth", build_core_depth_msg)'),
        evidence(
            paths["ros2_publish_mavros_cache_factories_py"],
            '("mavros_static_pressure", build_static_pressure_msg)',
        ),
        evidence(paths["ros2_publish_dvl_factories_py"], "state.dvl_vel_dvl_frd"),
    ]


def build_runtime_sensor_io_contract_check(runtime_paths: dict[str, Path]) -> Check:
    paths = _sensor_io_paths(runtime_paths)
    ok = all((_snapshot_and_sitl_ok(paths), _json_payload_ok(paths), _ros_publish_ok(paths)))
    return Check(
        check_id="active_runtime_sensor_io_snapshot_contract",
        status="PASS" if ok else "FAIL",
        title="Sensor input/output surfaces share one MuJoCo snapshot contract",
        conclusion=(
            "The active runtime builds one MuJoCo sensor snapshot, sends FRD IMU plus Bar30 pressure/depth "
            "to ArduSub JSON SITL, and publishes ROS/MAVROS/DVL observation topics from the same snapshot."
        ),
        evidence=_sensor_io_evidence(paths),
        official_refs=[OFFICIAL_REFS["ardupilot_json_sitl"], OFFICIAL_REFS["bar30_pressure_sensor"]],
    )


__all__ = ["build_runtime_sensor_io_contract_check"]
