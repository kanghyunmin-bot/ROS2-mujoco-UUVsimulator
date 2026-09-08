#!/usr/bin/env python3
"""Focused offline tests for timed IMU and Bar30 bridge integration."""

from __future__ import annotations

from contextlib import contextmanager
from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.ros2_imu_bar30_sensor_runtime import (  # noqa: E402
    DEFAULT_IMU_BAR30_SENSOR_CONFIG_PATH,
    advance_imu_bar30_sensor_runtime,
    configure_imu_bar30_sensor_runtime,
    reset_imu_bar30_sensor_runtime,
)
from bridge.ros2_sitl_sensor_types import (  # noqa: E402
    Bar30VerticalState,
    BaseKinematicState,
    ImuDvlState,
)
from bridge.sitl_types import VerticalEstimate  # noqa: E402


class PressureLaw:
    @staticmethod
    def frontend_depth_m_from_pressure(pressure_pa: float) -> float:
        return (float(pressure_pa) - 100_000.0) / 10_000.0

    @staticmethod
    def sitl_depth_m_for_frontend_match(pressure_pa: float) -> float:
        return (float(pressure_pa) - 100_000.0) / 10_000.0


class FakeBridge:
    def __init__(self) -> None:
        self._bmj_to_flu = np.eye(3, dtype=np.float64)
        self._baro_pressure_law = PressureLaw()
        self._sitl_home_alt_m = 0.0
        self._bar30_surface_pressure_pa = 100_000.0
        self._bar30_water_density = 1_000.0
        self._bar30_gravity = 10.0
        self._sitl_baro_depth_contract = "frontend_match"

    @staticmethod
    def _ros_imu_accel_surface(value):
        return np.asarray(value, dtype=np.float64)


class TruthTrapBase:
    sim_t = 0.0
    base_rot_enu = np.eye(3)

    @property
    def base_pos_enu(self):
        raise AssertionError("sensor model must not read ground-truth position")


def base_state(sim_t: float) -> BaseKinematicState:
    return BaseKinematicState(
        sim_t=sim_t,
        base_pos_enu=np.zeros(3),
        base_rot_enu=np.eye(3),
        quat_base=np.array((1.0, 0.0, 0.0, 0.0)),
        base_vel_enu=np.zeros(3),
        zero_vertical_reason=None,
    )


def imu_state() -> ImuDvlState:
    return ImuDvlState(
        gyro_bmj=np.array((0.1, -0.2, 0.3)),
        acc_bmj=np.array((0.0, 0.0, 9.8)),
        dvl_vel_body_bmj=None,
        dvl_altitude_m=None,
    )


def vertical_state(pressure_pa: float = 120_000.0) -> Bar30VerticalState:
    return Bar30VerticalState(
        bar30_pos_enu=np.zeros(3),
        bar30_vel_enu=np.zeros(3),
        vertical_estimate=VerticalEstimate(
            depth_m=2.0,
            pressure_pa=pressure_pa,
            pos_ned=np.zeros(3),
            vel_ned=np.zeros(3),
            alt_m=-2.0,
        ),
        bar30_pressure_pa=pressure_pa,
        ros_depth_m=2.0,
    )


def make_noise_free(data: dict) -> None:
    imu = data["imu"]["model"]
    imu.update(
        {
            "gyro_scale_cross_axis_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            "accel_scale_cross_axis_matrix": [1, 0, 0, 0, 1, 0, 0, 0, 1],
            "gyro_constant_bias_rad_s": [0, 0, 0],
            "gyro_turn_on_bias_std_rad_s": [0, 0, 0],
            "gyro_bias_instability_std_rad_s": [0, 0, 0],
            "gyro_random_walk_rad_s_per_sqrt_s": [0, 0, 0],
            "gyro_white_noise_density_rad_s_sqrt_hz": [0, 0, 0],
            "gyro_quantization_rad_s": [0, 0, 0],
            "accel_constant_bias_mps2": [0, 0, 0],
            "accel_turn_on_bias_std_mps2": [0, 0, 0],
            "accel_bias_instability_std_mps2": [0, 0, 0],
            "accel_random_walk_mps2_per_sqrt_s": [0, 0, 0],
            "accel_white_noise_density_mps2_sqrt_hz": [0, 0, 0],
            "accel_quantization_mps2": [0, 0, 0],
            "attitude_turn_on_bias_std_rad": [0, 0, 0],
            "attitude_white_noise_std_rad": [0, 0, 0],
        }
    )
    bar = data["bar30"]["model"]
    bar.update(
        {
            "reference_pressure_pa": 100_000.0,
            "constant_offset_pa": 0.0,
            "turn_on_offset_std_pa": 0.0,
            "bias_instability_std_pa": 0.0,
            "random_walk_pa_per_sqrt_s": 0.0,
            "white_noise_std_pa": 0.0,
            "temperature_coefficient_pa_per_c": 0.0,
            "quantization_pa": 0.0,
        }
    )
    for sensor in (data["imu"], data["bar30"]):
        for key in ("processing_latency", "transport_latency"):
            sensor["timing"][key].update(
                {"mean_s": 0.0, "jitter_std_s": 0.0, "min_s": 0.0, "max_s": 0.0}
            )
        sensor["packet_transport"]["dropout_probability"] = 0.0


@contextmanager
def profile(mutator=None, environment: dict[str, str] | None = None):
    data = json.loads(DEFAULT_IMU_BAR30_SENSOR_CONFIG_PATH.read_text(encoding="utf-8"))
    make_noise_free(data)
    if mutator is not None:
        mutator(data)
    with tempfile.TemporaryDirectory(prefix="imu-bar30-runtime-") as temp_dir:
        path = Path(temp_dir) / "profile.json"
        path.write_text(json.dumps(data), encoding="utf-8")
        clean = {
            key: value
            for key, value in os.environ.items()
            if not key.startswith("ROS2_UUV_IMU_SENSOR_")
            and not key.startswith("ROS2_UUV_BAR30_SENSOR_")
            and key != "ROS2_UUV_IMU_BAR30_SENSOR_CONFIG_PATH"
        }
        clean["ROS2_UUV_IMU_BAR30_SENSOR_CONFIG_PATH"] = str(path)
        clean.update(environment or {})
        with patch.dict(os.environ, clean, clear=True):
            yield


class TimedSensorRuntimeTest(unittest.TestCase):
    def test_environment_overrides_are_independent(self) -> None:
        environment = {
            "ROS2_UUV_IMU_SENSOR_RATE_HZ": "40",
            "ROS2_UUV_IMU_SENSOR_CLOCK_OFFSET_S": "0.1",
            "ROS2_UUV_IMU_SENSOR_PACKET_DROPOUT_PROBABILITY": "0.25",
            "ROS2_UUV_BAR30_SENSOR_RATE_HZ": "5",
            "ROS2_UUV_BAR30_SENSOR_CLOCK_DRIFT_PPM": "12",
            "ROS2_UUV_BAR30_SENSOR_PACKET_DROPOUT_PROBABILITY": "0.5",
            "ROS2_UUV_BAR30_SENSOR_AMBIENT_TEMPERATURE_C": "17.5",
        }
        with profile(environment=environment):
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
        self.assertEqual(bridge._imu_sensor_timing.config.schedule.rate_hz, 40.0)
        self.assertEqual(bridge._imu_sensor_model.config.nominal_rate_hz, 40.0)
        self.assertEqual(bridge._imu_sensor_timing.config.clock.offset_s, 0.1)
        self.assertEqual(bridge._imu_sensor_timing.config.transport.dropout_probability, 0.25)
        self.assertEqual(bridge._bar30_sensor_timing.config.schedule.rate_hz, 5.0)
        self.assertEqual(bridge._bar30_sensor_model.config.nominal_rate_hz, 5.0)
        self.assertEqual(bridge._bar30_sensor_timing.config.clock.drift_ppm, 12.0)
        self.assertEqual(bridge._bar30_sensor_timing.config.transport.dropout_probability, 0.5)
        self.assertEqual(bridge._bar30_sensor_ambient_temperature_c, 17.5)

    def test_each_model_can_be_disabled_without_affecting_the_other(self) -> None:
        environment = {
            "ROS2_UUV_IMU_SENSOR_ENABLE": "false",
            "ROS2_UUV_BAR30_SENSOR_ENABLE": "true",
        }
        with profile(environment=environment):
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            measured_imu, _, imu_deliveries, bar_deliveries = (
                advance_imu_bar30_sensor_runtime(
                    bridge, base_state(0.0), imu_state(), vertical_state()
                )
            )
        np.testing.assert_allclose(measured_imu.gyro_bmj, imu_state().gyro_bmj)
        self.assertEqual(imu_deliveries, ())
        self.assertEqual(len(bar_deliveries), 1)

    def test_independent_capture_rates(self) -> None:
        with profile():
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            counts = [0, 0]
            for sim_t in (0.0, 0.02, 0.04, 0.06, 0.08, 0.10):
                _, _, imu_deliveries, bar_deliveries = advance_imu_bar30_sensor_runtime(
                    bridge, base_state(sim_t), imu_state(), vertical_state()
                )
                counts[0] += len(imu_deliveries)
                counts[1] += len(bar_deliveries)
        self.assertEqual(counts, [6, 2])

    def test_capture_device_transmission_and_arrival_times_are_distinct(self) -> None:
        def mutate(data) -> None:
            imu = data["imu"]
            imu["timing"]["device_clock"]["offset_s"] = 0.25
            imu["timing"]["processing_latency"].update(
                {"mean_s": 0.01, "max_s": 0.01}
            )
            imu["timing"]["transport_latency"].update(
                {"mean_s": 0.02, "max_s": 0.02}
            )

        with profile(mutate):
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            result = advance_imu_bar30_sensor_runtime(
                bridge, base_state(0.0), imu_state(), vertical_state()
            )
            self.assertEqual(result[2], ())
            result = advance_imu_bar30_sensor_runtime(
                bridge, base_state(0.03), imu_state(), vertical_state()
            )
        delivery = result[2][0]
        self.assertEqual(delivery.capture_time_s, 0.0)
        self.assertEqual(delivery.device_time_s, 0.25)
        self.assertEqual(delivery.transmission_time_s, 0.01)
        self.assertEqual(delivery.arrival_time_s, 0.03)

    def test_dropout_is_independent_between_sensors_and_fcu_capture_survives(self) -> None:
        def mutate(data) -> None:
            data["imu"]["model"]["gyro_constant_bias_rad_s"] = [0.5, 0.0, 0.0]

        environment = {
            "ROS2_UUV_IMU_SENSOR_PACKET_DROPOUT_PROBABILITY": "1",
            "ROS2_UUV_BAR30_SENSOR_PACKET_DROPOUT_PROBABILITY": "0",
        }
        with profile(mutate, environment):
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            measured_imu, _, imu_deliveries, bar_deliveries = (
                advance_imu_bar30_sensor_runtime(
                    bridge, base_state(0.0), imu_state(), vertical_state()
                )
            )
        self.assertEqual(imu_deliveries, ())
        self.assertEqual(len(bar_deliveries), 1)
        self.assertAlmostEqual(measured_imu.gyro_bmj[0], 0.6)
        self.assertEqual(bridge._imu_sensor_timing.stats.probabilistic_drops, 1)
        self.assertEqual(bridge._bar30_sensor_timing.stats.probabilistic_drops, 0)

    def test_measured_pressure_drives_sitl_depth_without_truth_position_read(self) -> None:
        def mutate(data) -> None:
            data["bar30"]["model"]["constant_offset_pa"] = 1_000.0

        with profile(mutate):
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            _, measured_vertical, _, _ = advance_imu_bar30_sensor_runtime(
                bridge, base_state(0.0), imu_state(), vertical_state(120_000.0)
            )
        self.assertEqual(measured_vertical.bar30_pressure_pa, 121_000.0)
        self.assertEqual(measured_vertical.ros_depth_m, 2.1)
        self.assertEqual(measured_vertical.vertical_estimate.depth_m, 2.1)
        self.assertEqual(measured_vertical.vertical_estimate.alt_m, -2.1)

    def test_measurement_generation_does_not_read_truth_position(self) -> None:
        with profile():
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            _, _, imu_deliveries, bar_deliveries = advance_imu_bar30_sensor_runtime(
                bridge,
                TruthTrapBase(),
                imu_state(),
                vertical_state(),
            )
        self.assertEqual(len(imu_deliveries), 1)
        self.assertEqual(len(bar_deliveries), 1)

    def test_reset_replays_sensor_and_transport_streams(self) -> None:
        with profile():
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)

            def run() -> list[object]:
                results = []
                for sim_t in (0.0, 0.02, 0.10, 0.12):
                    measured_imu, measured_vertical, imu_packets, bar_packets = (
                        advance_imu_bar30_sensor_runtime(
                            bridge, base_state(sim_t), imu_state(), vertical_state()
                        )
                    )
                    results.append(
                        (
                            tuple(float(value) for value in measured_imu.gyro_bmj),
                            float(measured_vertical.bar30_pressure_pa),
                            imu_packets,
                            bar_packets,
                        )
                    )
                return results

            first = run()
            reset_imu_bar30_sensor_runtime(bridge)
            second = run()
        self.assertEqual(first, second)

    def test_backward_time_resets_and_reanchors(self) -> None:
        with profile():
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            advance_imu_bar30_sensor_runtime(
                bridge, base_state(5.0), imu_state(), vertical_state()
            )
            _, _, deliveries, _ = advance_imu_bar30_sensor_runtime(
                bridge, base_state(1.0), imu_state(), vertical_state()
            )
        self.assertEqual(deliveries[0].capture_time_s, 1.0)
        self.assertEqual(deliveries[0].sample.sample_index, 0)

    def test_large_time_jump_does_not_attach_current_truth_to_old_stamps(self) -> None:
        with profile():
            bridge = FakeBridge()
            configure_imu_bar30_sensor_runtime(bridge)
            advance_imu_bar30_sensor_runtime(
                bridge, base_state(0.0), imu_state(), vertical_state()
            )
            changed = imu_state()
            changed.gyro_bmj[0] = 9.0
            _, _, imu_deliveries, bar_deliveries = advance_imu_bar30_sensor_runtime(
                bridge, base_state(1.0), changed, vertical_state()
            )
        self.assertEqual(len(imu_deliveries), 1)
        self.assertEqual(imu_deliveries[0].capture_time_s, 1.0)
        self.assertEqual(imu_deliveries[0].sample.angular_velocity_rad_s[0], 9.0)
        self.assertEqual(len(bar_deliveries), 1)
        self.assertEqual(bar_deliveries[0].capture_time_s, 1.0)
        self.assertGreater(bridge._imu_sensor_missed_capture_count, 0)
        self.assertGreater(bridge._bar30_sensor_missed_capture_count, 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
