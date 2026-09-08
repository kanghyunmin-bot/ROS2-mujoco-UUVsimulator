#!/usr/bin/env python3
"""Focused offline tests for the timed A50 bridge runtime."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from contextlib import contextmanager
from dataclasses import dataclass
from pathlib import Path
from unittest.mock import patch

import numpy as np


CURRENT = Path(__file__).resolve().parents[1]
if str(CURRENT) not in sys.path:
    sys.path.insert(0, str(CURRENT))

from bridge.ros2_dvl_sensor_runtime import (  # noqa: E402
    DEFAULT_A50_SENSOR_CONFIG_PATH,
    advance_dvl_sensor_runtime,
    calibrate_dvl_gyro,
    configure_dvl_sensor_runtime,
    reset_dvl_dead_reckoning,
    reset_dvl_sensor_runtime,
)


ENV_PREFIX = "ROS2_UUV_DVL_SENSOR_"


class FakeBridge:
    def __init__(self) -> None:
        self._bmj_to_frd = np.eye(3, dtype=np.float64)
        self._dvl_body_frd_to_dvl_frd = np.eye(3, dtype=np.float64)
        self._odom_pos = np.array((99.0, 99.0, 99.0), dtype=np.float64)


@dataclass
class FakeState:
    sim_t: float
    dvl_vel_dvl_frd: np.ndarray | None
    dvl_altitude_m: float | None
    rot_world_body: np.ndarray
    gyro_bmj: np.ndarray | None = None


class TruthTrapState(FakeState):
    @property
    def base_pos_enu(self) -> np.ndarray:
        raise AssertionError("truth position must not be read by DVL dead reckoning")


def yaw_rotation(degrees: float) -> np.ndarray:
    angle = np.deg2rad(degrees)
    cosine = np.cos(angle)
    sine = np.sin(angle)
    return np.array(
        ((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)),
        dtype=np.float64,
    )


def _zero_latency(block: dict[str, object]) -> None:
    block.update(
        {
            "mean_s": 0.0,
            "jitter_std_s": 0.0,
            "min_s": 0.0,
            "max_s": 0.0,
        }
    )


def _noise_free_prior() -> dict[str, object]:
    data = json.loads(DEFAULT_A50_SENSOR_CONFIG_PATH.read_text(encoding="utf-8"))
    model = data["a50_sensor_model"]
    model.update(
        {
            "velocity_bias_frd_mps": [0.0, 0.0, 0.0],
            "beam_velocity_bias_mps": [0.0, 0.0, 0.0, 0.0],
            "white_noise_std_mps": 0.0,
            "velocity_noise_per_meter_mps": 0.0,
            "range_noise_std_m": 0.0,
            "range_noise_fraction": 0.0,
            "beam_dropout_probabilities": [0.0, 0.0, 0.0, 0.0],
        }
    )
    _zero_latency(data["timing"]["processing_latency"])
    _zero_latency(data["timing"]["transport_latency"])
    data["packet_transport"]["dropout_probability"] = 0.0
    data["dead_reckoning"].update(
        {
            "attitude_bias_std_deg": [0.0, 0.0, 0.0],
            "attitude_random_walk_std_deg_sqrt_s": [0.0, 0.0, 0.0],
            "attitude_white_noise_std_deg": [0.0, 0.0, 0.0],
            "position_std_initial_m": 0.0,
            "position_random_walk_std_m_sqrt_s": 0.0,
        }
    )
    return data


@contextmanager
def sensor_config(
    mutator=None,
    *,
    environment: dict[str, str] | None = None,
):
    data = _noise_free_prior()
    if mutator is not None:
        mutator(data)
    with tempfile.TemporaryDirectory(prefix="dvl-sensor-runtime-") as temp_dir:
        path = Path(temp_dir) / "a50.json"
        path.write_text(json.dumps(data), encoding="utf-8")
        clean_environment = {
            key: value
            for key, value in os.environ.items()
            if not key.startswith(ENV_PREFIX)
        }
        clean_environment[f"{ENV_PREFIX}CONFIG_PATH"] = str(path)
        clean_environment.update(environment or {})
        with patch.dict(os.environ, clean_environment, clear=True):
            yield path


class DvlSensorConfigurationTest(unittest.TestCase):
    def test_default_prior_separates_official_and_estimated_values(self) -> None:
        data = json.loads(DEFAULT_A50_SENSOR_CONFIG_PATH.read_text(encoding="utf-8"))

        self.assertEqual(data["schema"], "uuv_mujoco.sensor_model.a50.v1")
        self.assertEqual(data["calibration_status"], "unvalidated_prior")
        self.assertEqual(data["evidence"]["official_geometry_and_range"]["beam_angle_deg"], 22.5)
        self.assertEqual(data["a50_sensor_model"]["min_range_m"], 0.05)
        self.assertEqual(data["a50_sensor_model"]["max_range_m"], 50.0)
        self.assertEqual(data["evidence"]["bag_observed_rate"]["target_rate_hz"], 10.0)
        self.assertIn("engineering estimate", data["timing"]["processing_latency"]["basis"])
        self.assertEqual(data["timing"]["transport_latency"]["mean_s"], 0.004)
        self.assertEqual(data["timing"]["transport_latency"]["jitter_std_s"], 0.002)
        self.assertEqual(data["timing"]["transport_latency"]["max_s"], 0.013)
        self.assertEqual(data["dead_reckoning"]["report_rate_hz"], 5.0)
        self.assertEqual(data["dead_reckoning"]["packet_dropout_probability"], 0.0)
        self.assertEqual(
            data["dead_reckoning"]["attitude_source"],
            "modeled_body_gyro",
        )
        self.assertEqual(
            data["dead_reckoning"]["attitude_random_walk_std_deg_sqrt_s"],
            [0.005, 0.005, 0.01],
        )

    def test_environment_overrides_runtime_without_changing_json(self) -> None:
        environment = {
            f"{ENV_PREFIX}SEED": "91",
            f"{ENV_PREFIX}RATE_HZ": "20",
            f"{ENV_PREFIX}CLOCK_OFFSET_S": "0.25",
            f"{ENV_PREFIX}CLOCK_DRIFT_PPM": "12.5",
            f"{ENV_PREFIX}PROCESSING_LATENCY_MEAN_S": "0.003",
            f"{ENV_PREFIX}PROCESSING_LATENCY_JITTER_STD_S": "0",
            f"{ENV_PREFIX}PROCESSING_LATENCY_MAX_S": "0.003",
            f"{ENV_PREFIX}TRANSPORT_LATENCY_MEAN_S": "0.005",
            f"{ENV_PREFIX}TRANSPORT_LATENCY_JITTER_STD_S": "0",
            f"{ENV_PREFIX}TRANSPORT_LATENCY_MAX_S": "0.005",
            f"{ENV_PREFIX}PACKET_DROPOUT_PROBABILITY": "0.25",
            f"{ENV_PREFIX}POSITION_RATE_HZ": "4",
            f"{ENV_PREFIX}POSITION_PACKET_DROPOUT_PROBABILITY": "0.125",
        }
        with sensor_config(environment=environment):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)

        config = bridge._dvl_sensor_timing.config
        self.assertEqual(bridge._dvl_sensor_seed, 91)
        self.assertEqual(config.schedule.rate_hz, 20.0)
        self.assertEqual(config.clock.offset_s, 0.25)
        self.assertEqual(config.clock.drift_ppm, 12.5)
        self.assertEqual(config.transport.processing_latency.mean_s, 0.003)
        self.assertEqual(config.transport.transport_latency.mean_s, 0.005)
        self.assertEqual(config.transport.dropout_probability, 0.25)
        position_config = bridge._dvl_sensor_position_timing.config
        self.assertEqual(position_config.schedule.rate_hz, 4.0)
        self.assertEqual(position_config.transport.dropout_probability, 0.125)

    def test_legacy_prior_defaults_to_modeled_gyro_without_random_walk(self) -> None:
        def mutate(data) -> None:
            data["dead_reckoning"].pop("attitude_source")
            data["dead_reckoning"].pop("attitude_random_walk_std_deg_sqrt_s")

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)

        self.assertEqual(bridge._dvl_sensor_attitude_source, "modeled_body_gyro")
        self.assertEqual(
            bridge._dvl_sensor_dead_reckoning_prior.attitude_random_walk_std_deg_sqrt_s,
            (0.0, 0.0, 0.0),
        )

    def test_truth_attitude_source_cannot_be_selected(self) -> None:
        def mutate(data) -> None:
            data["dead_reckoning"]["attitude_source"] = "mujoco_truth_rotation"

        with sensor_config(mutate):
            bridge = FakeBridge()
            with self.assertRaisesRegex(ValueError, "modeled_body_gyro"):
                configure_dvl_sensor_runtime(bridge)

    def test_disabled_model_returns_none(self) -> None:
        environment = {f"{ENV_PREFIX}ENABLE": "false"}
        with sensor_config(environment=environment):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.ones(3), 2.0, np.eye(3))
            self.assertIsNone(advance_dvl_sensor_runtime(bridge, state))
            self.assertFalse(bridge._dvl_sensor_model_enabled)
            np.testing.assert_allclose(bridge._odom_pos, (99.0, 99.0, 99.0), atol=0.0)


class DvlSensorTimingTest(unittest.TestCase):
    def test_latency_defers_arrival_and_replaces_device_transmission_time(self) -> None:
        def mutate(data) -> None:
            processing = data["timing"]["processing_latency"]
            processing.update(
                {"mean_s": 0.010, "jitter_std_s": 0.0, "min_s": 0.0, "max_s": 0.010}
            )
            transport = data["timing"]["transport_latency"]
            transport.update(
                {"mean_s": 0.020, "jitter_std_s": 0.0, "min_s": 0.0, "max_s": 0.020}
            )
            data["timing"]["device_clock"]["offset_s"] = 0.25

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((1.0, 0.0, 0.0)), 2.0, np.eye(3))

            self.assertIsNone(advance_dvl_sensor_runtime(bridge, state))
            state.sim_t = 0.029
            self.assertIsNone(advance_dvl_sensor_runtime(bridge, state))
            state.sim_t = 0.030
            delivery = advance_dvl_sensor_runtime(bridge, state)

        self.assertIsNotNone(delivery)
        self.assertEqual(delivery.capture_time_s, 0.0)
        self.assertAlmostEqual(delivery.arrival_time_s, 0.030, places=12)
        self.assertEqual(delivery.sample.time_of_validity_us, 250_000)
        self.assertEqual(delivery.sample.time_of_transmission_us, 260_000)
        self.assertLessEqual(
            delivery.sample.time_of_validity_us,
            delivery.sample.time_of_transmission_us,
        )

    def test_seeded_sensor_transport_and_attitude_replay_after_reset(self) -> None:
        def mutate(data) -> None:
            data["a50_sensor_model"]["white_noise_std_mps"] = 0.004
            data["a50_sensor_model"]["velocity_noise_per_meter_mps"] = 0.0003
            data["timing"]["processing_latency"].update(
                {"mean_s": 0.006, "jitter_std_s": 0.002, "min_s": 0.0, "max_s": 0.012}
            )
            data["timing"]["transport_latency"].update(
                {"mean_s": 0.008, "jitter_std_s": 0.003, "min_s": 0.0, "max_s": 0.016}
            )
            data["dead_reckoning"]["attitude_bias_std_deg"] = [0.2, 0.2, 0.4]
            data["dead_reckoning"]["attitude_random_walk_std_deg_sqrt_s"] = [
                0.05,
                0.05,
                0.1,
            ]
            data["dead_reckoning"]["attitude_white_noise_std_deg"] = [0.1, 0.1, 0.2]

        def run(bridge: FakeBridge) -> tuple[object, ...]:
            state = FakeState(
                0.0,
                np.array((0.4, -0.1, 0.03)),
                3.0,
                np.eye(3),
                gyro_bmj=np.array((0.01, -0.02, 0.03), dtype=np.float64),
            )
            velocity_results = []
            position_results = []
            for sim_t in (0.0, 0.03, 0.10, 0.14, 0.20, 0.24, 0.30, 0.35):
                state.sim_t = sim_t
                delivery = advance_dvl_sensor_runtime(bridge, state)
                if delivery is not None:
                    velocity_results.append(delivery)
                position_results.extend(bridge._dvl_sensor_new_position_deliveries)
            return (
                tuple(velocity_results),
                tuple(position_results),
                tuple(float(value) for value in bridge._dvl_sensor_current_attitude_rpy_deg),
                tuple(float(value) for value in bridge._dvl_sensor_position_local_frd_m),
            )

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            first = run(bridge)
            reset_dvl_sensor_runtime(bridge)
            second = run(bridge)

        self.assertGreater(len(first[0]), 1)
        self.assertGreater(len(first[1]), 1)
        self.assertEqual(first, second)

    def test_packet_dropout_is_separate_from_beam_validity(self) -> None:
        environment = {f"{ENV_PREFIX}PACKET_DROPOUT_PROBABILITY": "1"}
        with sensor_config(environment=environment):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((0.2, 0.0, 0.0)), 2.0, np.eye(3))
            delivery = advance_dvl_sensor_runtime(bridge, state)

        self.assertIsNone(delivery)
        self.assertEqual(bridge._dvl_sensor_timing.stats.probabilistic_drops, 1)
        self.assertEqual(bridge._dvl_sensor_timing.stats.pending, 0)
        self.assertEqual(len(bridge._dvl_sensor_new_position_deliveries), 1)
        np.testing.assert_allclose(bridge._dvl_sensor_position_local_frd_m, 0.0)

    def test_velocity_report_period_reflects_dropped_packet_gap(self) -> None:
        environment = {
            f"{ENV_PREFIX}SEED": "5",
            f"{ENV_PREFIX}PACKET_DROPOUT_PROBABILITY": "0.5",
        }
        with sensor_config(environment=environment):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.zeros(3), 2.0, np.eye(3))
            first = advance_dvl_sensor_runtime(bridge, state)
            state.sim_t = 0.1
            dropped = advance_dvl_sensor_runtime(bridge, state)
            state.sim_t = 0.2
            after_gap = advance_dvl_sensor_runtime(bridge, state)

        self.assertAlmostEqual(first.report_period_s, 0.1, places=12)
        self.assertIsNone(dropped)
        self.assertAlmostEqual(after_gap.report_period_s, 0.2, places=12)

    def test_position_packet_dropout_does_not_drop_velocity_report(self) -> None:
        environment = {
            f"{ENV_PREFIX}POSITION_PACKET_DROPOUT_PROBABILITY": "1",
        }
        with sensor_config(environment=environment):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.zeros(3), 2.0, np.eye(3))
            velocity = advance_dvl_sensor_runtime(bridge, state)

        self.assertIsNotNone(velocity)
        self.assertEqual(bridge._dvl_sensor_new_position_deliveries, ())
        self.assertEqual(
            bridge._dvl_sensor_position_timing.stats.probabilistic_drops,
            1,
        )


class DvlDeadReckoningTest(unittest.TestCase):
    def test_position_reports_have_an_independent_five_hz_cadence(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((0.2, 0.0, 0.0)), 2.0, np.eye(3))
            position_deliveries = []
            for index in range(5):
                state.sim_t = index * 0.1
                advance_dvl_sensor_runtime(bridge, state)
                position_deliveries.extend(bridge._dvl_sensor_new_position_deliveries)

        self.assertEqual(
            [delivery.capture_time_s for delivery in position_deliveries],
            [0.0, 0.2, 0.4],
        )

    def test_position_reports_continue_when_every_velocity_packet_is_lost(self) -> None:
        environment = {f"{ENV_PREFIX}PACKET_DROPOUT_PROBABILITY": "1"}
        with sensor_config(environment=environment):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((0.2, 0.0, 0.0)), 2.0, np.eye(3))
            velocity_deliveries = []
            position_deliveries = []
            for index in range(5):
                state.sim_t = index * 0.1
                delivery = advance_dvl_sensor_runtime(bridge, state)
                if delivery is not None:
                    velocity_deliveries.append(delivery)
                position_deliveries.extend(bridge._dvl_sensor_new_position_deliveries)

        self.assertEqual(velocity_deliveries, [])
        self.assertEqual(
            [delivery.capture_time_s for delivery in position_deliveries],
            [0.0, 0.2, 0.4],
        )
        np.testing.assert_allclose(
            position_deliveries[-1].position_local_frd_m,
            (0.08, 0.0, 0.0),
            atol=1.0e-11,
        )

    def test_device_dr_reset_preserves_sensor_phase_and_random_stream(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((1.0, 0.0, 0.0)), 2.0, np.eye(3))
            advance_dvl_sensor_runtime(bridge, state)
            state.sim_t = 0.1
            advance_dvl_sensor_runtime(bridge, state)
            sample_index = bridge._dvl_sensor_model._sample_index
            next_capture_index = bridge._dvl_sensor_timing._next_capture_index
            position_next_capture_index = (
                bridge._dvl_sensor_position_timing._next_capture_index
            )
            generation = bridge._dvl_sensor_dr_generation
            reset_dvl_dead_reckoning(bridge)

        self.assertEqual(bridge._dvl_sensor_model._sample_index, sample_index)
        self.assertEqual(
            bridge._dvl_sensor_timing._next_capture_index,
            next_capture_index,
        )
        self.assertEqual(
            bridge._dvl_sensor_position_timing._next_capture_index,
            position_next_capture_index,
        )
        self.assertEqual(bridge._dvl_sensor_dr_generation, generation + 1)
        np.testing.assert_allclose(bridge._dvl_sensor_position_local_frd_m, 0.0)

    def test_device_dr_reset_discards_stale_pending_position_packet(self) -> None:
        def mutate(data) -> None:
            data["packet_transport"]["queue"].update(
                {"capacity": 1, "overflow_policy": "drop_newest"}
            )
            data["timing"]["transport_latency"].update(
                {
                    "mean_s": 0.5,
                    "jitter_std_s": 0.0,
                    "min_s": 0.0,
                    "max_s": 0.5,
                }
            )

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((0.2, 0.0, 0.0)), 2.0, np.eye(3))
            advance_dvl_sensor_runtime(bridge, state)
            self.assertEqual(bridge._dvl_sensor_position_timing.stats.pending, 1)

            reset_dvl_dead_reckoning(bridge)
            self.assertEqual(bridge._dvl_sensor_position_timing.stats.pending, 0)
            state.sim_t = 0.2
            advance_dvl_sensor_runtime(bridge, state)

            self.assertEqual(bridge._dvl_sensor_position_timing.stats.pending, 1)
            self.assertEqual(
                bridge._dvl_sensor_position_timing.stats.queue_overflow_drops,
                0,
            )
            state.sim_t = 0.7
            advance_dvl_sensor_runtime(bridge, state)
            delivery = bridge._dvl_sensor_new_position_deliveries[0]

        self.assertEqual(delivery.capture_time_s, 0.2)
        self.assertEqual(delivery.generation, 1)

    def test_device_dr_reset_between_captures_retains_post_reset_motion(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((1.0, 0.0, 0.0)), 2.0, np.eye(3))
            advance_dvl_sensor_runtime(bridge, state)

            reset_dvl_dead_reckoning(
                bridge,
                reset_time_s=0.05,
                rot_world_body=np.eye(3),
            )
            state.sim_t = 0.1
            advance_dvl_sensor_runtime(bridge, state)

        np.testing.assert_allclose(
            bridge._dvl_sensor_position_local_frd_m,
            (0.05, 0.0, 0.0),
            atol=1.0e-11,
        )

    def test_device_dr_reset_anchors_modeled_attitude_at_command_observation(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(
                0.0,
                np.zeros(3),
                2.0,
                np.eye(3),
                gyro_bmj=np.zeros(3, dtype=np.float64),
            )
            advance_dvl_sensor_runtime(bridge, state)

            reset_dvl_dead_reckoning(
                bridge,
                reset_time_s=0.05,
                rot_world_body=np.eye(3),
            )
            state.sim_t = 0.1
            state.rot_world_body = yaw_rotation(90.0)
            state.gyro_bmj = np.array((0.0, 0.0, np.pi), dtype=np.float64)
            advance_dvl_sensor_runtime(bridge, state)

        np.testing.assert_allclose(
            bridge._dvl_sensor_current_attitude_rpy_deg,
            (0.0, 0.0, 9.0),
            atol=1.0e-10,
        )

    def test_invalid_device_reset_is_atomic(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            generation = bridge._dvl_sensor_dr_generation
            position = bridge._dvl_sensor_position_local_frd_m.copy()

            with self.assertRaisesRegex(ValueError, "non-negative"):
                reset_dvl_dead_reckoning(
                    bridge,
                    reset_time_s=-0.1,
                    rot_world_body=np.eye(3),
                )

        self.assertEqual(bridge._dvl_sensor_dr_generation, generation)
        np.testing.assert_array_equal(
            bridge._dvl_sensor_position_local_frd_m,
            position,
        )

    def test_gyro_calibration_removes_attitude_bias_without_resetting_sensor(self) -> None:
        def mutate(data) -> None:
            data["dead_reckoning"]["attitude_bias_std_deg"] = [1.0, 1.0, 1.0]

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            sample_index = bridge._dvl_sensor_model._sample_index
            calibrate_dvl_gyro(bridge)

        np.testing.assert_allclose(bridge._dvl_sensor_attitude_bias_deg, 0.0)
        self.assertEqual(bridge._dvl_sensor_model._sample_index, sample_index)

    def test_invalid_beam_dropout_updates_time_without_position(self) -> None:
        def mutate(data) -> None:
            data["a50_sensor_model"]["beam_dropout_probabilities"] = [1.0, 1.0, 1.0, 1.0]

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((1.0, 0.0, 0.0)), 2.0, np.eye(3))
            first = advance_dvl_sensor_runtime(bridge, state)
            state.sim_t = 0.1
            second = advance_dvl_sensor_runtime(bridge, state)

        self.assertFalse(first.sample.velocity_valid)
        self.assertFalse(second.sample.velocity_valid)
        np.testing.assert_allclose(bridge._dvl_sensor_position_local_frd_m, 0.0, atol=0.0)
        np.testing.assert_allclose(bridge._odom_pos, 0.0, atol=0.0)
        self.assertEqual(bridge._dvl_sensor_last_capture_time_s, 0.1)

    def test_invalid_bottom_lock_still_grows_position_uncertainty(self) -> None:
        def mutate(data) -> None:
            data["a50_sensor_model"]["beam_dropout_probabilities"] = [
                1.0,
                1.0,
                1.0,
                1.0,
            ]
            data["dead_reckoning"]["position_random_walk_std_m_sqrt_s"] = 0.1

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.ones(3), 2.0, np.eye(3))
            advance_dvl_sensor_runtime(bridge, state)
            initial_variance = bridge._dvl_sensor_position_variance_m2
            state.sim_t = 1.0
            advance_dvl_sensor_runtime(bridge, state)

        np.testing.assert_allclose(bridge._dvl_sensor_position_local_frd_m, 0.0)
        self.assertGreater(
            bridge._dvl_sensor_position_variance_m2,
            initial_variance,
        )

    def test_measured_velocity_integrates_without_reading_truth_position(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = TruthTrapState(
                0.0,
                np.array((1.0, 0.0, 0.0)),
                2.0,
                np.eye(3),
            )
            advance_dvl_sensor_runtime(bridge, state)
            first_position = bridge._dvl_sensor_position_local_frd_m.copy()
            state.sim_t = 0.1
            advance_dvl_sensor_runtime(bridge, state)
            second_position = bridge._dvl_sensor_position_local_frd_m.copy()

        np.testing.assert_allclose(first_position, 0.0, atol=1.0e-12)
        np.testing.assert_allclose(second_position, (0.1, 0.0, 0.0), atol=1.0e-11)
        np.testing.assert_allclose(bridge._odom_pos, (0.1, 0.0, 0.0), atol=1.0e-11)

    def test_public_dead_reckoning_does_not_follow_private_truth_rotation(self) -> None:
        def run(truth_rotations: tuple[np.ndarray, ...]):
            with sensor_config():
                bridge = FakeBridge()
                configure_dvl_sensor_runtime(bridge)
                state = FakeState(
                    0.0,
                    np.array((1.0, 0.0, 0.0)),
                    2.0,
                    truth_rotations[0],
                    gyro_bmj=np.zeros(3, dtype=np.float64),
                )
                for index, rotation in enumerate(truth_rotations):
                    state.sim_t = index * 0.1
                    state.rot_world_body = rotation
                    advance_dvl_sensor_runtime(bridge, state)
                delivery = bridge._dvl_sensor_new_position_deliveries[-1]
                return (
                    np.asarray(delivery.position_local_frd_m),
                    np.asarray(delivery.attitude_rpy_deg),
                )

        identity = np.eye(3, dtype=np.float64)
        baseline = run((identity, identity, identity))
        perturbed = run((identity, yaw_rotation(90.0), yaw_rotation(90.0)))

        np.testing.assert_allclose(perturbed[0], baseline[0], atol=1.0e-12)
        np.testing.assert_allclose(perturbed[1], baseline[1], atol=1.0e-12)

    def test_modeled_gyro_drives_local_frame_and_rpy_degrees(self) -> None:
        def mutate(data) -> None:
            processing = data["timing"]["processing_latency"]
            processing.update(
                {"mean_s": 0.05, "jitter_std_s": 0.0, "min_s": 0.0, "max_s": 0.05}
            )

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(
                0.0,
                np.array((1.0, 0.0, 0.0)),
                2.0,
                np.eye(3),
                gyro_bmj=np.zeros(3, dtype=np.float64),
            )
            self.assertIsNone(advance_dvl_sensor_runtime(bridge, state))
            state.sim_t = 0.05
            self.assertIsNotNone(advance_dvl_sensor_runtime(bridge, state))

            state.sim_t = 0.1
            state.rot_world_body = yaw_rotation(90.0)
            captured_gyro = np.array((0.0, 0.0, np.pi / 2.0), dtype=np.float64)
            state.gyro_bmj = captured_gyro
            self.assertIsNone(advance_dvl_sensor_runtime(bridge, state))
            captured_gyro[:] = 0.0
            state.sim_t = 0.15
            advance_dvl_sensor_runtime(bridge, state)

        expected_position = 0.1 * np.array(
            (np.cos(np.deg2rad(9.0)), np.sin(np.deg2rad(9.0)), 0.0)
        )
        np.testing.assert_allclose(
            bridge._dvl_sensor_position_local_frd_m,
            expected_position,
            atol=1.0e-11,
        )
        np.testing.assert_allclose(bridge._odom_pos, expected_position, atol=1.0e-11)
        np.testing.assert_allclose(
            bridge._dvl_sensor_current_attitude_rpy_deg,
            (0.0, 0.0, 9.0),
            atol=1.0e-10,
        )

    def test_multiple_arrivals_apply_in_order_and_return_last(self) -> None:
        def mutate(data) -> None:
            transport = data["timing"]["transport_latency"]
            transport.update(
                {"mean_s": 0.25, "jitter_std_s": 0.0, "min_s": 0.0, "max_s": 0.25}
            )

        with sensor_config(mutate):
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((1.0, 0.0, 0.0)), 2.0, np.eye(3))
            for sim_t in (0.0, 0.1, 0.2):
                state.sim_t = sim_t
                self.assertIsNone(advance_dvl_sensor_runtime(bridge, state))
            state.sim_t = 0.45
            delivery = advance_dvl_sensor_runtime(bridge, state)

        self.assertAlmostEqual(delivery.capture_time_s, 0.2, places=12)
        np.testing.assert_allclose(
            bridge._dvl_sensor_position_local_frd_m,
            (0.45, 0.0, 0.0),
            atol=1.0e-11,
        )
        self.assertEqual(len(bridge._dvl_sensor_new_deliveries), 3)
        self.assertEqual(
            [item.capture_time_s for item in bridge._dvl_sensor_new_deliveries],
            [0.0, 0.1, 0.2],
        )

    def test_time_jump_drops_stale_captures_instead_of_backdating_current_state(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.zeros(3), 2.0, np.eye(3))
            advance_dvl_sensor_runtime(bridge, state)
            state.sim_t = 0.35
            state.dvl_vel_dvl_frd = np.array((3.5, 0.0, 0.0))
            delivery = advance_dvl_sensor_runtime(bridge, state)

        self.assertEqual(bridge._dvl_sensor_missed_capture_count, 2)
        self.assertAlmostEqual(delivery.capture_time_s, 0.35, places=12)
        self.assertEqual(delivery.sample.true_velocity_frd_mps, (3.5, 0.0, 0.0))

    def test_single_overdue_capture_uses_current_time_instead_of_backdating(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.zeros(3), 2.0, np.eye(3))
            advance_dvl_sensor_runtime(bridge, state)
            state.sim_t = 0.15
            state.dvl_vel_dvl_frd = np.array((1.5, 0.0, 0.0))
            delivery = advance_dvl_sensor_runtime(bridge, state)

        self.assertEqual(bridge._dvl_sensor_missed_capture_count, 0)
        self.assertAlmostEqual(delivery.capture_time_s, 0.15, places=12)
        self.assertEqual(delivery.sample.true_velocity_frd_mps, (1.5, 0.0, 0.0))

    def test_backward_time_performs_clean_seeded_reset(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(0.0, np.array((1.0, 0.0, 0.0)), 2.0, np.eye(3))
            initial = advance_dvl_sensor_runtime(bridge, state)
            state.sim_t = 0.1
            moved = advance_dvl_sensor_runtime(bridge, state)
            self.assertIsNotNone(moved)
            self.assertGreater(bridge._dvl_sensor_position_local_frd_m[0], 0.0)

            state.sim_t = 0.0
            rewound = advance_dvl_sensor_runtime(bridge, state)
            reset_dvl_sensor_runtime(bridge)
            explicit = advance_dvl_sensor_runtime(bridge, state)

        self.assertEqual(rewound.sample.sample_index, 0)
        self.assertEqual(rewound, initial)
        self.assertEqual(explicit, initial)
        np.testing.assert_allclose(bridge._odom_pos, 0.0, atol=0.0)

    def test_explicit_reset_at_nonzero_time_does_not_backfill_old_captures(self) -> None:
        with sensor_config():
            bridge = FakeBridge()
            configure_dvl_sensor_runtime(bridge)
            state = FakeState(1.0, np.array((1.0, 0.0, 0.0)), 2.0, np.eye(3))
            first = advance_dvl_sensor_runtime(bridge, state)
            self.assertEqual(first.capture_time_s, 1.0)
            self.assertEqual(bridge._dvl_sensor_timing.stats.captures, 1)

            state.sim_t = 1.1
            advance_dvl_sensor_runtime(bridge, state)
            reset_dvl_sensor_runtime(bridge)
            state.sim_t = 1.37
            restarted = advance_dvl_sensor_runtime(bridge, state)

        self.assertEqual(restarted.sample.sample_index, 0)
        self.assertEqual(restarted.capture_time_s, 1.37)
        self.assertEqual(bridge._dvl_sensor_timing.stats.captures, 1)
        np.testing.assert_allclose(bridge._dvl_sensor_position_local_frd_m, 0.0, atol=0.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
