"""Offline preparation separates observable voltages from guessed ESC supplies."""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from extract_rosbag_trends import sha256
from prepare_propulsion_replay import ROOT, prepare, sampling_quality, voltage_window
from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.physics.thruster_voltage import load_voltage_trace, update_supply_voltage


def window(volts, **overrides):
    options = {
        "reference": "esc_bus",
        "provenance": "Synthetic fixture",
        "max_gap_s": 0.11,
        "voltage_range": [10, 20],
    }
    options.update(overrides)
    return voltage_window(
        np.array([10, 10.1, 10.2, 10.3]), np.array(volts), 10.05, 10.25, **options
    )


def test_battery_reference_and_unsupported_voltage_never_produce_runtime_trace():
    for volts, reference in (
        ([16, 15, 14, 13], "battery_pack_unverified"),
        ([22, 21, 19, 18], "esc_bus"),
    ):
        status, trace = window(volts, reference=reference)
        assert not status["ready"] and trace is None


def test_boundary_interpolation_requires_valid_bracketing_samples():
    status, trace = window([16, 15, 14, 13])
    assert status["ready"]
    np.testing.assert_allclose(trace[:, 0], [0, 0.05, 0.15, 0.2])
    np.testing.assert_allclose(trace[:, 1], [15.5, 15, 14, 13.5])
    for volts, options in (
        ([16, float("nan"), 14, 13], {}),
        ([16, 15, 14, 13], {"max_gap_s": 0.05}),
    ):
        status, trace = window(volts, **options)
        assert not status["ready"] and trace is None


def test_average_rate_does_not_hide_a_pwm_outage():
    regular = np.arange(1001) / 1000
    times = regular[(regular < 0.3) | (regular > 0.5)]
    result = sampling_quality(times, 50, 0.05)
    assert result["received_hz"] > 50
    assert not result["ready"] and result["gaps_over_limit"] == 1
    assert sampling_quality(np.arange(101) / 100, 50, 0.05)["ready"]


def test_export_round_trip_uses_receipt_clock_and_recorded_sim_origin(tmp_path):
    source_dir = tmp_path / "source"
    source_dir.mkdir()
    origin = 1775133981020936721
    bt, pt = np.arange(99, 122) / 10, np.arange(1000, 1201) / 100

    def times_ns(t):
        stamps = origin + np.rint(t * 1e9).astype(np.int64)
        # Deliberately wrong header epoch must never shift the replay.
        return np.column_stack([stamps, stamps - 4000 * 10**9])

    np.savez(
        source_dir / "numeric.npz",
        __battery=np.column_stack(
            [bt, 16 - 0.5 * (bt - 10), np.ones(len(bt)), np.zeros(len(bt))]
        ),
        __battery__times_ns=times_ns(bt),
        __mavros__rc__out__times_ns=times_ns(pt),
    )
    manifest = {
        "record_origin_ns": origin,
        "sqlite_sha256": "synthetic_fixture",
        "derived_sha256": {"numeric.npz": sha256(source_dir / "numeric.npz")},
    }
    (source_dir / "source.json").write_text(json.dumps(manifest))
    replay = tmp_path / "replay.json"
    replay.write_text(
        json.dumps(
            {
                "complete": True,
                "begin_bag_time": 10,
                "end_bag_time": 12,
                "origin_sim_time": 6.25,
            }
        )
    )
    args = argparse.Namespace(
        source_dir=source_dir,
        replay_json=replay,
        output_dir=tmp_path / "prepared",
        voltage_topic="/battery",
        voltage_reference="esc_bus",
        provenance="Synthetic fixture; not measured hardware",
        performance_json=ROOT / "config/thruster_performance.json",
        maximum_voltage_gap_s=0.2,
        minimum_pwm_hz=50,
        maximum_pwm_gap_s=0.05,
        begin_bag_s=None,
        end_bag_s=None,
        outside_trace="hold",
    )
    result = prepare(args)
    assert result["identification_input_ready"]
    assert not result["physical_parameters_identified"]
    config = load_thruster_performance_config(
        args.performance_json, requested_voltage=16, direct=True
    )
    config["voltage_trace"] = load_voltage_trace(
        Path(result["runtime_voltage_csv"]),
        config,
        time_offset_s=result["voltage_time_offset_s"],
    )
    update_supply_voltage(config, 7.25)
    assert config["selected_voltage"] == 15.5
    # A short high-rate burst cannot pass the requested whole-interval coverage.
    args.begin_bag_s = 9.95
    args.output_dir = tmp_path / "missing_pwm_start"
    report = prepare(args)
    assert not report["identification_input_ready"]
    assert report["voltage_replay_ready"]
