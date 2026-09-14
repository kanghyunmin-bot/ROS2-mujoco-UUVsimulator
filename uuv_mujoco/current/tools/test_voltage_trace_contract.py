"""Voltage replay must preserve provenance, coverage, and replay clock alignment."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sim.physics.thruster_performance_loader import load_thruster_performance_config
from sim.physics.thruster_performance_selector import select_thruster_performance_config
from sim.physics.thruster_voltage import (
    force_curve_at_voltage,
    load_voltage_trace,
    update_supply_voltage,
)


@pytest.fixture
def config():
    return load_thruster_performance_config(
        ROOT / "config/thruster_performance.json", requested_voltage=16, direct=True
    )


def trace_file(tmp_path, content="sim_time,voltage_v\n0,16\n1,14\n2,12\n", **overrides):
    path = tmp_path / "voltage.csv"
    path.write_text(content)
    metadata = {
        "schema": "uuv_esc_voltage/v1",
        "csv_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "voltage_reference": "esc_bus",
        "provenance": "Synthetic voltage fixture; not a physical recording",
        "max_gap_s": 1.0,
        "outside_trace": "error",
    }
    metadata.update(overrides)
    path.with_suffix(".csv.json").write_text(json.dumps(metadata))
    return path


def test_rejects_pack_voltage_even_when_numerically_in_curve_range(tmp_path, config):
    path = trace_file(tmp_path, voltage_reference="battery_pack_unverified")
    with pytest.raises(ValueError, match="ESC"):
        load_voltage_trace(path, config)


def test_rejects_changed_csv_and_missing_voltage_coverage(tmp_path, config):
    path = trace_file(tmp_path)
    path.write_text(path.read_text().replace("1,14", "1,15"))
    with pytest.raises(ValueError, match="SHA256"):
        load_voltage_trace(path, config)
    path = trace_file(tmp_path, max_gap_s=0.2)
    with pytest.raises(ValueError, match="gap"):
        load_voltage_trace(path, config)


def test_frozen_trace_moves_with_replay_start_and_does_not_extrapolate(
    tmp_path, config
):
    path = trace_file(tmp_path)
    config["voltage_trace"] = load_voltage_trace(path, config, time_offset_s=6.25)
    update_supply_voltage(config, 7.25)
    assert config["selected_voltage"] == 14
    np.testing.assert_allclose(config["force"], force_curve_at_voltage(config, 14))
    for time in (6.24, 8.26, float("nan")):
        with pytest.raises(ValueError):
            update_supply_voltage(config, time)


def test_explicit_hold_covers_warmup_but_is_identified(tmp_path, config):
    path = trace_file(tmp_path, outside_trace="hold")
    config["voltage_trace"] = load_voltage_trace(path, config, time_offset_s=6.25)
    update_supply_voltage(config, 0)
    assert config["selected_voltage"] == 16
    assert config["voltage_trace"]["outside_trace"] == "hold"
    update_supply_voltage(config, 20)
    assert config["selected_voltage"] == 12


def test_legacy_csv_retains_hold_without_claiming_verified_measurement(
    tmp_path, config
):
    path = tmp_path / "legacy.csv"
    path.write_text("sim_time,voltage_v\n0,16\n2,12\n")
    config["voltage_trace"] = load_voltage_trace(path, config)
    assert config["voltage_trace"]["voltage_reference"] == "unverified_legacy_csv"
    update_supply_voltage(config, 5)
    assert config["selected_voltage"] == 12


def test_runtime_selector_passes_replay_offset(tmp_path):
    path = trace_file(tmp_path, outside_trace="hold")
    args = argparse.Namespace(
        disable_thruster_perf=False,
        thruster_perf_direct=True,
        thruster_voltage_trace=str(path),
        thruster_voltage_time_offset_s=6.25,
    )
    config = select_thruster_performance_config(
        args=args,
        active_thruster_voltage=16,
        path=ROOT / "config/thruster_performance.json",
        plant_replay_direct_rcout=False,
    )
    update_supply_voltage(config, 7.25)
    assert config["selected_voltage"] == 14


def test_actual_mujoco_actuator_uses_aligned_voltage_at_each_update(tmp_path):
    import contextlib
    import io

    import mujoco
    from check_research_pool_physics import _build_runtime

    with contextlib.redirect_stdout(io.StringIO()):
        runtime = _build_runtime(
            mujoco,
            profile_name="research_pool_distributed",
            fluid_model="legacy",
            use_custom_hydrodynamics=True,
        )
        config = load_thruster_performance_config(
            ROOT / "config/thruster_performance.json", requested_voltage=16, direct=True
        )
    config["voltage_trace"] = load_voltage_trace(
        trace_file(tmp_path), config, time_offset_s=6.25
    )
    actuator, model, data = runtime.thruster_actuator, runtime.model, runtime.data
    actuator.perf_cfg = config
    base, q, v = (
        int(runtime.state.base_id),
        int(runtime.state.world_qpos_adr),
        int(runtime.state.world_qvel_adr),
    )
    data.qpos[q + 2] = -1.5
    data.qpos[q + 3 : q + 7] = [1, 0, 0, 0]
    data.qvel[:] = 0
    mujoco.mj_forward(model, data)
    for name, command in (("yaw_lf", 0.5), ("yaw_rr", -0.5)):
        actuator.target.update({n: 0 for n in actuator.all_thruster_names})
        actuator.target[name] = command
        for sim_time, voltage in ((6.25, 16), (7.25, 14), (8.25, 12)):
            data.time = sim_time
            actuator.update_forces(10, base_id=base)
            # Independent manufacturer's table knot, not the runtime-selected curve.
            payload = json.loads(
                (ROOT / "config/thruster_performance.json").read_text()
            )
            curve = next(c for c in payload["curves"] if c["voltage_v"] == voltage)
            expected = np.interp(
                1500 + 400 * command, curve["pwm_us"], curve["force_n"]
            )
            assert actuator.force_cmd[name] == pytest.approx(expected, abs=1e-8)
            mujoco.mj_forward(model, data)
            np.testing.assert_allclose(
                data.qfrc_actuator[v : v + 3], actuator.last_force_body, atol=1e-6
            )
