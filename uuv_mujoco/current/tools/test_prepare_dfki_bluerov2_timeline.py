"""DFKI attitude must survive gaps in motion-capture tracks."""

from pathlib import Path

import numpy as np

from prepare_dfki_bluerov2 import prepare
from rosbag_trend_math import interpolate_valid


def _write(path: Path, names: list[str], rows: np.ndarray) -> None:
    np.savetxt(path, rows, delimiter=",", header=",".join(names), comments="")


def test_prepare_preserves_independent_attitude_and_motion_gaps(tmp_path: Path) -> None:
    source, output = tmp_path / "raw", tmp_path / "out"
    source.mkdir()
    t = np.round(np.arange(0, 4.001, .01), 3)
    vehicle_t = t[(t < 1) | (t > 2)]
    _write(source / "velocity.csv", ["time", "linear_x", "linear_y", "linear_z"],
           np.column_stack([vehicle_t, np.ones(len(vehicle_t)), np.zeros((len(vehicle_t), 2))]))
    _write(source / "pose.csv", ["time", "pos_x", "pos_y", "pos_z"],
           np.column_stack([vehicle_t, np.zeros((len(vehicle_t), 2)), np.ones(len(vehicle_t))]))
    fields = [f"rov_{name}_{axis}" for name in ("front_left", "front_right", "back_right") for axis in "xyz"]
    markers = np.tile([1, -1, 0, 1, 1, 0, -1, 1, 0], (len(t), 1))
    _write(source / "qtm_rov.csv", ["time", *fields], np.column_stack([t, markers]))
    att_t = t[::10]
    _write(source / "attitude.csv", ["time", "roll", "pitch", "yaw", "roll_speed", "pitch_speed", "yaw_speed"],
           np.column_stack([att_t, np.zeros((len(att_t), 2)), att_t * .1, np.zeros((len(att_t), 2)), np.full(len(att_t), .1)]))
    _write(source / "pressure2.csv", ["time", "press_abs"], np.column_stack([att_t, np.full(len(att_t), 120000)]))
    _write(source / "battery.csv", ["time", "voltages_01"], np.column_stack([att_t, np.full(len(att_t), 15)]))
    _write(source / "rc-out.csv", ["time", *[f"channel{i}" for i in range(1, 7)]],
           np.column_stack([att_t, np.full((len(att_t), 6), 1500)]))

    metadata = prepare(source, output, "random")
    real = np.genfromtxt(output / "real.csv", names=True, delimiter=",")
    mask = (real["t_s"] > 1.1) & (real["t_s"] < 1.9)
    assert metadata["start_s"] == vehicle_t[0]
    assert np.all(np.isfinite(real["yaw_deg"][mask]))
    assert np.all(np.isnan(real["surge_mps"][mask]))
    assert np.all(np.isnan(real["depth_m"][mask]))
    # The fitter and scorer resample at fixed timestamps with a 40 ms gap limit.
    assert np.isfinite(interpolate_valid(real["t_s"], real["yaw_deg"], [1.55], max_gap_s=.04)).all()
    assert np.isnan(interpolate_valid(real["t_s"], real["surge_mps"], [1.55], max_gap_s=.04)).all()
    assert np.all(np.diff(real["t_s"]) > 0)
