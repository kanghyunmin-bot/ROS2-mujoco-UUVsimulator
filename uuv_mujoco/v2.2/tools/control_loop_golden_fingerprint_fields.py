"""Field lists for control-loop golden fingerprints."""

from __future__ import annotations


RESPONSE_METRICS = (
    "samples",
    "armed_fraction",
    "rcin_max_delta",
    "rcout_max_delta",
    "rcin_mean_abs_delta",
    "rcout_mean_abs_delta",
    "expected_metric_mean",
    "expected_metric_rms",
    "expected_metric_peak_abs",
    "gyro_x_mean",
    "gyro_y_mean",
    "gyro_z_mean",
    "gyro_x_rms",
    "gyro_y_rms",
    "gyro_z_rms",
    "gyro_x_peak_abs",
    "gyro_y_peak_abs",
    "gyro_z_peak_abs",
    "dvl_vx_mean",
    "dvl_vy_mean",
    "dvl_vz_mean",
    "dvl_vx_rms",
    "dvl_vy_rms",
    "dvl_vz_rms",
    "dvl_vx_peak_abs",
    "dvl_vy_peak_abs",
    "dvl_vz_peak_abs",
    "odom_vx_mean",
    "odom_vy_mean",
    "odom_vz_mean",
    "roll_rad_span",
    "pitch_rad_span",
    "yaw_rad_span",
    "depth_m_mean",
    "depth_m_span",
)

METADATA_KEYS = (
    "mode",
    "input_mode",
    "command",
    "axis_s",
    "neutral_s",
    "baseline_s",
    "sample_hz",
    "axes",
    "release_initial_depth_hold",
    "post_release_neutral_s",
    "rc_neutral",
    "rc_span",
)


__all__ = ["METADATA_KEYS", "RESPONSE_METRICS"]
