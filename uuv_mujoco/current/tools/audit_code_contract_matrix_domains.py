"""Required source-contract domains for the top-level matrix gate."""

from __future__ import annotations


CONTRACT_DOMAINS = (
    {
        "domain": "source_identity",
        "required": (
            "ardupilot_source_tag",
            "top_level_ardupilot_gitlink",
            "active_runtime_alias_current",
        ),
        "warn_ok": (),
    },
    {
        "domain": "time_phase",
        "required": ("active_runtime_time_contract_sim_publish_wall_transport",),
        "warn_ok": (),
    },
    {
        "domain": "sensor_input_output",
        "required": (
            "json_sensor_no_direct_pressure_or_altitude",
            "baro_sitl_pressure_from_json_position_z",
            "active_runtime_baro_contract_frontend_match",
            "active_runtime_static_pressure_external_bar30",
            "active_runtime_sensor_io_snapshot_contract",
        ),
        "warn_ok": (),
    },
    {
        "domain": "rc_input_output",
        "required": (
            "rc_override_local_16_channel_limit",
            "rc_override_timeout_policy",
            "ardusub_joystick_axis_mapping",
            "active_runtime_rc_override_forward_mirror_contract",
        ),
        "warn_ok": ("rc_override_local_16_channel_limit",),
    },
    {
        "domain": "controller_output_and_plant_input",
        "required": (
            "json_servo_packet_16_raw_pwm",
            "servo_output_raw_halrcout_telemetry",
            "active_runtime_plant_input_raw_pwm_contract",
        ),
        "warn_ok": (),
    },
    {
        "domain": "thruster_contract",
        "required": (
            "thruster_contract_final_pwm_not_mot_direction_again",
            "active_runtime_thruster_conversion_contract",
            "plant_replay_gate_safe_targets",
        ),
        "warn_ok": ("plant_replay_gate_safe_targets",),
    },
    {
        "domain": "dynamic_ellipsoid_fluid",
        "required": (
            "active_runtime_dynamic_fluidcoef_contract",
            "active_runtime_integrated_flow_contract",
        ),
        "warn_ok": (),
    },
)


__all__ = ["CONTRACT_DOMAINS"]
