"""Source groups for active-runtime time contract audits."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from audit_code_contract_common import contains_all, evidence
from audit_code_contract_types import Evidence


@dataclass(frozen=True)
class RuntimeTimeSources:
    publish_timing_py: Path
    sensor_replay_clock_py: Path
    sensor_replay_servo_clock_py: Path
    sensor_replay_payload_time_py: Path
    servo_poll_loop_py: Path
    servo_request_py: Path
    servo_link_request_py: Path
    command_link_request_py: Path
    extnav_scheduler_py: Path
    extnav_runtime_py: Path
    simulation_loop_clocks_py: Path
    simulation_loop_catchup_py: Path
    simulation_step_catchup_py: Path
    simulation_sensor_catchup_py: Path


def runtime_time_sources(runtime_paths: dict[str, Path]) -> RuntimeTimeSources:
    return RuntimeTimeSources(
        publish_timing_py=runtime_paths["ros2_bridge_publish_timing_py"],
        sensor_replay_clock_py=runtime_paths["sitl_sensor_replay_clock_py"],
        sensor_replay_servo_clock_py=runtime_paths["sitl_sensor_replay_servo_clock_py"],
        sensor_replay_payload_time_py=runtime_paths["sitl_sensor_replay_payload_time_py"],
        servo_poll_loop_py=runtime_paths["sitl_json_servo_poll_loop_py"],
        servo_request_py=runtime_paths["sitl_mavlink_request_servo_py"],
        servo_link_request_py=runtime_paths["sitl_mavlink_request_servo_link_py"],
        command_link_request_py=runtime_paths["sitl_mavlink_request_command_link_py"],
        extnav_scheduler_py=runtime_paths["sitl_transport_extnav_scheduler_py"],
        extnav_runtime_py=runtime_paths["sitl_transport_extnav_runtime_py"],
        simulation_loop_clocks_py=runtime_paths["simulation_loop_clocks_py"],
        simulation_loop_catchup_py=runtime_paths["simulation_loop_catchup_py"],
        simulation_step_catchup_py=runtime_paths["simulation_step_catchup_py"],
        simulation_sensor_catchup_py=runtime_paths["simulation_sensor_catchup_py"],
    )


def runtime_time_contract_passes(sources: RuntimeTimeSources) -> bool:
    return (
        _publish_timing_contract_passes(sources.publish_timing_py)
        and _sensor_replay_clock_contract_passes(
            sources.sensor_replay_clock_py,
            sources.sensor_replay_servo_clock_py,
            sources.sensor_replay_payload_time_py,
        )
        and _mavlink_polling_contract_passes(
            sources.servo_poll_loop_py,
            sources.servo_link_request_py,
            sources.command_link_request_py,
        )
        and _external_nav_time_contract_passes(sources.extnav_scheduler_py, sources.extnav_runtime_py)
        and _simulation_clock_contract_passes(
            step_catchup_py=sources.simulation_step_catchup_py,
            sensor_catchup_py=sources.simulation_sensor_catchup_py,
        )
    )


def runtime_time_evidence(sources: RuntimeTimeSources) -> list[Evidence]:
    return [
        evidence(sources.publish_timing_py, "self.last_pub_t + self.sensor_dt"),
        evidence(sources.sensor_replay_servo_clock_py, "_sensor_replay_last_clock_t_s"),
        evidence(sources.sensor_replay_payload_time_py, "_sensor_replay_start_on_rc"),
        evidence(sources.servo_poll_loop_py, "now_wall = time.monotonic()"),
        evidence(sources.servo_link_request_py, "_sitl_mavlink_servo_hz"),
        evidence(sources.command_link_request_py, "_sitl_rcout_telemetry_hz"),
        evidence(sources.extnav_scheduler_py, 'transport._sitl_extnav_scheduler = "sim_time"'),
        evidence(sources.extnav_runtime_py, "transport._sitl_extnav_start_wall = time.monotonic()"),
        evidence(sources.simulation_step_catchup_py, "clocks.next_step_wall += cadence.target_dt"),
        evidence(sources.simulation_sensor_catchup_py, "clocks.next_sensor_wall += cadence.sensor_dt"),
    ]


def _publish_timing_contract_passes(publish_timing_py: Path) -> bool:
    return contains_all(
        publish_timing_py,
        [
            "def publish_time_due",
            "sim_t <= self.last_pub_t",
            "self.last_pub_t + self.sensor_dt",
        ],
    )


def _sensor_replay_clock_contract_passes(
    sensor_replay_clock_py: Path,
    sensor_replay_servo_clock_py: Path,
    sensor_replay_payload_time_py: Path,
) -> bool:
    return contains_all(
        sensor_replay_clock_py,
        [
            'self._sensor_replay_clock != "servo_frame"',
            "sensor_replay_servo_frame_clock_time_s(self)",
            "sensor_replay_payload_timestamp_for_sim_t(self, sim_t)",
        ],
    ) and contains_all(
        sensor_replay_servo_clock_py,
        [
            "transport._sitl_json_latest_frame_count",
            "transport._sensor_replay_last_clock_t_s",
        ],
    ) and contains_all(
        sensor_replay_payload_time_py,
        [
            "_sensor_replay_start_on_rc",
            "_sensor_replay_start_delay_s",
            "_sensor_replay_time_offset_s",
        ],
    )


def _mavlink_polling_contract_passes(
    servo_poll_loop_py: Path,
    servo_link_request_py: Path,
    command_link_request_py: Path,
) -> bool:
    return contains_all(
        servo_poll_loop_py,
        [
            "now_wall = time.monotonic()",
            "_sitl_command_poll_hz",
            "_sitl_mavlink_poll_hz",
            "_send_neutral_rc_keepalive(service_wall)",
        ],
    ) and contains_all(
        servo_link_request_py,
        [
            "request_servo_output_raw",
            "_sitl_mavlink_servo_hz",
        ],
    ) and contains_all(
        command_link_request_py,
        [
            "request_servo_output_raw",
            "_sitl_rcout_telemetry_hz",
        ],
    )


def _external_nav_time_contract_passes(extnav_scheduler_py: Path, extnav_runtime_py: Path) -> bool:
    return contains_all(
        extnav_scheduler_py,
        [
            'transport._sitl_extnav_scheduler = "sim_time"',
            'transport._sitl_extnav_scheduler = "wall_time"',
            "transport._sensor_replay_frames or transport._native_vpd_events",
            "transport._sitl_extnav_rate_hz = max(float(transport._sitl_extnav_rate_hz), 15.0)",
        ],
    ) and contains_all(
        extnav_runtime_py,
        [
            "transport._sitl_extnav_start_wall = time.monotonic()",
            "transport._sitl_extnav_last_send_wall = -1.0",
            "transport._sitl_extnav_tx_window_start_wall = transport._sitl_extnav_start_wall",
        ],
    )


def _simulation_clock_contract_passes(*, step_catchup_py: Path, sensor_catchup_py: Path) -> bool:
    return contains_all(
        step_catchup_py,
        [
            "time.perf_counter()",
            "clocks.next_step_wall += cadence.target_dt",
            "max_catchup_steps",
        ],
    ) and contains_all(
        sensor_catchup_py,
        [
            "time.perf_counter()",
            "clocks.next_sensor_wall += cadence.sensor_dt",
            "max_sensor_catchup",
        ],
    )


__all__ = [
    "RuntimeTimeSources",
    "runtime_time_contract_passes",
    "runtime_time_evidence",
    "runtime_time_sources",
]
