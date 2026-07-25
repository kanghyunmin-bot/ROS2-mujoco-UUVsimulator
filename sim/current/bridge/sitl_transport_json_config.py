"""JSON servo/socket configuration for SitlTransport."""

from __future__ import annotations

from sim.transport import JsonServoReceiver


def initialize_json_servo_transport(transport: object, *, sitl_ip: str, sitl_port: int, sitl_send_port: int) -> None:
    transport.sitl_addr = (sitl_ip, int(sitl_port))
    transport.sitl_send_addr = (sitl_ip, int(sitl_send_port))
    transport.sitl_listen_addr = ("0.0.0.0", int(sitl_port))
    transport._json_servo_receiver = JsonServoReceiver(
        listen_addr=transport.sitl_listen_addr,
        servo_target=transport.sitl_addr,
        sensor_target=transport.sitl_send_addr,
    )
    transport.sitl_sock = transport._json_servo_receiver.socket
    transport._sitl_client_addr = None
    transport._sitl_send_target = None
    transport._sitl_client_last_wall = -1.0
    transport._sitl_client_logged = False
    transport._sitl_no_client_warn_interval_s = 3.0
    transport._sitl_last_client_missing_wall = -1.0
    transport._sitl_last_command_stale_wall = -1.0
    transport._sitl_last_send_wall = -1.0
    transport._sitl_last_send_err_wall = -1.0
    transport._sitl_last_no_client_wall = -1.0
    transport._sitl_last_sensor_log_wall = -1.0
    transport._sitl_send_counter = 0
    transport._sitl_first_servo_wall = -1.0
    transport._sitl_last_nonneutral_servo_wall = -1.0
    transport._sitl_last_neutral_warn_wall = -1.0
    transport._sitl_nonfinite_warned = False
    transport._sitl_prev_sim_t = None
    transport._sitl_prev_pos_enu = None
    transport._sitl_last_cmd_log = -1.0
    transport._sitl_last_servo_pkt = None
    transport._sitl_servo_callback = None
    transport._sitl_servo_telemetry_callback = None
