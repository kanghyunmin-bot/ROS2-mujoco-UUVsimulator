#!/usr/bin/env python3
"""Run an isolated, normal-EKF recorded-sensor diagnostic without MuJoCo.

Native VPD, RC, heartbeat and command events retain their recorded receipt
schedule. Filtered IMU telemetry is causally held at the JSON boundary; it is
not represented as recovered high-rate inertial data. MAVLink output and JSON
plant output are saved separately. This runner cannot certify controller
parity when initial filter state or raw sensor history is unavailable.
"""

from __future__ import annotations

import argparse
import bisect
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time

import numpy as np

os.environ["MAVLINK20"] = "1"
from pymavlink import mavutil  # noqa: E402

RUNTIME = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RUNTIME))
from sim.contracts.baro import BaroPressureLaw  # noqa: E402
from sim.transport.json_servo import decode_json_servo_packet  # noqa: E402
from recorded_imu_gyro_input import gyro_frd_from_recorded_imu, gyro_frd_from_recorded_raw_imu  # noqa: E402
from recorded_initial_mode import recorded_mode_id, select_initial_mode_row  # noqa: E402
from recorded_barometer_mapping import barometer_parameters_from_recorded_primary  # noqa: E402


def latest(values: np.ndarray, t_s: float) -> np.ndarray | None:
    """Select only telemetry already received by this replay instant [s]."""
    index = np.searchsorted(values[:, 0], t_s, side="right") - 1
    return values[index] if index >= 0 else None


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.r_[a[0] * b[0] - np.dot(a[1:], b[1:]),
                 a[0] * b[1:] + b[0] * a[1:] + np.cross(a[1:], b[1:])]


def quaternion_ned_frd(q_xyzw: np.ndarray) -> np.ndarray:
    enu_ned = np.array([0.0, 2**-0.5, 2**-0.5, 0.0])
    frd_flu = np.array([0.0, 1.0, 0.0, 0.0])
    q = quaternion_multiply(enu_ned, quaternion_multiply(q_xyzw[[3, 0, 1, 2]], frd_flu))
    return q / np.linalg.norm(q)


def read_parameters(path: Path) -> dict[str, str]:
    return dict(line.split()[:2] for line in path.read_text().splitlines()
                if line.strip() and not line.lstrip().startswith("#"))


def run(case_path: Path, output_dir: Path, native_vpd: bool) -> dict:
    case = json.loads(case_path.read_text())
    output_dir.mkdir(parents=True, exist_ok=False)
    paths = {key: Path(value) for key, value in case["paths"].items()}
    parameters = read_parameters(paths["simulation_parameters"])
    field = json.loads(paths["field_parameters"].read_text())["values"]
    # Restore the recorded estimator choices. Sensor-device calibration and
    # virtual hardware remain the explicit simulation adaptation, not a fit.
    for key, value in field.items():
        if key.startswith(("EK3_", "VISO_")) or key in {
            "AHRS_EKF_TYPE", "SYSID_MYGCS", "SYSID_THISMAV", "BARO_PRIMARY",
            "BARO1_GND_PRESS", "BARO2_GND_PRESS", "BARO_SPEC_GRAV",
        }:
            parameters[key] = str(value)
    parameters.update(LOG_DISARMED="1", SCHED_LOOP_RATE="400", ARMING_CHECK="0")
    barometer_policy = case.get("barometer_mapping", "legacy_indices")
    barometer_mapping = None
    if barometer_policy == "recorded_primary_to_virtual_zero":
        parameters, barometer_mapping = barometer_parameters_from_recorded_primary(parameters, field)
    elif barometer_policy != "legacy_indices":
        raise ValueError(f"Unsupported barometer mapping: {barometer_policy}")
    if int(float(parameters["AHRS_EKF_TYPE"])) != 3:
        raise ValueError("Only the normal EKF3 diagnostic is supported")
    parameter_file = output_dir / "params.parm"
    parameter_file.write_text("".join(f"{key} {value}\n" for key, value in parameters.items()))
    sensors = dict(np.load(paths["native_events"]))
    gyro_source = case.get("gyro_source", "attitude")
    ahrs_history = sensors.get("ahrs_mavlink", np.empty((0, 12)))
    raw_imu_history = sensors.get("imu_raw", np.empty((0, 12)))
    if gyro_source not in {"attitude", "attitude_minus_ahrs", "imu_raw"}:
        raise ValueError(f"Unsupported recorded gyro source: {gyro_source}")
    if gyro_source == "imu_raw" and len(raw_imu_history) == 0:
        raise ValueError("Calibrated INS gyro policy requires recorded ROS data_raw")
    if gyro_source == "attitude_minus_ahrs" and len(ahrs_history) == 0:
        raise ValueError("Gyro-stage inversion requires recorded AHRS history")
    raw = dict(np.load(paths["raw_events"]))
    commands = json.loads(paths["raw_details"].read_text())
    state = json.loads(paths["field_states"].read_text())
    mode_ids = {"MANUAL": 19, "STABILIZE": 0, "ALT_HOLD": 2, "POSHOLD": 16}
    initial_mode_index = select_initial_mode_row(state, mode_ids)
    initial_mode_id = recorded_mode_id(state[initial_mode_index][3], mode_ids)
    events = []
    for kind, key in [("rc", "gcs_RC_CHANNELS_OVERRIDE"), ("heartbeat", "gcs_HEARTBEAT")]:
        for index, row in enumerate(raw[key]):
            events.append((float(row[0]), kind, index))
    for index, row in enumerate(commands):
        if row["direction"] == "gcs" and row["mavpackettype"] in {"SET_MODE", "COMMAND_LONG"}:
            events.append((row["t"], "command", index))
    if native_vpd:
        for index, row in enumerate(sensors["vpd"]):
            events.append((float(row[0]), "vpd", index))
    # Match the observed initial mode once, without forcing arming or outputs.
    events.append((float(state[initial_mode_index][0]), "initial_mode", initial_mode_index))
    events.sort(key=lambda row: row[0])
    event_times = [row[0] for row in events]
    cursor = 0
    pre_roll_s = float(case["pre_roll_s"])
    source_end_s = float(case["source_end_s"])
    post_source_drain_s = float(case.get("post_source_drain_s", 0.0))
    if not 0 <= post_source_drain_s <= 2.0:
        raise ValueError("Post-source log drain must be between 0 and 2 s")
    pressure_law = BaroPressureLaw(case["surface_pressure_pa"], case["surface_pressure_pa"])
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 9002))
    sock.settimeout(0.25)
    mav = mavutil.mavlink_connection("udpin:0.0.0.0:14660", source_system=255, source_component=240)
    command = [str(paths["binary"]), "-S", "-w", "--model", "JSON", "--rate", "400",
               "--speedup", str(case["speedup"]),
               "--sim-address=127.0.0.1", "--sim-port-out=9002", "--sim-port-in=9003",
               "--defaults", str(parameter_file), "-I0", "--home", "33.810313,-118.393867,0,90",
               "--serial0=udpclient:127.0.0.1:14660"]
    environment = {**os.environ, "UUV_SITL_YAW_BRAKE": "0"}
    process = subprocess.Popen(command, cwd=output_dir, env=environment,
                               stdout=(output_dir / "sitl.log").open("w"), stderr=subprocess.STDOUT)
    outputs, json_servo, attitude, pressure, pressure2, local, heartbeats, notices, sent = [], [], [], [], [], [], [], [], []
    sensor_trace = []
    gyro_input_trace = []
    last_frame = first_frame = None
    reply = None
    last_bootstrap_hb = -1.0
    now = 0.0
    frame_rates = {}
    fcu_seen = False
    telemetry_request_bag_s = None
    telemetry_intervals = [(36, 100), (30, 20), (29, 10), (137, 10), (32, 10)]
    wall_start = time.monotonic()
    complete = False
    try:
        while time.monotonic() - wall_start < float(case["timeout_wall_s"]):
            try:
                packet, address = sock.recvfrom(65535)
            except socket.timeout:
                if process.poll() is not None:
                    raise RuntimeError(f"SITL exited with {process.returncode}")
                continue
            servo = decode_json_servo_packet(packet)
            if servo is None:
                continue
            if servo.frame_count == last_frame:
                if reply is not None:
                    sock.sendto(reply, address)
                continue
            if servo.frame_rate_hz <= 0 or (now >= pre_roll_s and not 399 <= servo.frame_rate_hz <= 401):
                raise RuntimeError(f"Unexpected JSON frame rate {servo.frame_rate_hz}")
            frame_rates[servo.frame_rate_hz] = frame_rates.get(servo.frame_rate_hz, 0) + 1
            if first_frame is None:
                first_frame = servo.frame_count
            if last_frame is not None and servo.frame_count != last_frame + 1:
                raise RuntimeError(f"Nonsequential JSON frame {last_frame}->{servo.frame_count}")
            last_frame = servo.frame_count
            bag_t = now - pre_roll_s
            if (servo.frame_count - first_frame) % 4000 == 0:
                print(f"JSON time={now:.1f}s bag={bag_t:.1f}s advertised_rate={servo.frame_rate_hz}", flush=True)
            while message := mav.recv_match(blocking=False):
                if message.get_srcSystem() == int(float(parameters["SYSID_THISMAV"])):
                    fcu_seen = True
                kind = message.get_type()
                if kind == "SERVO_OUTPUT_RAW" and message.port == 0:
                    outputs.append([bag_t, message.time_usec / 1e6 - pre_roll_s,
                                    *[getattr(message, f"servo{i}_raw") for i in range(1, 9)]])
                elif kind == "ATTITUDE":
                    attitude.append([bag_t, message.time_boot_ms / 1000 - pre_roll_s,
                                     message.roll, message.pitch, message.yaw,
                                     message.rollspeed, message.pitchspeed, message.yawspeed])
                elif kind == "SCALED_PRESSURE":
                    pressure.append([bag_t, message.time_boot_ms / 1000 - pre_roll_s, message.press_abs * 100])
                elif kind == "SCALED_PRESSURE2":
                    pressure2.append([bag_t, message.time_boot_ms / 1000 - pre_roll_s, message.press_abs * 100])
                elif kind == "LOCAL_POSITION_NED":
                    local.append([bag_t, message.time_boot_ms / 1000 - pre_roll_s,
                                  message.x, message.y, message.z, message.vx, message.vy, message.vz])
                elif kind == "HEARTBEAT":
                    heartbeats.append([bag_t, message.base_mode, message.custom_mode])
                elif kind in {"STATUSTEXT", "COMMAND_ACK", "PARAM_VALUE", "AUTOPILOT_VERSION"}:
                    notices.append({"receipt_bag_s": bag_t, **message.to_dict()})
            if now < pre_roll_s and now - last_bootstrap_hb >= 1:
                mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_bootstrap_hb = now
            # An udpin connection has no destination until it receives a peer's
            # first packet. Requests sent on the first JSON frame can silently
            # disappear, leaving default 2 Hz servo telemetry in the comparison.
            if telemetry_request_bag_s is None and fcu_seen:
                for message_id, frequency in telemetry_intervals:
                    mav.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                              message_id, 1e6 / frequency, 0, 0, 0, 0, 0)
                telemetry_request_bag_s = bag_t
            if bag_t >= 0:
                end = bisect.bisect_right(event_times, min(bag_t, source_end_s) + 1e-9)
                for event_t, kind, index in events[cursor:end]:
                    if kind == "rc":
                        row = raw["gcs_RC_CHANNELS_OVERRIDE"][index]
                        mav.mav.rc_channels_override_send(int(row[3]), int(row[4]), *map(int, row[5:23]))
                    elif kind == "heartbeat":
                        row = raw["gcs_HEARTBEAT"][index]
                        mav.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                               int(row[3]), int(row[4]), int(row[5]))
                    elif kind == "vpd":
                        row = sensors["vpd"][index]
                        mav.mav.vision_position_delta_send(int(row[2]), int(row[3]), row[4:7], row[7:10], row[10])
                    elif kind == "initial_mode":
                        mav.mav.set_mode_send(1, 1, initial_mode_id)
                    else:
                        row = commands[index]
                        if row["mavpackettype"] == "SET_MODE":
                            mav.mav.set_mode_send(row["target_system"], row["base_mode"], row["custom_mode"])
                        else:
                            mav.mav.command_long_send(row["target_system"], row["target_component"],
                                                      row["command"], row["confirmation"],
                                                      *[row[f"param{i}"] for i in range(1, 8)])
                    sent.append([bag_t, event_t, kind, index])
                cursor = end
                json_servo.append([bag_t, *servo.pwm_values[:8]])
                if bag_t >= source_end_s + post_source_drain_s:
                    complete = True
                    break
            # A bounded log drain advances only the post-record transport
            # clock. No later inputs or inferred sensor response are injected,
            # and times beyond source_end_s are excluded from field scoring.
            sensor_t = min(bag_t, source_end_s)
            imu_row = latest(sensors["imu"], sensor_t)
            pressure_row = latest(sensors["pressure"], sensor_t)
            if imu_row is None:
                gyro, accel, quat = np.zeros(3), np.array([0.0, 0.0, -9.80665]), np.array([1.0, 0.0, 0.0, 0.0])
                imu_age = -1.0
            else:
                gyro_receipt = imu_row[0]
                if gyro_source == "imu_raw":
                    gyro, gyro_receipt = gyro_frd_from_recorded_raw_imu(raw_imu_history, sensor_t)
                    # Unknown pre-observation input retains the explicit zero
                    # bootstrap. It is not filled from filtered ATTITUDE rates.
                    if gyro is None:
                        gyro, gyro_receipt = np.zeros(3), -1.
                    removed_drift, ahrs_receipt = np.zeros(3), None
                else:
                    gyro, removed_drift, ahrs_receipt = gyro_frd_from_recorded_imu(
                        imu_row, ahrs_history, gyro_source=gyro_source)
                accel = imu_row[9:12] * [1, -1, -1]
                quat = quaternion_ned_frd(imu_row[2:6])
                imu_age = bag_t - imu_row[0]
                if bag_t >= 0:
                    gyro_input_trace.append([bag_t, gyro_receipt,
                                             -1. if ahrs_receipt is None else ahrs_receipt,
                                             *gyro, *removed_drift])
            p_pa = float(case["surface_pressure_pa"] if pressure_row is None else pressure_row[2])
            depth = pressure_law.sitl_depth_m_for_pressure(p_pa)
            # JSON advertises its initial model rate (1200 Hz) before the first
            # reply. The experiment explicitly drives the recorded controller
            # cadence at 400 Hz; subsequent advertised rate must agree.
            next_time = (servo.frame_count - first_frame + 1) / 400.0
            payload = {"timestamp": next_time, "imu": {"gyro": gyro.tolist(), "accel_body": accel.tolist()},
                       "position": [0.0, 0.0, depth], "velocity": [0.0, 0.0, 0.0],
                       "quaternion": quat.tolist(), "no_time_sync": False, "no_lockstep": False}
            reply = ("\n" + json.dumps(payload, separators=(",", ":")) + "\n").encode()
            sock.sendto(reply, address)
            if bag_t >= 0:
                sensor_trace.append([bag_t, imu_age, -1 if pressure_row is None else bag_t - pressure_row[0], p_pa, depth])
            now = next_time
        if not complete:
            raise RuntimeError("Recorded sensor diagnostic timed out")
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        sock.close()
        mav.close()
        np.savez_compressed(output_dir / "controller.npz", servo_telemetry=np.asarray(outputs),
                            json_plant_input=np.asarray(json_servo), attitude=np.asarray(attitude),
                            pressure=np.asarray(pressure), pressure2=np.asarray(pressure2), local_position=np.asarray(local),
                            heartbeats=np.asarray(heartbeats), sensor_trace=np.asarray(sensor_trace))
        np.savez_compressed(output_dir / "gyro_input_trace.npz", rows=np.asarray(gyro_input_trace))
        receipt = {
            "complete": complete, "case": case, "native_vpd_enabled": native_vpd,
            "wall_s": time.monotonic() - wall_start, "final_bag_s": now - pre_roll_s,
            "source_end_s": source_end_s, "post_source_drain_s": post_source_drain_s,
            "observed_json_frame_rates": frame_rates,
            "telemetry_request_bag_s": telemetry_request_bag_s,
            "requested_telemetry_intervals_hz": telemetry_intervals,
            "gyro_source": gyro_source,
            "gyro_input_semantics": "Causal held source receipt; imu_raw is already calibrated INS0, not AHRS-corrected ATTITUDE. A source receipt of -1 marks an unknown bootstrap, not a measurement.",
            "recovered_high_rate_inertial_history": False,
            "barometer_mapping_policy": barometer_policy, "barometer_mapping": barometer_mapping,
            "barometer_mapping_sha256": hashlib.sha256((Path(__file__).parent / "recorded_barometer_mapping.py").read_bytes()).hexdigest(),
            "initial_mode_row": initial_mode_index, "initial_mode_id": initial_mode_id,
            "initial_mode_policy_sha256": hashlib.sha256((Path(__file__).parent / "recorded_initial_mode.py").read_bytes()).hexdigest(),
            "gyro_adapter_sha256": hashlib.sha256((Path(__file__).parent / "recorded_imu_gyro_input.py").read_bytes()).hexdigest(),
            "gyro_input_trace_columns": ["bag_time_s", "imu_receipt_s", "ahrs_receipt_s_or_minus_one",
                                          "gyro_x_rad_s", "gyro_y_rad_s", "gyro_z_rad_s",
                                          "removed_drift_x_rad_s", "removed_drift_y_rad_s", "removed_drift_z_rad_s"],
            "event_counts": {kind: sum(row[2] == kind for row in sent) for kind in {row[2] for row in sent}},
            "command": command, "input_sha256": {key: hashlib.sha256(path.read_bytes()).hexdigest()
                                                  for key, path in paths.items()},
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "source_commit": case["source_commit"], "ahrs_truth_mode": False,
            "physical_ground_truth_feedback": False, "approved_parity": False,
            "limitations": case["limitations"], "notices": notices,
        }
        (output_dir / "sent_events.json").write_text(json.dumps(sent))
        (output_dir / "run.json").write_text(json.dumps(receipt, indent=2) + "\n")
    return receipt


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case_json", required=True, type=Path)
    parser.add_argument("--output_dir", required=True, type=Path)
    parser.add_argument("--native_vpd", choices=["0", "1"], required=True)
    args = parser.parse_args()
    receipt = run(args.case_json, args.output_dir, args.native_vpd == "1")
    print(json.dumps({key: receipt[key] for key in ["complete", "native_vpd_enabled", "event_counts", "wall_s"]}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
