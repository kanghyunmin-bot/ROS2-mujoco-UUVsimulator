#!/usr/bin/env python3
"""Run bounded visual-FSM demonstrations against an isolated simulation GUI."""
from __future__ import annotations

import argparse
import hashlib
import shutil
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import urllib.request
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from mavros_msgs.msg import OverrideRCIn, State
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String
from outcome import classify_release

ROOT = Path(__file__).resolve().parents[2]
UPSTREAM = 'cd9c61f8d964ca2c0a01edd0941ce18f2cdd1d0e'
TEACHER_REVISION = 'sim-depth-hand-servo-v10-stop-align'


def main():
    calibration = json.loads((ROOT / 'uuv_mujoco/current/config/hand_camera_calibration.json').read_text())
    inverse = calibration['error_to_right_down_m']
    hand_parameters = dict(hand_target_x=calibration['hand_target_uv'][0],
                           hand_target_y=calibration['hand_target_uv'][1],
                           hand_right_error_x=inverse[0][0] / .1,
                           hand_right_error_y=inverse[0][1] / .1,
                           hand_depth_error_x=inverse[1][0] / .7,
                           hand_depth_error_y=inverse[1][1] / .7)
    hand_arguments = [item for name, value in hand_parameters.items()
                      for item in ('-p', f'{name}:={value}')]

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui_url', default='http://127.0.0.1:8878')
    parser.add_argument('--episodes', type=int, default=3)
    parser.add_argument('--episode_seconds', type=float, default=120.0)
    parser.add_argument('--wall_seconds', type=float, default=360.0)
    parser.add_argument('--model', type=Path, default=ROOT / 'YOLO/best.pt')
    parser.add_argument('--detector_python', default=str(ROOT / '.venv-vla/bin/python'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.episodes < 1 or min(args.episode_seconds, args.wall_seconds) <= 0:
        parser.error('episode count and limits must be positive')
    if not args.model.is_file():
        parser.error('Model file does not exist')
    args.output.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(Path(__file__).with_name('index.html'), args.output / 'index.html')
    model_sha256 = hashlib.sha256(args.model.read_bytes()).hexdigest()
    run_id = 'fsm-' + uuid.uuid4().hex[:12]
    state = {'vehicle': None, 'buoys': {}, 'buoys_wall': 0., 'hand': None,
             'hand_wall': 0., 'rc': None, 'rc_wall': 0., 'phase': 'IDLE'}
    results = []
    children = []
    logs = []
    recording = False
    cancelled = False
    prepared = False
    owns_environment = False
    sequence = 0
    stop = threading.Event()
    rclpy.init()
    node = rclpy.create_node('simulation_auto_collection')
    node.create_subscription(State, '/mavros/state', lambda m: state.update(vehicle=m), qos_profile_sensor_data)
    def on_buoys(m):
        state.update(buoys=json.loads(m.data), buoys_wall=time.monotonic())
    node.create_subscription(String, '/mujoco/course_buoys/status', on_buoys, 10)
    node.create_subscription(CompressedImage, '/imx219/camera1/image_raw/compressed',
                             lambda m: state.update(hand=m, hand_wall=time.monotonic()), qos_profile_sensor_data)
    node.create_subscription(OverrideRCIn, '/auto_collection/proposed_rc',
                             lambda m: state.update(rc=m, rc_wall=time.monotonic()), 10)
    node.create_subscription(String, '/auto_collection/state', lambda m: state.update(phase=m.data), 10)
    handshake_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    request = node.create_publisher(Bool, '/auto_collection/search', handshake_qos)
    grant = node.create_publisher(Bool, '/auto_collection/grant', handshake_qos)
    def spin():
        while not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    def interrupt(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGINT, interrupt)

    def api(command=None, **payload):
        data = None if command is None else json.dumps({
            'command': command, '_auto_collection_token': os.environ.get('UUV_AUTO_COLLECTION_TOKEN', ''),
            **payload}).encode()
        req = urllib.request.Request(args.gui_url + ('/api/status' if data is None else '/api/command'),
                                     data=data, headers={'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=3) as response:
            result = json.load(response)
        if result.get('error'):
            raise RuntimeError(result['error'])
        return result

    def progress(phase, **extra):
        payload = dict(run_id=run_id, phase=phase, results=results, episodes=args.episodes,
                       teacher_commit=UPSTREAM, teacher_revision=TEACHER_REVISION, **extra)
        temp = args.output / 'status.tmp'
        temp.write_text(json.dumps(payload, indent=2))
        temp.replace(args.output / 'status.json')
        print(json.dumps(payload, ensure_ascii=False), flush=True)

    def wait_for(predicate, label, timeout=60):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if prepared and (recording or label in {"recorder readiness", "ready between episodes", "FSM handshake", "recording start"}):
                send_rc(False)
            value = predicate()
            if value:
                return value
            time.sleep(.15)
        raise RuntimeError('Timed out: ' + label)

    def spawn(command, name):
        log = (args.output / (name + '.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        children.append(process)
        return process

    def terminate(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)

    def send_rc(enabled):
        nonlocal sequence
        sequence += 1
        axes = dict(forward=0., lateral=0., heave=0., yaw=0.)
        if enabled and state['rc'] is not None:
            for key, index in [('forward', 4), ('lateral', 5), ('heave', 2), ('yaw', 3)]:
                pwm = state['rc'].channels[index]
                if 1100 <= pwm <= 1900:
                    axes[key] = (pwm - 1500) / 400.
        result = api('rc', client_id=run_id, seq=sequence, enabled=True, axes=axes)
        if result.get('accepted') is False:
            raise RuntimeError('GUI rejected control ownership')

    def recorder_idle():
        rec = api()['recorder']
        return rec if not rec.get('active') and not rec.get('busy') and rec.get('online') else None

    try:
        status = api()
        if (status['recorder'].get('configuration_locked')
                or status['control'].get('enabled')
                or status['control'].get('vla_prepared')
                or status['processes'].get('mission_running')
                or status['processes'].get('pinger_homing_running')):
            raise RuntimeError('Use a separate idle simulation GUI, without an existing session')
        if not status['processes']['sim_running']:
            api('stack_start', sim_preset='research_pool_distributed')
        progress('waiting_simulation')
        wait_for(lambda: state['buoys'].get('source') == 'mujoco_live'
                 and state['vehicle'] and state['vehicle'].connected, 'simulation/MAVROS', 120)
        owns_environment = True
        api('mode', mode='STABILIZE')
        wait_for(lambda: state['vehicle'].mode == 'STABILIZE', 'STABILIZE')
        api('arm', value=True)
        wait_for(lambda: state['vehicle'].armed, 'arming')
        api('recorder_prepare', task='Approach the buoy, align the fixed fork, and detach it.', mode='STABILIZE')
        prepared = True
        wait_for(lambda: api()['recorder'].get('ready'), 'recorder readiness', 90)
        api('release', client_id=run_id)
        api('recorder_action', action='reset')
        initial_reset = wait_for(recorder_idle, 'map reset', 30)
        if not initial_reset.get('message', '').startswith('완료:'):
            raise RuntimeError('Initial map reset failed: ' + initial_reset.get('message', ''))
        detector = spawn([args.detector_python, str(ROOT / 'rospkg/src/kmu26_auv_buoy_vision_control/scripts/yolo_buoy_detector.py'),
                          '--ros-args', '-p', 'use_sim_time:=true', '-p', f'model_path:={args.model}',
                          '-p', 'show_preview:=false', '-p', 'publish_annotated_image:=false',
                          '-p', 'publish_all_detections:=true', '-p', 'simulation_yellow_assist:=true',
                          '-p', 'bbox_topic:=/auto_collection/bbox'], 'detector')
        hand_detector = spawn([args.detector_python, str(ROOT / 'rospkg/src/kmu26_auv_buoy_vision_control/scripts/yolo_buoy_detector.py'),
                               '--ros-args', '-r', '__node:=hand_yolo_detector',
                               '-p', 'use_sim_time:=true', '-p', f'model_path:={args.model}',
                               '-p', 'image_topic:=/imx219/camera1/image_raw/compressed',
                               '-p', 'show_preview:=false', '-p', 'publish_annotated_image:=false',
                               '-p', 'publish_all_detections:=true', '-p', 'simulation_yellow_assist:=true',
                               '-p', 'simulation_stick_assist:=true',
                               '-p', 'bbox_topic:=/auto_collection/hand_bbox'], 'hand-detector')
        (args.output / 'provenance.json').write_text(json.dumps(dict(
            teacher_commit=UPSTREAM, teacher_revision=TEACHER_REVISION, model=str(args.model), model_sha256=model_sha256,
            arguments=vars(args), hand_camera_calibration=calibration), default=str, indent=2))
        for trial in range(1, args.episodes + 1):
            progress('preparing', trial=trial)
            wait_for(lambda: api()['recorder'].get('ready'), 'ready between episodes')
            if time.monotonic() - state['buoys_wall'] > 2:
                raise RuntimeError('Stale simulation truth')
            initial_ids = {b['id'] for b in state['buoys']['buoys'] if b.get('has_magnet') and b.get('eq_active')}
            if not initial_ids:
                raise RuntimeError('No attached magnetic targets after reset')
            request.publish(Bool(data=False))
            grant.publish(Bool(data=False))
            state.update(rc=None, rc_wall=0., phase='IDLE')
            fsm = spawn(['ros2', 'run', 'auv_buoy_vision_control', 'collection_fsm_node', '--ros-args',
                         '-p', 'use_sim_time:=true', '-p', 'bbox_topic:=/auto_collection/bbox',
                         '-p', 'rc_override_topic:=/auto_collection/proposed_rc',
                         '-p', 'state_topic:=/auto_collection/state',
                         '-p', 'vision_search_request_topic:=/auto_collection/search',
                         '-p', 'vision_control_granted_topic:=/auto_collection/grant'] + hand_arguments, f'fsm-{trial}')
            wait_for(lambda: request.get_subscription_count() > 0 and grant.get_subscription_count() > 0,
                     'FSM handshake')
            api('recorder_action', action='start')
            recording = True
            wait_for(lambda: api()['recorder'].get('active'), 'recording start')
            start_s = state['buoys']['time_s']
            wall_start = time.monotonic()
            request.publish(Bool(data=True))
            time.sleep(.2)
            grant.publish(Bool(data=True))
            outcome = dict(success=False, reason='episode_time_limit')
            phases = []
            last_progress = 0.
            while state['buoys']['time_s'] - start_s < args.episode_seconds:
                now = time.monotonic()
                phase = state['phase']
                if not phases or phases[-1]['state'] != phase:
                    phases.append(dict(state=phase, time_s=state['buoys']['time_s']))
                if detector.poll() is not None or hand_detector.poll() is not None or fsm.poll() is not None:
                    raise RuntimeError('Detector/FSM exited; inspect logs')
                if now - state['buoys_wall'] > 2 or now - state['hand_wall'] > 2:
                    raise RuntimeError('Lost simulation status or hand camera')
                if not state['vehicle'].armed or state['vehicle'].mode != 'STABILIZE':
                    raise RuntimeError('Arming/mode changed during episode')
                found = classify_release(initial_ids, state['buoys']['buoys'], start_s)
                if found:
                    outcome = found
                    send_rc(False)
                    release_s = found['buoy']['release_time_s']
                    wait_for(lambda: state['hand'] is not None and
                             state['hand'].header.stamp.sec +
                             state['hand'].header.stamp.nanosec * 1e-9 >= release_s,
                             'hand frame after release', 5)
                    hand = state['hand']
                    (args.output / f'hand-release-{trial}.jpg').write_bytes(bytes(hand.data))
                    outcome['hand_stamp_s'] = hand.header.stamp.sec + hand.header.stamp.nanosec * 1e-9
                    # Camera frame may precede the physics event. Retain video tail.
                    tail_end = time.monotonic() + 1.0
                    while time.monotonic() < tail_end:
                        send_rc(False)
                        time.sleep(.07)
                    break
                if now - wall_start >= args.wall_seconds:
                    outcome['reason'] = 'wall_time_limit'
                    break
                if phase in ('COMPLETE', 'FAILSAFE'):
                    outcome['reason'] = 'fsm_' + phase.lower() + '_without_verified_release'
                    break
                if now - state['rc_wall'] > 2 and now - wall_start > 8:
                    raise RuntimeError('FSM has no fresh RC commands')
                send_rc(now - state['rc_wall'] < .5)
                if now - last_progress > 10:
                    progress('recording', trial=trial, fsm=phase,
                             sim_seconds=state['buoys']['time_s'] - start_s)
                    last_progress = now
                time.sleep(.07)
            send_rc(False)
            api('recorder_action', action='success' if outcome['success'] else 'failure')
            saved = wait_for(recorder_idle, 'episode save', 90)['last_result']
            recording = False
            terminate(fsm)
            api('release', client_id=run_id)
            if saved.get('frames', 0) < 1 or saved.get('success') != outcome['success']:
                raise RuntimeError('Recorder result does not match outcome')
            evidence = dict(trial=trial, **outcome, saved=saved, phases=phases,
                            start_s=start_s, end_s=state['buoys']['time_s'],
                            teacher_commit=UPSTREAM, teacher_revision=TEACHER_REVISION, model_sha256=model_sha256, label_source='simulation_contact_and_weld',
                            visual_release_classifier=False, hand_camera_calibration=calibration)
            (Path(saved['path']) / 'auto_collection.json').write_text(json.dumps(evidence, indent=2))
            results.append(evidence)
            progress('resetting', trial=trial)
            api('recorder_action', action='reset')
            reset_result = wait_for(recorder_idle, 'map reset', 30)
            if not reset_result.get('message', '').startswith('완료:'):
                raise RuntimeError('Map reset failed: ' + reset_result.get('message', ''))
            wait_for(lambda: all(any(b['id'] == target and b.get('eq_active') and not b.get('detached')
                                      for b in state['buoys']['buoys']) for target in initial_ids), 'attached targets restored')
            evidence['reset_verified'] = True
            evidence['reset_message'] = reset_result['message']
            (Path(saved['path']) / 'auto_collection.json').write_text(json.dumps(evidence, indent=2))
            (args.output / f'trial-{trial}.json').write_text(json.dumps(evidence, indent=2))
        progress('finalizing')
    except KeyboardInterrupt:
        cancelled = True
        progress('stopping')
    except BaseException as exc:
        progress('error', error=str(exc) or type(exc).__name__)
        raise
    finally:
        for child in reversed(children):
            terminate(child)
        try:
            if owns_environment:
                api('release', client_id=run_id)
            if recording and api()['recorder'].get('active'):
                api('recorder_action', action='failure')
                interrupted = wait_for(recorder_idle, 'interrupted recording save', 30)
                saved = interrupted.get('last_result', {})
                if saved.get('path'):
                    evidence = dict(trial=len(results)+1, success=False,
                                    reason='collection_interrupted', label_valid=False, saved=saved,
                                    teacher_commit=UPSTREAM, teacher_revision=TEACHER_REVISION, model_sha256=model_sha256,
                                    hand_camera_calibration=calibration)
                    (Path(saved['path']) / 'auto_collection.json').write_text(json.dumps(evidence, indent=2))
                    results.append(evidence)
            if prepared:
                api('recorder_action', action='close')
            if owns_environment:
                api('arm', value=False)
                wait_for(lambda: state['vehicle'] is not None and
                         not state['vehicle'].armed, 'disarming', 15)
        finally:
            stop.set()
            thread.join(timeout=2)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            for log in logs:
                log.close()
    progress('stopped' if cancelled else 'complete')


if __name__ == '__main__':
    main()
