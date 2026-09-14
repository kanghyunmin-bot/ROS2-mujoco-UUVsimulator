"""Web adapter for the existing VLA collector; never publishes RC or arms."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import threading
import time
from datetime import datetime, timezone
from uuid import uuid4

from rclpy.clock import Clock, ClockType
from std_srvs.srv import Trigger, SetBool

from .config_paths import APP_ROOT, SIM_STACK_DIR
from .config_rc import RC_PWM_SPAN
from .ros_tools import ros_bash_command


class WebRecorder:
    def __init__(self, node, processes):
        self.node = node
        self.processes = processes
        self.lock = threading.RLock()
        self.process = None
        self.status = {}
        self.received = 0.0
        self.message = "레코더 준비를 눌러 시작하세요."
        self.future = None
        self.command_client = None
        self.status_future = None
        self.status_at = 0.0
        self.command_at = 0.0
        self.closed = False
        self.session = ""
        self.clients = {name: node.create_client(kind, '/vla_data_collector/' + name)
                        for name, kind in [('get_status', Trigger), ('start_episode', Trigger),
                                           ('stop_episode', SetBool), ('discard_episode', Trigger)]}
        self.timer = node.create_timer(0.5, self.poll, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def poll(self):
        with self.lock:
            if self.closed:
                return
            if self.future is not None and time.monotonic() - self.command_at > 15:
                # Do not retry an uncertain mutation: first reconcile recorder status.
                self.message = "요청 응답 지연: 재요청하지 말고 녹화 상태를 확인하세요."
            if self.status_future is not None:
                if time.monotonic() - self.status_at < 3:
                    return
                stale = self.status_future
                self.status_future = None
                self.clients["get_status"].remove_pending_request(stale)
                stale.cancel()
            client = self.clients['get_status']
            if not client.service_is_ready():
                return
            self.status_at = time.monotonic()
            self.status_future = client.call_async(Trigger.Request())
            self.status_future.add_done_callback(self._status_done)

    def _status_done(self, future):
        with self.lock:
            if future is not self.status_future:
                return
            try:
                result = future.result()
                if result.success:
                    self.status = json.loads(result.message)
                    self.received = time.monotonic()
            except Exception as exc:
                self.message = f"레코더 상태 확인 실패: {exc}"
            finally:
                self.status_future = None

    def payload(self):
        with self.lock:
            fresh = time.monotonic() - self.received < 3
            owned = bool(self.session and self.status.get('session_id') == self.session)
            running = self.process is not None and self.process.poll() is None
            return {**self.status, 'online': fresh, 'owned': owned, 'running': running,
                    'configuration_locked': bool(self.session),
                    'ready': fresh and owned and self.status.get('ready', False),
                    'busy': self.future is not None, 'message': self.message}

    def require_configuration_unlocked(self):
        """Protect the provenance captured at prepare, including idle episodes."""
        if self.session:
            raise ValueError('센서·시뮬레이션 설정은 레코더 세션 종료 후 변경하세요. 미리보기 속도는 변경할 수 있습니다.')

    def prepare(self, task, mode):
        task = str(task).strip()
        if not task or len(task) > 1000:
            raise ValueError('작업 지시문을 1~1000자로 입력하세요.')
        if mode not in {'STABILIZE', 'ALT_HOLD', 'MANUAL'}:
            raise ValueError('지원하지 않는 수집 모드입니다.')
        with self.lock:
            if self.closed:
                raise ValueError('GUI가 종료 중입니다.')
            self.require_configuration_unlocked()
            if self.process is not None and self.process.poll() is None:
                raise ValueError('레코더가 이미 실행 중입니다. 현재 세션을 계속 사용하세요.')
            if self.clients['get_status'].service_is_ready() or self.clients['start_episode'].service_is_ready():
                raise ValueError('별도 레코더가 실행 중입니다. 중복 실행하지 않습니다.')
            camera = self.processes.camera_config_payload()
            launched_camera = camera.get('launched') or camera
            if launched_camera.get('enabled') is False:
                raise ValueError('실행 요청에서 카메라 출력이 꺼져 있습니다. 카메라를 켜고 시뮬을 재시작하세요.')
            # A saved selection cannot override a lower-rate running launch.
            # This gate checks known requests, not achieved sensor throughput.
            if float(launched_camera.get('hz', 0)) < 10.0:
                raise ValueError('수집에는 카메라 10Hz 이상 설정이 필요합니다. VLA lite를 적용 후 입력 상태를 확인하세요.')
            self.session = datetime.now(timezone.utc).strftime('sim_%Y%m%dT%H%M%SZ_') + uuid4().hex[:8]
            root = APP_ROOT / 'outputs' / 'vla-demonstrations' / self.session
            root.mkdir(parents=True, exist_ok=False)
            context = {'session_id': self.session, 'data_source': 'simulation',
                       'captured_utc': datetime.now(timezone.utc).isoformat(),
                       'simulation': self.processes.simulation_config_payload(),
                       'camera': camera,
                       'calibration_status': 'uncalibrated_or_profile_specific',
                       'note': 'GUI configuration/launch request snapshot; not measured sensor rates or FCU parameter readback.'}
            for name, args in [('commit', ['rev-parse', 'HEAD']), ('status', ['status', '--porcelain'])]:
                result = subprocess.run(['git', '-C', str(APP_ROOT), *args], capture_output=True, text=True, timeout=8)
                context[name] = result.stdout.strip() if result.returncode == 0 else 'unavailable'
            diff = subprocess.run(['git', '-C', str(APP_ROOT), 'diff', 'HEAD', '--', 'uuv_mujoco/current',
                                   'rospkg/src/auv_vla_data_collector'], capture_output=True, timeout=15)
            (root / 'source.patch').write_bytes(diff.stdout)
            # Store configuration content, not just mutable paths.
            context['config_files'] = {}
            config_paths = [SIM_STACK_DIR / 'config/sim_profiles.json',
                         SIM_STACK_DIR / 'config/sensor_mounts_2026.json',
                         Path(context['simulation'].get('active_scene') or SIM_STACK_DIR / 'scenes/research_pool_slam_scene.xml')]
            optics_path = launched_camera.get('sensor_model_config')
            if optics_path:
                path = Path(optics_path).expanduser()
                config_paths.append(path if path.is_absolute() else SIM_STACK_DIR / path)
            for path in config_paths:
                if path.is_file():
                    context['config_files'][str(path)] = path.read_text()
            (root / 'context.json').write_text(json.dumps(context, ensure_ascii=False, indent=2))
            import yaml
            source = APP_ROOT / 'rospkg/src/auv_vla_data_collector'
            cfg = yaml.safe_load((source / 'config/collector.yaml').read_text())
            params = cfg['vla_data_collector']['ros__parameters']
            params.update(dataset_root=str(root / 'staging'), collection_kind='task_demonstration',
                          data_source='simulation', session_id=self.session,
                          provenance_file=str(root / 'context.json'), expected_mode=mode,
                          default_task=task, use_sim_time=True, pwm_span=int(RC_PWM_SPAN))
            params.update(imu_motion_topic='/mavros/imu/data_raw',
                          imu_motion_frame='fcu_link', imu_motion_convention='FLU')
            # Current front camera has a compatibility alias; use an observed source.
            topics = dict(self.node.get_topic_names_and_types())
            if '/imx219/camera0/image_raw/compressed' not in topics and 'sensor_msgs/msg/CompressedImage' in topics.get('/camera/camera/color/image_raw/compressed', []):
                params['ego_image_topic'] = '/camera/camera/color/image_raw/compressed'
            config = root / 'collector.yaml'
            config.write_text(yaml.safe_dump(cfg, allow_unicode=True))
            command = ' '.join(['env', shlex.quote('PYTHONPATH=' + str(source) + os.pathsep + os.environ.get('PYTHONPATH', '')),
                                'python3', '-m', 'kmu26_auv_vla_data_collector.collector',
                                '--ros-args', '--params-file', shlex.quote(str(config))])
            with (root / 'recorder.log').open('w') as log:
                self.process = subprocess.Popen(ros_bash_command(command, cwd=APP_ROOT), cwd=APP_ROOT,
                                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            self.message = '레코더 준비 중 · 입력 상태를 확인합니다.'
            self.status = {}
            self.received = 0.0
            return {'message': self.message, 'session_id': self.session}

    def command(self, action):
        if action == 'close':
            return self.close_session()
        with self.lock:
            state = self.payload()
            if not state['online'] or not state['owned']:
                raise ValueError('이 GUI에서 준비한 레코더의 최신 상태가 필요합니다.')
            if self.future is not None:
                raise ValueError('이전 요청 처리 중입니다.')
            if action == 'start':
                if state.get('active') or not state['ready']:
                    raise ValueError('녹화 준비가 안 됐습니다: ' + ', '.join(state.get('missing', [])))
                name, request = 'start_episode', Trigger.Request()
            elif action in {'success', 'failure', 'discard'}:
                if not state.get('active'):
                    raise ValueError('녹화 중인 시연이 없습니다.')
                if action == 'discard':
                    name, request = 'discard_episode', Trigger.Request()
                else:
                    name, request = 'stop_episode', SetBool.Request()
                    request.data = action == 'success'
            else:
                raise ValueError('알 수 없는 레코더 명령입니다.')
            client = self.clients[name]
            if not client.service_is_ready():
                raise ValueError('레코더 서비스가 연결되지 않았습니다.')
            self.command_at = time.monotonic()
            self.message = '레코더 요청 처리 중…'
            self.command_client = client
            self.future = client.call_async(request)
            self.future.add_done_callback(self._command_done)
            return {'message': self.message}

    def close_session(self):
        """Stop an idle owned collector so a new task can be configured."""
        with self.lock:
            state = self.payload()
            if not self.session:
                raise ValueError('이 GUI에서 준비한 레코더 세션이 없습니다.')
            if state['running'] and (not state['online'] or not state['owned'] or state.get('active') or state['busy']):
                raise ValueError('녹화를 저장/폐기하고 최신 대기 상태에서 세션을 종료하세요.')
            self.closed = True
        try:
            self.shutdown()
            if self.process is not None and self.process.poll() is None:
                raise ValueError('레코더 종료 대기 중입니다. 잠시 후 세션 종료를 다시 누르세요.')
            with self.lock:
                # A dead collector cannot answer an outstanding command. Drop
                # its request before allowing a new session to reuse clients.
                pending_command = self.future
                self.future = None
                command_client = getattr(self, 'command_client', None)
                self.command_client = None
                if pending_command is not None:
                    if command_client is not None:
                        command_client.remove_pending_request(pending_command)
                    pending_command.cancel()
                pending = self.status_future
                self.status_future = None
                if pending is not None:
                    self.clients['get_status'].remove_pending_request(pending)
                    pending.cancel()
                self.received = 0.0
                self.status = {}
                self.session = ''
                self.closed = False
                self.message = '세션 종료 완료. 지시문과 모드를 바꿔 새 세션을 준비할 수 있습니다.'
        finally:
            with self.lock:
                self.closed = False
        return {'message': self.message}

    def _command_done(self, future):
        with self.lock:
            if future is not self.future:
                return
            try:
                result = future.result()
                self.message = ('완료: ' if result.success else '거절: ') + result.message
            except Exception as exc:
                self.message = f'요청 결과 확인 실패: {exc}'
            finally:
                self.future = None
                self.command_client = None
                # Disable buttons until a post-command status is received.
                self.received = 0.0

    def shutdown(self):
        with self.lock:
            self.closed = True
            proc = self.process
        if proc is not None and proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
