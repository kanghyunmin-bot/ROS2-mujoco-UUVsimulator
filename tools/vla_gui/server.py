"""Local, simulation-only VLA process manager for the web GUI."""
import json
import os
from pathlib import Path
import shlex
import signal
import socket
import subprocess
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.request import Request, urlopen
from urllib.error import HTTPError

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'outputs/vla-gui'
OUT.mkdir(parents=True, exist_ok=True)
from runtime_config import load_models, DEFAULT_TASK, U0_ROOT, HOST_PYTHON, CONTAINER_PYTHON
PREFIX = 'source /opt/ros/humble/setup.bash; source /workspace/rospkg/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42; '
MODELS, DEFAULT_MODEL = load_models()


def request(path, payload=None):
    data = None if payload is None else json.dumps(payload).encode()
    req = Request('http://127.0.0.1:8878' + path, data=data,
                  headers={'Content-Type': 'application/json'})
    try:
        with urlopen(req, timeout=5) as response:
            return json.load(response)
    except HTTPError as error:
        detail = json.loads(error.read()).get('error', str(error))
        raise RuntimeError(detail) from error


def command(name, **kwargs):
    return request('/api/command', {'command': name, **kwargs})


def container():
    names = subprocess.check_output(['docker', 'ps', '--filter',
        'label=com.docker.compose.service=uuv-dev', '--format', '{{.Names}}'], text=True).split()
    matches = []
    for name in names:
        info = json.loads(subprocess.check_output(['docker', 'inspect', name]))[0]
        if any(m.get('Source') == str(ROOT) and m.get('Destination') == '/workspace'
               for m in info['Mounts']) and info['HostConfig']['NetworkMode'] == 'host':
            matches.append(name)
    if len(matches) != 1:
        raise RuntimeError('이 프로젝트의 실행 중인 개발 컨테이너가 정확히 하나 필요합니다.')
    return matches[0]


class Manager:
    def __init__(self):
        self.lock = threading.Lock()
        self.cancel = threading.Event()
        self.children = {}
        self.state = {'phase': 'idle', 'message': '모델을 선택하고 준비를 누르세요.',
                      'model': None, 'error': None, 'result': None, 'busy': False}
        self.name = None

    def update(self, **values):
        with self.lock:
            self.state.update(values)

    def snapshot(self):
        with self.lock:
            state = dict(self.state)
        if state['phase'] == 'ready' and (self.children.get('model') is None or
                                         self.children['model'].poll() is not None):
            self.update(phase='error', error='모델 서버가 종료됐습니다. 다시 준비하세요.')
            state.update(phase='error', error='모델 서버가 종료됐습니다. 다시 준비하세요.')
        return {**state, 'models': MODELS, 'default_model': DEFAULT_MODEL, 'default_task': DEFAULT_TASK, 'service': 'uuv-vla-gui', 'workspace': str(ROOT)}

    def spawn(self, key, args, env=None):
        log = OUT / f'{key}.log'
        with log.open('w') as stream:
            process = subprocess.Popen(args, cwd=ROOT, env=env, stdout=stream,
                                       stderr=subprocess.STDOUT, start_new_session=True)
        self.children[key] = process
        return process

    def wait(self, process, key, timeout):
        deadline = time.monotonic() + timeout
        while process.poll() is None:
            if self.cancel.wait(.15):
                raise RuntimeError('사용자가 중지했습니다.')
            if time.monotonic() > deadline:
                raise RuntimeError(f'{key} 시간 초과')
        if process.returncode:
            raise RuntimeError(f'{key} 실패: ' + (OUT / f'{key}.log').read_text(errors='replace')[-1400:])

    def gpu_command(self, args):
        if HOST_PYTHON:
            return [HOST_PYTHON, '-u', *args]
        translated = []
        for value in args:
            path = Path(value)
            if path.is_absolute():
                try:
                    value = '/workspace/' + str(path.relative_to(ROOT))
                except ValueError:
                    raise ValueError('Container VLA paths must be inside the checkout')
            translated.append(value)
        check = subprocess.run(['docker', 'exec', self.name, 'test', '-x', CONTAINER_PYTHON], timeout=5)
        if check.returncode:
            raise RuntimeError('VLA Python 환경이 없습니다. ./release.sh install-vla를 실행하세요.')
        return ['docker', 'exec', '-w', '/workspace', '-e', 'OMP_NUM_THREADS=4',
                '-e', 'TOKENIZERS_PARALLELISM=false', '-e', 'NO_ALBUMENTATIONS_UPDATE=1',
                self.name, CONTAINER_PYTHON, '-u', *translated]

    def docker(self, script):
        return ['docker', 'exec', self.name, 'bash', '-lc', PREFIX + script]

    def stop_child(self, key):
        if key == 'model' and self.name and not HOST_PYTHON:
            subprocess.run(['docker', 'exec', self.name, 'pkill', '-INT', '-f',
                            '^/workspace/.venv-vla/bin/python -u rospkg/src/auv_vla_data_collector/tools/serve_transfer.py'],
                           timeout=8, capture_output=True)
        process = self.children.pop(key, None)
        if process and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)

    def stop_policy(self):
        # docker exec client signals do not reliably reach its container child.
        if self.name:
            script = "import os,signal; from pathlib import Path\nfor p in Path('/proc').glob('[0-9]*'):\n try:\n  a=(p/'cmdline').read_bytes().split(b'\\0')\n  policy=b'/workspace/rospkg/install/kmu26_auv_vla_policy/lib/kmu26_auv_vla_policy/policy' in a\n  trial=b'/workspace/tools/vla_gui/run_rollout.py' in a and any(x.startswith(b'/workspace/outputs/vla-gui/') for x in a)\n  if policy or trial: os.kill(int(p.name),signal.SIGINT)\n except (OSError,ValueError): pass"
            subprocess.run(['docker', 'exec', self.name, 'python3', '-c', script], timeout=8, capture_output=True)
        self.stop_child('policy')

    def release(self):
        self.stop_child('trial')
        self.stop_policy()
        status = request('/api/status')
        if self.name and status['telemetry'].get('connected'):
            completed = subprocess.run(self.docker(
                'timeout 6 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: false}"'),
                timeout=9, capture_output=True, text=True)
            if 'success=True' not in completed.stdout:
                raise RuntimeError('DISARM 확인 실패. GUI의 DISARM 상태를 확인하세요.')
        for _ in range(20):
            try:
                command('vla_restore_manual')
                return
            except Exception:
                time.sleep(.2)
        raise RuntimeError('수동 제어 복귀 실패: 외부 RC 발행자를 확인하세요.')

    def stop_previous_model(self):
        """Stop only this repository's serving entry point, never an arbitrary port owner."""
        expected = ROOT / 'rospkg/src/auv_vla_data_collector/tools/serve_transfer.py'
        for directory in Path('/proc').glob('[0-9]*'):
            try:
                args = (directory / 'cmdline').read_bytes().decode().split('\0')
                cwd = (directory / 'cwd').resolve()
                if not any((cwd / arg).resolve() == expected for arg in args[1:4] if arg):
                    continue
                pid = int(directory.name)
                started = (directory / 'stat').read_text().split()[21]
                os.kill(pid, signal.SIGINT)
                for _ in range(40):
                    if not directory.exists():
                        break
                    if (directory / 'stat').read_text().split()[21] != started:
                        break
                    time.sleep(.1)
            except (OSError, ValueError, UnicodeError):
                continue

    def prepare(self, model):
        if model not in MODELS:
            raise ValueError('알 수 없는 모델')
        if not U0_ROOT.is_dir():
            raise RuntimeError('VLA 환경이 없습니다. ./release.sh install-vla를 먼저 실행하세요. 데이터 수집은 모델 없이 가능합니다.')
        self.name = container()
        self.update(phase='preparing', message='정책 정리 및 실행 환경 확인', model=None, result=None)
        self.release()
        self.stop_child('model')
        self.stop_previous_model()
        # Never silently connect to an old model or kill an unrelated service.
        with socket.socket() as sock:
            if sock.connect_ex(('127.0.0.1', 8000)) == 0:
                raise RuntimeError('8000 포트에 별도 모델 서버가 실행 중입니다. 해당 서버 터미널에서 Ctrl+C 후 준비하세요.')
        deps = self.spawn('dependencies', self.docker(
            'python3 -m pip install -r /workspace/rospkg/src/kmu26_auv_vla_policy/requirements-ros.txt'))
        self.wait(deps, 'dependencies', 180)
        env = dict(os.environ, OMP_NUM_THREADS='4', TOKENIZERS_PARALLELISM='false', NO_ALBUMENTATIONS_UPDATE='1')
        env['PYTHONPATH'] = str(U0_ROOT) + ':' + str(ROOT / 'rospkg/src/auv_vla_data_collector')
        self.update(message='선택 모델 로딩 중…')
        args = ['rospkg/src/auv_vla_data_collector/tools/serve_transfer.py',
                str(U0_ROOT), MODELS[model]['path'], '--port', '8000']
        process = self.spawn('model', self.gpu_command(args), env)
        for _ in range(180):
            if self.cancel.wait(.5):
                raise RuntimeError('사용자가 중지했습니다.')
            if process.poll() is not None:
                raise RuntimeError((OUT / 'model.log').read_text(errors='replace')[-1200:])
            try:
                with urlopen('http://127.0.0.1:8000/health', timeout=1) as response:
                    if response.status == 200:
                        break
            except OSError:
                continue
        else:
            raise RuntimeError('모델 서버 준비 시간 초과')
        self.update(message='모델 예열 및 행동 형식·응답 시간 검사 중…')
        warm = self.spawn('warmup', self.gpu_command(['tools/vla_gui/warmup.py']), env)
        self.wait(warm, 'warmup', 180)
        if process.poll() is not None:
            raise RuntimeError('예열 도중 모델 서버 종료')
        timing = json.loads((OUT / 'warmup.log').read_text().strip().splitlines()[-1])
        self.update(phase='ready', model=model, message=f"준비 완료 · 예열 응답 {timing['warmup_ms']}ms", **timing)

    def run(self, seconds, task):
        if self.snapshot()['phase'] != 'ready':
            raise RuntimeError('모델 준비가 먼저 필요합니다.')
        seconds = float(seconds)
        if not 1 <= seconds <= 90 or not task.strip() or len(task) > 1000:
            raise ValueError('실행 시간은 1~90초, 지시문은 1~1000자여야 합니다.')
        if container() != self.name:
            raise RuntimeError('컨테이너가 바뀌었습니다. 모델을 다시 준비하세요.')
        runtime = subprocess.run(['docker', 'exec', self.name, 'pgrep', '-f',
            '^/workspace/.venv/bin/python run_uuv_mujoco.py'], capture_output=True, timeout=5)
        if runtime.returncode != 0:
            raise RuntimeError('MuJoCo 시뮬레이션을 먼저 시작하세요. 실물에는 실행하지 않습니다.')
        self.update(phase='running', message='STABILIZE · ARM 상태 확인 중', result=None)
        try:
            command('mode', mode='STABILIZE')
            command('arm', value=True)
            for _ in range(80):
                if self.cancel.wait(.2):
                    raise RuntimeError('사용자가 중지했습니다.')
                telemetry = request('/api/status')['telemetry']
                if (telemetry.get('connected') and telemetry.get('armed') and
                    telemetry.get('mode') == 'STABILIZE' and
                    (telemetry.get('state_age_s') or 0) < 1):
                    break
            else:
                raise RuntimeError('ARM/STABILIZE 확인 실패. GUI 상태를 확인하세요.')
            command('vla_prepare_control')
            self.spawn('policy', self.docker('ros2 launch kmu26_auv_vla_policy sim_policy.launch.py config:=/workspace/tools/vla_gui/policy-live.yaml'))
            run_id = self.state['model'] + '-' + uuid.uuid4().hex[:10]
            output = OUT / (run_id + '.json')
            self.update(message=f'{seconds:g}초 임무 실행 중', started_at=time.time(), seconds=seconds)
            trial = self.spawn('trial', self.docker('python3 /workspace/tools/vla_gui/run_rollout.py --seconds ' + str(seconds) + ' --task ' + shlex.quote(task) + ' --output ' + shlex.quote('/workspace/' + str(output.relative_to(ROOT)))))
            self.wait(trial, 'trial', 150)
            result = json.loads(output.read_text())
            result = {k: v for k, v in result.items() if k != 'telemetry'}
            self.update(result=result, result_file=str(output.relative_to(ROOT)))
            if result['outcome'] in ('error', 'no_policy_output'):
                lines = (OUT / 'policy.log').read_text(errors='replace').splitlines()
                diagnostic = '\n'.join(line for line in lines if any(word in line for word in ('WARN', 'ERROR', 'Traceback')))
                raise RuntimeError(str(result) + '\n' + diagnostic[-1800:])
        finally:
            self.release()
        self.update(phase='ready', message='시험 종료 · DISARM 및 수동 복귀 완료')

    def submit(self, action, data):
        if action not in {'prepare', 'run', 'stop', 'cleanup'}:
            raise ValueError('알 수 없는 명령')
        if action == 'stop':
            self.cancel.set()
            if not self.state['busy']:
                self.submit('cleanup', {})
            return
        with self.lock:
            if self.state['busy']:
                raise ValueError('작업 중입니다. 중지 후 다시 시도하세요.')
            self.state.update(busy=True, error=None)
        self.cancel.clear()

        def worker():
            try:
                if action == 'prepare':
                    self.prepare(data['model'])
                elif action == 'run':
                    if data.get('model') != self.state['model']:
                        raise ValueError('선택한 모델과 준비된 모델이 다릅니다. 다시 준비하세요.')
                    self.run(data.get('seconds', 50), data.get('task', ''))
                elif action == 'cleanup':
                    self.release()
                    self.stop_child('model')
                    self.update(phase='idle', model=None, message='종료 완료')
                else:
                    raise ValueError('알 수 없는 명령')
            except Exception as error:
                self.update(phase='error', error=str(error), message='작업 중단')
                try:
                    self.release()
                except Exception as cleanup_error:
                    self.update(error=str(error) + '\n' + str(cleanup_error))
                for key in ('warmup', 'dependencies', 'model'):
                    self.stop_child(key)
                self.update(model=None)
                if self.cancel.is_set() and self.state['error'] == str(error):
                    self.update(phase='idle', error=None, message='중지 완료 · DISARM 및 수동 복귀 완료')
            finally:
                self.update(busy=False)
        threading.Thread(target=worker, daemon=True).start()


manager = Manager()


class Handler(BaseHTTPRequestHandler):
    def send(self, value, code=200):
        data = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == '/api/status':
            self.send(manager.snapshot())
        elif self.path == '/':
            data = Path(__file__).with_name('index.html').read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send({'error': 'Not found'}, 404)

    def do_POST(self):
        if self.headers.get('Origin') != 'http://127.0.0.1:8882' or self.path != '/api/command':
            self.send({'error': 'Invalid origin or endpoint'}, 403)
            return
        try:
            size = int(self.headers.get('Content-Length', 0))
            if not 0 < size <= 8192:
                raise ValueError('Invalid request size')
            data = json.loads(self.rfile.read(size))
            manager.submit(data['action'], data)
            self.send({'ok': True})
        except (ValueError, KeyError) as error:
            self.send({'error': str(error)}, 400)

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    def terminate(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, terminate)
    server = ThreadingHTTPServer(('127.0.0.1', 8882), Handler)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        manager.cancel.set()
        if manager.name:
            try:
                manager.release()
            except Exception:
                manager.stop_policy()
        for key in list(manager.children):
            manager.stop_child(key)
