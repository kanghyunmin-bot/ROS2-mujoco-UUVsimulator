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

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from rl_config import defaults, validate
from rl_manager import RLManager

ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / 'outputs/vla-gui'
OUT.mkdir(parents=True, exist_ok=True)
PYTHON = '/home/khm/miniforge3/envs/gr00t/bin/python'
PREFIX = 'source /opt/ros/humble/setup.bash; source /workspace/rospkg/install/setup.bash; export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID=42; '
MODELS = {f'{w}-{s}': {'label': f'{label} · 추가 {s // 10000}만',
          'path': f'outputs/vla-sixhour-20260920/weight-{w}/model/checkpoint-{s}'}
          for w, label in [(1, '기본 1배'), (2, '중간 강조'), (3, '강한 강조')]
          for s in (20000, 40000, 60000)}


DEFAULT_TASK = 'Approach the yellow buoy, align the fixed fork, and detach the buoy.'
FORK_RUN = ROOT / 'outputs/vla-fork-medium-10k-20260920'
DEFAULT_MODEL = '3-40000'
if (FORK_RUN / 'task_instruction.txt').is_file():
    fork_task = DEFAULT_TASK + ' ' + (FORK_RUN / 'task_instruction.txt').read_text().strip()
    for step in (10000, 5000):
        checkpoint = FORK_RUN / 'model' / f'checkpoint-{step}'
        if (checkpoint / 'model.safetensors.index.json').is_file() and (checkpoint / 'trainer_state.json').is_file():
            saved_step = json.loads((checkpoint / 'trainer_state.json').read_text())['global_step']
            if saved_step != step:
                continue
            key = f'fork-medium-{step}'
            MODELS[key] = {
                'label': f'포크 정렬 · 중간 2배 · 추가 {step:,}스텝' + (' (최신)' if step == 10000 else ''),
                'path': str(checkpoint.relative_to(ROOT)),
                'task': fork_task,
            }
            if step == 10000:
                DEFAULT_MODEL = key



PLAIN_RUN = ROOT / 'outputs/vla-experiments/camera-df7a8516ac67/models/plain-18-5000-20260920'
plain_checkpoint = PLAIN_RUN / 'model' / 'checkpoint-5000'
if (plain_checkpoint / 'trainer_state.json').is_file() and (plain_checkpoint / 'model.safetensors.index.json').is_file():
    if json.loads((plain_checkpoint / 'trainer_state.json').read_text()).get('global_step') == 5000:
        for model in MODELS.values():
            model['label'] = model['label'].replace(' (최신)', '')
        MODELS['plain-18-5000'] = {
            'label': '현재 손 카메라 · 18개 · 강조 없음 · 5,000스텝 (최신)',
            'path': str(plain_checkpoint.relative_to(ROOT)),
            'task': DEFAULT_TASK,
        }
        DEFAULT_MODEL = 'plain-18-5000'


SUCCESS_RUN = ROOT / 'outputs/training-59-sequential-20260921'
for key, folder, steps, label in (
    ('success-59-10000', 'current59', 10000, '현재 성공 59개 · 강조 없음 · 10,000스텝'),
    ('success-mixed-84-15000', 'mixed84', 15000, '성공 84개 · 현재 80% / 이전 20% · 15,000스텝'),
):
    checkpoint = SUCCESS_RUN / folder / 'model'
    state_file = checkpoint / 'trainer_state.json'
    index_file = checkpoint / 'model.safetensors.index.json'
    if state_file.is_file() and index_file.is_file():
        state = json.loads(state_file.read_text())
        shards = set(json.loads(index_file.read_text())['weight_map'].values())
        if state.get('global_step') == steps and all((checkpoint / shard).is_file() for shard in shards):
            for model in MODELS.values():
                model['label'] = model['label'].replace(' (최신)', '')
            MODELS[key] = {'label': label, 'path': str(checkpoint.relative_to(ROOT)), 'task': DEFAULT_TASK}
if 'success-59-10000' in MODELS:
    DEFAULT_MODEL = 'success-59-10000'



NEW_RUN = ROOT / 'outputs/training-135-emphasis110-20260921'
NEW_MODEL_KEY = 'success-135-110-25000'
PPO_MODEL_KEY = 'success-135-ppo15'
PPO_CHECKPOINT = ROOT / 'outputs/rl-training/20260922-025444-1e76ee/success-135-110-25000/collection-00046.pt'

def refresh_new_model():
    global DEFAULT_MODEL
    for key in list(MODELS):
        if key != NEW_MODEL_KEY and not (ROOT / MODELS[key]['path'] / 'model.safetensors.index.json').is_file():
            del MODELS[key]
    DEFAULT_MODEL = NEW_MODEL_KEY
    path = NEW_RUN / 'all135/model'
    ready = False
    try:
        run = json.loads((NEW_RUN / 'status.json').read_text())['runs'][0]
        saved = json.loads((path / 'trainer_state.json').read_text())
        index = json.loads((path / 'model.safetensors.index.json').read_text())
        ready = (run['status'] == 'completed' and saved['global_step'] == 25000
                 and all((path / shard).is_file() for shard in set(index['weight_map'].values())))
    except (OSError, ValueError, KeyError, IndexError):
        pass
    MODELS[NEW_MODEL_KEY] = {'label': '성공 135개 · 검토 구간 1.1배 · 25,000스텝' + ('' if ready else ' (학습 준비/진행 중)'),
                           'path': str(path.relative_to(ROOT)), 'task': DEFAULT_TASK, 'ready': ready}
    if ready and PPO_CHECKPOINT.is_file():
        MODELS[PPO_MODEL_KEY] = {
            'label': '성공 135개 + PPO 15회 · 평가 분리 5/6',
            'path': str(path.relative_to(ROOT)), 'task': DEFAULT_TASK, 'ready': True,
            'residual_checkpoint': str(PPO_CHECKPOINT.relative_to(ROOT)),
            'description': '일반 분리 5/6 · 손 시야 0/6 · 포크 영역 0/6. 소수 평가 후보이며 성능 보장은 아닙니다.',
        }
        DEFAULT_MODEL = PPO_MODEL_KEY
    else:
        MODELS.pop(PPO_MODEL_KEY, None)

refresh_new_model()

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

    def docker(self, script):
        return ['docker', 'exec', self.name, 'bash', '-lc', PREFIX + script]

    def stop_child(self, key):
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
            script = "import os,signal; from pathlib import Path\nfor p in Path('/proc').glob('[0-9]*'):\n try:\n  a=(p/'cmdline').read_bytes().split(b'\\0')\n  policy=b'/workspace/rospkg/install/kmu26_auv_vla_policy/lib/kmu26_auv_vla_policy/policy' in a\n  trial=b'/workspace/outputs/vla-first-training-20260920/run_rollout.py' in a and any(x.startswith(b'/workspace/outputs/vla-gui/') for x in a)\n  if policy or trial: os.kill(int(p.name),signal.SIGINT)\n except (OSError,ValueError): pass"
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
        env = dict(os.environ, HF_HUB_OFFLINE='1', TRANSFORMERS_OFFLINE='1', OMP_NUM_THREADS='4',
                   TOKENIZERS_PARALLELISM='false', NO_ALBUMENTATIONS_UPDATE='1')
        env['PYTHONPATH'] = str(ROOT / 'outputs/vla-transfer-audit-20260910/upstream/auv_vla') + ':' + str(ROOT / 'rospkg/src/auv_vla_data_collector')
        self.update(message='선택 모델 로딩 중…')
        serve_args = [PYTHON, '-u', 'rospkg/src/auv_vla_data_collector/tools/serve_transfer.py',
            'outputs/vla-transfer-audit-20260910/upstream/auv_vla', MODELS[model]['path'], '--port', '8000']
        if MODELS[model].get('residual_checkpoint'):
            serve_args += ['--residual_checkpoint', MODELS[model]['residual_checkpoint']]
        process = self.spawn('model', serve_args, env)
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
        warm = self.spawn('warmup', [PYTHON, str(Path(__file__).with_name('warmup.py'))], env)
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
            self.spawn('policy', self.docker('ros2 launch kmu26_auv_vla_policy sim_policy.launch.py config:=/workspace/outputs/vla-first-training-20260920/policy-live.yaml'))
            run_id = self.state['model'] + '-' + uuid.uuid4().hex[:10]
            output = OUT / (run_id + '.json')
            self.update(message=f'{seconds:g}초 임무 실행 중', started_at=time.time(), seconds=seconds)
            trial = self.spawn('trial', self.docker('python3 /workspace/outputs/vla-first-training-20260920/run_rollout.py --seconds ' + str(seconds) + ' --task ' + shlex.quote(task) + ' --output ' + shlex.quote('/workspace/' + str(output.relative_to(ROOT)))))
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
        if action in ('prepare', 'run') and rl_manager.running():
            raise ValueError('강화학습 중입니다. 중지 후 모델을 실행하세요.')
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
rl_manager = RLManager(ROOT, PYTHON)


class Handler(BaseHTTPRequestHandler):
    def send(self, value, code=200):
        data = json.dumps(value, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        refresh_new_model()
        if self.path == '/api/rl/status':
            self.send(rl_manager.snapshot())
        elif self.path == '/api/rl/config':
            path = OUT / 'rl_config.json'
            self.send({'config': {**defaults(), **json.loads(path.read_text())} if path.exists() else defaults(), 'models': MODELS, 'runner_ready': True})
        elif self.path == '/api/status':
            self.send(manager.snapshot())
        elif self.path in ('/', '/rl'):
            data = Path(__file__).with_name('rl.html' if self.path == '/rl' else 'index.html').read_bytes()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send({'error': 'Not found'}, 404)

    def do_POST(self):
        refresh_new_model()
        if self.path in ('/api/rl/start', '/api/rl/stop'):
            if self.headers.get('Origin') != 'http://127.0.0.1:8882':
                self.send({'error': 'Invalid origin'}, 403)
                return
            try:
                if self.path.endswith('/stop'):
                    self.send(rl_manager.stop())
                else:
                    if manager.state['busy'] or manager.state['model'] is not None:
                        raise ValueError('VLA 실행 패널에서 기존 모델을 종료한 뒤 시작하세요.')
                    path = OUT / 'rl_config.json'
                    config = validate(json.loads(path.read_text()) if path.exists() else defaults(), MODELS)
                    self.send(rl_manager.start(config, MODELS))
            except (ValueError, OSError) as error:
                self.send({'error': str(error)}, 400)
            return
        if self.path == '/api/rl/config':
            if self.headers.get('Origin') != 'http://127.0.0.1:8882':
                self.send({'error': 'Invalid origin'}, 403)
                return
            try:
                size = int(self.headers.get('Content-Length', 0))
                if not 0 < size <= 8192:
                    raise ValueError('Invalid request size')
                config = validate(json.loads(self.rfile.read(size)), MODELS)
                path = OUT / 'rl_config.json'
                temp = path.with_suffix('.tmp')
                temp.write_text(json.dumps(config, ensure_ascii=False, indent=2))
                temp.replace(path)
                self.send({'ok': True, 'config': config})
            except (ValueError, TypeError, KeyError) as error:
                self.send({'error': str(error)}, 400)
            return
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
        rl_manager.close()
        manager.cancel.set()
        if manager.name:
            try:
                manager.release()
            except Exception:
                manager.stop_policy()
        for key in list(manager.children):
            manager.stop_child(key)
