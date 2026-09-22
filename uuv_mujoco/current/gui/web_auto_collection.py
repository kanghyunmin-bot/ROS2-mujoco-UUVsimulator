"""GUI lifecycle for the bounded simulation collection worker."""
from __future__ import annotations

import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import threading
import time

from .config_paths import APP_ROOT


class WebAutoCollection:
    def __init__(self, controller):
        self.controller = controller
        self.lock = threading.RLock()
        self.process = None
        self.output = None
        self.token = ''
        self.stopping = False
        self.gui_url = ''
        self.log = None

    def running(self):
        return self.process is not None and self.process.poll() is None

    def authorize(self, payload):
        """Keep manual commands out of an active automatic episode."""
        with self.lock:
            if self.running() and payload.get('command') != 'auto_collection_stop':
                supplied = str(payload.get('_auto_collection_token', ''))
                if not secrets.compare_digest(supplied, self.token):
                    raise ValueError('자동 수집 중입니다. 자동 수집 중지 후 수동 조작하세요.')

    def start(self, episodes):
        with self.lock:
            if self.running():
                raise ValueError('자동 수집이 이미 실행 중입니다.')
            if isinstance(episodes, bool) or not str(episodes).isdigit() or not 1 <= int(episodes) <= 10000:
                raise ValueError('수집 횟수는 1~10000 사이의 정수여야 합니다.')
            controller = self.controller
            controller.recorder.require_configuration_unlocked()
            if (controller.replay.running() or controller.processes.pinger_homing_running()
                    or controller.processes.mission_running()
                    or getattr(controller, '_vla_control_prepared', False)
                    or controller._control_enabled):
                raise ValueError('수동 조종·재생·다른 자율제어를 종료한 뒤 시작하세요.')
            if controller.node.snapshot().armed:
                raise ValueError('현재 ARM 상태입니다. 해제 후 자동 수집을 시작하세요.')
            if not controller.processes.simulation_runtime_available():
                raise ValueError('시뮬레이터를 먼저 시작하세요. 자동 수집은 시뮬레이터 전용입니다.')
            executable = APP_ROOT / 'rospkg/install/auv_buoy_vision_control/lib/auv_buoy_vision_control/collection_fsm_node'
            python = APP_ROOT / '.venv-vla/bin/python'
            if not executable.is_file() or not python.is_file():
                raise ValueError('자동 수집용 FSM 빌드와 VLA Python 환경을 먼저 준비하세요.')
            if not self.gui_url:
                raise ValueError('GUI 서버 주소가 준비되지 않았습니다.')
            self.token = secrets.token_hex(24)
            self.output = APP_ROOT / 'outputs/auto-collection' / ('gui-' + time.strftime('%Y%m%d-%H%M%S') + '-' + secrets.token_hex(3))
            self.output.parent.mkdir(parents=True, exist_ok=True)
            if self.log:
                self.log.close()
            self.log = self.output.with_suffix('.log').open('w')
            env = dict(os.environ, UUV_AUTO_COLLECTION_TOKEN=self.token)
            self.process = subprocess.Popen(
                ['/usr/bin/python3', str(APP_ROOT / 'tools/auto_collection/run.py'),
                 '--gui_url', self.gui_url, '--episodes', str(episodes), '--output', str(self.output)],
                cwd=APP_ROOT, env=env, stdout=self.log, stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            self.stopping = False
            return {'message': '자동 수집 시작 · STABILIZE·ARM 후 녹화합니다.'}

    def stop(self):
        with self.lock:
            if self.running() and not self.stopping:
                # Only signal the worker. It owns child cleanup and failure save.
                self.process.send_signal(signal.SIGTERM)
                self.stopping = True
            return {'message': '자동 수집 중지 중 · 현재 녹화를 실패로 저장하고 ARM을 해제합니다.'}

    def payload(self):
        with self.lock:
            data = {}
            if self.output:
                try:
                    data = json.loads((self.output / 'status.json').read_text())
                except (OSError, ValueError):
                    pass
            running = self.running()
            if self.process is not None and not running and self.process.returncode != 0:
                data['phase'] = 'stopped' if self.stopping else 'error'
                if self.stopping:
                    data.pop('error', None)
                if not self.stopping:
                    data.setdefault('error', '실행 실패 · 자동 수집 로그를 확인하세요.')
            return {**data, 'running': running, 'stopping': running and self.stopping,
                    'output': str(self.output or '')}
