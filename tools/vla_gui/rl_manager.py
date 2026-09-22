"""Own only the reinforcement-learning process launched from this GUI."""
import json,subprocess,threading,time,uuid,fcntl
from pathlib import Path


class RLManager:
    def __init__(self, root, python):
        self.root=Path(root);self.python=python;self.process=None;self.run=None
        self.lock=threading.Lock();self.stopping=False
        completed = sorted((self.root/'outputs/rl-training').glob('*/status.json'))
        if completed:
            try:
                latest = completed[-1]
                if json.loads(latest.read_text()).get('phase') in ('stopped', 'completed', 'error'):
                    self.run = latest.parent
            except (OSError, ValueError):
                pass

    def running(self):
        return self.process is not None and self.process.poll() is None

    def start(self, config, models):
        with self.lock:
            if self.running():raise ValueError('강화학습이 이미 실행 중입니다.')
            if any(models[k].get('ready') is False for k in config['models']):
                raise ValueError('선택한 모델의 25,000스텝 학습이 아직 완료되지 않았습니다.')
            training = self.root/'outputs/training-135-emphasis110-20260921/status.json'
            if training.exists() and any(r.get('status') == 'running' for r in json.loads(training.read_text()).get('runs', [])):
                raise ValueError('135개 모델 지도학습이 GPU를 사용 중입니다. 완료 후 강화학습을 시작하세요.')
            with (self.root/'outputs/rl-training.lock').open('a') as lock:
                try:fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
                except BlockingIOError:raise ValueError('별도 강화학습 실행이 진행 중입니다.')
            self.run=self.root/'outputs/rl-training'/ (time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:6])
            self.run.mkdir(parents=True)
            data={**config,'model_paths':{k:models[k]['path'] for k in config['models']}}
            (self.run/'config.json').write_text(json.dumps(data,ensure_ascii=False,indent=2))
            with (self.run/'run.log').open('w') as log:
                self.process=subprocess.Popen([self.python,'-u',str(self.root/'tools/rl_training/run.py'),str(self.run)],cwd=self.root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            self.stopping=False
            return self.snapshot()

    def stop(self):
        if self.running():
            self.process.terminate();self.stopping=True
        return self.snapshot()

    def close(self):
        self.stop()
        if self.process is not None:
            try:self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:pass

    def snapshot(self):
        data={'phase':'idle','updates':0,'episodes':[],'workers':[]}
        if self.run and (self.run/'status.json').exists():
            data=json.loads((self.run/'status.json').read_text())
        if self.process and not self.running() and data['phase'] not in ('completed','stopped','error'):
            data.update(phase='error',error='실행 프로세스 종료. run.log를 확인하세요.')
        return {**data,'running':self.running(),'stopping':self.stopping and self.running(),'output':str(self.run or '')}
