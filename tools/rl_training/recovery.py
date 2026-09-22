"""Isolate failed simulators without labelling infrastructure faults task failures."""
from pathlib import Path
import os


def physics_failure(log_directory):
    """Read only the latest simulator log; old crashes do not taint a restart."""
    logs = list(Path(log_directory).glob('mujoco_*.log'))
    if not logs:
        return None
    latest = max(logs, key=lambda p: p.stat().st_mtime_ns)
    with latest.open('rb') as f:
        f.seek(max(0, latest.stat().st_size-16384))
        tail = f.read().decode(errors='replace')
    lines = [line for line in tail.splitlines() if 'RuntimeError: Physics instability at ' in line]
    if not lines:
        return None
    return {'kind': 'physics_instability', 'detail': lines[-1], 'log': str(latest)}


class FailureIsolation:
    """Retire failed workers for this run; stop on the third infrastructure fault."""
    recoverable = {'physics_instability', 'simulation_tick_timeout', 'sensor_timeout', 'worker_exit'}

    def __init__(self, failure_limit=3):
        self.failure_limit = failure_limit
        self.retired = set()

    def discard(self, worker, kind):
        if kind not in self.recoverable:
            raise RuntimeError('학습 코드 오류: ' + kind)
        self.retired.add(worker)
        if len(self.retired) >= self.failure_limit:
            raise RuntimeError(f'여러 환경에서 오류 반복 · {self.failure_limit}개 환경 오류로 전체 중단')


def failure_kind(message):
    if message == 'Waiting failed: simulation tick':
        return 'simulation_tick_timeout'
    if message in ('Waiting failed: observation', 'Waiting failed: command boundary observation',
                   'Stale simulation reward telemetry'):
        return 'sensor_timeout'
    return 'unknown'


def find_resume(root, key, config):
    """Choose a saved learner with identical reward/time settings, or legacy seed."""
    import torch
    root = Path(root)
    ignored = {'models','model_paths','environments_per_model','episodes_per_environment',
               'resume_candidate','save_successes'}
    # New optional defaults must not invalidate older legacy checkpoints.
    added_defaults = dict(policy_groups={}, reward_version='legacy', evaluation_repeats=1,
                          ppo_learning_rate=.0003, ppo_clip=.2, ppo_kl_limit=.01,
                          ppo_drift_limit=.005, ppo_gamma=.99, ppo_lambda=.95, ppo_entropy=0.0)
    def signature_of(value):
        return {k:v for k,v in {**added_defaults, **value}.items() if k not in ignored}
    signature = signature_of(config)
    explicit = os.environ.get('UUV_RL_RESUME_CHECKPOINT')
    if explicit:
        path = Path(explicit).resolve(strict=True)
        checkpoint = torch.load(path, map_location='cpu', weights_only=False)
        prior = signature_of(checkpoint['config'])
        if (checkpoint.get('schema') != 'uuv.residual_rl.v1' or prior != signature
                or checkpoint['base_model'] != config['model_paths'][key]):
            raise ValueError('Explicit resume checkpoint is incompatible with this run')
        return path, checkpoint, True
    for path in sorted((root/'outputs/rl-training').glob(f'*/{key}/training-latest.pt'), reverse=True):
        checkpoint = torch.load(path, map_location='cpu', weights_only=False)
        prior = signature_of(checkpoint['config'])
        if (checkpoint.get('schema') == 'uuv.residual_rl.v1' and prior == signature
                and checkpoint['base_model'] == config['model_paths'][key]):
            return path, checkpoint, True
    path = root/'outputs/preserved-models/135-rl-update12/collection-00013.pt'
    return path, torch.load(path, map_location='cpu', weights_only=False), False
