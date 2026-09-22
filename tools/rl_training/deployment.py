"""Serve a frozen VLA together with its evaluated deterministic residual."""
from pathlib import Path

import numpy as np
import torch

from residual_policy import ResidualActorCritic


STATE_KEYS = ('prev_command', 'dvl_velocity', 'angular_velocity',
              'linear_acceleration', 'attitude', 'depth', 'altitude', 'validity')


class ResidualDeployment:
    def __init__(self, checkpoint, base_model, repository):
        checkpoint = Path(checkpoint).resolve()
        saved = torch.load(checkpoint, map_location='cpu', weights_only=False)
        if saved.get('schema') not in ('uuv.residual_rl.v1', 'uuv.residual_rl.v2'):
            raise ValueError('Unsupported residual checkpoint schema')
        expected = (Path(repository) / saved['base_model']).resolve()
        if expected != Path(base_model).resolve():
            raise ValueError('Residual checkpoint belongs to a different base VLA')
        limit = float(saved['residual_limit'])
        if not np.isfinite(limit) or not 0 < limit <= 1:
            raise ValueError('Invalid residual action limit')
        self.policy = ResidualActorCritic(residual_limit=limit, learned_brake=saved.get('learned_brake', False))
        self.policy.load_state_dict(saved['policy'], strict=True)
        if not all(torch.isfinite(p).all() for p in self.policy.parameters()):
            raise ValueError('Nonfinite residual checkpoint')
        self.policy.eval().requires_grad_(False)
        self.metadata = {'checkpoint': str(checkpoint), 'updates': saved['updates'],
                         'lineage_updates': saved.get('lineage_updates'),
                         'residual_limit': limit, 'deterministic': True}

    def apply(self, observations, motion):
        """Use the first base action, as in rollouts; hold until a new response.

        The legacy ROS client indexes a 16-action horizon by inference age. Repeat
        this evaluated one-step command across that horizon rather than applying
        a residual trained only on action zero to unobserved future states.
        Existing client timeouts and command limiting remain in force.
        """
        motion = np.asarray(motion)
        if motion.shape not in ((16, 4), (1, 16, 4)) or not np.isfinite(motion).all():
            raise ValueError('Expected a finite, single-observation motion chunk')
        state = np.concatenate([np.asarray(observations['state.' + k]).reshape(-1)
                                for k in STATE_KEYS]).astype(np.float32)
        if state.shape != (23,) or not np.isfinite(state).all():
            raise ValueError('Residual deployment needs finite 23-dimensional state')
        base = torch.tensor(motion.reshape(-1, 4)[0].clip(-1, 1), dtype=torch.float32)[None]
        action, _, _, _ = self.policy.act(torch.from_numpy(state)[None], base, deterministic=True)
        result = np.broadcast_to(action.numpy(), (16, 4)).copy()
        return result.reshape(motion.shape)
