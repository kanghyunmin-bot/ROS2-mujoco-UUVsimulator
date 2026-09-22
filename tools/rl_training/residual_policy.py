"""On-policy actor/critic updates for a frozen-VLA residual controller.

Inputs contain only deployable sensor state and the frozen base action. Simulation
truth belongs exclusively in the reward stream. A checkpoint is a residual policy,
not a replacement for the original VLA checkpoint.
"""
from pathlib import Path
import copy
import math
import torch
from torch import nn
from torch.distributions import Normal


class ResidualActorCritic(nn.Module):
    def __init__(self, state_dim=23, action_dim=4, residual_limit=.15, learned_brake=False):
        super().__init__()
        self.residual_limit = residual_limit
        self.learned_brake = learned_brake
        self.features = nn.Sequential(nn.Linear(state_dim+action_dim, 64), nn.Tanh(),
                                      nn.Linear(64, 64), nn.Tanh())
        latent_dim = action_dim + int(learned_brake)
        self.actor = nn.Linear(64, latent_dim)
        self.critic = nn.Linear(64, 1)
        self.log_std = nn.Parameter(torch.full((latent_dim,), -2.))
        nn.init.zeros_(self.actor.weight)
        nn.init.zeros_(self.actor.bias)

    def distribution(self, state, base_action):
        x = self.features(torch.cat((state, base_action), dim=-1))
        return Normal(self.actor(x), self.log_std.clamp(-5., 0.).exp()), self.critic(x).squeeze(-1)

    def compose(self, raw, base_action):
        """Compose bounded RC commands; learned braking attenuates forward RC.

        No simulator geometry or stage switch is consulted at inference. Zero
        brake mean preserves the old deterministic action exactly on migration.
        """
        action = (base_action+self.residual_limit*torch.tanh(raw[..., :4])).clamp(-1., 1.)
        if self.learned_brake:
            action = action.clone()
            action[..., 0] = action[..., 0] * (1-torch.tanh(raw[..., 4]).clamp_min(0.0))
        return action

    def seed_legacy(self, saved):
        """Transfer actor/features, reset critic for the new reward; no Adam reuse."""
        target = self.state_dict()
        for name in ('features.0.weight', 'features.0.bias', 'features.2.weight', 'features.2.bias'):
            target[name] = saved[name].clone()
        for name in ('actor.weight', 'actor.bias', 'log_std'):
            if saved[name].shape[0] != 4:
                raise ValueError('Precision warm start expects a four-action legacy policy')
            target[name][:4] = saved[name]
        target['critic.weight'].zero_()
        target['critic.bias'].zero_()
        self.load_state_dict(target)

    @torch.no_grad()
    def act(self, state, base_action, deterministic=False):
        distribution, value = self.distribution(state, base_action)
        raw = distribution.mean if deterministic else distribution.sample()
        # Score the latent Gaussian; the fixed tanh and clipping transformation
        # is identical during collection and update.
        action = self.compose(raw, base_action)
        return action, raw, distribution.log_prob(raw).sum(-1), value


def advantages(rewards, values, next_values, terminals, boundaries, durations,
               gamma=.99, lam=.95, nominal_dt=.1):
    """GAE over [time, environment], handling terminal and truncated rollouts.

    durations is measured simulation time [s], not wall time. Terminal means a
    task termination; boundaries also includes a nonterminal rollout cutoff.
    next_values must describe the final pre-reset observation.
    """
    discount = gamma ** (durations/nominal_dt)
    trace = lam ** (durations/nominal_dt)
    result = torch.zeros_like(rewards)
    carry = torch.zeros_like(rewards[0])
    for t in reversed(range(len(rewards))):
        delta = rewards[t]+discount[t]*(~terminals[t])*next_values[t]-values[t]
        carry = delta+discount[t]*trace[t]*(~boundaries[t])*carry
        result[t] = carry
    return result, result+values


def update(policy, optimizer, batch, *, epochs=4, clip=.2, kl_limit=.01,
           drift_limit=.005, entropy_coefficient=0.0):
    """Consume an on-policy batch; caller must freeze collection during updates."""
    required = ('state', 'base_action', 'raw_action', 'old_log_prob', 'advantage', 'return')
    if any(k not in batch or not torch.isfinite(batch[k]).all() for k in required):
        raise ValueError('Missing or nonfinite rollout data')
    advantage = batch['advantage'].detach()
    if len(advantage)>1 and advantage.std(unbiased=False)>1e-8:
        advantage = (advantage-advantage.mean())/(advantage.std(unbiased=False)+1e-8)
    with torch.no_grad():
        initial_dist, _ = policy.distribution(batch['state'], batch['base_action'])
        initial_mean = initial_dist.mean.clone()
        initial_std = initial_dist.stddev.clone()

    def diagnostics():
        with torch.no_grad():
            dist, _ = policy.distribution(batch['state'], batch['base_action'])
            kl = torch.distributions.kl_divergence(
                Normal(initial_mean, initial_std), dist).sum(-1).mean()
            drift = (policy.compose(dist.mean, batch['base_action'])
                     -policy.compose(initial_mean, batch['base_action'])).abs().max()
            ratio = (dist.log_prob(batch['raw_action']).sum(-1)-batch['old_log_prob']).exp()
            return dict(kl=float(kl), max_correction_change=float(drift),
                        clip_fraction=float(((ratio-1).abs()>clip).float().mean()),
                        entropy=float(dist.entropy().sum(-1).mean()))

    metrics = {}
    accepted_epochs = backtracks = 0
    original_lrs = [group['lr'] for group in optimizer.param_groups]
    try:
        for _ in range(epochs):
            # A rejected step must not undo earlier safe epochs. Retry from the
            # exact pre-step policy AND Adam moments, reducing only this step.
            prior_policy = copy.deepcopy(policy.state_dict())
            prior_optimizer = copy.deepcopy(optimizer.state_dict())
            distribution, value = policy.distribution(batch['state'], batch['base_action'])
            log_ratio = distribution.log_prob(batch['raw_action']).sum(-1)-batch['old_log_prob'].detach()
            ratio = log_ratio.exp()
            policy_loss = -torch.minimum(ratio*advantage, ratio.clamp(1-clip,1+clip)*advantage).mean()
            value_loss = .5*(value-batch['return'].detach()).square().mean()
            loss = policy_loss+value_loss-entropy_coefficient*distribution.entropy().sum(-1).mean()
            optimizer.zero_grad(); loss.backward()
            norm = nn.utils.clip_grad_norm_(policy.parameters(), .5)
            if not torch.isfinite(norm):
                raise ValueError('Nonfinite policy gradient')
            accepted = False
            for retry in range(7):
                policy.load_state_dict(prior_policy)
                optimizer.load_state_dict(prior_optimizer)
                for group, lr in zip(optimizer.param_groups, original_lrs):
                    group['lr'] = lr * .5**retry
                optimizer.step()
                trial = diagnostics()
                finite = all(torch.isfinite(v).all() for v in policy.state_dict().values())
                if finite and math.isfinite(trial['kl']) and trial['kl'] <= kl_limit and trial['max_correction_change'] <= drift_limit:
                    accepted = True
                    accepted_epochs += 1
                    metrics.update(loss=float(loss.detach()), policy_loss=float(policy_loss.detach()),
                                   value_loss=float(value_loss.detach()), gradient_norm=float(norm),
                                   effective_learning_rate=optimizer.param_groups[0]['lr'])
                    break
                backtracks += 1
            if not accepted:
                policy.load_state_dict(prior_policy)
                optimizer.load_state_dict(prior_optimizer)
                break
    finally:
        # Retrying a difficult batch must not permanently anneal Adam to zero.
        for group, lr in zip(optimizer.param_groups, original_lrs):
            group['lr'] = lr
    metrics.update(diagnostics())  # Report the policy actually retained.
    metrics.update(accepted_epochs=accepted_epochs, backtracks=backtracks,
                   guard_rejected=accepted_epochs == 0)
    return metrics


def save_checkpoint(path, policy, optimizer, *, base_model, updates, config):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temp=path.with_suffix('.tmp')
    torch.save({'schema':'uuv.residual_rl.v2' if policy.learned_brake else 'uuv.residual_rl.v1','policy':policy.state_dict(),
                'learned_brake':policy.learned_brake,
                'source_checkpoint':getattr(policy,'source_checkpoint',None),
                'source_lineage_updates':getattr(policy,'source_lineage_updates',None),
                'optimizer':optimizer.state_dict(), 'base_model':str(base_model),
                'updates':updates,'lineage_updates':updates+getattr(policy,'inherited_updates',0),'config':config,'residual_limit':policy.residual_limit}, temp)
    temp.replace(path)
