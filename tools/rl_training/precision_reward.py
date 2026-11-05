"""Smooth, bounded stage-progress rewards; privileged geometry is reward-only.

Distances [m], angles [rad], durations [s]. This is experimental record-progress
shaping, not policy-invariant potential shaping or a certified insertion test.
"""
import math
from collections import deque

from reward import ForkReward


def sigmoid(value):
    return 1.0 / (1.0 + math.exp(-max(-60.0, min(60.0, value))))


def profiles(observation):
    """Return overlapping approach/alignment/insertion scores in [0, 1]."""
    x, y, z = observation['slot_offset_m']
    distance = math.sqrt(x*x + y*y + z*z)
    near = sigmoid((0.65-distance)/0.15)
    entry = math.exp(-(x/0.22)**2)
    lateral = math.exp(-(y/0.05)**2)
    height = math.exp(-(z/0.05)**2)
    # Bearing of the rod relative to the slot's +X entry direction. A round
    # vertical rod has no meaningful intrinsic yaw orientation.
    bearing = math.atan2(y, max(x, 0.08))
    aim = math.exp(-(bearing/0.20)**2)
    alignment = lateral*height*aim
    return dict(approach=math.exp(-distance),
                alignment=near*lateral*height,
                aiming=near*alignment,
                insertion=entry*alignment*sigmoid((0.04-x)/0.025),
                near=near, alignment_quality=alignment,
                bearing_error_rad=bearing, distance_m=distance)


class PrecisionReward(ForkReward):
    """Keep terminal evidence; replace legacy shaping with staged progress."""

    weights = {'approach': 1.0, 'alignment': 2.0, 'aiming': 1.5, 'insertion': 3.0}

    def reset(self, observation):
        super().reset(observation)
        score = profiles(observation)
        self.best = {key: score[key] for key in self.weights}
        self.previous_offset = observation['slot_offset_m']
        self.precision_best = score['insertion']

    def step(self, observation):
        dt = observation['time_s']-self.previous_time
        previous_offset = self.previous_offset
        result = super().step(observation)
        parts = {k: v for k, v in result['terms'].items()
                 if k in ('time', 'terminal', 'right_fork_contact')}
        # Only intended slot-region releases earn a speed bonus. Geometric
        # hand visibility is never treated as contact evidence.
        if result['success']:
            parts['fast_release'] = result['terms'].get('fast_release', 0.0)
        if not observation['detached']:
            score = profiles(observation)
            for key, weight in self.weights.items():
                parts['precision_'+key] = weight*max(0.0, score[key]-self.best[key])
                self.best[key] = max(self.best[key], score[key])
            self.precision_best = max(self.precision_best, score['insertion'])
            closing = max(0.0, (previous_offset[0]-observation['slot_offset_m'][0])/dt)
            # Allowed approach speed decreases continuously with distance;
            # alignment opens a small insertion-speed allowance near the slot.
            target_speed = (0.04 + 0.18*(1-score['near'])
                            + 0.04*score['alignment_quality'])
            parts['overspeed'] = -0.5*score['near']*min(4.0, (max(0.0, closing-target_speed)/0.2)**2)*dt
            parts['unaligned_advance'] = -0.4*score['near']*(1-score['alignment_quality'])*min(2.0, closing/0.2)*dt
            # Penalize excessive yaw rate, not corrective turning itself.
            parts['yaw_overshoot'] = -0.1*score['near']*min(4.0, (max(0.0, abs(observation['yaw_rate_rad_s'])-0.15)/0.3)**2)*dt
            result['precision'] = {**score, 'closing_speed_m_s': closing,
                                   'target_speed_m_s': target_speed}
        self.previous_offset = observation['slot_offset_m']
        result.update(reward=sum(parts.values()), terms=parts,
                      precision_best=self.precision_best)
        return result


class PrecisionRewardV4(PrecisionReward):
    """Pay progress only after a continuous aligned, slow approach interval [s].

    Old v3 remains unchanged for checkpoint reproducibility. The gate is a
    training reward qualification, not an inference-time truth controller.
    """

    dwell_s = .75
    max_sample_gap_s = .25

    def reset(self, observation):
        super().reset(observation)
        self.qualified_best = dict(self.best)
        self.qualified_since = None
        self.window = deque()

    def step(self, observation):
        dt = observation['time_s'] - self.previous_time
        previous = self.previous_offset
        result = super().step(observation)
        terms = {k: v for k, v in result['terms'].items() if not k.startswith('precision_')}
        x, y, z = observation['slot_offset_m']
        speed = math.sqrt(sum((a-b)**2 for a, b in zip(previous, (x,y,z))))/dt
        score = profiles(observation)
        qualified = (not observation['detached'] and 0 < dt <= self.max_sample_gap_s
                     and -.02 <= x <= 1.0 and abs(y) <= .05 and abs(z) <= .05
                     and abs(score['bearing_error_rad']) <= .20
                     and speed <= .08 and abs(observation['yaw_rate_rad_s']) <= .15)
        now = observation['time_s']
        if not qualified:
            self.qualified_since = None
            self.window.clear()
        else:
            if self.qualified_since is None:
                self.qualified_since = now
            self.window.append((now, score))
            # Keep the sample straddling the window boundary so a transient
            # peak cannot be credited as sustained quality.
            while len(self.window) > 1 and self.window[1][0] < now-self.dwell_s:
                self.window.popleft()
        held = qualified and now-self.qualified_since >= self.dwell_s
        for key, weight in self.weights.items():
            sustained = min(s[key] for _,s in self.window) if held else self.qualified_best[key]
            terms['precision_'+key] = weight*max(0.0, sustained-self.qualified_best[key])
            if held:
                self.qualified_best[key] = max(self.qualified_best[key], sustained)
        # A fleeting collision also cannot earn intermediate contact credit.
        if not held:
            terms.pop('right_fork_contact', None)
        result.update(terms=terms, reward=sum(terms.values()),
                      progress_qualified=bool(held),
                      dwell_s=0.0 if self.qualified_since is None else now-self.qualified_since,
                      relative_speed_m_s=speed,
                      qualified_precision_best=self.qualified_best['insertion'])
        return result
