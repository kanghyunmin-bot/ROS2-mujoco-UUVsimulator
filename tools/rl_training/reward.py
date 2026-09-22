"""Episode-local rewards for instrumented right-fork buoy release.

All positions are world coordinates [m], elapsed times are simulation seconds.
The environment supplies verified geometry IDs; camera visibility is measured,
not inferred from a missing or stale camera frame.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RewardConfig:
    deadline_s: float = 35.0
    success: float = 20.0
    general_release: float = 3.0
    hand_release: float = 8.0
    fast_success: float = 5.0
    timeout: float = -2.0
    wrong_release: float = -5.0
    time_cost_per_s: float = .02
    height_progress: float = 2.0
    approach_progress: float = 1.0
    contact_bonus: float = 2.0
    height_scale_m: float = .1
    approach_scale_m: float = 1.0
    rediscovery: float = .5
    search_depth_m: float = .03
    search_yaw_rad: float = .1


class ForkReward:
    def __init__(self, config=RewardConfig()):
        self.config = config
        self.started = False
        self.done = False
        self.rediscovered = False
        self.lost_pose = None

    def potential(self, observation):
        c = self.config
        return math.exp(-abs(observation['height_error_m']) / c.height_scale_m
                        -observation['fork_stem_distance_m'] / c.approach_scale_m)

    def reset(self, observation):
        self.start = self.previous_time = observation['time_s']
        self.best_alignment = self.potential(observation)
        self.best_approach = math.exp(-observation['fork_stem_distance_m'] / self.config.approach_scale_m)
        self.contact_paid = False
        self.last_hand_seen = observation['time_s'] if observation.get('hand_visible') and observation['camera_valid'] else None
        self.target = observation['target_id']
        if observation['detached']:
            raise ValueError('Episode must start with attached target')
        self.started = True
        self.done = self.rediscovered = False
        self.lost_pose = None

    def step(self, observation):
        if not self.started or self.done:
            raise ValueError('Reward episode is not active')
        c = self.config
        if observation['target_id'] != self.target:
            raise ValueError('Target changed within episode')
        now = observation['time_s']
        if not math.isfinite(now) or now <= self.previous_time:
            raise ValueError('Simulation time must advance')
        dt = now - self.previous_time
        elapsed = now - self.start
        parts = {'time': -c.time_cost_per_s * dt}
        alignment = self.potential(observation)
        parts['height_alignment'] = c.height_progress * max(0., alignment-self.best_alignment)
        self.best_alignment = max(self.best_alignment, alignment)
        approach = math.exp(-observation['fork_stem_distance_m'] / c.approach_scale_m)
        parts['approach'] = c.approach_progress * max(0., approach-self.best_approach)
        self.best_approach = max(self.best_approach, approach)
        if (not self.contact_paid and not observation['detached']
                and observation.get('right_fork_stem_contact_active') is True):
            parts['right_fork_contact'] = c.contact_bonus
            self.contact_paid = True
        # No reward for spinning or changing depth alone; one reacquisition
        # payment per episode prevents intentional visibility-loss cycling.
        if observation['camera_valid']:
            pose = (observation['depth_m'], observation['yaw_rad'])
            if not observation['visible'] and self.lost_pose is None:
                self.lost_pose = pose
            elif observation['visible'] and self.lost_pose is not None:
                depth, yaw = self.lost_pose
                turned = abs(math.atan2(math.sin(pose[1]-yaw), math.cos(pose[1]-yaw)))
                if not self.rediscovered and (abs(pose[0]-depth) >= c.search_depth_m or turned >= c.search_yaw_rad):
                    parts['rediscovery'] = c.rediscovery
                    self.rediscovered = True
                self.lost_pose = None
        success = False
        released = False
        tier = 'none'
        if observation.get('hand_visible') and observation['camera_valid'] and not observation['detached']:
            self.last_hand_seen = now
        if observation['detached']:
            release_elapsed = observation['release_time_s'] - self.start
            released = (release_elapsed <= c.deadline_s and observation['eq_active'] is False
                        and self.start < observation['release_time_s'] <= now)
            success = released and observation.get('release_right_fork_region') is True
            hand_confirmed = (released and self.last_hand_seen is not None
                              and 0 <= observation['release_time_s']-self.last_hand_seen <= .25)
            tier = 'right_fork' if success else 'hand_view' if hand_confirmed else 'general' if released else 'invalid'
            base = c.success if success else c.hand_release if hand_confirmed else c.general_release
            speed_scale = c.fast_success if success else min(2., c.fast_success) if hand_confirmed else min(1., c.fast_success)
            parts['terminal'] = base if released else c.wrong_release
            if released:
                parts['fast_release'] = speed_scale * (1-release_elapsed/c.deadline_s)
            self.done = True
        elif elapsed >= c.deadline_s:
            parts['terminal'] = c.timeout
            self.done = True
        self.previous_time = now
        return {'reward': sum(parts.values()), 'terms': parts, 'success': success, 'released': released, 'release_tier': tier, 'done': self.done}
