"""Precision shaping, migration, and repeated-evaluation regressions."""
import unittest

import torch

from evaluation import assessment
from precision_reward import PrecisionReward, profiles
from residual_policy import ResidualActorCritic, update
from reward import RewardConfig
from schedule import round_plan


def observation(t=0., x=.6, y=.1, z=.05, **extra):
    return dict(time_s=t, target_id='rod', detached=False, eq_active=True,
                release_time_s=-1., release_right_fork_region=False,
                height_error_m=z, fork_stem_distance_m=(x*x+y*y+z*z)**.5,
                camera_valid=True, visible=True, hand_visible=False,
                depth_m=1., yaw_rad=0., yaw_rate_rad_s=0.,
                slot_offset_m=[x,y,z], **extra)


class PrecisionTests(unittest.TestCase):
    def test_alignment_gates_insertion(self):
        good=profiles(observation(x=0.,y=0.,z=0.))
        bad=profiles(observation(x=0.,y=.15,z=0.))
        self.assertGreater(good['insertion'],.8)
        self.assertLess(bad['insertion'],.001)

    def test_continuity_and_yaw_direction(self):
        a=profiles(observation(x=.650001))
        b=profiles(observation(x=.649999))
        self.assertLess(abs(a['alignment']-b['alignment']),1e-5)
        self.assertGreater(profiles(observation(x=.2,y=.01,z=0))['aiming'],
                           profiles(observation(x=.2,y=.10,z=0))['aiming'])

    def test_no_stationary_or_repeat_bonus(self):
        r=PrecisionReward();r.reset(observation())
        stationary=r.step(observation(t=.2))
        self.assertLess(stationary['reward'],0)
        r.step(observation(t=.4,x=.2,y=0,z=0))
        r.step(observation(t=.6))
        repeated=r.step(observation(t=.8,x=.2,y=0,z=0))
        self.assertTrue(all(v==0 for k,v in repeated['terms'].items() if k.startswith('precision_')))

    def test_slow_aligned_motion_better_than_fast(self):
        def run(dt):
            r=PrecisionReward();r.reset(observation(x=.2,y=0,z=0))
            return r.step(observation(t=dt,x=.18,y=0,z=0))
        self.assertGreater(run(.3)['reward'],run(.02)['reward'])

    def test_release_teleport_cannot_pay_progress_or_general_speed(self):
        r=PrecisionReward();r.reset(observation())
        o=observation(t=.2,x=0,y=0,z=0)
        o.update(detached=True,eq_active=False,release_time_s=.2)
        result=r.step(o)
        self.assertEqual(result['release_tier'],'general')
        self.assertNotIn('fast_release',result['terms'])
        self.assertFalse(any(k.startswith('precision_') for k in result['terms']))

    def test_warm_start_preserves_deterministic_actions(self):
        torch.manual_seed(1)
        old=ResidualActorCritic()
        with torch.no_grad():old.actor.weight.normal_(0,.03)
        new=ResidualActorCritic(learned_brake=True);new.seed_legacy(old.state_dict())
        s=torch.randn(32,23);b=torch.rand(32,4)*2-1
        torch.testing.assert_close(old.act(s,b,True)[0],new.act(s,b,True)[0],rtol=0,atol=0)
        self.assertEqual(new.critic.weight.abs().sum().item(),0)
        self.assertTrue(torch.isfinite(new.act(s,b)[2]).all())

    def test_brake_can_reduce_full_forward(self):
        p=ResidualActorCritic(learned_brake=True)
        raw=torch.zeros(1,5);raw[0,4]=3
        a=p.compose(raw,torch.tensor([[1.,.3,-.2,.1]]))
        self.assertLess(a[0,0].item(),.01)
        torch.testing.assert_close(a[0,1:],torch.tensor([.3,-.2,.1]))

    def test_guard_covers_brake_and_actual_action(self):
        torch.manual_seed(4)
        p=ResidualActorCritic(learned_brake=True)
        opt=torch.optim.Adam(p.parameters(),lr=.001)
        s=torch.randn(64,23);b=torch.ones(64,4)
        a,raw,logp,v=p.act(s,b)
        before=p.act(s,b,True)[0]
        m=update(p,opt,dict(state=s,base_action=b,raw_action=raw,old_log_prob=logp,
                 advantage=raw[:,4],**{'return':torch.ones(64)}),kl_limit=.005,drift_limit=.003)
        self.assertGreater(m['accepted_epochs'],0)
        self.assertLessEqual((p.act(s,b,True)[0]-before).abs().max().item(),.003001)
        self.assertLessEqual(m['kl'],.005)

    def test_all_failed_reward_gain_not_best(self):
        a=dict(n=6,fork=0,hand_or_fork=0,released=0,mean_reward=-2.)
        b={**a,'mean_reward':2.}
        self.assertEqual(assessment(a,b,require_outcome_gain=True),'continue_training')
        b.update(fork=1,hand_or_fork=1,released=1)
        self.assertEqual(assessment(a,b,require_outcome_gain=True),'promote')
        self.assertEqual(assessment(b,a,require_outcome_gain=True),'rollback')

    def test_repeated_evaluation_schedule(self):
        self.assertEqual([round_plan(i,2) for i in range(7)],
          [('baseline_evaluation',0),('training',0),('incumbent_evaluation',0),
           ('incumbent_evaluation',1),('candidate_evaluation',0),
           ('candidate_evaluation',1),('training',0)])


if __name__=='__main__':unittest.main()
