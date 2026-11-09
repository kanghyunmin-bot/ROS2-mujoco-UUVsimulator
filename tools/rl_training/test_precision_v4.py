"""Regressions for deployable vision and dwell-qualified progress."""
import unittest
import numpy as np
import torch
from test_precision import observation
from precision_reward import PrecisionRewardV4
from visual_observation import visual_features, residual_state, STATE_KEYS, VISUAL_CONTRACT
from residual_policy import ResidualActorCritic
from spawn_schedule import spawn_request


class V4Tests(unittest.TestCase):
    def test_spawn_schedule_pairs_evaluation_and_separates_training(self):
        region=dict(seed=100, x_m=[-1.5,1.5], y_m=[-1,1])
        train=set(); evaluate=set()
        for worker in range(5):
            for cycle in range(4):
                wave=1+5*cycle
                train.add(spawn_request(region,wave,worker,2)['seed'])
                for repeat in range(2):
                    left=spawn_request(region,wave+1+repeat,worker,2)
                    right=spawn_request(region,wave+3+repeat,worker,2)
                    self.assertEqual(left,right)
                    evaluate.add(left['seed'])
        self.assertEqual(len(train),20);self.assertEqual(len(evaluate),10)
        self.assertFalse(train & evaluate)

    def test_fast_pass_never_pays_progress(self):
        r=PrecisionRewardV4();r.reset(observation(x=.8,y=0,z=0))
        for i,x in enumerate([.6,.4,.2,0,-.2,-.4],1):
            result=r.step(observation(t=i*.1,x=x,y=0,z=0))
            self.assertEqual(sum(v for k,v in result['terms'].items() if k.startswith('precision_')),0)

    def test_fleeting_contact_and_progress_after_dwell_break_do_not_pay(self):
        r=PrecisionRewardV4();r.reset(observation(x=.8,y=0,z=0))
        hit=r.step(observation(t=.1,x=.6,y=0,z=0,right_fork_stem_contact_active=True))
        self.assertNotIn('right_fork_contact',hit['terms'])
        for i in range(2,16):
            result=r.step(observation(t=i*.1,x=.6-(i-1)*.005,y=0,z=0))
        self.assertTrue(result['progress_qualified'])
        flyby=r.step(observation(t=1.6,x=.0,y=0,z=0))
        self.assertFalse(flyby['progress_qualified'])
        self.assertEqual(sum(v for k,v in flyby['terms'].items() if k.startswith('precision_')),0)

    def test_sustained_slow_aligned_approach_pays_once(self):
        r=PrecisionRewardV4();r.reset(observation(x=.8,y=0,z=0))
        total=0
        for i in range(1,31):
            result=r.step(observation(t=i*.1,x=.8-i*.005,y=0,z=0))
            total+=sum(v for k,v in result['terms'].items() if k.startswith('precision_'))
        self.assertGreater(total,0)
        for i in range(31,46):
            result=r.step(observation(t=i*.1,x=.65,y=0,z=0))
        self.assertEqual(sum(v for k,v in result['terms'].items() if k.startswith('precision_')),0)

    def test_misaligned_hover_and_long_sampling_gap_do_not_pay(self):
        r=PrecisionRewardV4();r.reset(observation(x=.8,y=.15,z=0))
        for i in range(1,21):
            result=r.step(observation(t=i*.1,x=.8-i*.005,y=.15,z=0))
            self.assertEqual(sum(v for k,v in result['terms'].items() if k.startswith('precision_')),0)
        result=r.step(observation(t=10,x=.1,y=0,z=0))
        self.assertEqual(sum(v for k,v in result['terms'].items() if k.startswith('precision_')),0)

    def test_visual_features_use_only_rgb_and_have_missing_flags(self):
        image=np.zeros((1,120,160,3),dtype=np.uint8)
        obs={'video.ego':image.copy(),'video.buoy_release':image.copy()}
        np.testing.assert_array_equal(visual_features(obs),np.zeros(26))
        obs['video.ego'][0,30:55,60:80]=[255,240,0]
        obs['video.ego'][0,55:75,68:72]=[245,245,245]
        f=visual_features(obs)
        self.assertEqual(f.shape,(26,));self.assertEqual(f[0],1)
        self.assertEqual(f[7],1)
        obs['rl_slot_offset_m']=[900,900,900]
        np.testing.assert_array_equal(f,visual_features(obs))
        with self.assertRaises(ValueError):visual_features({'video.ego':image})

    def test_warm_start_keeps_old_actions_with_expanded_input(self):
        old=ResidualActorCritic(learned_brake=True)
        with torch.no_grad():old.actor.weight.normal_(0,.05);old.actor.bias[4]=.5
        new=ResidualActorCritic(state_dim=49,learned_brake=True,observation_contract=VISUAL_CONTRACT)
        new.seed_legacy(old.state_dict())
        state=torch.randn(20,23);base=torch.randn(20,4)
        extended=torch.cat([state,torch.randn(20,26)],-1)
        torch.testing.assert_close(old.act(state,base,True)[0],new.act(extended,base,True)[0],atol=1e-6,rtol=1e-6)


if __name__=='__main__':unittest.main()
