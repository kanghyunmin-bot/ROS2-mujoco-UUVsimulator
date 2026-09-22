import unittest
import torch
from residual_policy import ResidualActorCritic, advantages, update

class PolicyTests(unittest.TestCase):
    def test_actual_update_and_bounds(self):
        torch.manual_seed(4)
        p=ResidualActorCritic();optimizer=torch.optim.Adam(p.parameters(),lr=3e-4)
        state=torch.randn(16,23);base=torch.zeros(16,4)
        action,raw,logp,value=p.act(state,base)
        self.assertTrue((action.abs()<=.15).all())
        before=p.actor.weight.detach().clone()
        metric=update(p,optimizer,dict(state=state,base_action=base,raw_action=raw,
            old_log_prob=logp,advantage=action[:,0]*10,**{'return':action[:,0]*10}))
        self.assertFalse(torch.equal(before,p.actor.weight));self.assertTrue(metric['gradient_norm']>0)

    def test_large_update_rolls_back(self):
        torch.manual_seed(7)
        p=ResidualActorCritic();o=torch.optim.Adam(p.parameters(),lr=10.)
        state=torch.randn(16,23);base=torch.zeros(16,4)
        action,raw,logp,value=p.act(state,base)
        before={k:v.clone() for k,v in p.state_dict().items()}
        m=update(p,o,dict(state=state,base_action=base,raw_action=raw,
            old_log_prob=logp,advantage=torch.arange(16.),**{'return':torch.ones(16)}),epochs=1)
        self.assertTrue(m['guard_rejected'])
        for k,v in p.state_dict().items():torch.testing.assert_close(v,before[k])

    def test_guard_keeps_small_accepted_steps(self):
        torch.manual_seed(7)
        p=ResidualActorCritic();o=torch.optim.Adam(p.parameters(),lr=.003)
        state=torch.randn(32,23);base=torch.zeros(32,4)
        _,raw,logp,_=p.act(state,base)
        before=p.actor.weight.detach().clone()
        m=update(p,o,dict(state=state,base_action=base,raw_action=raw,
            old_log_prob=logp,advantage=raw[:,0],**{'return':torch.ones(32)}),epochs=64)
        self.assertFalse(torch.equal(before,p.actor.weight))
        self.assertGreater(m['accepted_epochs'],0)
        self.assertLess(m['accepted_epochs'],64)  # Last step rejected; earlier steps survive.
        self.assertGreater(m['backtracks'],0)
        self.assertLessEqual(m['kl'],.01)
        self.assertLessEqual(m['max_correction_change'],.005)

    def test_terminal_no_bootstrap_or_leak(self):
        reward=torch.tensor([[1.],[2.]])
        value=torch.zeros_like(reward);next_value=torch.full_like(reward,100.)
        terminal=torch.ones_like(reward,dtype=torch.bool)
        adv,ret=advantages(reward,value,next_value,terminal,terminal,torch.full_like(reward,.1))
        torch.testing.assert_close(adv,reward)

    def test_nonterminal_cutoff_bootstraps(self):
        r=torch.zeros((1,1));v=torch.ones((1,1));yes=torch.ones((1,1),dtype=torch.bool)
        a,_=advantages(r,v,v*2,~yes,yes,torch.full_like(r,.1))
        self.assertAlmostEqual(float(a),.98,places=5)

if __name__=='__main__':unittest.main()
