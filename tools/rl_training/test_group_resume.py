"""A same-contract capture pause must not reset the critic or Adam state."""
import copy
import pytest
import torch

from recovery import restore_group_training_state
from residual_policy import ResidualActorCritic


def prepared():
    actor=ResidualActorCritic(state_dim=49,learned_brake=True,observation_contract='rgb_target_cues_v1')
    optimizer=torch.optim.Adam(actor.parameters(),lr=1e-4)
    sum(p.square().sum() for p in actor.parameters()).backward();optimizer.step()
    config={'reward_version':'precision_v4','models':['release-best'],'model_paths':{'release-best':'base'},
            'policy_groups':{'release-best':{'checkpoint':'old.pt','environments':2}},'ppo_learning_rate':1e-4}
    checkpoint={'schema':'uuv.residual_rl.v3','state_dim':49,'observation_contract':'rgb_target_cues_v1',
                'learned_brake':True,'policy':copy.deepcopy(actor.state_dict()),
                'optimizer':copy.deepcopy(optimizer.state_dict()),'config':config,
                'updates':4,'lineage_updates':4}
    return actor,optimizer,config,checkpoint


def test_restore_same_contract_including_critic_and_optimizer():
    old,_,config,checkpoint=prepared()
    config=copy.deepcopy(config);config['policy_groups']['release-best']['checkpoint']='training-latest.pt'
    actor=ResidualActorCritic(state_dim=49,learned_brake=True,observation_contract='rgb_target_cues_v1')
    optimizer=torch.optim.Adam(actor.parameters(),lr=1e-4)
    assert restore_group_training_state(actor,optimizer,checkpoint,config)
    assert all(torch.equal(v,old.state_dict()[k]) for k,v in actor.state_dict().items())
    assert optimizer.state_dict()['state']
    assert actor.inherited_updates==4


@pytest.mark.parametrize('field,value',[('reward_version','precision_v3'),('ppo_learning_rate',2e-4)])
def test_changed_training_contract_is_not_exact_resume(field,value):
    actor,optimizer,config,checkpoint=prepared()
    config=copy.deepcopy(config);config[field]=value
    assert not restore_group_training_state(actor,optimizer,checkpoint,config)


def test_old_observation_is_not_exact_resume():
    actor,optimizer,config,checkpoint=prepared();checkpoint['state_dim']=23
    assert not restore_group_training_state(actor,optimizer,checkpoint,config)
