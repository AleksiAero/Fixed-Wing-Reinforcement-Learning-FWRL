import json
import numpy as np
import pytest
torch=pytest.importorskip('torch')
from fwrl.mountain import make_mountain
from fwrl.curriculum import stage_world,eligible,STAGES
from fwrl.training import VectorFlight
from fwrl.learning_support import NormalizedPolicy,flight_shaping,save_checkpoint,evaluate_policy

def test_curriculum_changes_tasks_without_changing_physics_or_observation_size():
    base=make_mountain();dims=[]
    for stage in range(len(STAGES)):
        world=stage_world(base,stage)
        assert world['aerodynamics']==base['aerodynamics']
        assert world['launcher']==base['launcher']
        env=VectorFlight(world,2)
        dims.append(env.observation().shape[1])
        assert torch.isfinite(env.observation()).all()
    assert len(set(dims))==1 and dims[0]<100
    assert len(stage_world(base,0)['waypoints'])==3
    assert stage_world(base,4)['waypoints']==base['waypoints']

def test_progress_requires_repeated_quality_metrics_not_just_gates():
    good=dict(success_rate=.8,stall_fraction=.05,speed_band_fraction=.7)
    assert eligible(good)
    assert not eligible(dict(good,stall_fraction=.4))
    assert not eligible(dict(good,speed_band_fraction=.1))
    assert not eligible(dict(good,success_rate=.2))

def test_flight_shaping_prefers_safe_speed_and_low_aoa_without_throttle_bribe():
    env=VectorFlight(stage_world(make_mountain(),0),1)
    s=env.state.clone();s[:,2]=100;s[:,7:11]=torch.tensor([1.,0,0,0])
    s[:,4]=env.world['target_speed_mps'];s[:,11:14]=torch.tensor([s[0,4],0,0])
    command=torch.zeros(1,3)
    safe=flight_shaping(env,s,command,torch.ones(1))
    stalled=s.clone();a=np.deg2rad(30);stalled[:,7:11]=torch.tensor([np.cos(a/2),0,-np.sin(a/2),0])
    assert flight_shaping(env,stalled,command,torch.ones(1))<safe
    slow=s.clone();slow[:,4]=15;slow[:,11]=15
    assert flight_shaping(env,slow,command,torch.ones(1))<safe
    assert flight_shaping(env,s,torch.ones(1,3),torch.ones(1))<safe

def test_rail_is_not_treated_as_a_learned_control_action_and_no_guidance():
    env=VectorFlight(stage_world(make_mountain(),0),1)
    env.baseline=lambda: (_ for _ in ()).throw(AssertionError('Expert guidance used'))
    for _ in range(40):
        _,_,_,info=env.step(torch.zeros(1,3))
        assert not info['airborne_action'].item()
    _,_,_,info=env.step(torch.zeros(1,3))
    assert info['airborne_action'].item()

def test_normalization_survives_checkpoint_and_evaluation_does_not_update_weights(tmp_path):
    world=stage_world(make_mountain(),0);env=VectorFlight(world,2)
    model=NormalizedPolicy(env.observation().shape[1])
    model.update_normalization(env.observation())
    before={k:v.clone() for k,v in model.state_dict().items()}
    path=tmp_path/'policy.pt';save_checkpoint(path,dict(model=before))
    loaded=NormalizedPolicy(env.observation().shape[1]);loaded.load_state_dict(torch.load(path,weights_only=True)['model'])
    torch.testing.assert_close(model(env.observation())[0].mean,loaded(env.observation())[0].mean)
    result=evaluate_policy(model,world,count=2,max_seconds=.1)
    assert result['incomplete']==2
    for k,v in model.state_dict().items():torch.testing.assert_close(v,before[k])
