import json
import numpy as np
import pytest
from fwrl.mountain import make_mountain
from fwrl.world import load_world
from fwrl.aerodynamics import advance, initial_state
from fwrl.landscape import terrain_height
from fwrl.live import publish


def test_mountain_course_is_clear_and_has_reversing_turns(tmp_path):
    w = make_mountain()
    path = tmp_path/'world.json'
    path.write_text(json.dumps(w),encoding='utf-8')
    load_world(path)
    points = np.array(w['waypoints'])
    assert len(points) == 20
    assert points[:,2].max()>1000
    assert np.ptp(points[:,2])>800
    assert (points[1:,2]-terrain_height(points[1:,0],points[1:,1],w,np)>90).all()
    assert points[0,2]-terrain_height(*points[0,:2],w)>w['gate_radii_m'][0]
    directions = np.diff(points[:,:2],axis=0)
    turns = directions[:-1,0]*directions[1:,1]-directions[:-1,1]*directions[1:,0]
    assert turns.min()<0 and turns.max()>0
    assert w['control_mode']=='elevons' and 'residual_scale' not in w


def test_direct_controls_supply_no_lift_or_speed_assistance():
    w = make_mountain()
    state = initial_state(w)[None]
    state[0,:3] = [0,0,1400]
    state[0,7:11] = [1,0,0,0]
    state[0,11:14] = [15,0,0]
    result = advance(state,np.array([[0.,0.,-1.]]),w,np.array([10.]))
    assert result[0,6]<0  # zero lift coefficient means gravity wins
    assert result[0,4]<15.01  # engine off and drag, no speed hold
    assert result[0,5]==0
    # Target route does not affect any aerodynamic control.
    w['waypoints'] = [[9000,9000,9000]]
    np.testing.assert_allclose(result,advance(state,np.array([[0.,0.,-1.]]),w,np.array([10.])))


def test_training_never_calls_guidance_and_observes_mountains(monkeypatch):
    torch = pytest.importorskip('torch')
    from fwrl.training import VectorFlight, Policy
    env = VectorFlight(make_mountain(),count=2)
    def forbidden(*args):
        raise AssertionError('Guidance must never be invoked')
    monkeypatch.setattr(env,'baseline',forbidden)
    obs,reward,done,info = env.step(torch.zeros(2,3))
    assert torch.isfinite(obs).all()
    assert obs.shape[1]==11+len(env.centers)*6+20+13
    policy = Policy(obs.shape[1],from_scratch=True)
    assert torch.count_nonzero(policy.actor.weight)>0
    assert env.state[0,4]>0  # rail provides launch, not a pretrained pilot


def test_direct_numpy_and_torch_agree():
    torch = pytest.importorskip('torch')
    w = make_mountain()
    s = initial_state(w)[None].astype(np.float32)
    s[0,:3] = [100,-3000,1300]
    s[0,11:14] = [80,0,0]
    action = np.array([[.2,.1,.5]],dtype=np.float32)
    expected = advance(s,action,w,np.array([20.]))
    actual = advance(torch.tensor(s),torch.tensor(action),w,torch.tensor([20.]),ops=torch)
    np.testing.assert_allclose(actual.numpy(),expected,atol=1e-5)


def test_live_packet_replaces_current_state(tmp_path):
    path = tmp_path/'state.json'
    publish(path,dict(live_tick=1,state=[0]*7))
    publish(path,dict(live_tick=2,state=[1]*7))
    saved = json.loads(path.read_text())
    assert saved['live_tick']==2 and saved['state']==[1]*7
    assert saved['published_at']>0
    assert not path.with_suffix('.tmp').exists()


def test_windows_reader_contention_does_not_abort_training(tmp_path,monkeypatch):
    import fwrl.live as live
    original = live.os.replace
    calls = []
    def locked(source,destination):
        calls.append(1)
        if len(calls)<3:
            raise PermissionError('Windows reader briefly holds file')
        return original(source,destination)
    monkeypatch.setattr(live.os,'replace',locked)
    assert publish(tmp_path/'state.json',dict(tick=1))
    assert len(calls)==3
    monkeypatch.setattr(live.os,'replace',lambda *args: (_ for _ in ()).throw(PermissionError()))
    assert publish(tmp_path/'state.json',dict(tick=2)) is False
    assert json.loads((tmp_path/'state.json').read_text())['tick']==1
