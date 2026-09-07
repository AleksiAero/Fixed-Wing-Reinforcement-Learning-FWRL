import numpy as np
import pytest
from fwrl.mountain import make_mountain
from fwrl.aerodynamics import initial_state,advance
from fwrl.elevon import aero


def flying(world, speed=22.352, pitch=0.):
    s = initial_state(world)[None]
    s[0,:3] = [0,0,1400]
    s[0,7:11] = [np.cos(pitch/2),0,-np.sin(pitch/2),0]
    s[0,11:14] = [speed,0,0]
    return s


def test_stall_reference_and_post_stall_lift_loss():
    w = make_mountain();p=w['aerodynamics']
    vs = (2*p['mass_kg']*9.81/(p['density_kg_m3']*p['wing_area_m2']*p['cl_max']))**.5
    assert vs/.44704 == pytest.approx(45)
    lift_at_40 = .5*p['density_kg_m3']*(40*.44704)**2*p['wing_area_m2']*p['cl_max']
    assert lift_at_40 < p['mass_kg']*9.81
    attached = aero(flying(w,pitch=np.deg2rad(15)),w)[2][0]
    stalled = aero(flying(w,pitch=np.deg2rad(35)),w)[2][0]
    assert stalled < attached*.7
    assert aero(flying(w,pitch=np.deg2rad(35)),w)[3][0] > aero(flying(w,pitch=np.deg2rad(15)),w)[3][0]


def test_vertical_full_throttle_cannot_sustain_climb():
    w=make_mountain();s=flying(w,pitch=np.pi/2)
    s[0,11:14]=[0,0,15]
    s[0,19]=1
    new=advance(s,np.array([[0,0,1.]]),w,np.array([10.]))
    assert new[0,13] < s[0,13]  # even aligned thrust is below weight
    assert np.isfinite(new).all()  # no vertical Euler singularity
    for tick in range(160):
        new=advance(new,np.array([[0,0,1.]]),w,np.array([10.05+tick*.05]))
    assert new[0,13] < 0


def test_elevon_mixing_servo_limits_and_inertia():
    w=make_mountain();s=flying(w)
    common=advance(s,np.array([[1.,1.,-1.]]),w,np.array([10.]))
    differential=advance(s,np.array([[1.,-1.,-1.]]),w,np.array([10.]))
    assert common[0,15]<0 and abs(common[0,14])<1e-8  # common up = nose up
    assert differential[0,14]<0  # left up/right down = left roll
    assert 0<common[0,17]<w['aerodynamics']['elevon_limit_rad']
    assert common[0,17] <= w['aerodynamics']['servo_rate_rad_s']*.05


def test_first_gate_positive_and_next_gate_deadline_negative():
    torch=pytest.importorskip('torch')
    from fwrl.training import VectorFlight
    env=VectorFlight(make_mountain(),1)
    env.wind[:]=0
    # Exercise the gate reward with a controlled crossing, not a scripted launch.
    env.age[:]=100
    env.state[0]=torch.as_tensor(flying(env.world,speed=30.)[0],dtype=env.state.dtype)
    env.state[0,:3]=env.points[0]
    env.state[0,0]-=.5
    _,reward,done,info=env.step(torch.zeros(1,3))
    assert env.index.item()==1 and reward.item()>99 and not done.item()
    assert env.gate_age.item()==0
    env.gate_age[:]=int(np.ceil(env.gate_timeouts[1].item()/env.cfg.dt))
    _,reward,done,info=env.step(torch.zeros(1,3))
    assert done.item() and info['timeout'].item() and info['terminal'].item()
    assert not info['crash'].item() and not info['truncated'].item()
    assert reward.item()<-140
    np.testing.assert_allclose(env.state[0].numpy(),initial_state(env.world),atol=1e-5)
    assert env.gate_age.item()==0 and env.index.item()==0


def test_ground_crash_negative_and_reset():
    torch=pytest.importorskip('torch')
    from fwrl.training import VectorFlight
    env=VectorFlight(make_mountain(),1)
    env.age[:]=100
    env.state[0,2]=-20
    _,reward,done,info=env.step(torch.zeros(1,3))
    assert info['crash'].item() and done.item() and reward.item()<-140
    assert env.state[0,4]==0
