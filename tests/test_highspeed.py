import numpy as np
import pytest
from fwrl.aerodynamics import advance, events, initial_state, MPH
from fwrl.landscape import gate_normal, terrain_height
from fwrl.world import load_world


def world():
    return load_world('worlds/training_airfield.json')


def test_launch_rest_exit_speed_and_ballistic_stall():
    w = world()
    s = initial_state(w)[None]
    assert s[0, 4] == 0
    for i in range(40):
        s = advance(s, np.array([[0., .18, 200*MPH]]), w, np.array([i*.05]))
    assert s[0, 4] == pytest.approx(50*MPH)
    assert np.linalg.norm(s[0, :3]-w['start']) == pytest.approx(w['launcher']['length_m'])
    s[0, 2:7] = [100, 0, 5, 0, 0]
    new = advance(s, np.array([[0., 0., 5.]]), w, np.array([10.]))
    assert new[0, 6] < 0  # Lift-limited aircraft falls rather than flying below stall.


def test_fast_ring_crossing_and_tube_strike():
    w = world()
    center = np.array(w['waypoints'][0])
    n = np.array(gate_normal(w, 0))
    a, b = center-10*n, center+10*n
    hit, crash = events(a[None], b[None], np.array([0]), w)
    assert hit[0] and not crash[0]
    a[2] += w['goal_radius_m']; b[2] += w['goal_radius_m']
    hit, crash = events(a[None], b[None], np.array([0]), w)
    assert crash[0] and not hit[0]
    hit, _ = events((center+10*n)[None], (center-10*n)[None], np.array([0]), w)
    assert not hit[0]


def test_cuda_path_ground_crash_resets_to_rest():
    torch = pytest.importorskip('torch')
    from fwrl.training import VectorFlight
    w = world()
    env = VectorFlight(w, 1)
    env.age[:] = 100
    env.state[0] = torch.tensor([0, 0, terrain_height(0, 0, w), 0, 89.408, 0, 0])
    _, _, done, info = env.step(torch.zeros((1, 3)))
    assert done.item() and info['crash'].item()
    np.testing.assert_allclose(env.state[0].numpy(), initial_state(w), atol=1e-6)
    assert env.age.item() == 0 and env.index.item() == 0


def test_force_integrator_torch_numpy_agree_in_flight():
    torch = pytest.importorskip('torch')
    w = world()
    s = np.array([[0., 0., 100., .3, 75., .2, .1]], dtype=np.float32)
    command = np.array([[.4, .05, 89.408]], dtype=np.float32)
    expected = advance(s, command, w, np.array([10.], dtype=np.float32))
    actual = advance(torch.tensor(s), torch.tensor(command), w, torch.tensor([10.]), ops=torch)
    np.testing.assert_allclose(actual.numpy(), expected, atol=1e-5)
