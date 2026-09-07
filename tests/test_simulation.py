from pathlib import Path
import numpy as np
import pytest
from fwrl.dynamics import step, guidance
from fwrl.world import load_world, collision
from fwrl.demo import rollout

WORLD = Path(__file__).resolve().parents[1]/'worlds/training_airfield.json'


def test_level_speed_and_turn_direction():
    state = np.array([0., 0., 80., 0., 22., 0., 0.])
    assert step(state, [0, 0, 22]) == pytest.approx([1.1, 0, 80, 0, 22, 0, 0])
    assert step(state, [.5, 0, 22])[3] > 0
    assert guidance(state, [100, 100, 80])[0] > 0


def test_bounds_and_obstacles():
    world = load_world(WORLD)
    assert collision(world['obstacles'][0]['center'], world)
    assert collision([0, 0, 0], world)
    assert not collision(world['start'], world)


def test_baseline_finishes_course():
    states, result = rollout(load_world(WORLD))
    assert result['result'] == 'success'
    assert np.isfinite(states).all()
    assert np.abs(states[:, 5]).max() <= .7


def test_torch_matches_scalar_integrator():
    torch = pytest.importorskip('torch')
    from fwrl.training import VectorFlight
    env = VectorFlight(load_world(WORLD), count=2)
    env.wind[:] = 0
    state = env.state[0].cpu().numpy().copy()
    command = env.baseline()[0].cpu().numpy()
    if env.physical:
        from fwrl.aerodynamics import advance
        expected = advance(state[None], command[None], env.world, np.array([0.]), env.cfg.dt)[0]
    else:
        expected = step(state, command, env.cfg)
    env.step(torch.zeros((2, 3)))
    np.testing.assert_allclose(env.state[0].cpu().numpy(), expected, atol=2e-5)


def test_termination_and_timeout_are_distinct():
    torch = pytest.importorskip('torch')
    from fwrl.training import VectorFlight
    env = VectorFlight(load_world(WORLD), count=2, max_steps=1)
    env.age[:] = 50
    env.state[0, 2] = -20
    obs, reward, done, info = env.step(torch.zeros((2, 3)))
    assert done.tolist() == [True, True]
    assert info['terminal'].tolist() == [True, False]
    assert info['truncated'].tolist() == [False, True]
    assert torch.isfinite(obs).all()
    assert env.age.tolist() == [0, 0]
