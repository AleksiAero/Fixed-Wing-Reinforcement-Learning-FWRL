import json
import math
import numpy as np
import pytest
from fwrl.landscape import make_meadow, terrain_height, gate_normal, distance_to_route
from fwrl.world import collision, load_world


def test_seeded_course_and_vertical_gates():
    world = make_meadow(27)
    assert world == make_meadow(27)
    assert world['waypoints'] != make_meadow(28)['waypoints']
    assert len(world['waypoints']) == 9
    assert len(world['obstacles']) == 105
    for i in range(9):
        normal = np.array(gate_normal(world, i))
        assert normal[2] == 0
        assert np.linalg.norm(normal) == pytest.approx(1)
        # A vertical gate plane contains world up, perpendicular to its normal.
        assert np.dot(normal, [0, 0, 1]) == 0


def test_meadow_has_real_relief_and_ground_collision():
    world = make_meadow()
    x, y = np.meshgrid(np.linspace(-650, 650, 60), np.linspace(-650, 650, 60))
    ground = terrain_height(x, y, world, np)
    assert np.ptp(ground) > 20
    z = terrain_height(0., 0., world)
    assert collision([0, 0, z+1], world)
    assert not collision([0, 0, 150], world)


def test_tree_positions_match_ground_and_avoid_route(tmp_path):
    world = make_meadow(32)
    route = [world['start'], *world['waypoints']]
    for tree in world['obstacles']:
        x, y, z = tree['center']
        assert distance_to_route(x, y, route) >= 65
        assert z-tree['size'][2]/2 == pytest.approx(terrain_height(x, y, world), abs=.003)
    path = tmp_path/'world.json'
    path.write_text(json.dumps(world))
    assert load_world(path)['seed'] == 32
    world['terrain']['hills'][0]['width_m'] = 0
    path.write_text(json.dumps(world))
    with pytest.raises(ValueError, match='Hill width'):
        load_world(path)


def test_torch_and_scalar_terrain_agree():
    torch = pytest.importorskip('torch')
    world = make_meadow()
    x = np.array([-400., -70., 260.], dtype=np.float32)
    y = np.array([150., 60., -160.], dtype=np.float32)
    expected = terrain_height(x, y, world, np)
    actual = terrain_height(torch.tensor(x), torch.tensor(y), world, torch)
    np.testing.assert_allclose(actual.numpy(), expected, atol=1e-5)


def test_torch_terrain_collision_resets_episode():
    torch = pytest.importorskip('torch')
    from fwrl.training import VectorFlight
    world = make_meadow()
    env = VectorFlight(world, count=1)
    env.state[0, :3] = torch.tensor([0., 0., terrain_height(0., 0., world)])
    _, _, done, info = env.step(torch.zeros((1, 3)))
    assert done.item() and info['crash'].item()
