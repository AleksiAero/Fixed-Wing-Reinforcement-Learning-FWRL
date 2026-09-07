"""Shared procedural meadow geometry for Blender, plotting and physics."""
import math
import random


def terrain_height(x, y, world, ops=math):
    cfg = world.get('terrain')
    if not cfg:
        return x * 0 + world['bounds'][0][2]
    z = x * 0 + cfg['base_m']
    for wave in cfg['waves']:
        z = z + wave['amplitude_m'] * ops.sin(x * wave['kx'] + y * wave['ky'] + wave['phase'])
    for hill in cfg['hills']:
        shape = ops.exp(-((x-hill['x_m'])**2 + (y-hill['y_m'])**2) / (2*hill['width_m']**2))
        ruggedness = hill.get('ruggedness', 0.)
        if ruggedness:
            shape = shape*(1 + ruggedness*ops.sin(x*.008)*ops.sin(y*.009)
                          + ruggedness*.4*ops.sin(x*.021+y*.014))
        z = z + hill['height_m'] * shape
    return z


def gate_normal(world, index):
    points = [world['start'], *world['waypoints']]
    previous = points[index]
    following = points[min(index+2, len(points)-1)]
    dx, dy = following[0]-previous[0], following[1]-previous[1]
    length = math.hypot(dx, dy)
    return (dx/length, dy/length, 0.) if length > 1e-9 else (1., 0., 0.)


def distance_to_route(x, y, route):
    best = float('inf')
    for a, b in zip(route, route[1:]):
        dx, dy = b[0]-a[0], b[1]-a[1]
        u = min(1, max(0, ((x-a[0])*dx+(y-a[1])*dy)/(dx*dx+dy*dy or 1)))
        best = min(best, math.hypot(x-a[0]-u*dx, y-a[1]-u*dy))
    return best


def make_meadow(seed=27):
    rng = random.Random(seed)
    world = {
        'name': 'Meadow Park', 'frame': 'ENU', 'seed': seed,
        'bounds': [[-700, -700, 0], [700, 700, 250]],
        'start': [-490, -160, 72], 'goal_radius_m': 30, 'aircraft_radius_m': 2,
        'terrain': {'type': 'rolling_meadow', 'base_m': 8,
                    'waves': [{'amplitude_m': 2.7, 'kx': .018, 'ky': .008, 'phase': .5},
                              {'amplitude_m': 1.8, 'kx': -.013, 'ky': .024, 'phase': 2.1},
                              {'amplitude_m': .45, 'kx': .085, 'ky': .064, 'phase': 1.2}],
                    'hills': [{'x_m': -130, 'y_m': 90, 'height_m': 24, 'width_m': 160},
                              {'x_m': 470, 'y_m': 340, 'height_m': 18, 'width_m': 150},
                              {'x_m': -460, 'y_m': -410, 'height_m': 16, 'width_m': 140}]},
        'waypoints': [], 'obstacles': [], 'foliage': [],
        'hoops': {'orientation': 'vertical', 'tube_radius_m': .85, 'seed': seed},
    }
    for x, y, z in [(-250, -335, 66), (65, -355, 76), (365, -235, 70), (435, 55, 83),
                    (280, 340, 77), (-55, 370, 84), (-355, 250, 75), (-470, -20, 70), (-285, -230, 68)]:
        world['waypoints'].append([round(x+rng.uniform(-22, 22), 2), round(y+rng.uniform(-22, 22), 2), round(z+rng.uniform(-4, 4), 2)])
    route = [world['start'], *world['waypoints']]
    patches = [(-550, 390), (-200, 30), (170, 20), (540, -150), (180, -555), (-400, -490), (70, 540)]
    attempts = 0
    while len(world['obstacles']) < 105 and attempts < 5000:
        attempts += 1
        cx, cy = rng.choice(patches)
        x, y = rng.gauss(cx, 80), rng.gauss(cy, 75)
        if max(abs(x), abs(y)) > 630 or distance_to_route(x, y, route) < 65:
            continue
        if any(math.hypot(x-o['center'][0], y-o['center'][1]) < 15 for o in world['obstacles']):
            continue
        height, radius = rng.uniform(18, 33), rng.uniform(5.5, 10)
        ground = terrain_height(x, y, world)
        world['obstacles'].append({'name': f'Tree {len(world["obstacles"])+1:03d}', 'kind': 'tree',
            'center': [round(x, 3), round(y, 3), round(ground+height/2, 3)],
            'size': [round(2*radius, 3), round(2*radius, 3), round(height, 3)], 'variant': rng.randrange(3)})
    for _ in range(310):
        x, y = rng.uniform(-660, 660), rng.uniform(-660, 660)
        world['foliage'].append({'position': [x, y, terrain_height(x, y, world)], 'radius': rng.uniform(.8, 2.7), 'height': rng.uniform(.7, 2.3)})
    return world
