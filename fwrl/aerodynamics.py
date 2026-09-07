"""Force-based 3-DOF flight with an idealized bank/lift autopilot (SI units).

Not a calibrated 6-DOF airframe: coefficients and propulsion are assumptions.
One implementation serves NumPy, CUDA training, and Blender playback.
"""
import numpy as np
from .landscape import gate_normal, terrain_height

MPH = .44704
PARAMETERS = dict(mass_kg=5., wing_area_m2=.45, cl_max=1.4, cd0=.025,
                  induced_drag=.055, max_thrust_n=100., propulsive_power_w=6500.,
                  max_load_g=6., density_kg_m3=1.225)


def initial_state(world):
    if world.get('control_mode') == 'elevons':
        from .elevon import initial
        return initial(world)
    launch = world['launcher']
    return np.array([*world['start'], launch['heading_rad'], 0., 0., launch['pitch_rad']])


def advance(state, command, world, time_s, dt=.05, wind=None, ops=np):
    """Batch states [N,7]; bank/gamma/speed commands; semi-implicit force integration."""
    if world.get('control_mode') == 'elevons':
        from .elevon import advance as elevon_advance
        return elevon_advance(state,command,world,time_s,dt,wind,ops)
    s = state.copy() if ops is np else state.clone()
    clip = ops.clip if ops is np else ops.clamp
    stack = lambda xs: ops.stack(xs, -1)
    p = world['aerodynamics']
    v = clip(s[:, 4], 1., None)
    direct = world.get('control_mode') == 'direct'
    bank = (clip(s[:, 5]+dt*command[:, 0]*1.2, -1.3, 1.3) if direct else
            s[:, 5] + dt * clip((command[:, 0]-s[:, 5])/.6, -1.2, 1.2))
    rho = p['density_kg_m3'] * ops.exp(-clip(s[:, 2], 0., None)/8500.)
    qs = .5*rho*v*v*p['wing_area_m2']
    lift_request = p['mass_kg']*(9.81*ops.cos(s[:, 6]) + v*clip((command[:, 1]-s[:, 6]), -.4, .4))/clip(ops.cos(bank), .3, None)
    cl = clip(lift_request/clip(qs, .01, None), -p['cl_max'], p['cl_max'])
    if direct:
        # Raw normalized roll-rate, lift-coefficient and throttle. No heading,
        # altitude, speed or gravity compensation is provided to this policy.
        cl = command[:, 1]*p['cl_max']
    lift = clip(qs*cl, -p['max_load_g']*p['mass_kg']*9.81, p['max_load_g']*p['mass_kg']*9.81)
    drag = qs*(p['cd0']+p['induced_drag']*cl*cl)
    thrust_limit = clip(p['propulsive_power_w']/v, 0., p['max_thrust_n'])
    requested = drag + p['mass_kg']*(9.81*ops.sin(s[:, 6]) + (command[:, 2]-v)/3.)
    thrust = clip(requested, 0., None)
    thrust = ops.minimum(thrust, thrust_limit)
    if direct:
        thrust = (command[:, 2]+1)*.5*thrust_limit
    s[:, 4] = clip(v + dt*((thrust-drag)/p['mass_kg']-9.81*ops.sin(s[:, 6])), .1, None)
    s[:, 6] = clip(s[:, 6]+dt*(lift*ops.cos(bank)/p['mass_kg']-9.81*ops.cos(s[:, 6]))/v, -1.4, 1.4)
    s[:, 3] += dt*lift*ops.sin(bank)/(p['mass_kg']*v*clip(ops.cos(s[:, 6]), .15, None))
    s[:, 3] = ops.atan2(ops.sin(s[:, 3]), ops.cos(s[:, 3])) if ops is not np else np.arctan2(np.sin(s[:, 3]), np.cos(s[:, 3]))
    s[:, 5] = bank
    velocity = s[:, 4:5]*stack([ops.cos(s[:, 6])*ops.cos(s[:, 3]), ops.cos(s[:, 6])*ops.sin(s[:, 3]), ops.sin(s[:, 6])])
    s[:, :3] += dt*(velocity + (0 if wind is None else wind))
    launch = world['launcher']
    duration = 2*launch['length_m']/launch['exit_speed_mps']
    # Rail constraints provide external reaction force until release. Split-step
    # release is avoided by choosing duration as an integer number of physics ticks.
    t = clip(time_s+dt, 0., duration)
    active = time_s < duration-1e-6
    h, g = launch['heading_rad'], launch['pitch_rad']
    direction = np.array([np.cos(g)*np.cos(h), np.cos(g)*np.sin(h), np.sin(g)])
    for axis in range(3):
        s[:, axis] = ops.where(active, world['start'][axis]+.5*launch['exit_speed_mps']/duration*t*t*direction[axis], s[:, axis])
    for axis, value in [(3,h),(4,launch['exit_speed_mps']/duration*t),(5,0.),(6,g)]:
        s[:, axis] = ops.where(active, value, s[:, axis])
    return s


def events(previous, current, indices, world, ops=np):
    """Swept collision using conservative distance bounds and exact plane crossing.

    Sample each segment at <=0.5m (speed is limited by thrust and episode bounds).
    Inflate by half the sample spacing: torus distance is 1-Lipschitz, so thin
    rings cannot be skipped between samples. Ground includes a slope margin.
    """
    tensor = lambda x: np.asarray(x) if ops is np else ops.as_tensor(x, dtype=current.dtype, device=current.device)
    norm = lambda x: (x*x).sum(-1)**.5
    # 32 intervals cover a 16m step; additional margin handles longer segments.
    u = tensor(np.linspace(0, 1, 33))
    delta = current-previous
    samples = previous[:, None, :] + delta[:, None, :]*u[None, :, None]
    margin = norm(delta)/64
    radius = world['aircraft_radius_m']
    lo, hi = tensor(world['bounds'])
    crash = ((samples <= lo+radius) | (samples >= hi-radius)).any(-1).any(-1)
    crash |= (samples[:, :, 2] <= terrain_height(samples[:, :, 0], samples[:, :, 1], world, ops)+radius+margin[:, None]*world.get('terrain_collision_margin',1.5)).any(-1)
    centers = tensor(world['waypoints'])
    normals = tensor([gate_normal(world, i) for i in range(len(centers))])
    rel = samples[:, :, None, :]-centers[None, None, :, :]
    axial = (rel*normals[None, None]).sum(-1)
    radial = norm(rel-axial[:, :, :, None]*normals[None, None])
    radii = tensor(world.get('gate_radii_m',[world['goal_radius_m']]*len(centers)))
    tube_distance = (axial**2+(radial-radii[None,None,:])**2)**.5
    crash |= (tube_distance <= world['hoops']['tube_radius_m']+radius+margin[:, None, None]).any(-1).any(-1)
    if world['obstacles']:
        boxes = tensor([o['center'] for o in world['obstacles']])
        halves = tensor([o['size'] for o in world['obstacles']])/2+radius
        crash |= (abs(samples[:, :, None, :]-boxes[None, None]) <= halves[None, None]+margin[:, None, None, None]).all(-1).any(-1).any(-1)
    center, normal = centers[indices], normals[indices]
    a, b = ((previous-center)*normal).sum(-1), ((current-center)*normal).sum(-1)
    fraction = -a/ops.where(abs(b-a)>1e-8, b-a, b*0+1)
    crossing = previous+fraction[:, None]*delta
    hit = (a<0) & (b>=0) & (norm(crossing-center)<radii[indices]-world['hoops']['tube_radius_m']-radius) & ~crash
    return hit, crash
