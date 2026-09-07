import json
from pathlib import Path
import numpy as np
from .landscape import terrain_height


def load_world(path):
    world = json.loads(Path(path).read_text(encoding="utf-8"))
    if world.get("frame") != "ENU":
        raise ValueError("World must use ENU coordinates")
    lo, hi = np.array(world["bounds"], dtype=float)
    points = np.array([world["start"], *world["waypoints"]], dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 2 or not np.isfinite(points).all() or not np.isfinite([lo, hi]).all() or np.any(lo >= hi):
        raise ValueError("Invalid bounds/start/waypoints")
    if np.any(points <= lo) or np.any(points >= hi):
        raise ValueError("Start and waypoints must be strictly inside world bounds")
    if not np.isfinite(world["goal_radius_m"]) or world["goal_radius_m"] <= 0 or not np.isfinite(world["aircraft_radius_m"]) or world["aircraft_radius_m"] <= 0:
        raise ValueError("Radii must be finite and positive")
    if 'launcher' in world:
        from .aerodynamics import PARAMETERS
        launch = world['launcher']
        values = [launch[k] for k in ('length_m', 'exit_speed_mps', 'heading_rad', 'pitch_rad')]
        if not np.isfinite(values).all() or min(values[:2]) <= 0 or not 0 <= launch['pitch_rad'] < .7:
            raise ValueError('Invalid launch rail parameters')
        duration_ticks = 2*launch['length_m']/launch['exit_speed_mps']/.05
        if abs(duration_ticks-round(duration_ticks)) > 1e-5:
            raise ValueError('Launch duration must align with 0.05 second physics ticks')
        if not 0 < world['target_speed_mps'] <= 105:
            raise ValueError('Target speed must be between 0 and 105 m/s')
        coefficients = [world['aerodynamics'][key] for key in PARAMETERS]
        if not np.isfinite(coefficients).all() or min(coefficients) <= 0:
            raise ValueError('Aerodynamic coefficients must be finite and positive')
        tube = world['hoops']['tube_radius_m']
        if not np.isfinite(tube) or tube <= 0 or world['goal_radius_m'] <= tube+world['aircraft_radius_m']:
            raise ValueError('Hoop aperture must clear the aircraft')
        for key in ('gate_radii_m','gate_timeout_s'):
            if key in world:
                values = np.asarray(world[key],dtype=float)
                if values.shape != (len(world['waypoints']),) or not np.isfinite(values).all() or np.any(values<=0):
                    raise ValueError(f'Invalid {key}')
        if 'gate_radii_m' in world and min(world['gate_radii_m'])<=tube+world['aircraft_radius_m']:
            raise ValueError('Every hoop must clear the aircraft')
        if world.get('control_mode') == 'elevons':
            p = world['aerodynamics']
            values = [p[k] for k in ('wingspan_m','mean_chord_m','stall_speed_mps','stall_angle_rad','elevon_limit_rad','servo_rate_rad_s','servo_tau_s','motor_tau_s')]
            values.extend(p['inertia_kg_m2'])
            if len(p['inertia_kg_m2'])!=3 or not np.isfinite(values).all() or min(values)<=0:
                raise ValueError('Invalid elevon dynamics coefficients')
            if p['max_thrust_n'] >= p['mass_kg']*9.81:
                raise ValueError('This non-VTOL aircraft requires thrust below weight')
    terrain = world.get('terrain')
    if terrain:
        if terrain.get('type') != 'rolling_meadow':
            raise ValueError('Unsupported terrain type')
        values = [terrain['base_m']]
        for wave in terrain['waves']:
            values.extend(wave[k] for k in ('amplitude_m', 'kx', 'ky', 'phase'))
        for hill in terrain['hills']:
            values.extend(hill[k] for k in ('x_m', 'y_m', 'height_m', 'width_m'))
            if hill['width_m'] <= 0:
                raise ValueError('Hill width must be positive')
        if not np.isfinite(values).all():
            raise ValueError('Terrain parameters must be finite')
        ground = terrain_height(points[:, 0], points[:, 1], world, np)
        if np.any(points[:, 2] <= ground + world['aircraft_radius_m']):
            raise ValueError('Start or waypoint intersects terrain')
    for o in world["obstacles"]:
        c, size = np.array(o["center"]), np.array(o["size"])
        if c.shape != (3,) or size.shape != (3,) or not np.isfinite([c, size]).all() or np.any(size <= 0):
            raise ValueError("Invalid obstacle box")
        if np.any(np.all(np.abs(points - c) <= size / 2 + world["aircraft_radius_m"], axis=1)):
            raise ValueError("Start or waypoint intersects an obstacle")
    return world


def collision(position, world):
    if "launcher" in world:
        from .aerodynamics import events
        p = np.asarray(position, dtype=float)[None]
        return bool(events(p, p, np.array([0]), world)[1][0])
    p = np.asarray(position)
    r = world["aircraft_radius_m"]
    lo, hi = np.asarray(world["bounds"])
    if np.any(p <= lo + r) or np.any(p >= hi - r):
        return True
    if p[2] <= terrain_height(p[0], p[1], world) + r:
        return True
    return any(np.all(np.abs(p - o["center"]) <= np.asarray(o["size"]) / 2 + r) for o in world["obstacles"])
