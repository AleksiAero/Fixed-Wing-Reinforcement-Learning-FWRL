import math
from .landscape import make_meadow, terrain_height
from .aerodynamics import PARAMETERS, MPH

def make_highspeed(seed=27):
    w = make_meadow(seed)
    w.update(name='SYNTHOSAR 200 mph launch course', bounds=[[-700,-1000,-20],[7200,1000,600]],
             residual_scale=[.08,.025,2.], target_speed_mps=200*MPH, aircraft_radius_m=.9, goal_radius_m=35,
             aerodynamics=PARAMETERS.copy())
    w['start'] = [-490., -160., terrain_height(-490., -160., w)+2.]
    w['launcher'] = dict(length_m=50*MPH, exit_speed_mps=50*MPH, heading_rad=0., pitch_rad=.18)
    w['waypoints'] = [[400+i*750, -160+70*math.sin(i*.65), 100+10*math.sin(i*.5)] for i in range(9)]
    # Clear the complete launch/flight corridor, retaining surrounding scenery.
    from fwrl.landscape import distance_to_route
    w['obstacles'] = [o for o in w['obstacles'] if distance_to_route(*o['center'][:2], [w['start'],*w['waypoints']])>90]
    w['physics_version'] = 2
    return w
