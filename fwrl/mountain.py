"""Mountain slalom world for direct-control learning from random initialization."""
import random
import math
from .highspeed import make_highspeed
from .landscape import terrain_height, distance_to_route
from .deadlines import gate_budgets


def make_mountain(seed=27):
    w = make_highspeed(seed)
    w.update(name='Mountain Gauntlet — elevon learning', control_mode='elevons',
             bounds=[[-2200,-5000,-30],[11200,5600,1800]], goal_radius_m=32,
             physics_version=4)
    p = w['aerodynamics']
    p.update(wing_area_m2=.31,cd0=.012,max_thrust_n=45.,propulsive_power_w=3000.,
             wingspan_m=2.106,mean_chord_m=.31/2.106,inertia_kg_m2=[.4,.30,.60],
             stall_speed_mps=45*.44704,stall_angle_rad=math.radians(15),
             elevon_limit_rad=math.radians(25),servo_rate_rad_s=math.radians(180),
             servo_tau_s=.12,motor_tau_s=.12)
    p.update(pitch_stability=.50,pitch_damping=6.,yaw_stability=.16,yaw_damping=.50)
    w['aircraft_radius_m']=1.6
    p['cl_max'] = 2*p['mass_kg']*9.81/(p['density_kg_m3']*p['wing_area_m2']*p['stall_speed_mps']**2)
    w['powertrain'] = dict(battery_series_cells=12, nominal_voltage_v=44.4, calibration='Estimated thrust and power; motor, propeller, capacity and current limit not supplied')
    w['rewards'] = dict(hoop=100.,crash=-150.,timeout=-150.,complete=500.,progress_per_m=.02)
    w['terrain_collision_margin'] = 5.
    w.pop('residual_scale', None)
    w['terrain']['hills'] = [
        dict(x_m=2600,y_m=-2200,height_m=520,width_m=1100),
        dict(x_m=5100,y_m=2000,height_m=900,width_m=1400),
        dict(x_m=8500,y_m=1000,height_m=670,width_m=1100),
        dict(x_m=6700,y_m=-2600,height_m=760,width_m=1200),
        dict(x_m=2200,y_m=3500,height_m=620,width_m=1300)]
    for hill in w['terrain']['hills']:
        hill['ruggedness'] = .16
    w['start'] = [-1300.,-3500.,terrain_height(-1300.,-3500.,w)+2.]
    w['launcher']['pitch_rad'] = math.radians(20)
    w['launcher']['exit_speed_mps'] = 65*.44704
    w['launcher']['length_m'] = 65*.44704  # Two-second acceleration stroke.
    route = [(-700,-3500),(0,-3500),(1400,-3200),(2600,-2300),(3400,-800),
             (3900,900),(4900,2300),(6400,2800),(8000,2200),(9200,900),
             (9500,-800),(8700,-2400),(7100,-3100),(5500,-2300),(4700,-700),
             (4400,1100),(3400,3000),(1500,4100),(-200,3700),(-1200,2200)]
    rng = random.Random(seed)
    route = [(x,y) if i<2 else (x+rng.uniform(-40,40),y+rng.uniform(-40,40))
             for i,(x,y) in enumerate(route)]
    w['waypoints'] = [[x,y,round(terrain_height(x,y,w)+100+(i%3)*15,2)] for i,(x,y) in enumerate(route)]
    g,h = w['launcher']['pitch_rad'],w['launcher']['heading_rad']
    # Stay aligned with the rail, but require sustained airborne control.
    # The gate is 100 m from the start, about 78 m beyond rail release.
    w['waypoints'][0] = [w['start'][0]+100*math.cos(g)*math.cos(h),
                         w['start'][1]+100*math.cos(g)*math.sin(h),
                         w['start'][2]+100*math.sin(g)-13.]
    w['gate_radii_m'] = [8.]+[32.]*(len(w['waypoints'])-1)
    route_points = [w['start'],*w['waypoints']]
    w['gate_timeout_s'] = gate_budgets(w)
    rng = random.Random(seed)
    w['obstacles'], w['foliage'] = [], []
    for i in range(170):
        x,y = rng.uniform(-1800,10800),rng.uniform(-4500,5200)
        if distance_to_route(x,y,[w['start'],*w['waypoints']])<110:
            continue
        z = terrain_height(x,y,w)
        if z>550:
            continue
        height = rng.uniform(15,32)
        w['obstacles'].append(dict(name=f'Mountain pine {i:03}',kind='tree',
            center=[x,y,z+height/2],size=[14,14,height],variant=i%3))
    return w
