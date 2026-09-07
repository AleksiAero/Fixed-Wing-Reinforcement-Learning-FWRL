"""Training tasks without demonstration trajectories or a flight controller."""
from copy import deepcopy
import math
from .deadlines import gate_budgets
from .landscape import terrain_height

STAGES = [('Launch and straight flight',70), ('Gentle turns',80),
          ('Mountain course',100), ('Mountain speed',150), ('Full speed',200)]

def stage_world(base, stage):
    w = deepcopy(base)
    name,mph = STAGES[stage]
    w.update(name=f'{name} — learning from zero', curriculum_stage=stage,
             training_recipe=2, target_speed_mps=mph*.44704,
             wind_scale=[.2,.5,1.,1.,1.][stage])
    if stage < 2:
        x,y,z = w['start']; h = w['launcher']['heading_rad']
        offsets = [(100,0,22),(350,0,30),(700,0,40)] if stage == 0 else [
            (100,0,22),(400,0,40),(700,100,65),(1050,-80,90),(1400,0,110)]
        w['waypoints'] = []
        for dx,dy,dz in offsets:
            px,py=x+dx*math.cos(h)-dy*math.sin(h),y+dx*math.sin(h)+dy*math.cos(h)
            w['waypoints'].append([px,py,max(z+dz,terrain_height(px,py,w)+20)])
        w['gate_radii_m'] = [18.]+[24.]*(len(offsets)-1)
    else:
        w['gate_radii_m'] = [max(r,[0,0,40,36,0][stage]) for r in w['gate_radii_m']]
    w['gate_timeout_s'] = gate_budgets(w)
    w['rewards']['progress_per_m'] = .05
    return w

def eligible(report):
    return (report['success_rate'] >= .75 and report['stall_fraction'] <= .10
            and report['speed_band_fraction'] >= .60)
