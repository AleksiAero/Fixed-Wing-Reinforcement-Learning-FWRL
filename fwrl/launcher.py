"""Ground-contact launcher hardware shared by Blender and the live viewer.

The launch carriage station and powered stroke are unchanged. The rails extend
back from that station to a ground-contact foot, instead of floating in space.
"""
import math
from .landscape import terrain_height


def geometry(world):
    cfg = world['launcher']
    h,g = cfg['heading_rad'],cfg['pitch_rad']
    start = world['start']
    direction = [math.cos(g)*math.cos(h),math.cos(g)*math.sin(h),math.sin(g)]
    side = [-math.sin(h),math.cos(h),0]
    def axis(distance):
        return [start[i]+distance*direction[i]-(.3 if i==2 else 0) for i in range(3)]
    def clearance(distance):
        p = axis(distance)
        return p[2]-.08*math.cos(g)-terrain_height(p[0],p[1],world)
    # Find where the inclined rails meet the actual terrain behind the carriage.
    back = -1.
    while clearance(back)>0 and back>-200:
        back *= 2
    lo,hi = back,0.
    for _ in range(48):
        mid = (lo+hi)/2
        if clearance(mid)>0: hi=mid
        else: lo=mid
    foot = (lo+hi)/2
    end = cfg['length_m']
    rails=[]
    for sign in [-1,1]:
        p=axis((foot+end)/2)
        rails.append(dict(center=[p[i]+sign*.5*side[i] for i in range(3)],size=[end-foot,.12,.16]))
    supports=[]
    for distance in [foot,0,end*.5,end]:
        p=axis(distance)
        ground=terrain_height(p[0],p[1],world)
        top=p[2]-.08*math.cos(g)
        if top-ground>.15:
            supports.append(dict(center=[p[0],p[1],(ground+top)/2],size=[.3,1.25,top-ground]))
        supports.append(dict(center=[p[0],p[1],ground-.10],size=[1.,1.6,.4]))
    return dict(rails=rails,supports=supports,pitch_rad=g,heading_rad=h,foot=axis(foot))
