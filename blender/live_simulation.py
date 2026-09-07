"""Run this embedded text once (Alt-P) to enable live physics, then press Space.

Blender intentionally requires permission to execute scripts in downloaded files.
The scene also includes a baked flight that plays without enabling scripts.
"""
import bpy
import json
import sys
from pathlib import Path
import numpy as np
from bpy.app.handlers import persistent
from mathutils import Euler

scene = bpy.context.scene
root = Path(scene['project_root'])
if not root.exists():
    root = Path(bpy.data.filepath).resolve().parents[1]
sys.path.insert(0, str(root))
from fwrl.aerodynamics import initial_state, advance, events
from fwrl.dynamics import guidance, Aircraft

world = json.loads(scene['navigation_world_json'])
weights = json.loads(scene.get('policy_weights_json', '{}'))
airframe = bpy.data.objects['Research aircraft 5kg']
airframe.animation_data_clear()
airframe.rotation_mode = 'ZYX'
cfg = Aircraft(max_speed=105.)


def residual(state, index):
    if not weights:
        return np.zeros(3)
    d = np.asarray(world['waypoints'][index])-state[:3]
    h = np.arctan2(d[1], d[0])-state[3]
    obs = np.r_[d/700, np.sin(h), np.cos(h), state[4]/105, state[5:7], [0,0,0]]
    if world['obstacles']:
        centers = np.array([o['center'] for o in world['obstacles']])
        halves = np.array([o['size'] for o in world['obstacles']])/2+world['aircraft_radius_m']
        obs = np.r_[obs, ((centers-state[:3])/700).ravel(), halves.ravel()/700]
    for layer in ['body.0', 'body.2']:
        obs = np.tanh(np.array(weights[layer+'.weight'])@obs+weights[layer+'.bias'])
    return np.tanh(np.array(weights['actor.weight'])@obs+weights['actor.bias'])*world.get('residual_scale', [.35,.12,5])


simulation = dict(state=initial_state(world), index=0, tick=0, last_frame=0, resets=0)


@persistent
def fwrl_live_tick(scene, depsgraph=None):
    sim = simulation
    frame = scene.frame_current
    if frame == 1 or frame <= sim['last_frame']:
        sim.update(state=initial_state(world), index=0, tick=0, last_frame=frame)
    else:
        old = sim['state']
        command = guidance(old, world['waypoints'][sim['index']], cfg)
        command[2] = world['target_speed_mps']
        command += residual(old, sim['index'])
        command = np.clip(command, [-.7,-.25,12], [.7,.25,105])
        new = advance(old[None], command[None], world, np.array([sim['tick']*.05]))[0]
        hit, crash = events(old[None,:3], new[None,:3], np.array([sim['index']]), world)
        sim['state'], sim['tick'] = new, sim['tick']+1
        sim['index'] += int(hit[0])
        if crash[0] or sim['index'] == len(world['waypoints']) or sim['tick']>=6000:
            scene['last_episode'] = 'Crash — launcher reset' if crash[0] else 'Complete / timeout — launcher reset'
            sim.update(state=initial_state(world), index=0, tick=0, resets=sim['resets']+1)
        sim['last_frame'] = frame
    s = sim['state']
    airframe.location = s[:3]
    airframe.rotation_euler = Euler((-s[5], -s[6], s[3]), 'ZYX')
    scene['airspeed_mph'] = float(s[4]/.44704)
    scene['next_hoop'] = sim['index']+1
    scene['crash_reset_count'] = sim['resets']


for handler in list(bpy.app.handlers.frame_change_pre):
    if handler.__name__ == 'fwrl_live_tick':
        bpy.app.handlers.frame_change_pre.remove(handler)
bpy.app.handlers.frame_change_pre.append(fwrl_live_tick)
scene.render.fps = 20
scene.frame_end = 120000
scene.frame_set(1)
scene['playback_mode'] = 'LIVE physics; Space to start/pause; frame 1 to reset'
