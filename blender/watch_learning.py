"""Live training viewer and controls; no keyframes, policy replay or local pilot."""
import bpy
import blf
import json
import os
import subprocess
import time
from pathlib import Path
from mathutils import Euler, Quaternion, Vector

scene = bpy.context.scene
root = Path(scene['project_root'])
if not root.exists():
    root = Path(bpy.data.filepath).resolve().parents[1]
telemetry = root/'runs'/'live'/'state.json'
airframe = bpy.data.objects['Research aircraft 5kg']
airframe.animation_data_clear()
airframe.rotation_mode = 'ZYX'
scene['live_status'] = 'Ready — Start learning from zero'
scene['live_episode'] = 0
scene['live_steps'] = 0
scene['airspeed_mph'] = 0.
scene['next_hoop'] = 1
state = {'process': None, 'run': None, 'packet': None, 'pending_stop': False}


def linux_path(path):
    return subprocess.check_output(['wsl','-d','Ubuntu-24.04','--','wslpath','-a',str(path)], text=True,
                                   creationflags=subprocess.CREATE_NO_WINDOW).strip()


class FWRLStartLearning(bpy.types.Operator):
    bl_idname = 'fwrl.start_learning'
    bl_label = 'Start learning from zero'
    bl_description = 'New random network, no previous policy or flight controller'

    def execute(self, context):
        global telemetry
        if state['process'] and state['process'].poll() is None:
            (state['run']/'PAUSE').unlink(missing_ok=True)
            return {'FINISHED'}
        run = root/'runs'/f'mountain-zero-{time.time_ns()}'
        # Trainer creates this folder exclusively; viewer stores its log beside it.
        state['run'] = run
        telemetry = root/'runs'/'live'/f'{run.name}.json'
        telemetry.parent.mkdir(parents=True, exist_ok=True)
        world_path = telemetry.with_name(f'{run.name}-world.json')
        world_path.write_text(scene['navigation_world_json'], encoding='utf-8')
        def target(p):
            return linux_path(p) if os.name == 'nt' else str(p)
        command = ['bash','scripts/python.sh','-m','fwrl.training','--device',os.environ.get('FWRL_DEVICE','auto'),
                   '--world',target(world_path),'--envs','64','--steps',str(scene.get('learning_steps',10000000)),
                   '--seed',str(time.time_ns() % 2147483647),
                   '--realtime','--live-state',target(telemetry),'--output',target(run)]
        flags = 0
        if os.name != 'nt':
            command += ['--owner-pid',str(os.getpid())]
        if os.name == 'nt':
            command = ['wsl','-d','Ubuntu-24.04','--cd',str(root),'--',*command]
            flags = subprocess.CREATE_NO_WINDOW
        log = telemetry.with_suffix('.log').open('w', encoding='utf-8')
        state['process'] = subprocess.Popen(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,creationflags=flags)
        log.close()
        state['packet'] = None
        state['pending_stop'] = False
        scene['live_status'] = 'Starting fresh random policy…'
        return {'FINISHED'}


class FWRLPauseLearning(bpy.types.Operator):
    bl_idname = 'fwrl.pause_learning'
    bl_label = 'Pause / resume learning'

    def execute(self, context):
        run = state['run']
        if run and run.exists():
            pause = run/'PAUSE'
            if pause.exists():
                pause.unlink()
            else:
                pause.touch()
        return {'FINISHED'}


class FWRLStopLearning(bpy.types.Operator):
    bl_idname = 'fwrl.stop_learning'
    bl_label = 'Stop and save checkpoint'

    def execute(self, context):
        state['pending_stop'] = True
        if state['run'] and state['run'].exists():
            (state['run']/'STOP').touch()
        return {'FINISHED'}


class FWRLLearningPanel(bpy.types.Panel):
    bl_label = 'LIVE · Learn from zero'
    bl_idname = 'FWRL_PT_learning'
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = 'Flight Lab'

    def draw(self, context):
        layout = self.layout
        layout.label(text=scene.get('live_status','Ready'))
        layout.label(text=f"{scene['airspeed_mph']:.1f} mph | Hoop {scene['next_hoop']}/20")
        layout.label(text=f"Episode {scene['live_episode']} | {scene['live_steps']:,} transitions")
        layout.operator('fwrl.start_learning', icon='PLAY')
        layout.operator('fwrl.pause_learning', icon='PAUSE')
        layout.operator('fwrl.stop_learning', icon='FILE_TICK')
        layout.label(text='Current training environment · 1× time')
        layout.label(text='Random start · no flight guidance')


def hud():
    for y,text,size in [(100,'LIVE TRAINING · MOUNTAIN GAUNTLET',20),
                        (72,f"{scene['airspeed_mph']:.1f} mph   |   Hoop {scene['next_hoop']}/20   |   Episode {scene['live_episode']}",17),
                        (46,scene.get('live_status','Ready'),15)]:
        blf.position(0,24,y,0)
        blf.size(0,size)
        blf.color(0,1.,.85,.2,1.)
        blf.draw(0,text)


def poll():
    try:
        if state['pending_stop'] and state['run'] and state['run'].exists():
            (state['run']/'STOP').touch()
        packet = json.loads(telemetry.read_text(encoding='utf-8'))
        # Ignore telemetry from another session/course, including an old file.
        if state['run'] and Path(packet['run']).name == state['run'].name:
            state['packet'] = packet
            s = packet['state']
            airframe.location = s[:3]
            if len(s)>=20:
                airframe.rotation_mode = 'QUATERNION'
                airframe.rotation_quaternion = Quaternion(s[7:11])
                for name,value in [('Left elevon',s[17]),('Right elevon',s[18])]:
                    if name in bpy.data.objects:
                        surface=bpy.data.objects[name]
                        surface.rotation_mode='QUATERNION'
                        surface.rotation_quaternion=Quaternion(Vector(surface.get('hinge_axis',(0,1,0))),value)
                        sign=1 if name.startswith('Left') else -1
                        label='Left' if sign==1 else 'Right'
                        rod=bpy.data.objects.get(label+' pushrod')
                        if rod:
                            start=Vector((.55*(-.22)-.35*.43+.06-.12,sign*.43*1.35,.065))
                            end=surface.rotation_quaternion @ Vector((.55*(-.055)-.35*.18,sign*.18*1.35,.065)) + surface.location
                            rod.location=(start+end)/2
                            rod.rotation_quaternion=(end-start).to_track_quat('Z','Y')
                            rod.scale.z=(end-start).length
            else:
                airframe.rotation_euler = Euler((-s[5],-s[6],s[3]),'ZYX')
            scene['airspeed_mph'] = s[4]/.44704
            scene['next_hoop'] = packet['index']+1
            scene['live_episode'] = packet['episode']
            scene['live_steps'] = packet['steps']
            scene['live_status'] = packet['status'] + f" · {packet.get('gate_time_left_s',0):.1f}s to hoop · reward {packet.get('reward',0):+.1f}"
            if packet.get('event'):
                scene['live_status'] += ' · '+packet['event']+' → launcher reset'
            if time.time()-packet['published_at']>3:
                scene['live_status'] = 'Waiting for trainer — view held'
        proc = state['process']
        if proc and proc.poll() is not None:
            scene['live_status'] = 'Stopped · checkpoint saved' if proc.returncode == 0 else 'Trainer error — see runs/live session log'
        for screen in bpy.data.screens:
            for area in screen.areas:
                if area.type == 'VIEW_3D':
                    area.tag_redraw()
    except (OSError,ValueError,KeyError):
        pass
    return .05


def shutdown():
    if state['run'] and state['run'].exists():
        (state['run']/'STOP').touch()


import atexit
atexit.register(shutdown)
for cls in [FWRLStartLearning,FWRLPauseLearning,FWRLStopLearning,FWRLLearningPanel]:
    bpy.utils.register_class(cls)
bpy.types.SpaceView3D.draw_handler_add(hud,(), 'WINDOW','POST_PIXEL')
bpy.app.timers.register(poll,first_interval=.05,persistent=True)
keyconfig = bpy.context.window_manager.keyconfigs.addon
if keyconfig:
    keymap = keyconfig.keymaps.new(name='3D View',space_type='VIEW_3D')
    keymap.keymap_items.new('fwrl.pause_learning',type='SPACE',value='PRESS')
for screen in bpy.data.screens:
    for area in screen.areas:
        if area.type == 'VIEW_3D':
            area.spaces.active.show_region_ui = True
scene['playback_mode'] = 'LIVE training telemetry — no replay'
