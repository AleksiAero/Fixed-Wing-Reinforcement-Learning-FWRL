"""Real-time live evaluation of the policy while independent training runs faster."""
import argparse
import json
import time
from pathlib import Path
import torch
from .training import VectorFlight
from .learning_support import NormalizedPolicy
from .elevon import aero
from .live import publish

def run(folder,path):
    torch.set_num_threads(1)
    checkpoint=folder/'policy.pt';env=model=None;seen=-1;episode=1;tick=0
    next_frame=time.monotonic();last_reward=0.;event=None;status={}
    while True:
        try: status=json.loads((folder/'training-status.json').read_text())
        except (OSError,ValueError):pass
        if env is None or env.age[0]==0:
            try:
                saved=torch.load(checkpoint,map_location='cpu',weights_only=True)
                if saved['steps']!=seen or env is None:
                    world=saved['metadata']['world']
                    model=NormalizedPolicy(saved['metadata']['observation_dim']);model.load_state_dict(saved['model']);model.eval()
                    env=VectorFlight(world,1,'cpu',8000+episode)
                    companion=path.with_name(path.stem+'-world.json')
                    publish(companion,world)
                    seen=saved['steps']
            except (OSError,RuntimeError,EOFError):
                if env is None:time.sleep(.2);continue
        paused=(folder/'PAUSE').exists()
        finished=status.get('status') in ('finished','stopped')
        if not paused and not finished:
            with torch.no_grad():
                _,reward,done,info=env.step(model(env.observation())[0].mean)
            last_reward=float(reward[0]);tick+=1
            event=('crash' if info['crash'][0] else 'complete' if info['success'][0] else 'timeout') if done[0] else None
            if done[0]:episode+=1
        publish(path,dict(status='paused' if paused else status.get('status','learning'),
            view_mode='live policy evaluation',state=env.state[0].tolist(),index=int(env.index[0]),
            age=int(env.age[0]),episode=episode,steps=status.get('steps',seen),policy_steps=seen,live_tick=tick,
            run=str(folder),world_name=world['name'],control_mode='elevons',reward=last_reward,
            stage=world['curriculum_stage'],stage_name=world['name'],target_mph=world['target_speed_mps']/.44704,
            gate_time_left_s=max(0.,float(env.gate_timeouts[env.index[0]]-env.gate_age[0]*env.cfg.dt)),
            alpha_deg=float(aero(env.state,world,torch)[0][0])*180/3.141592653589793,event=event,
            evaluation=status.get('evaluation')))
        if finished:break
        next_frame=max(next_frame+.05,time.monotonic());time.sleep(max(0,next_frame-time.monotonic()))

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--live-state',type=Path,required=True)
    args=p.parse_args();run(args.run.resolve(),args.live_state.resolve())
