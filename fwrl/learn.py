"""Fresh PPO training with measured curriculum progression and a live policy viewer."""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
import torch
from torch.nn import functional as F
from .training import VectorFlight
from .world import load_world
from .curriculum import stage_world, STAGES, eligible
from .learning_support import NormalizedPolicy, save_checkpoint, evaluate_policy
from .live import publish

ROOT=Path(__file__).resolve().parents[1]

def train(args):
    device=('cuda' if torch.cuda.is_available() else 'cpu') if args.device=='auto' else args.device
    if min(args.envs,args.horizon,args.steps,args.eval_every,args.eval_episodes)<=0:
        raise ValueError('Training counts must be positive')
    torch.manual_seed(args.seed);torch.set_num_threads(4)
    base=load_world(args.world);stage=0;world=stage_world(base,stage)
    env=VectorFlight(world,args.envs,device,args.seed);obs=env.observation()
    model=NormalizedPolicy(obs.shape[1]).to(device)
    optimizer=torch.optim.Adam(model.parameters(),lr=3e-4)
    out=Path(args.output).resolve();out.mkdir(parents=True,exist_ok=False)
    metadata=dict(vars(args),world=world,base_world=base,observation_dim=obs.shape[1],
                  algorithm='PPO curriculum from random initialization; no expert or loaded policy',recipe=2)
    (out/'config.json').write_text(json.dumps(metadata,indent=2),encoding='utf-8')
    steps=updates=stage_steps=streak=0;start=time.monotonic();last_eval=None
    def checkpoint():
        metadata['world']=world
        save_checkpoint(out/'policy.pt',dict(model=model.state_dict(),optimizer=optimizer.state_dict(),
                        steps=steps,stage=stage,metadata=metadata))
    def status(state):
        publish(out/'training-status.json',dict(status=state,steps=steps,stage=stage,
                   stage_name=STAGES[stage][0],target_mph=STAGES[stage][1],evaluation=last_eval))
    checkpoint();status('learning')
    if args.live_state:
        subprocess.Popen([sys.executable,'-m','fwrl.live_policy','--run',str(out),
                          '--live-state',str(Path(args.live_state).resolve())],cwd=ROOT)
    def check_owner():
        if args.owner_pid:
            try:os.kill(args.owner_pid,0)
            except ProcessLookupError:(out/'STOP').touch()
    while steps<args.steps and not (out/'STOP').exists():
        check_owner()
        records=[];crashes=hits=0
        for _ in range(args.horizon):
            check_owner()
            while (out/'PAUSE').exists() and not (out/'STOP').exists():
                check_owner();status('paused');time.sleep(.2)
            if (out/'STOP').exists():break
            with torch.no_grad():
                dist,value=model(obs);action=dist.sample();logp=dist.log_prob(action).sum(-1)
                next_obs,reward,done,info=env.step(action)
                next_value=model(info['final_obs'])[1]*(~info['terminal'])
            records.append((obs,action,logp,value,reward*.01,done,next_value,info['airborne_action']))
            obs=next_obs;crashes+=int(info['crashes']);hits+=int(info['waypoints'])
        if not records:break
        ob,ac,lp,val,rew,done,nv,control=[torch.stack([r[i] for r in records]) for i in range(8)]
        advantage=torch.zeros_like(rew);gae=torch.zeros(args.envs,device=device)
        for t in reversed(range(len(records))):
            delta=rew[t]+args.gamma*nv[t]-val[t]
            gae=delta+args.gamma*args.gae_lambda*(~done[t])*gae
            advantage[t]=gae
        returns=(advantage+val).flatten();adv=advantage.flatten()
        valid=control.flatten();actor_adv=adv[valid]
        adv=(adv-actor_adv.mean())/(actor_adv.std(unbiased=False)+1e-8) if actor_adv.numel() else adv*0
        flat_obs=ob.flatten(0,1);flat_action=ac.flatten(0,1);old_logp=lp.flatten()
        loss=torch.tensor(0.);kl_value=0.
        for epoch in range(4):
            stop_epoch=False
            for ids in torch.randperm(len(adv),device=device).split(1024):
                dist,predicted=model(flat_obs[ids]);logratio=dist.log_prob(flat_action[ids]).sum(-1)-old_logp[ids]
                ratio=logratio.exp();mask=valid[ids]
                surrogate=torch.minimum(ratio*adv[ids],ratio.clamp(.8,1.2)*adv[ids])
                actor=-surrogate[mask].mean() if bool(mask.any()) else surrogate.sum()*0
                entropy=dist.entropy().sum(-1)[mask].mean() if bool(mask.any()) else actor*0
                loss=actor+.5*F.smooth_l1_loss(predicted,returns[ids])-.003*entropy
                if not torch.isfinite(loss):raise RuntimeError('Non-finite PPO loss')
                kl_value=float(((ratio-1)-logratio)[mask].mean().detach()) if bool(mask.any()) else 0.
                if kl_value>.03:stop_epoch=True;break
                optimizer.zero_grad();loss.backward();torch.nn.utils.clip_grad_norm_(model.parameters(),.5);optimizer.step()
            if stop_epoch:break
        # Freeze normalization during rollout and PPO, then update for next batch.
        model.update_normalization(flat_obs)
        added=args.envs*len(records);steps+=added;stage_steps+=added;updates+=1
        metrics=dict(steps=steps,stage=stage,stage_name=STAGES[stage][0],crashes=crashes,waypoints=hits,
                     loss=float(loss.detach()),approx_kl=kl_value,mean_reward=float(rew.mean()),
                     transitions_per_second=steps/(time.monotonic()-start),airborne_samples=int(valid.sum()))
        with (out/'metrics.jsonl').open('a') as f:f.write(json.dumps(metrics)+'\n')
        print(json.dumps(metrics),flush=True);checkpoint();status('learning')
        if updates%args.eval_every==0 and not (out/'STOP').exists():
            status('evaluating')
            # Evaluation gets its own environment and fixed wind seeds. It never updates weights.
            last_eval=evaluate_policy(model,world,device,args.eval_episodes,1001,
                                      max_seconds=120 if stage<2 else 1800,
                                      stop_requested=lambda:(out/"STOP").exists())
            last_eval.update(steps=steps,stage=stage)
            with (out/'evaluations.jsonl').open('a') as f:f.write(json.dumps(last_eval)+'\n')
            streak=streak+1 if eligible(last_eval) else 0
            if streak>=2 and stage_steps>=50000 and stage<len(STAGES)-1:
                stage+=1;stage_steps=streak=0;world=stage_world(base,stage)
                env=VectorFlight(world,args.envs,device,args.seed+stage);obs=env.observation()
                checkpoint()
            status('learning')
    checkpoint();status('stopped' if (out/'STOP').exists() else 'finished')
    return out

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--world',default='worlds/mountain_gauntlet.json')
    p.add_argument('--output',default=f'runs/mountain-zero-curriculum-{time.time_ns()}')
    p.add_argument('--live-state')
    p.add_argument('--owner-pid',type=int)
    p.add_argument('--device',choices=['auto','cpu','cuda'],default='auto')
    p.add_argument('--steps',type=int,default=10000000);p.add_argument('--envs',type=int,default=64)
    p.add_argument('--horizon',type=int,default=512);p.add_argument('--seed',type=int,default=1)
    p.add_argument('--gamma',type=float,default=.999);p.add_argument('--gae-lambda',type=float,default=.98)
    p.add_argument('--eval-every',type=int,default=5);p.add_argument('--eval-episodes',type=int,default=12)
    args=p.parse_args()
    if not 0<args.gamma<1 or not 0<args.gae_lambda<=1:p.error('Invalid gamma or GAE lambda')
    train(args)

if __name__=='__main__':main()
