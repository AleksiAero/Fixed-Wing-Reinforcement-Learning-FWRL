"""Compact observations, flight shaping and checkpointed normalization."""
import os
import time
import numpy as np
import torch
from .elevon import aero, basis
from .landscape import terrain_height
from .training import Policy

def compact_observation(env):
    s=env.state
    axes=torch.stack(basis(s[:,7:11],torch),dim=2)
    body=lambda v: torch.bmm(v[:,None,:],axes).squeeze(1)
    alpha,beta,*_=aero(s,env.world,torch)
    obstacles=torch.zeros((env.n,8,6),device=env.device)
    if len(env.centers):
        offsets=env.centers[None]-s[:,None,:3]
        ids=offsets.square().sum(-1).topk(min(8,len(env.centers)),largest=False).indices
        rel=offsets.gather(1,ids[:,:,None].expand(-1,-1,3))
        obstacles[:,:ids.shape[1],:3]=torch.bmm(rel,axes)/250
        obstacles[:,:ids.shape[1],3:]=env.halves[ids]/50
    clearance=(s[:,2]-terrain_height(s[:,0],s[:,1],env.world,torch))/100
    parts=[body(env.points[env.index]-s[:,:3])/500,body(s[:,11:14])/100,
           s[:,7:11],s[:,14:17]/5,s[:,17:20],alpha[:,None],beta[:,None],
           clearance[:,None],obstacles.flatten(1),body(env.wind)/5,
           torch.full((env.n,1),env.world['target_speed_mps']/100,device=env.device),
           (env.gate_age*env.cfg.dt/env.gate_timeouts[env.index])[:,None]]
    for distance in (75.,200.,500.):
        ground=terrain_height(s[:,0]+distance*torch.cos(s[:,3]),s[:,1]+distance*torch.sin(s[:,3]),env.world,torch)
        parts.append(((s[:,2]-ground)/500)[:,None])
    return torch.cat(parts,dim=1).clamp(-10,10)

def flight_shaping(env,s,command,progress_m):
    alpha,*_=aero(s,env.world,torch)
    stall=(alpha.abs()/env.world['aerodynamics']['stall_angle_rad']-1).clamp(0,2)
    safe_speed=env.world['aerodynamics']['stall_speed_mps']*1.15
    underspeed=((safe_speed-s[:,4])/safe_speed).clamp(0,1)
    target=env.world['target_speed_mps']
    speed_score=torch.exp(-((s[:,4]-target)/(.25*target)).square())
    progress=(progress_m/(env.cfg.dt*target)).clamp(0,1)
    smooth=(command-env.previous_command).square().sum(1)
    return env.cfg.dt*(2*speed_score*progress-8*stall-4*underspeed.square()
                       -.1*s[:,14:17].square().sum(1).clamp(max=25)-2*smooth)

class NormalizedPolicy(Policy):
    def __init__(self,dim):
        super().__init__(dim,from_scratch=True)
        self.register_buffer('obs_mean',torch.zeros(dim))
        self.register_buffer('obs_var',torch.ones(dim))
        self.register_buffer('obs_count',torch.tensor(1e-4))

    @torch.no_grad()
    def update_normalization(self,obs):
        mean,var=obs.mean(0),obs.var(0,unbiased=False)
        count=obs.shape[0];total=self.obs_count+count;delta=mean-self.obs_mean
        self.obs_var.copy_((self.obs_var*self.obs_count+var*count+delta.square()*self.obs_count*count/total)/total)
        self.obs_mean.add_(delta*count/total);self.obs_count.copy_(total)

    def forward(self,obs):
        return super().forward(((obs-self.obs_mean)/(self.obs_var+.01).sqrt()).clamp(-5,5))

def save_checkpoint(path,data):
    temporary=path.with_suffix('.tmp')
    torch.save(data,temporary)
    for attempt in range(6):
        try:os.replace(temporary,path);break
        except PermissionError:
            if attempt==5:raise
            time.sleep(.05)

@torch.no_grad()
def evaluate_policy(model,world,device='cpu',count=12,seed=1001,max_seconds=240,stop_requested=None):
    from .training import VectorFlight
    env=VectorFlight(world,count,device,seed)
    live=torch.ones(count,dtype=torch.bool,device=device)
    first=torch.zeros_like(live);successes=crashes=timeouts=0
    airborne=stalled=in_band=0;speed_sum=duration_sum=0.
    for _ in range(int(max_seconds/env.cfg.dt)):
        if stop_requested and stop_requested():break
        _,_,done,info=env.step(model(env.observation())[0].mean)
        flight=live & info['airborne_action']
        alpha,*_=aero(info['final_state'],world,torch)
        speed=info['final_state'][:,4]
        first |= live & info['hit'] & (env.index==1)
        airborne+=int(flight.sum());stalled+=int((flight & (alpha.abs()>world['aerodynamics']['stall_angle_rad'])).sum())
        in_band+=int((flight & ((speed-world['target_speed_mps']).abs()<.2*world['target_speed_mps'])).sum())
        speed_sum+=float(speed[flight].sum());duration_sum+=int(live.sum())*env.cfg.dt
        finish=live & done
        successes+=int((finish & info['success']).sum());crashes+=int((finish & info['crash']).sum())
        timeouts+=int((finish & (info['timeout']|info['truncated'])).sum())
        live &= ~done
        if not bool(live.any()):break
    return dict(seed=seed,episodes=count,success_rate=successes/count,first_hoop_rate=float(first.float().mean()),
                crashes=crashes,timeouts=timeouts,incomplete=int(live.sum()),
                stall_fraction=stalled/max(1,airborne),speed_band_fraction=in_band/max(1,airborne),
                mean_airborne_mph=speed_sum/max(1,airborne)/.44704,mean_episode_seconds=duration_sum/count)
