"""Batched CUDA point-mass environment and PPO with residual waypoint guidance."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import time
import numpy as np
import torch
from torch import nn
from torch.distributions import Normal
from .dynamics import Aircraft
from .world import load_world
from .landscape import terrain_height
from .aerodynamics import advance, events, initial_state, MPH
from .live import publish


class VectorFlight:
    def __init__(self, world, count=128, device="cpu", seed=1, max_steps=None):
        self.device = torch.device(device)
        self.world = world
        self.n = count
        self.elevons = world.get("control_mode") == "elevons"
        self.direct = world.get("control_mode") in ("direct", "elevons")
        self.physical = "launcher" in world
        self.cfg = Aircraft(dt=.05, max_speed=105.) if self.physical else Aircraft(dt=.1)
        self.max_steps = max_steps if max_steps is not None else (max(12000,int(np.ceil(sum(world.get('gate_timeout_s',[]))/self.cfg.dt))+1) if self.direct else 6000)
        self.rng = torch.Generator(device=self.device).manual_seed(seed)
        self.points = self.tensor(world["waypoints"])
        self.lo, self.hi = self.tensor(world["bounds"])
        self.centers = self.tensor([o["center"] for o in world["obstacles"]]).reshape(-1, 3)
        self.halves = self.tensor([o["size"] for o in world["obstacles"]]).reshape(-1, 3) / 2 + world["aircraft_radius_m"]
        self.state = torch.zeros((count, 20 if self.elevons else 7), device=self.device)
        self.index = torch.zeros(count, dtype=torch.long, device=self.device)
        self.age = torch.zeros_like(self.index)
        self.gate_age = torch.zeros_like(self.index)
        self.gate_timeouts = self.tensor(world.get("gate_timeout_s", [1e9]*len(self.points)))
        self.wind = torch.zeros((count, 3), device=self.device)
        self.reset(torch.ones(count, dtype=torch.bool, device=self.device))

    def tensor(self, value):
        return torch.tensor(value, dtype=torch.float32, device=self.device)

    def reset(self, mask):
        k = int(mask.sum())
        if not k:
            return
        self.state[mask] = self.tensor(initial_state(self.world) if self.physical else [*self.world["start"], 0, 22, 0, 0])
        d = self.points[0] - self.state[mask, :3]
        if not self.physical:
            self.state[mask, 3] = torch.atan2(d[:, 1], d[:, 0]) + .1 * torch.randn(k, generator=self.rng, device=self.device)
        self.index[mask] = 0
        self.age[mask] = 0
        self.gate_age[mask] = 0
        self.wind[mask] = torch.randn((k, 3), generator=self.rng, device=self.device) * self.tensor([1., 1., .1])

    def observation(self):
        s = self.state
        d = self.points[self.index] - s[:, :3]
        heading = torch.atan2(d[:, 1], d[:, 0]) - s[:, 3]
        pieces = [d / 700, torch.sin(heading)[:, None], torch.cos(heading)[:, None], s[:, 4:5] / self.cfg.max_speed, s[:, 5:7], self.wind / 5]
        # Every obstacle box is observed; network dimensions are tied to the world.
        if len(self.centers):
            pieces.extend([((self.centers[None] - s[:, None, :3]) / 700).flatten(1), self.halves.flatten()[None].expand(self.n, -1) / 700])
        if self.direct:
            # Body-relative terrain clearance at three bearings and four ranges.
            for bearing in [-.45, 0., .45]:
                angle = s[:, 3]+bearing
                for distance in [75., 200., 500., 1000.]:
                    ground = terrain_height(s[:, 0]+distance*torch.cos(angle), s[:, 1]+distance*torch.sin(angle), self.world, torch)
                    pieces.append(((s[:, 2]-ground)/1000)[:, None])
            pieces.append(((s[:, 2]-terrain_height(s[:, 0], s[:, 1], self.world, torch))/1000)[:, None])
            for ahead in [1,2]:
                target = self.points[(self.index+ahead).clamp(max=len(self.points)-1)]
                pieces.append((target-s[:, :3])/2000)
            pieces.append((self.age*self.cfg.dt/300)[:, None])
        if self.elevons:
            from .elevon import aero
            alpha,beta,*_ = aero(s,self.world,torch)
            pieces += [s[:,7:11],s[:,14:17]/5,s[:,17:20],alpha[:,None],beta[:,None],
                       (1-self.gate_age*self.cfg.dt/self.gate_timeouts[self.index])[:,None]]
        return torch.cat(pieces, dim=1)

    def baseline(self):
        d = self.points[self.index] - self.state[:, :3]
        error = torch.atan2(d[:, 1], d[:, 0]) - self.state[:, 3]
        error = torch.atan2(torch.sin(error), torch.cos(error))
        return torch.stack([(1.2 * error).clamp(-.7, .7), torch.atan2(d[:, 2], d[:, :2].norm(dim=1).clamp(min=30)).clamp(-.25, .25), torch.full_like(error, self.world.get("target_speed_mps", 22))], dim=1)

    def step(self, action):
        c = self.cfg
        s = self.state
        distance = (self.points[self.index] - s[:, :3]).norm(dim=1)
        if self.direct:
            command = torch.tanh(action)
        else:
            command = self.baseline() + torch.tanh(action) * self.tensor(self.world.get("residual_scale", [.35, .12, 5]))
            command = torch.maximum(torch.minimum(command, self.tensor([c.max_bank, c.max_gamma, c.max_speed])), self.tensor([-c.max_bank, -c.max_gamma, c.min_speed]))
        previous = s[:, :3].clone()
        if self.physical:
            self.state = advance(s, command, self.world, self.age*c.dt, c.dt, self.wind, torch)
            s = self.state
        else:
            s[:, 5] += c.dt * (command[:, 0] - s[:, 5]) / c.bank_tau
            s[:, 6] += c.dt * (command[:, 1] - s[:, 6]) / c.gamma_tau
            s[:, 4] += c.dt * (command[:, 2] - s[:, 4]) / c.speed_tau
            s[:, 3] += c.dt * 9.81 * torch.tan(s[:, 5]) / s[:, 4].clamp(min=c.min_speed)
            s[:, 3] = torch.atan2(torch.sin(s[:, 3]), torch.cos(s[:, 3]))
            velocity = torch.stack([torch.cos(s[:, 6]) * torch.cos(s[:, 3]), torch.cos(s[:, 6]) * torch.sin(s[:, 3]), torch.sin(s[:, 6])], dim=1) * s[:, 4:5]
            s[:, :3] += c.dt * (velocity + self.wind)
        self.age += 1
        self.gate_age += 1
        after = (self.points[self.index] - s[:, :3]).norm(dim=1)
        hit = after < self.world["goal_radius_m"]
        r = self.world["aircraft_radius_m"]
        crash = ((s[:, :3] <= self.lo + r) | (s[:, :3] >= self.hi - r)).any(dim=1)
        crash |= s[:, 2] <= terrain_height(s[:, 0], s[:, 1], self.world, torch) + r
        if len(self.centers):
            crash |= ((s[:, None, :3] - self.centers[None]).abs() <= self.halves[None]).all(dim=2).any(dim=1)
        if self.physical:
            hit, crash = events(previous, s[:, :3], self.index, self.world, torch)
        hit &= ~crash
        success = hit & (self.index == len(self.points) - 1)
        timeout = (self.gate_age*c.dt >= self.gate_timeouts[self.index]) & ~hit & ~crash
        terminal = crash | success | timeout
        truncated = (self.age >= self.max_steps) & ~terminal
        done = terminal | truncated
        reward = (distance - after) * .1 - .01 - .005 * torch.tanh(action).square().sum(dim=1) + hit.float() * 15 + success.float() * 50 - crash.float() * 50
        if self.direct:
            rewards = self.world.get('rewards',dict(hoop=100.,crash=-100.,timeout=-60.,complete=500.,progress_per_m=.02))
            reward = rewards['progress_per_m']*(distance-after) - .01 + hit.float()*rewards['hoop'] + success.float()*rewards['complete'] + crash.float()*rewards['crash'] + (timeout|truncated).float()*rewards['timeout']
        if self.physical:
            flying = self.age*c.dt > 2*self.world["launcher"]["length_m"]/self.world["launcher"]["exit_speed_mps"]
            reward -= (flying & (self.index>0) if self.elevons else flying).float() * .15 * ((s[:, 4]-self.world["target_speed_mps"])/self.world["target_speed_mps"]).square()
            reward += hit.float()*10*torch.exp(-((s[:, 4]-self.world["target_speed_mps"])/4.47).square())
        self.gate_age = torch.where(hit,torch.zeros_like(self.gate_age),self.gate_age)
        self.index = torch.minimum(self.index + hit.long(), torch.full_like(self.index, len(self.points) - 1))
        final_obs = self.observation()
        info = {"terminal": terminal.clone(), "timeout": timeout.clone(), "truncated": truncated.clone(), "final_obs": final_obs.clone(), "final_state": s.clone(), "success": success.clone(), "crash": crash.clone(), "successes": success.sum(), "crashes": crash.sum(), "waypoints": hit.sum()}
        self.reset(done)
        return self.observation(), reward, done, info


class Policy(nn.Module):
    def __init__(self, obs_dim, from_scratch=False):
        super().__init__()
        self.body = nn.Sequential(nn.Linear(obs_dim, 128), nn.Tanh(), nn.Linear(128, 128), nn.Tanh())
        self.actor = nn.Linear(128, 3)
        if not from_scratch:
            nn.init.zeros_(self.actor.weight)
            nn.init.zeros_(self.actor.bias)
        self.critic = nn.Linear(128, 1)
        self.log_std = nn.Parameter(torch.full((3,), -.3 if from_scratch else -1.))

    def forward(self, obs):
        h = self.body(obs)
        return Normal(self.actor(h), self.log_std.clamp(-5, 1).exp()), self.critic(h).squeeze(-1)


def train(args):
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable; install a Blackwell-capable PyTorch wheel in WSL and check nvidia-smi")
    if min(args.envs, args.steps, args.horizon) <= 0:
        raise ValueError("envs, steps and horizon must be positive")
    torch.manual_seed(args.seed)
    torch.set_num_threads(4)
    world = load_world(args.world)
    env = VectorFlight(world, args.envs, args.device, args.seed)
    obs = env.observation()
    model = Policy(obs.shape[1], from_scratch=env.direct).to(args.device)
    optimizer = torch.optim.Adam(model.parameters(), lr=3e-4)
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    metadata = {**vars(args), "world": world, "aircraft": asdict(env.cfg), "torch": torch.__version__, "gpu": torch.cuda.get_device_name() if args.device == "cuda" else None, "observation_dim": obs.shape[1], "algorithm": "PPO from random initialization; left/right elevons and throttle; no guidance" if env.elevons else "PPO from random initialization; direct roll-rate/lift/throttle; no guidance" if env.direct else "PPO residual guidance; tanh-squashed command residuals", "simulation": "quaternion 6-DOF, elevons, post-stall lift/drag, finite thrust" if env.elevons else "force-based 3-DOF with rail launch" if env.physical else "idealized coordinated-turn point mass"}
    metadata['torch'] = str(metadata['torch'])
    (out / "config.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    transitions = 0
    if getattr(args,'resume',None):
        saved = torch.load(args.resume,map_location=args.device,weights_only=True)
        previous = saved['metadata']
        if previous['observation_dim'] != obs.shape[1] or previous['world']['control_mode'] != world['control_mode']:
            raise ValueError('Resume checkpoint has incompatible controls or observations')
        if previous['world']['aerodynamics'] != world['aerodynamics'] and not getattr(args,'allow_physics_change',False):
            raise ValueError('Resume requires unchanged aircraft physics')
        if previous['world']['waypoints'] != world['waypoints'] and not getattr(args,'allow_course_change',False):
            raise ValueError('Changed course requires explicit --allow-course-change')
        model.load_state_dict(saved['model'])
        optimizer.load_state_dict(saved['optimizer'])
        transitions = saved['steps']
    if getattr(args,'start_paused',False):
        (out/'PAUSE').touch()
    start = time.monotonic()
    live_path = getattr(args, 'live_state', None)
    pace = getattr(args, 'realtime', False)
    episode = 1
    live_tick = transitions//env.n
    deadline = time.monotonic()
    def check_owner():
        pid = getattr(args, 'owner_pid', None)
        if pid:
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                (out/'STOP').touch()
    last_reward = 0.
    def stream(status, final=None):
        if live_path:
            publish(live_path, dict(status=status, state=env.state[0].tolist(),
                index=int(env.index[0]), age=int(env.age[0]), episode=episode,
                steps=live_tick*env.n, live_tick=live_tick, run=str(out.resolve()),
                world_name=world['name'], control_mode=world.get('control_mode','residual'),
                reward=last_reward, gate_time_left_s=max(0.,float(env.gate_timeouts[env.index[0]]-env.gate_age[0]*env.cfg.dt)),
                alpha_deg=(float(__import__('fwrl.elevon',fromlist=['aero']).aero(env.state[:1],world,torch)[0][0])*180/np.pi if env.elevons else None),
                event=final, target_mph=world.get('target_speed_mps',22)/MPH))
    stream('starting')
    while transitions < args.steps:
        records = []
        successes = crashes = waypoints = timeouts = 0
        for _ in range(args.horizon):
            check_owner()
            while (out/'PAUSE').exists() and not (out/'STOP').exists():
                check_owner()
                stream('paused')
                time.sleep(.1)
                deadline = time.monotonic()
            if pace:
                deadline = max(deadline+env.cfg.dt, time.monotonic())
                time.sleep(max(0, deadline-time.monotonic()))
            with torch.no_grad():
                distribution, value = model(obs)
                action = distribution.sample()
                logp = distribution.log_prob(action).sum(-1)
                next_obs, reward, done, info = env.step(action)
                # Time-limit transitions bootstrap from pre-reset observations.
                _, next_value = model(info["final_obs"])
                next_value = next_value * (~info["terminal"]).float()
            live_tick += 1
            last_reward = float(reward[0])
            event = None
            if bool(done[0]):
                episode += 1
                event = 'crash' if bool(info['crash'][0]) else ('complete' if bool(info['success'][0]) else 'timeout')
            stream('learning', event)
            records.append((obs, action, logp, value, reward, done, next_value))
            obs = next_obs
            successes += int(info["successes"])
            crashes += int(info["crashes"])
            timeouts += int((info["timeout"] | info["truncated"]).sum())
            waypoints += int(info["waypoints"])
        stream('updating policy')
        ob, ac, lp, val, rew, done, nv = [torch.stack([r[i] for r in records]) for i in range(7)]
        advantage = torch.zeros_like(rew)
        gae = torch.zeros(args.envs, device=args.device)
        for t in reversed(range(args.horizon)):
            delta = rew[t] + .99 * nv[t] - val[t]
            gae = delta + .99 * .95 * (~done[t]).float() * gae
            advantage[t] = gae
        returns = (advantage + val).flatten()
        adv = advantage.flatten()
        adv = (adv - adv.mean()) / (adv.std(unbiased=False) + 1e-8)
        flat_obs, flat_action, old_logp = ob.flatten(0, 1), ac.flatten(0, 1), lp.flatten()
        for _ in range(4):
            for ids in torch.randperm(len(adv), device=args.device).split(1024):
                dist, predicted = model(flat_obs[ids])
                ratio = (dist.log_prob(flat_action[ids]).sum(-1) - old_logp[ids]).exp()
                actor = -torch.minimum(ratio * adv[ids], ratio.clamp(.8, 1.2) * adv[ids]).mean()
                loss = actor + .5 * (predicted - returns[ids]).square().mean() - .001 * dist.entropy().sum(-1).mean()
                if not torch.isfinite(loss):
                    raise RuntimeError("Non-finite training loss")
                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(model.parameters(), .5)
                optimizer.step()
        transitions += args.envs * args.horizon
        metrics = {"steps": transitions, "mean_reward": float(rew.mean()), "successes": successes, "crashes": crashes, "timeouts": timeouts, "waypoints": waypoints, "loss": float(loss.detach()), "transitions_per_second": transitions / (time.monotonic() - start)}
        with (out / "metrics.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(metrics) + "\n")
        print(json.dumps(metrics), flush=True)
        torch.save({"model": model.state_dict(), "optimizer": optimizer.state_dict(), "steps": transitions, "metadata": metadata}, out / "policy.pt")
        if (out / "STOP").exists():
            break
    stream('stopped' if (out/'STOP').exists() else 'finished')
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="worlds/mountain_gauntlet.json")
    parser.add_argument("--live-state", help="Atomic telemetry file for live Blender view")
    parser.add_argument("--resume", help="Continue a compatible checkpoint when changing time budgets")
    parser.add_argument("--allow-physics-change", action="store_true", help="Explicitly transfer compatible policy to revised physics")
    parser.add_argument("--allow-course-change", action="store_true", help="Explicitly retain policy learning on a changed course")
    parser.add_argument("--start-paused", action="store_true")
    parser.add_argument("--owner-pid", type=int, help="Stop and save when the Linux viewer exits")
    parser.add_argument("--realtime", action="store_true", help="Pace training physics to wall time so every launch is visible")
    parser.add_argument("--device", choices=["cpu", "cuda"], default="cuda")
    parser.add_argument("--envs", type=int, default=128)
    parser.add_argument("--steps", type=int, default=1_000_000)
    parser.add_argument("--horizon", type=int, default=128)
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--output", default=f"runs/ppo-{int(time.time())}")
    train(parser.parse_args())


if __name__ == "__main__":
    main()
