"""Evaluate a saved navigation policy on held-out wind seeds."""
import argparse
import json
from pathlib import Path
import torch
from .training import VectorFlight, Policy


def evaluate(checkpoint, device='cuda', count=16, seed=1001):
    saved = torch.load(checkpoint, map_location=device, weights_only=True)
    metadata = saved['metadata']
    env = VectorFlight(metadata['world'], count, device, seed)
    model = Policy(metadata['observation_dim']).to(device)
    model.load_state_dict(saved['model'])
    model.eval()
    live = torch.ones(count, dtype=torch.bool, device=device)
    successes = crashes = timeouts = 0
    returns = torch.zeros(count, device=device)
    trace = []
    gate_speeds = []
    with torch.no_grad():
        for _ in range(env.max_steps):
            dist, _ = model(env.observation())
            if bool(live[0]):
                trace.append(env.state[0].cpu().tolist())
            old_index = env.index.clone()
            old_speed = env.state[:, 4].clone()
            _, reward, done, info = env.step(dist.mean)
            reached = ((env.index > old_index) | info["success"]) & live
            gate_speeds.extend(old_speed[reached].cpu().tolist())
            returns += reward * live
            finished = live & done
            if bool(finished[0]):
                trace.append(info["final_state"][0].cpu().tolist())
            successes += int((finished & info['success']).sum())
            crashes += int((finished & info['crash']).sum())
            timeouts += int((finished & (info['truncated'] | info['timeout'])).sum())
            live &= ~done
            if not bool(live.any()):
                break
    result = {'seed': seed, 'episodes': count, 'successes': successes, 'crashes': crashes, 'timeouts': timeouts, 'success_rate': successes/count, 'mean_return': float(returns.mean()), 'checkpoint': str(checkpoint)}
    result['gate_speed_mph_mean'] = sum(gate_speeds)/len(gate_speeds)/.44704 if gate_speeds else None
    result['gate_speed_mph_min'] = min(gate_speeds)/.44704 if gate_speeds else None
    output = Path(checkpoint).parent
    (output/'blender_flight.json').write_text(json.dumps({'dt': env.cfg.dt, 'states': trace, 'world': metadata['world'], 'source': str(checkpoint)}))
    (output/'evaluation.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    import numpy as np
    np.savetxt(output/'evaluation_flight.csv', np.column_stack([np.arange(len(trace)) * env.cfg.dt, np.array(trace)[:, :3], np.rad2deg(np.array(trace)[:, 5])]), delimiter=',', header='t_s,x_m,y_m,z_m,roll_deg', comments='')
    print(json.dumps(result, indent=2))
    return result


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('checkpoint')
    p.add_argument('--device', choices=['cpu', 'cuda'], default='cuda')
    p.add_argument('--episodes', type=int, default=16)
    p.add_argument('--seed', type=int, default=1001)
    a = p.parse_args()
    if a.episodes <= 0:
        p.error('episodes must be positive')
    evaluate(a.checkpoint, a.device, a.episodes, a.seed)
