from pathlib import Path
import numpy as np
from .dynamics import step, guidance
from .world import collision


def rollout(world, max_steps=12000):
    if "launcher" in world:
        return physical_rollout(world, max_steps)
    state = np.array([*world["start"], 0., 22., 0., 0.])
    states = [state.copy()]
    index = 0
    result = "timeout"
    for _ in range(max_steps):
        state = step(state, guidance(state, world["waypoints"][index]))
        states.append(state.copy())
        if collision(state[:3], world):
            result = "collision"
            break
        if np.linalg.norm(state[:3] - world["waypoints"][index]) < world["goal_radius_m"]:
            index += 1
            if index == len(world["waypoints"]):
                result = "success"
                break
    return np.asarray(states), {"result": result, "waypoints_reached": index, "time_s": (len(states) - 1) * .05}


def make_examples(folder):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    t = np.arange(0, 121, .5)
    examples = {"straight": (22*t, 0*t, 80+0*t), "climb": (22*t, 0*t, 40+2*t),
                "loiter": (150*np.cos(t*22/150), 150*np.sin(t*22/150), 80+0*t),
                "figure_eight": (220*np.sin(t*.075), 130*np.sin(t*.15), 80+0*t)}
    for name, xyz in examples.items():
        np.savetxt(folder / f"{name}.csv", np.column_stack([t, *xyz]), delimiter=",", header="t_s,x_m,y_m,z_m", comments="")


def save_flight(states, path):
    np.savetxt(path, np.column_stack([np.arange(len(states))*.05, states[:, :3], np.rad2deg(states[:, 5])]), delimiter=",", header="t_s,x_m,y_m,z_m,roll_deg", comments="")


def physical_rollout(world, max_steps=12000, policy=None):
    from .aerodynamics import initial_state, advance, events
    from .dynamics import Aircraft
    cfg = Aircraft(max_speed=105.)
    state = initial_state(world)
    states = [state.copy()]
    index = 0
    result = "timeout"
    for tick in range(max_steps):
        command = guidance(state, world['waypoints'][index], cfg)
        command[2] = world['target_speed_mps']
        if policy is not None:
            command += policy(state, index)
        command = np.clip(command, [-cfg.max_bank, -cfg.max_gamma, cfg.min_speed], [cfg.max_bank, cfg.max_gamma, cfg.max_speed])
        new = advance(state[None], command[None], world, np.array([tick*.05]))[0]
        hit, crash = events(state[None, :3], new[None, :3], np.array([index]), world)
        states.append(new.copy())
        state = new
        if crash[0]:
            result = 'collision'
            break
        if hit[0]:
            index += 1
            if index == len(world['waypoints']):
                result = 'success'
                break
    return np.asarray(states), {'result': result, 'waypoints_reached': index, 'time_s': (len(states)-1)*.05}
